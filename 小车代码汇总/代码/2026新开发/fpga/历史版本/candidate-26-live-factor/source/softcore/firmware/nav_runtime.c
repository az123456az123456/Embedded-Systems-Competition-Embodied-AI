#include "nav_runtime.h"
#include "mmio.h"
#include <limits.h>
static void clear(void *v,unsigned n){uint8_t *p=v;while(n--)*p++=0;}
static unsigned event_reason(uint32_t mask)
{
 if(mask&NH_EV_MOT)return NV_R_MANUAL;
 if(mask&(NH_EV_CANCEL|NH_EV_REVOKE))return NV_R_CANCEL;
 if(mask&NH_EV_AUTH)return NV_R_AUTH_EXPIRED;
 if(mask&NH_EV_MOTION)return NV_R_MOTION_EXPIRED;
 if(mask&NH_EV_POSE)return NV_R_POSE_EXPIRED;
 if(mask&NH_EV_REFERENCE)return NV_R_REFERENCE_RESET;
 return NV_R_INTERNAL;
}
void nav_runtime_init(struct nav_runtime *r,struct nav_server *server)
{
 clear(r,sizeof(*r));nav_exec_init(&r->exec,server);nav_track_init(&r->track);nav_avoid_init(&r->avoid);r->last_poll=UINT32_MAX-2u;r->state_pending=1;
}
int nav_runtime_adopt(struct nav_runtime *r)
{
 struct nav_hw_event ev;struct nav_exec *e=&r->exec;unsigned reason;int already,arrived_release;
 if(!nav_mmio_event(&ev)||!(ev.flags&1u)||!ev.session||ev.session!=e->guard.session||!ev.id||!ev.new_epoch||ev.new_epoch<ev.old_epoch)return 0;
 if(r->adopted.session==ev.session&&ev.id<=r->adopted.id)return 0;
 if(r->adopted.session==ev.session&&ev.id-r->adopted.id>1u){uint64_t gap=ev.id-r->adopted.id-1u;r->missed_events=gap>(uint64_t)(UINT32_MAX-r->missed_events)?UINT32_MAX:r->missed_events+(uint32_t)gap;}
 reason=event_reason(ev.reasons);already=e->state==NV_CANCELLED&&e->nav_epoch==ev.new_epoch&&e->reason==reason;
 arrived_release=e->state==NV_ARRIVED&&e->reason==NV_R_ARRIVED&&r->release_ticket&&r->release_session==ev.session&&r->release_run==e->task.run&&r->release_epoch==ev.old_epoch&&r->release_event!=UINT64_MAX&&ev.id==r->release_event+1u&&ev.reasons==NH_EV_REVOKE&&nav_mmio_read(NM_REVOKE_RESULT)==NH_OK;
 e->resume=0;e->plan_due=0;e->owner=0;e->start_request=0;e->intent.linear_mmps=0;e->intent.angular_mradps=0;e->plan_ready=0;e->path_count=0;e->cursor=0;e->remaining_mm=0;
 nx_revoke(&e->guard);e->server->path_valid=0;e->server->path_length=0;astar_invalidate_path(&e->server->planner);e->nav_epoch=ev.new_epoch;
 if(ev.reasons&(NH_EV_REFERENCE|NH_EV_EXHAUSTED))e->rebind_required=1;
 if(e->task.run&&!arrived_release){e->state=NV_CANCELLED;e->reason=(uint16_t)reason;
 if(!already&&e->state_seq!=UINT64_MAX)++e->state_seq;}else if(!e->task.run)e->state=NV_IDLE;
 r->adopted=ev;r->hw_claimed=0;r->task_pending=r->claim_pending=r->fence_pending=r->hw_started=0;r->release_ticket=0;r->state_pending=1;r->plan_pending=1;
 nav_track_init(&r->track);nav_avoid_init(&r->avoid);clear(&r->stop,sizeof(r->stop));return 1;
}
/* Commands are single outstanding asynchronous transactions. Every BUSY result
 * returns to main/A* cooperation; no source or TX wait owns the CPU. */
enum { NR_IDLE=0,NR_AUTH=1,NR_CLAIM=2,NR_PWM=3,NR_REVOKE=4 };
static unsigned status_from_hw(unsigned s)
{
 if(s==NH_OK)return NE_OK;
 if(s==NH_OLD_SOURCE)return NE_STALE;
 if(s==NH_OLD_EPOCH)return NE_CANCELLED;
 if(s==NH_IDENTITY||s==NH_BAD_TOKEN)return NE_IDENTITY;
 if(s==NH_EXPIRED)return NE_NO_PERMIT;
 if(s==NH_BUSY||s==NH_NO_START||s==NH_ACTIVE_OWNER)return NE_BUSY;
 return NE_INTERNAL;
}
static uint32_t next_ticket(struct nav_runtime *r)
{
 if(r->ticket==UINT32_MAX){r->ready=0;r->hw_claimed=0;r->exec.intent.linear_mmps=0;r->exec.intent.angular_mradps=0;return 0;}
 return ++r->ticket;
}
static void write_uuid(unsigned o,const uint8_t *id){unsigned i;for(i=0;i<4;++i)nav_mmio_write(o+4u*i,ne_get32(id+4u*i));}
static void authority(struct nav_runtime *r,unsigned action,unsigned claim)
{
 struct nav_exec *e=&r->exec;unsigned i;uint32_t t=next_ticket(r),pose_words[7];const uint8_t *raw=r->rx+96;
 static const uint8_t pose_offsets[7]={20,16,44,56,0,4,8};
 if(!t)return;
 for(i=0;i<64;++i)nav_mmio_write(NM_AUTH_SHADOW+4u*i,0);
 nav_mmio_write(0x900,action);nav_mmio_write(0x904,t);nav_mmio_write64(0x908,r->hardware.epoch);nav_mmio_write64(0x910,action==NH_INSTALL_SESSION?ne_get64(r->rx+96):e->guard.session);nav_mmio_write64(0x918,e->task.run);write_uuid(0x920,e->task.goal);
 nav_mmio_write64(0x930,action==NH_INSTALL_SESSION?1:action==NH_MAP_FENCE?r->hardware.epoch:e->nav_epoch);nav_mmio_write64(0x938,e->task.map_epoch);nav_mmio_write64(0x940,e->task.map_revision);nav_mmio_write64(0x948,e->task.localization_epoch);nav_mmio_write64(0x950,e->task.plan);nav_mmio_write(0x958,(uint32_t)e->guard.frame_token);
 if(action==NH_ACCEPT_POSE){nav_mmio_write64(0x938,e->pose.map_epoch);nav_mmio_write64(0x940,e->pose.map_revision);nav_mmio_write64(0x948,e->pose.localization_epoch);}
 if(action==NH_MAP_FENCE&&r->lane<4u){nav_mmio_write64(0x938,r->reply_header.map_epoch);nav_mmio_write64(0x940,r->reply_header.map_revision);if(r->lane==NL_POSE)nav_mmio_write64(0x948,r->reply_header.localization_epoch);}
 nav_mmio_write(0x95c,action==NH_MAP_FENCE?(r->lane==NL_POSE?r->token:0):action==NH_ACCEPT_POSE?r->token:r->hardware.pose_token);
 if(action==NH_ACCEPT_POSE||(action==NH_MAP_FENCE&&r->lane==NL_POSE)){
  for(i=0;i<7;++i)pose_words[i]=ne_get32(raw+pose_offsets[i]);
 }else{
  pose_words[0]=(uint32_t)e->guard.pose_seq;pose_words[1]=(uint32_t)(e->guard.pose_seq>>32);pose_words[2]=e->guard.pose_deadline;pose_words[3]=3;pose_words[4]=(uint32_t)e->pose.x_mm;pose_words[5]=(uint32_t)e->pose.y_mm;pose_words[6]=(uint32_t)e->pose.yaw_urad;
 }
 for(i=0;i<7;++i)nav_mmio_write(0x960+i*4u,pose_words[i]);
 nav_mmio_write(0x97c,r->lane==NL_REQUEST?r->token:0);
 nav_mmio_write(0x980,action==NH_ACCEPT_PERMIT?r->token:r->hardware.permit_token);nav_mmio_write64(0x984,e->guard.permit_seq);nav_mmio_write(0x98c,e->guard.permit_deadline);nav_mmio_write64(0x990,e->guard.auth_epoch);nav_mmio_write64(0x998,e->guard.phone_seq);nav_mmio_write(0x9a0,e->guard.auth_deadline);nav_mmio_write(0x9a4,claim==1u?0u:(e->resume!=0));nav_mmio_write(0x9a8,claim<=2u?claim:0);nav_mmio_write(0x9ac,r->hardware.context);nav_mmio_write64(0x9b0,e->start_request);nav_mmio_write64(0x9b8,r->lane==NL_REQUEST?r->reply_header.transport_seq:0);nav_mmio_write(0x9c0,e->active.crc32);
 r->publish_action=(uint8_t)action;r->command_cycles=nav_mmio_read(0x08);
 if(claim==3u){r->stage=NR_REVOKE;nav_mmio_write(NM_SW_REVOKE,t);}
 else if(claim){r->stage=NR_CLAIM;nav_mmio_write(NM_CLAIM,t);}else{r->stage=NR_AUTH;nav_mmio_write(NM_AUTH_PUBLISH,t);}
}
static uint32_t prepare_frame(unsigned class_id,const uint8_t *wire)
{
 unsigned i;uint32_t ticket;nav_mmio_write(NM_TX_CLASS,class_id);
 if(!(nav_mmio_read(NM_TX_READY)&1u))return 0;
 ticket=nav_mmio_read(NM_TX_TICKET);nav_mmio_write(NM_TX_BYTES,NE_BODY_BYTES);
 for(i=0;i<64;++i)nav_mmio_write(NM_TX_BODY+i*4u,ne_get32(wire+i*4u));
 return ticket;
}
static int fast_ack(struct nav_runtime *r,const uint8_t *wire,unsigned lane)
{
 unsigned i,slot=2;if(lane!=NL_PERMIT)return 0;for(i=0;i<2;++i)if(!r->ack_used[i]){slot=i;break;}
 if(slot==2u){
  /* Both slots can contain only real PERMIT ACKs: other kinds return above. */
  slot=0;if(r->ack_dropped!=UINT32_MAX)++r->ack_dropped;
 }
 for(i=0;i<NE_WIRE_BYTES;++i){r->ack[slot][i]=wire[i];}
 r->ack_used[slot]=1;r->ack_lane[slot]=(uint8_t)lane;r->ack_newest=(uint8_t)slot;return 1;
}
static void prefix(struct nav_runtime *r,uint8_t *p,unsigned status,uint32_t now)
{
 clear(p,32);ne_put16(p,(uint16_t)status);ne_put32(p+4,now);ne_put64(p+8,r->exec.nav_epoch);ne_put64(p+16,r->exec.state_seq);
}
static void reply(struct nav_runtime *r,unsigned status,uint32_t now)
{
 struct ne_header h=r->reply_header;struct ne_message m;uint16_t cells[NE_PAGE_CELLS];unsigned n=0,i,off=0;uint32_t ticket;uint8_t *p=r->rx+96;
 if(h.family==NE_NAV3&&h.opcode==NE_PATH_PAGE&&status==NE_OK){m.h=h;m.payload=p;off=ne_get16(p);status=(unsigned)nav_exec_page(&r->exec,&m,cells,NE_PAGE_CELLS,&n);}
 prefix(r,p,status,now);h.opcode|=0x80u;h.payload_bytes=32;
 if((h.opcode&0x7fu)==NE_INFO&&h.family==NE_NAV3){h.payload_bytes=64;ne_put32(p+32,63);ne_put32(p+36,50000000);ne_put64(p+40,r->exec.guard.info_cookie);ne_put64(p+48,r->exec.guard.session);ne_put32(p+56,r->exec.active.crc32);ne_put32(p+60,r->ready?1:0);}
 if((h.opcode&0x7fu)==NE_PATH_PAGE&&h.family==NE_NAV3){h.payload_bytes=(uint16_t)(32+2*n);ne_put16(p+24,(uint16_t)off);ne_put16(p+26,(uint16_t)n);for(i=0;i<n;++i)ne_put16(p+32+2*i,cells[i]);}
 if(nav_mmio_encode_body(&h,r->rx)!=NE_CODEC_OK){nav_mmio_release(r->lane,r->token);return;}
 r->last_ack_seq=h.transport_seq;r->last_request=h.request_id;
 if((ticket=prepare_frame(NT_ACK,r->rx))){nav_mmio_release(r->lane,r->token);nav_mmio_write(NM_TX_SEND,ticket);return;}
 if(r->lane==NL_PERMIT&&fast_ack(r,r->rx,r->lane)){nav_mmio_release(r->lane,r->token);return;}
 /* Ordinary/PAGE/CANCEL cannot be displaced by fast ACK coalescing. Leave its
  * immutable ingress pin held until an actual hardware ACK slot is reserved. */
 for(i=0;i<NE_WIRE_BYTES;++i){r->tx[i]=r->rx[i];}
 r->have_reply=1;r->held_token=r->token;r->held_lane=r->lane;
}
static void finish_job(struct nav_runtime *r,unsigned status,uint32_t now)
{
 r->stage=NR_IDLE;
 if(r->lane==NL_POSE){nav_mmio_release(NL_POSE,r->token);}else if(r->lane<4u){reply(r,status,now);}r->lane=4;
}
static int cancel_notice(struct nav_runtime *r,const struct ne_message *m)
{
 struct nav_exec *e=&r->exec;struct nx_guard g;int rc;uint64_t epoch;
 (void)nav_runtime_adopt(r);g=e->guard;
 if((rc=nx_check_session(&g,&m->h))!=NE_OK)return rc;
 if((rc=nx_identity_check(&e->task,&m->h,NX_ID_TASK))!=NE_OK)return rc;
 /* Hardware has already revoked this exact task before publishing its CPU
  * notification. Adoption is the only local retirement/epoch transition. */
 if(m->h.family==NE_NVA1){
  if(!nav_mmio_live(&r->hardware))return NE_BUSY;
  epoch=ne_get64(m->payload);
  if(epoch!=r->hardware.epoch&&!(r->adopted.session==g.session&&r->adopted.id&&(r->adopted.reasons&NH_EV_REVOKE)&&epoch==r->adopted.old_epoch))return NE_IDENTITY;
 }
 if(e->state!=NV_CANCELLED&&e->state!=NV_ARRIVED)return NE_BUSY;
 g=e->guard; /* Adoption revoked qualification; only advance its lane floor. */
 rc=nx_accept_transport(&g,&m->h);
 if(rc==NE_OK){
  e->guard=g;
  /* After verified arrival SW_REVOKE has already released the hardware owner.
   * A new explicit CANCEL closes that terminal task for stopped map upload;
   * it is not another revoke and must not invent a second hardware epoch. */
  if(e->state==NV_ARRIVED){
   e->state=NV_CANCELLED;e->reason=NV_R_CANCEL;
   if(e->state_seq!=UINT64_MAX)++e->state_seq;
   e->map_quiet=0;clear(&r->stop,sizeof(r->stop));r->state_pending=1;r->plan_pending=1;
  }
 }else if(rc!=NE_STALE)return rc;
 return NE_OK;
}
static void process(struct nav_runtime *r,unsigned lane,uint32_t token,uint32_t rx_ms,uint32_t now)
{
 struct nav_exec *e=&r->exec;struct ne_message m;uint64_t old_epoch=e->nav_epoch;int status;
 r->lane=(uint8_t)lane;r->token=token;r->rx_ms=rx_ms;
 if(lane==NL_POSE||lane==NL_PERMIT)r->fast_turn=(uint8_t)(lane==NL_POSE);
 status=nav_mmio_message(lane,r->rx,&m);
 if(status!=NE_OK){nav_mmio_release(lane,token);r->lane=4;return;}
 r->reply_header=m.h;
 if(lane==NL_POSE&&e->rebind_required){nav_mmio_release(lane,token);r->lane=4;return;}
 if(lane==NL_CANCEL){status=cancel_notice(r,&m);finish_job(r,(unsigned)status,now);return;}
 if(r->ready&&m.h.family==NE_NAV3&&m.h.opcode==NE_SESSION_BIND){
  struct nx_guard shadow=e->guard;status=nx_bind_session(&shadow,&m,now,e->external_quiet&&e->owner==0&&!e->intent.linear_mmps&&!e->intent.angular_mradps&&!astar_busy(&e->server->planner));
  if(status==NE_OK){r->job_ms=now;authority(r,NH_INSTALL_SESSION,0);}else finish_job(r,(unsigned)status,now);return;
 }
 status=r->ready||m.h.opcode==NE_INFO?nav_exec_handle(e,&m,now,nav_mmio_read(0x08)):NE_BUSY;
 if(status==NE_OK){
  if(m.h.family==NE_NAV3&&m.h.opcode==NE_GOAL_PLAN&&e->task.run!=r->installed_run){r->hw_claimed=0;r->hw_started=0;authority(r,NH_INSTALL_TASK,0);return;}
  if(m.h.family==NE_NAV3&&m.h.opcode==NE_START&&!r->hw_claimed){authority(r,0,1);return;}
  if(m.h.family==NE_NAV3&&m.h.opcode==NE_POSE){e->pose.source_age_ms+=(uint32_t)(now-rx_ms);if(e->task.run&&e->state!=NV_CANCELLED&&e->state!=NV_ARRIVED){authority(r,NH_ACCEPT_POSE,0);return;}}
  if(m.h.family==NE_NVA1&&m.h.opcode==NE_PERMIT){authority(r,NH_ACCEPT_PERMIT,0);return;}
 }
 if(e->nav_epoch!=old_epoch&&e->rebind_required&&m.h.family==NE_NAV3&&m.h.opcode==NE_POSE){r->hw_claimed=0;r->reply_status=(uint16_t)status;authority(r,NH_MAP_FENCE,0);return;}
 finish_job(r,(unsigned)status,now);
}
static void command_result(struct nav_runtime *r,uint32_t now)
{
 unsigned off=r->stage==NR_AUTH?NM_AUTH_RESULT:r->stage==NR_CLAIM?NM_CLAIM_RESULT:r->stage==NR_PWM?NM_PWM_RESULT:NM_REVOKE_RESULT;
 unsigned result=nav_mmio_read(off),stage=r->stage;
 /* BUSY also denotes final timeout; hardware ends by1025 cycles. */
 if(result==NH_BUSY&&(uint32_t)(nav_mmio_read(0x08)-r->command_cycles)<5000u)return;
 if(stage==NR_AUTH&&result==NH_OK){
  if(r->publish_action==NH_INSTALL_SESSION){
   struct ne_message m;int committed;m.h=r->reply_header;m.payload=r->rx+96;committed=nav_exec_handle(&r->exec,&m,r->job_ms,nav_mmio_read(0x08));
   if(committed!=NE_OK){r->ready=0;r->hw_claimed=0;r->reply_status=NE_INTERNAL;nav_mmio_write(0x28,0x3101);nav_mmio_write(0x14,nav_mmio_read(0x14)|NAV_FW_FAILED);}
   else{r->adopted.session=r->exec.guard.session;r->adopted.id=0;r->hw_claimed=0;r->hw_started=0;r->installed_run=0;r->installed_plan=0;r->task_pending=0;r->claim_pending=0;r->fence_pending=0;nav_track_init(&r->track);nav_avoid_init(&r->avoid);clear(&r->stop,sizeof(r->stop));(void)nav_runtime_adopt(r);}
  }
  if(r->publish_action==NH_INSTALL_TASK){r->installed_run=r->exec.task.run;r->installed_plan=r->exec.task.plan;r->task_pending=0;
 if(r->hw_started&&r->exec.resume&&r->exec.plan_ready)r->claim_pending=1;}
  if(r->publish_action==NH_MAP_FENCE)r->fence_pending=0;
 }
 if(stage==NR_CLAIM){r->hw_claimed=(uint8_t)(result==NH_OK);
 if(result==NH_OK){r->hw_started=1;r->claim_pending=0;}}
 if(result!=NH_OK&&stage!=NR_PWM)r->hw_claimed=0;
 if(stage==NR_PWM||stage==NR_REVOKE){r->stage=NR_IDLE;r->lane=4;return;}
 finish_job(r,r->reply_status?r->reply_status:status_from_hw(result),now);r->reply_status=0;
}
static __attribute__((noinline)) void event_header(struct nav_runtime *r,struct ne_header *h,unsigned opcode,unsigned bytes)
{
 unsigned i;struct nav_exec *e=&r->exec;clear(h,sizeof(*h));h->opcode=(uint8_t)opcode;h->payload_bytes=(uint16_t)bytes;h->session_id=e->guard.session;h->transport_seq=++r->tx_seq;h->run_generation=e->task.run;for(i=0;i<16;++i)h->goal_id[i]=e->task.goal[i];h->map_epoch=e->active.epoch;h->map_revision=e->active.revision;h->localization_epoch=e->task.localization_epoch;h->plan_id=e->task.plan;
}
static void state_frame(struct nav_runtime *r,uint32_t now)
{
 struct nav_exec *e=&r->exec;struct ne_header h;uint8_t *p=r->rx+96;uint32_t flags=0,ticket;clear(p,160);event_header(r,&h,NE_STATE,160);
 if(r->hw_claimed&&nav_exec_motion_allowed(e,now)&&!r->hardware.gates)flags|=1;
 if(e->plan_ready)flags|=2;
 if(e->state==NV_ARRIVED)flags|=4;
 if(e->reason==NV_R_NO_ROUTE)flags|=8;
 if(e->rebind_required)flags|=16;
 if(nx_pose_valid(&e->guard,now))flags|=32;
 if(nx_motion_valid(&e->guard,now))flags|=64;
 if(nx_authorized(&e->guard,now))flags|=128;
 p[0]=e->state;p[1]=e->avoid_phase;ne_put16(p+2,e->reason);ne_put32(p+4,flags);ne_put64(p+8,e->nav_epoch);ne_put64(p+16,e->state_seq);ne_put64(p+24,e->guard.pose_seq);ne_put32(p+32,(uint32_t)e->pose.x_mm);ne_put32(p+36,(uint32_t)e->pose.y_mm);ne_put32(p+40,(uint32_t)e->pose.yaw_urad);ne_put32(p+44,e->guard.pose_seq?e->pose.source_age_ms+(uint32_t)(now-e->pose.accepted_ms):UINT32_MAX);
 ne_put16(p+48,e->cursor);ne_put16(p+50,e->path_count);ne_put32(p+52,e->remaining_mm);ne_put32(p+56,(flags&64u)?e->guard.permit_deadline-now:0);ne_put32(p+60,(flags&128u)?e->guard.auth_deadline-now:0);ne_put32(p+64,e->active.crc32);ne_put32(p+68,0);ne_put32(p+72,e->upload.valid?e->upload.crc32:0);ne_put32(p+76,r->hardware.gates);
 ne_put64(p+80,0);ne_put64(p+88,0);ne_put64(p+96,e->upload.valid?e->upload.epoch:0);ne_put64(p+104,e->upload.valid?e->upload.revision:0);ne_put64(p+112,r->last_ack_seq);ne_put32(p+120,now);ne_put32(p+124,r->hardware.owner==NH_OWNER_NAV?NE_OWNER_NAV:NE_OWNER_NONE);ne_put16(p+128,e->upload_chunks);ne_put32(p+132,r->ack_dropped);ne_put64(p+136,r->last_request);ne_put32(p+144,e->active.frame_token);ne_put32(p+148,0);
 if(nav_mmio_encode_body(&h,r->rx)==NE_CODEC_OK&&(ticket=prepare_frame(NT_STATE,r->rx))){nav_mmio_write(NM_TX_SEND,ticket);r->state_pending=0;r->last_state_ms=now;r->last_state_seq=e->state_seq;}
}
static void plan_frame(struct nav_runtime *r,uint32_t now)
{
 struct nav_exec *e=&r->exec;struct ne_header h;uint8_t *p=r->rx+96;uint32_t ticket;unsigned status=e->reason==NV_R_NO_ROUTE?NE_NO_ROUTE:e->plan_ready?NE_OK:NE_CANCELLED;clear(p,160);event_header(r,&h,NE_PLAN_RESULT,64);
 ne_put16(p,(uint16_t)status);ne_put16(p+2,e->reason);ne_put32(p+4,now);ne_put64(p+8,e->nav_epoch);ne_put64(p+16,e->state_seq);ne_put64(p+24,e->start_pose_seq);ne_put32(p+32,e->server->planner.expanded);ne_put32(p+36,e->server->plan_cycles);ne_put16(p+40,e->plan_ready?e->path_count:0);ne_put16(p+42,e->start_cell);ne_put16(p+44,e->goal_cell);ne_put16(p+46,e->plan_ready?1:0);ne_put32(p+48,e->planned_revision);ne_put32(p+52,e->active.frame_token);ne_put32(p+56,50000000);
 if(nav_mmio_encode_body(&h,r->rx)==NE_CODEC_OK&&(ticket=prepare_frame(NT_EVENT,r->rx))){nav_mmio_write(NM_TX_SEND,ticket);r->plan_pending=0;}
}
static void transmit(struct nav_runtime *r,uint32_t now)
{
 struct nav_exec *e=&r->exec;unsigned i;uint32_t ticket;
 if(e->state_seq!=r->last_state_seq)r->state_pending=1;
 if(e->task.plan!=r->last_plan||e->plan_ready!=r->last_plan_valid){r->plan_pending=1;r->last_plan=e->task.plan;r->last_plan_valid=e->plan_ready;}
 if(r->have_reply&&(ticket=prepare_frame(NT_ACK,r->tx))){nav_mmio_release(r->held_lane,r->held_token);nav_mmio_write(NM_TX_SEND,ticket);r->have_reply=0;}
 for(i=0;i<2;++i)if(r->ack_used[i]&&(ticket=prepare_frame(NT_ACK,r->ack[i]))){nav_mmio_write(NM_TX_SEND,ticket);r->ack_used[i]=0;}
 if(!r->exec.guard.session||r->stage)return;
 if(r->plan_pending&&r->exec.task.run)plan_frame(r,now);
 if(r->state_pending||(uint32_t)(now-r->last_state_ms)>=100u)state_frame(r,now);
}
static void pwm(struct nav_runtime *r,const struct nv_intent *intent,const struct nav_hw_live *computed,const struct lc_output *out,uint32_t now)
{
 struct nav_exec *e=&r->exec;struct nav_hw_live current;uint32_t t;unsigned i;
 if(nav_runtime_adopt(r)||!nav_mmio_live(&current)||!r->hw_claimed||!nav_exec_motion_allowed(e,now)||current.context!=computed->context||current.epoch!=computed->epoch||current.pose_seq!=intent->pose_seq||current.permit_seq!=intent->permit_seq||intent->nav_epoch!=e->nav_epoch||intent->run!=e->task.run||intent->plan!=e->task.plan||intent->pose_seq!=e->guard.pose_seq||intent->permit_seq!=e->guard.permit_seq)return;
 t=next_ticket(r);if(!t)return;
 for(i=0;i<64;++i)nav_mmio_write(NM_PWM_SHADOW+i*4u,0);
 nav_mmio_write(0xa00,t);nav_mmio_write(0xa04,NH_OWNER_NAV);nav_mmio_write64(0xa08,e->guard.session);nav_mmio_write64(0xa10,intent->run);write_uuid(0xa18,e->task.goal);nav_mmio_write64(0xa28,intent->nav_epoch);nav_mmio_write64(0xa30,intent->pose_seq);nav_mmio_write64(0xa38,intent->permit_seq);nav_mmio_write64(0xa40,computed->auth_epoch);nav_mmio_write64(0xa48,intent->plan);nav_mmio_write64(0xa50,e->task.map_epoch);nav_mmio_write64(0xa58,e->task.map_revision);nav_mmio_write64(0xa60,e->task.localization_epoch);nav_mmio_write(0xa68,(uint32_t)e->guard.frame_token);nav_mmio_write(0xa6c,out->flags);
 for(i=0;i<4;++i)nav_mmio_write(0xa70+i*4u,(uint32_t)(int32_t)out->duty[i]);
 nav_mmio_write(0xa80,(uint32_t)intent->linear_mmps);nav_mmio_write(0xa84,(uint32_t)intent->angular_mradps);nav_mmio_write(0xa88,computed->pose_token);nav_mmio_write(0xa8c,computed->permit_token);nav_mmio_write(0xa90,computed->context);
 r->stage=NR_PWM;r->lane=4;r->command_cycles=nav_mmio_read(0x08);nav_mmio_write(NM_PWM_COMMIT,t);
}
static void control(struct nav_runtime *r,uint32_t now)
{
 struct nav_exec *e=&r->exec;struct nv_stop_sample sample;struct nv_intent computed;struct nav_hw_live hw=r->hardware;unsigned before=e->state;int nav,commit,avoid=NV_AVOID_INACTIVE;
 if(!layered_snapshot()){nav_exec_set_map_quiet(e,0);return;}
 if((e->state==NV_IDLE||e->state==NV_CANCELLED)&&r->hardware.owner!=NH_OWNER_NAV&&e->external_quiet){
  if(!r->stop.active)nv_stop_begin(&r->stop,now);
  layered_stop_sample(&sample);nav_exec_set_map_quiet(e,nv_stop_update(&r->stop,&sample));
 }else nav_exec_set_map_quiet(e,0);
 nav=e->resume||r->hardware.owner==NH_OWNER_NAV;
 if(!r->stage&&r->hw_claimed&&nav_exec_motion_allowed(e,now)){
  layered_stop_sample(&sample);avoid=nav_avoid_update(e,&r->avoid,&r->stop,&sample,layered_raw(),&layered_state);
  if(avoid==NV_AVOID_INACTIVE&&e->state==NV_FOLLOWING)nav_track_update(e,&r->track,&r->stop,&sample);
 }
 if(before==NV_AVOIDING&&e->state==NV_PLANNING){r->fence_pending=1;r->hw_claimed=0;}
 computed=e->intent;
 if(!r->hw_claimed||!nav_exec_motion_allowed(e,now)){computed.linear_mmps=0;computed.angular_mradps=0;}
 /* A pending source publication or an old high-level intent must not consume
  * a PI release which cannot be submitted with its original source context.
  * Zero/authority-loss/calibration observations keep the immediate path. */
 if((computed.linear_mmps||computed.angular_mradps)&&
    (r->stage||computed.pose_seq!=e->guard.pose_seq||computed.permit_seq!=e->guard.permit_seq))return;
 commit=layered_step(nav,computed.linear_mmps,computed.angular_mradps,(uint32_t)e->nav_epoch);
 if(commit){layered_publish_legacy(nav);if(!r->stage&&r->hw_claimed)pwm(r,&computed,&hw,layered_output(),now);}
 if(e->state==NV_ARRIVED&&r->hw_started&&!r->stage&&!r->release_ticket){
  r->lane=4;r->release_session=e->guard.session;r->release_run=e->task.run;r->release_epoch=r->hardware.epoch;r->release_event=r->adopted.session==e->guard.session?r->adopted.id:0;authority(r,0,3);r->release_ticket=r->ticket;
 }
}
void nav_runtime_service(struct nav_runtime *r)
{
 uint32_t now=nav_mmio_read(NM_NOW),cycles=nav_mmio_read(0x08),gap,token,rx_ms;struct nav_exec *e=&r->exec;struct nav_hw_live hw;unsigned lane;
 gap=cycles-r->last_service_cycle;
 if(r->last_service_cycle&&gap>r->max_service_gap_cycles)r->max_service_gap_cycles=gap;
 r->last_service_cycle=cycles;
 if((uint32_t)(now-r->last_poll)<2u)return;
 r->last_poll=now;(void)nav_runtime_adopt(r);
 if(!nav_mmio_live(&hw))return;
 r->hardware=hw;nav_exec_set_external_quiet(e,(hw.live&NH_LIVE_MANUAL_QUIET)!=0);
 if(r->stage)command_result(r,now);
 nav_exec_poll(e,now);control(r,now);
 /* Publish a replan before another source transaction or continuation claim.
  * TX backpressure retains the pending notice but never blocks fast ingress. */
 if(!r->stage&&r->claim_pending)transmit(r,now);
 /* A newly computed PI transaction has priority over another authority update;
  * otherwise phase-locked50Hz POSE traffic could starve every20ms PWM result. */
 if(!r->stage){
  if(r->fence_pending){r->lane=4;authority(r,NH_MAP_FENCE,0);}
  else if(r->task_pending&&e->plan_ready){r->lane=4;authority(r,NH_INSTALL_TASK,0);}
  else{
   uint8_t order[4];order[0]=NL_CANCEL;order[1]=(uint8_t)(r->fast_turn?NL_PERMIT:NL_POSE);order[2]=(uint8_t)(r->fast_turn?NL_POSE:NL_PERMIT);order[3]=NL_REQUEST;
   for(lane=0;lane<4;++lane){unsigned which=order[lane];
    if(r->have_reply&&(which==NL_REQUEST||which==NL_CANCEL))continue;
    if(nav_mmio_pin(which,&token,&rx_ms)){process(r,which,token,rx_ms,now);break;}
   }
   if(!r->stage&&r->claim_pending&&r->hw_started&&e->resume&&e->plan_ready&&r->hardware.pose_seq==e->guard.pose_seq&&r->hardware.pose_token){r->lane=4;authority(r,0,2);}
  }
 }
 transmit(r,now);
}
void nav_runtime_plan(struct nav_runtime *r)
{
 struct nav_exec *e=&r->exec;uint32_t began,now;int status;
 if(!r->ready||r->stage||r->task_pending||r->fence_pending||!e->plan_due||astar_busy(&e->server->planner))return;
 now=nav_mmio_read(NM_NOW);began=nav_mmio_read(0x08);status=nav_exec_plan(e,now);e->server->plan_cycles=nav_mmio_read(0x08)-began;
 if(status!=NE_BUSY&&status!=NE_POSE_STALE){r->plan_pending=1;r->state_pending=1;}
 if(status==NE_OK&&e->plan_ready){r->task_pending=1;r->hw_claimed=0;}
 NAV_PATH_LENGTH=e->plan_ready?e->path_count:0;NAV_ASTAR_CYCLES=e->server->plan_cycles;NAV_EXPANDED=e->server->planner.expanded;
}
