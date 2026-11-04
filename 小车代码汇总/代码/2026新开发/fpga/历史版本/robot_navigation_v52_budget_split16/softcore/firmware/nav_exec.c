#include "nav_exec.h"
#include <limits.h>

static void zero(void *v,unsigned n){uint8_t *p=v;while(n--)*p++=0;}
static void copy(void *v,const void *s,unsigned n){uint8_t *p=v;const uint8_t *q=s;while(n--)*p++=*q++;}
static int same(const void *a,const void *b,unsigned n){const uint8_t *p=a,*q=b;while(n--)if(*p++!=*q++)return 0;return 1;}
static void stop(struct nav_exec *e){e->intent.linear_mmps=0;e->intent.angular_mradps=0;}
static void changed(struct nav_exec *e,unsigned state,unsigned reason){e->state=(uint8_t)state;e->reason=(uint16_t)reason;if(e->state_seq!=UINT64_MAX)++e->state_seq;stop(e);}
static void invalidate(struct nav_exec *e){e->plan_ready=0;e->path_count=0;e->cursor=0;e->remaining_mm=0;e->server->path_valid=0;e->server->path_length=0;astar_invalidate_path(&e->server->planner);}
static void retire(struct nav_exec *e,unsigned reason)
{
 e->resume=0;e->plan_due=0;e->owner=NE_OWNER_NONE;e->start_request=0;
 if(e->nav_epoch!=UINT64_MAX)++e->nav_epoch;
 nx_revoke(&e->guard);invalidate(e);changed(e,NV_CANCELLED,reason);
}
static int task_live(const struct nav_exec *e){return e->task.run&&!e->rebind_required&&e->state!=NV_CANCELLED&&e->state!=NV_ARRIVED;}
static int map_identity(const struct nv_map *map,const struct ne_header *h)
{return map->valid&&map->revision==h->map_revision;}
static int map_same(const struct nv_map *a,const struct nv_map *b)
{return a->valid&&b->valid&&a->revision==b->revision&&a->transaction==b->transaction&&a->origin_x_mm==b->origin_x_mm&&a->origin_y_mm==b->origin_y_mm&&a->frame_token==b->frame_token&&a->crc32==b->crc32;}
static int transport(struct nav_exec *e,const struct ne_message *m){return nx_accept_transport(&e->guard,&m->h);}
static void replan(struct nav_exec *e,unsigned why)
{
 invalidate(e);e->plan_due=1;changed(e,NV_PLANNING,why);
}
void nav_exec_init(struct nav_exec *e,struct nav_server *server)
{zero(e,sizeof(*e));e->server=server;nx_guard_init(&e->guard);e->nav_epoch=1;}
int nav_exec_cell(const struct nv_map *m,int32_t x,int32_t y,uint16_t *cell)
{
 uint32_t dx,dy;if(!m->valid||!cell||x<m->origin_x_mm||y<m->origin_y_mm)return 0;
 dx=(uint32_t)x-(uint32_t)m->origin_x_mm;dy=(uint32_t)y-(uint32_t)m->origin_y_mm;
 if(dx>=6400u||dy>=6400u)return 0;
 *cell=(uint16_t)((dy/100u)*64u+dx/100u);return 1;
}
static void activate(struct nav_exec *e,const struct nv_map *m,const uint8_t *bitmap)
{
 astar_clear_dynamic(&e->server->planner);copy(e->server->planner.occupied,bitmap,NE_MAP_BYTES);copy(&e->active,m,sizeof(e->active));
 e->server->map_valid=1;e->server->generation=(uint32_t)m->revision;
 invalidate(e);
}
/* Only the idle/cancelled, physically stopped owner may replace the map.
 * No pending map exists; a running task keeps exactly one immutable bitmap. */
static int map_writable(const struct nav_exec *e)
{return (e->state==NV_IDLE||e->state==NV_CANCELLED)&&!e->resume&&!e->owner&&e->external_quiet&&e->map_quiet&&!astar_busy(&e->server->planner);}
static int map_begin(struct nav_exec *e,const struct ne_message *m)
{
 struct nv_map n;const uint8_t *p=m->payload;int rc;
 if(!map_writable(e))return NE_BUSY;
 zero(&n,sizeof(n));n.epoch=1;n.revision=m->h.map_revision;n.transaction=ne_get64(p+20);
 n.origin_x_mm=(int32_t)ne_get32(p+4);n.origin_y_mm=(int32_t)ne_get32(p+8);n.frame_token=ne_get32(p);n.crc32=ne_get32(p+16);n.valid=1;
 if(!e->guard.frame_token||n.frame_token!=e->guard.frame_token)return NE_REBIND;
 if(n.origin_x_mm>INT32_MAX-6400||n.origin_y_mm>INT32_MAX-6400)return NE_BAD_REQUEST;
 if(map_same(&e->upload,&n)||map_same(&e->active,&n))return NE_OK;
 if(e->active.valid&&n.revision<=e->active.revision)return NE_STALE;
 rc=transport(e,m);if(rc)return rc;copy(&e->upload,&n,sizeof(n));e->upload_chunks=0;e->server->upload_active=1;
 return NE_OK;
}
static int map_chunk(struct nav_exec *e,const struct ne_message *m)
{
 const uint8_t *p=m->payload;unsigned off=ne_get16(p+8),bit=1u<<(off/32u);int rc;
 if(!map_writable(e))return NE_BUSY;
 if(!map_identity(&e->upload,&m->h)||e->upload.transaction!=ne_get64(p))return NE_IDENTITY;
 if(e->upload_chunks&bit)return same(e->server->staging+off,p+16,32)?NE_OK:NE_BAD_REQUEST;
 rc=transport(e,m);if(rc)return rc;copy(e->server->staging+off,p+16,32);e->upload_chunks|=(uint16_t)bit;return NE_OK;
}
static int map_commit(struct nav_exec *e,const struct ne_message *m)
{
 const uint8_t *p=m->payload;int rc;
 if(!map_writable(e))return NE_BUSY;
 if(map_identity(&e->active,&m->h)&&e->active.transaction==ne_get64(p)&&e->active.crc32==ne_get32(p+8))return NE_OK;
 if(!map_identity(&e->upload,&m->h)||e->upload.transaction!=ne_get64(p))return NE_IDENTITY;
 if(e->upload_chunks!=UINT16_MAX)return NE_INCOMPLETE;
 if(e->upload.crc32!=ne_get32(p+8)||nav_crc32(e->server->staging,NE_MAP_BYTES)!=e->upload.crc32)return NE_MAP_CRC;
 rc=transport(e,m);if(rc)return rc;
 activate(e,&e->upload,e->server->staging);
 e->upload.valid=0;e->server->upload_active=0;
 /* A new map never inherits a task, pose qualification or START. Run replay
  * floor stays intact. New POSE + GOAL + PERMIT + START must be explicit. */
 zero(&e->task,sizeof(e->task));e->goal_request=0;e->start_request=0;e->plan_due=0;e->resume=0;nx_revoke(&e->guard);
 changed(e,NV_IDLE,NV_R_NONE);return NE_OK;
}
static int pose(struct nav_exec *e,const struct ne_message *m,uint32_t now)
{
 struct nx_guard g;const uint8_t *p=m->payload;int rc;
 copy(&g,&e->guard,sizeof(g));
 if(!map_identity(&e->active,&m->h))return NE_IDENTITY;
 /* Reject old transport before a semantic reference-reset side effect.
  * Both checks use a shadow: a rejected pose still cannot consume a floor. */
 rc=nx_accept_transport(&g,&m->h);if(rc)return rc;
 rc=nx_accept_pose(&g,m,now);if(rc){if(rc==NE_REBIND){e->rebind_required=1;retire(e,NV_R_REFERENCE_RESET);}return rc;}
 if(e->task.run&&m->h.localization_epoch!=e->task.localization_epoch){e->rebind_required=1;retire(e,NV_R_REFERENCE_RESET);g.live&=NX_LIVE_POSE;g.nav_epoch=0;g.permit_deadline=0;g.auth_deadline=0;}
 copy(&e->guard,&g,sizeof(g));e->pose.x_mm=(int32_t)ne_get32(p);e->pose.y_mm=(int32_t)ne_get32(p+4);e->pose.yaw_urad=(int32_t)ne_get32(p+8);
 e->pose.map_epoch=m->h.map_epoch;e->pose.map_revision=m->h.map_revision;e->pose.localization_epoch=m->h.localization_epoch;
 e->pose.accepted_ms=now;e->pose.source_age_ms=(uint32_t)ne_get64(p+36)/1000000u;return NE_OK;
}
static int goal(struct nav_exec *e,const struct ne_message *m,uint32_t now)
{
 uint16_t start,end;const uint8_t *p=m->payload;int rc;
 if(m->h.request_id==e->goal_request&&nx_identity_check(&e->task,&m->h,NX_ID_ALL&~NX_ID_PLAN)==NE_OK&&
    e->goal_x_mm==(int32_t)ne_get32(p)&&e->goal_y_mm==(int32_t)ne_get32(p+4)&&e->start_pose_seq==ne_get64(p+12))return task_live(e)?NE_OK:NE_CANCELLED;
 if(astar_busy(&e->server->planner))return NE_BUSY;
 if(e->upload.valid)return NE_BUSY;
 if(!map_identity(&e->active,&m->h))return e->active.valid?NE_IDENTITY:NE_NO_MAP;
 if(ne_get32(p+8)!=e->active.frame_token||m->h.localization_epoch!=e->pose.localization_epoch)return NE_REBIND;
 if(!nx_pose_valid(&e->guard,now)||ne_get64(p+12)!=e->guard.pose_seq)return NE_POSE_STALE;
 if(e->pose.map_epoch!=e->active.epoch||e->pose.map_revision!=e->active.revision)return NE_POSE_STALE;
 if(m->h.run_generation<=e->run_floor)return NE_STALE;
 if(!nav_exec_cell(&e->active,e->pose.x_mm,e->pose.y_mm,&start)||!nav_exec_cell(&e->active,(int32_t)ne_get32(p),(int32_t)ne_get32(p+4),&end))return NE_GOAL_OUTSIDE_WINDOW;
 if(e->nav_epoch==UINT64_MAX||e->plan_counter==UINT64_MAX)return NE_INTERNAL;
 rc=transport(e,m);if(rc)return rc;
 e->resume=0;e->owner=0;e->guard.live&=(uint8_t)~(NX_LIVE_MOTION|NX_LIVE_AUTH);e->guard.nav_epoch=0;
 astar_clear_dynamic(&e->server->planner);
 ++e->nav_epoch;e->run_floor=m->h.run_generation;nx_identity_from_header(&e->task,&m->h);e->task.plan=0;

 e->goal_request=m->h.request_id;e->start_request=0;e->goal_x_mm=(int32_t)ne_get32(p);e->goal_y_mm=(int32_t)ne_get32(p+4);
 e->start_pose_seq=e->guard.pose_seq;e->start_cell=start;e->goal_cell=end;e->rebind_required=0;replan(e,NV_R_AWAIT_PLAN);return NE_OK;
}
int nav_exec_motion_allowed(const struct nav_exec *e,uint32_t now)
{
 return e->owner==NE_OWNER_NAV&&e->resume&&!e->rebind_required&&
 (e->state==NV_FOLLOWING||e->state==NV_AVOIDING)&&
 nx_pose_valid(&e->guard,now)&&nx_motion_valid(&e->guard,now)&&nx_authorized(&e->guard,now)&&
 e->guard.nav_epoch==e->nav_epoch&&e->pose.localization_epoch==e->task.localization_epoch;
}
static int start(struct nav_exec *e,const struct ne_message *m,uint32_t now)
{
 const uint8_t *p=m->payload;uint16_t current;int rc;
 if(nx_identity_check(&e->task,&m->h,NX_ID_ALL))return NE_IDENTITY;
 if(!task_live(e))return NE_CANCELLED;
 if(e->resume&&m->h.request_id==e->start_request){
  if(ne_get64(p)!=e->nav_epoch||ne_get64(p+8)!=e->start_permit_seq||ne_get64(p+16)!=e->start_auth_epoch)return NE_IDENTITY;
  return nav_exec_motion_allowed(e,now)?NE_OK:NE_NO_PERMIT;
 }
 if(!e->external_quiet)return NE_BUSY;
 if(!e->plan_ready||e->planned_revision!=astar_path_revision(&e->server->planner)||!e->path_count)return NE_BUSY;
 if(ne_get64(p)!=e->nav_epoch||ne_get64(p+8)!=e->guard.permit_seq||ne_get64(p+16)!=e->guard.auth_epoch||e->guard.nav_epoch!=e->nav_epoch)return NE_IDENTITY;
 if(!nx_pose_valid(&e->guard,now)||e->pose.map_epoch!=e->active.epoch||e->pose.map_revision!=e->active.revision)return NE_POSE_STALE;
 if(!nav_exec_cell(&e->active,e->pose.x_mm,e->pose.y_mm,&current)||current!=e->start_cell)return NE_POSE_STALE;
 if(!nx_motion_valid(&e->guard,now)||!nx_authorized(&e->guard,now))return NE_NO_PERMIT;
 if(e->state!=NV_PLANNING)return NE_BUSY;
 rc=transport(e,m);if(rc)return rc;e->resume=1;e->owner=NE_OWNER_NAV;e->start_request=m->h.request_id;e->start_permit_seq=e->guard.permit_seq;e->start_auth_epoch=e->guard.auth_epoch;changed(e,NV_FOLLOWING,NV_R_NONE);return NE_OK;
}
static int page_status(const struct nav_exec *e,const struct ne_message *m)
{
 if(nx_identity_check(&e->task,&m->h,NX_ID_ALL))return NE_IDENTITY;
 if(!e->plan_ready||astar_busy(&e->server->planner)||e->planned_revision!=ne_get32(m->payload+4)||e->planned_revision!=astar_path_revision(&e->server->planner))return NE_BUSY;
 return NE_OK;
}
int nav_exec_handle(struct nav_exec *e,const struct ne_message *m,uint32_t now,uint32_t cycles)
{
 struct nx_guard g;int rc;const uint8_t *p;
 if(!e||!e->server||!m||ne_validate_shape(m)!=NE_CODEC_OK||(m->h.opcode&0x80))return NE_BAD_REQUEST;
 p=m->payload;
 if(!(m->h.family==NE_NAV3&&(m->h.opcode==NE_INFO||m->h.opcode==NE_SESSION_BIND||m->h.opcode==NE_CANCEL))&&
    !(m->h.family==NE_NVA1&&(m->h.opcode==NE_PROBE||m->h.opcode==NE_REVOKE))&&
    (m->h.map_epoch!=1||m->h.localization_epoch!=1||m->h.map_revision>UINT32_MAX)){
  if(m->h.family==NE_NAV3&&m->h.opcode==NE_POSE&&nx_check_session(&e->guard,&m->h)==NE_OK&&m->h.transport_seq>e->guard.last_transport[0]){e->rebind_required=1;retire(e,NV_R_REFERENCE_RESET);}
  return NE_REBIND;
 }
 if(m->h.family==NE_NAV3&&m->h.opcode==NE_INFO){nx_info_cookie(&e->guard,m->h.request_id,now,cycles);return NE_OK;}
 if(m->h.family==NE_NVA1&&m->h.opcode==NE_PROBE)return NE_OK;
 if(m->h.family==NE_NAV3&&m->h.opcode==NE_SESSION_BIND){
  copy(&g,&e->guard,sizeof(g));rc=nx_bind_session(&g,m,now,e->external_quiet&&e->owner==0&&!e->intent.linear_mmps&&!e->intent.angular_mradps&&!astar_busy(&e->server->planner));
  if(rc)return rc;
  invalidate(e);e->server->upload_active=0;
  nav_exec_init(e,e->server);copy(&e->guard,&g,sizeof(g));return NE_OK;
 }
 rc=nx_check_session(&e->guard,&m->h);if(rc)return rc;
 nav_exec_poll(e,now);
 if(m->h.family==NE_NVA1){
  if(nx_identity_check(&e->task,&m->h,NX_ID_TASK)||!task_live(e))return NE_IDENTITY;
  if(m->h.opcode==NE_REVOKE){if(ne_get64(p)!=e->nav_epoch)return NE_IDENTITY;rc=transport(e,m);if(rc)return rc;retire(e,NV_R_CANCEL);return NE_OK;}
  if(nx_identity_check(&e->task,&m->h,NX_ID_ALL))return NE_IDENTITY;
  copy(&g,&e->guard,sizeof(g));rc=nx_accept_permit(&g,m,now,e->nav_epoch);if(rc)return rc;rc=nx_accept_transport(&g,&m->h);if(rc)return rc;copy(&e->guard,&g,sizeof(g));return NE_OK;
 }
 switch(m->h.opcode){
 case NE_FRAME_BIND:case NE_CLOCK_ANCHOR:
  copy(&g,&e->guard,sizeof(g));rc=m->h.opcode==NE_FRAME_BIND?nx_bind_frame(&g,m):nx_bind_anchor(&g,m,now);if(rc)return rc;rc=nx_accept_transport(&g,&m->h);if(rc)return rc;copy(&e->guard,&g,sizeof(g));return NE_OK;
 case NE_MAP_BEGIN:return map_begin(e,m);
 case NE_MAP_CHUNK:return map_chunk(e,m);
 case NE_MAP_COMMIT:return map_commit(e,m);
 case NE_POSE:return pose(e,m,now);

 case NE_GOAL_PLAN:return goal(e,m,now);
 case NE_START:return start(e,m,now);
 case NE_CANCEL:
  if(nx_identity_check(&e->task,&m->h,NX_ID_TASK))return NE_IDENTITY;
  if(e->state==NV_CANCELLED)return NE_OK;
  rc=transport(e,m);if(rc)return rc;retire(e,NV_R_CANCEL);return NE_OK;
 case NE_PATH_PAGE:return page_status(e,m); /* read-only: does not consume transport floor */
 default:return NE_BAD_REQUEST;
 }
}
void nav_exec_poll(struct nav_exec *e,uint32_t now)
{
 unsigned ex=nx_expire(&e->guard,now);
 if(e->resume&&(ex&(NX_LIVE_AUTH|NX_LIVE_MOTION|NX_LIVE_POSE)))
  retire(e,(ex&NX_LIVE_AUTH)?NV_R_AUTH_EXPIRED:(ex&NX_LIVE_MOTION)?NV_R_MOTION_EXPIRED:NV_R_POSE_EXPIRED);
 if(!nav_exec_motion_allowed(e,now))stop(e);
}
int nav_exec_plan(struct nav_exec *e,uint32_t now)
{
 uint64_t run,epoch;uint16_t start,goalcell;int n;
 if(!e->plan_due||e->state!=NV_PLANNING||astar_busy(&e->server->planner))return NE_BUSY;
 if(!task_live(e))return NE_CANCELLED;
 if(!nx_pose_valid(&e->guard,now)||e->pose.map_epoch!=e->active.epoch||e->pose.map_revision!=e->active.revision||e->pose.localization_epoch!=e->task.localization_epoch||e->task.map_epoch!=e->active.epoch||e->task.map_revision!=e->active.revision){e->reason=NV_R_POSE_EXPIRED;return NE_POSE_STALE;}
 if(!nav_exec_cell(&e->active,e->pose.x_mm,e->pose.y_mm,&start)||!nav_exec_cell(&e->active,e->goal_x_mm,e->goal_y_mm,&goalcell)){e->plan_due=0;e->reason=NV_R_OUTSIDE;return NE_GOAL_OUTSIDE_WINDOW;}
 e->plan_due=0;run=e->task.run;epoch=e->nav_epoch;e->start_pose_seq=e->guard.pose_seq;e->start_cell=start;e->goal_cell=goalcell;
 n=astar_plan(&e->server->planner,start%64u,start/64u,goalcell%64u,goalcell/64u);
 if(run!=e->task.run||epoch!=e->nav_epoch||!task_live(e)){return NE_CANCELLED;}
 if(n<0){e->reason=n==ASTAR_NO_ROUTE||n==ASTAR_BLOCKED_ENDPOINT?NV_R_NO_ROUTE:NV_R_INTERNAL;e->plan_ready=0;return e->reason==NV_R_NO_ROUTE?NE_NO_ROUTE:NE_INTERNAL;}
 if(e->plan_counter==UINT64_MAX){retire(e,NV_R_INTERNAL);return NE_INTERNAL;}
 e->task.plan=++e->plan_counter;e->plan_ready=1;e->path_count=(uint16_t)n;e->planned_revision=astar_path_revision(&e->server->planner);
 e->server->path_valid=1;e->server->path_length=(uint16_t)n;e->server->path_revision=e->planned_revision;e->cursor=0;
 /* Replanning alone cannot grant permission: resume was set only by START. */
 changed(e,e->resume?NV_FOLLOWING:NV_PLANNING,e->resume?NV_R_NONE:NV_R_AWAIT_START);return NE_OK;
}
void nav_exec_set_map_quiet(struct nav_exec *e,int quiet){e->map_quiet=(uint8_t)(quiet!=0);}
void nav_exec_set_external_quiet(struct nav_exec *e,int quiet){e->external_quiet=(uint8_t)(quiet!=0);}
void nav_exec_manual_takeover(struct nav_exec *e){e->external_quiet=0;if(e->task.run)retire(e,NV_R_MANUAL);else{e->resume=0;e->owner=0;stop(e);}}
int nav_exec_submit_intent(struct nav_exec *e,const struct nv_intent *i,uint32_t now)
{
 if(!i||!nav_exec_motion_allowed(e,now)||i->nav_epoch!=e->nav_epoch||i->run!=e->task.run||i->plan!=e->task.plan||i->pose_seq!=e->guard.pose_seq||i->permit_seq!=e->guard.permit_seq)return 0;
 copy(&e->intent,i,sizeof(e->intent));return 1;
}
void nav_exec_begin_avoid(struct nav_exec *e){if(e->state==NV_FOLLOWING&&e->resume){e->avoid_phase=NV_STOP;changed(e,NV_AVOIDING,NV_R_OBSTACLE);}}
void nav_exec_set_avoid_phase(struct nav_exec *e,enum nv_avoid_phase phase){if(e->state==NV_AVOIDING&&phase<=NV_ADVANCE)e->avoid_phase=(uint8_t)phase;}
int nav_exec_finish_avoid(struct nav_exec *e,uint32_t now)
{
 if(e->state!=NV_AVOIDING||e->avoid_phase!=NV_STOP||!nav_exec_motion_allowed(e,now))return NE_BUSY;
 replan(e,NV_R_AVOID_COMPLETE);return NE_OK;
}
int nav_exec_arrive(struct nav_exec *e,uint32_t now)
{
 if(e->state!=NV_FOLLOWING||!nav_exec_motion_allowed(e,now))return NE_BUSY;
 e->resume=0;e->owner=0;e->plan_due=0;nx_revoke(&e->guard);changed(e,NV_ARRIVED,NV_R_ARRIVED);return NE_OK;
}
int nav_exec_page(struct nav_exec *e,const struct ne_message *m,uint16_t *cells,unsigned cap,unsigned *count)
{
 unsigned off,n,i;if(count)*count=0;
 if(!count||!cells||!m||m->h.family!=NE_NAV3||m->h.opcode!=NE_PATH_PAGE||ne_validate_shape(m)!=NE_CODEC_OK)return NE_BAD_REQUEST;
 if(nx_check_session(&e->guard,&m->h))return NE_SESSION;
 {int rc=page_status(e,m);if(rc)return rc;}
 off=ne_get16(m->payload);n=ne_get16(m->payload+2);if(n>cap||off>e->path_count)return NE_BAD_REQUEST;if(n>e->path_count-off)n=e->path_count-off;
 for(i=0;i<n;++i)if(!astar_path_get(&e->server->planner,off+i,cells+i))return NE_BUSY;
 *count=n;return NE_OK;
}
