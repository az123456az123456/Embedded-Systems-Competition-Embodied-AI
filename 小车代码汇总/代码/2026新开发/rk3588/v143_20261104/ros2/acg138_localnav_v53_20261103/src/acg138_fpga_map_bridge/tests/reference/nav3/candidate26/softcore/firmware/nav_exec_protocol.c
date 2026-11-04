#include "nav_exec_protocol.h"

static int same(const uint8_t *a,const uint8_t *b,unsigned n)
{unsigned i;for(i=0;i<n;++i)if(a[i]!=b[i])return 0;return 1;}
void nx_guard_init(struct nx_guard *g)
{unsigned i;uint8_t *p=(uint8_t*)g;for(i=0;i<sizeof(*g);++i)p[i]=0;}
void nx_identity_from_header(struct nx_identity *id,const struct ne_header *h)
{
 unsigned i;id->session=h->session_id;id->run=h->run_generation;
 id->map_epoch=h->map_epoch;id->map_revision=h->map_revision;
 id->localization_epoch=h->localization_epoch;id->plan=h->plan_id;
 for(i=0;i<16;++i)id->goal[i]=h->goal_id[i];
}
int nx_identity_check(const struct nx_identity *id,const struct ne_header *h,unsigned mask)
{
 if(((mask&NX_ID_SESSION)&&id->session!=h->session_id)||
    ((mask&NX_ID_RUN)&&id->run!=h->run_generation)||
    ((mask&NX_ID_GOAL)&&!same(id->goal,h->goal_id,16))||
    ((mask&NX_ID_MAP_EPOCH)&&h->map_epoch!=1)||
    ((mask&NX_ID_MAP_REVISION)&&id->map_revision!=h->map_revision)||
    ((mask&NX_ID_LOCALIZATION)&&h->localization_epoch!=1)||
    ((mask&NX_ID_PLAN)&&id->plan!=h->plan_id))return NE_IDENTITY;
 return NE_OK;
}
uint64_t nx_info_cookie(struct nx_guard *g,uint64_t request,uint32_t now,uint32_t cycles)
{
 /* Correlates the recent round trip on a trusted RK link. Not a cryptographic
  * boot nonce or a claim of replay authentication across a full power loss. */
 g->info_cookie=request^((uint64_t)cycles<<32)^((uint64_t)now<<1)^UINT64_C(0x4e41563300000001);
 if(!g->info_cookie)g->info_cookie=1;
 g->info_ms=now;return g->info_cookie;
}
int nx_bind_session(struct nx_guard *g,const struct ne_message *m,uint32_t now,int quiet)
{
 uint64_t session=ne_get64(m->payload),cookie=ne_get64(m->payload+8);
 if(!quiet)return NE_BUSY;
 if(!cookie||cookie!=g->info_cookie||(uint32_t)(now-g->info_ms)>1000u)return NE_STALE;
 if(!session||session==g->session)return NE_SESSION;
 /* Consume this INFO challenge exactly once. A captured BIND cannot cycle
  * sessions A->B->A and erase sequence floors. CLOCK_ANCHOR requires a fresh
  * INFO in the newly bound session. A lost BIND ACK is recovered via INFO. */
 nx_guard_init(g);g->session=session;
 return NE_OK;
}
int nx_check_session(const struct nx_guard *g,const struct ne_header *h)
{return !g->session?NE_UNBOUND:h->session_id!=g->session?NE_SESSION:NE_OK;}
static unsigned lane(const struct ne_header *h)
{
 if(h->family==NE_NVA1)return 1;
 if(h->opcode==NE_POSE)return 0;
 if(h->opcode==NE_CANCEL)return 4;
 if(h->opcode>=NE_MAP_BEGIN&&h->opcode<=NE_MAP_COMMIT)return 2;
 return 3;
}
int nx_accept_transport(struct nx_guard *g,const struct ne_header *h)
{
 unsigned k=lane(h);int e=nx_check_session(g,h);if(e)return e;
 if(!h->transport_seq||h->transport_seq<=g->last_transport[k])return NE_STALE;
 g->last_transport[k]=h->transport_seq;return NE_OK;
}
int nx_bind_frame(struct nx_guard *g,const struct ne_message *m)
{
 const uint8_t *p=m->payload;uint64_t token=ne_get32(p);unsigned i,n=p[4];
 int e=nx_check_session(g,&m->h);if(e)return e;
 if(!token||!n||n>63)return NE_BAD_REQUEST;
 if(g->frame_token)return g->frame_token==token&&g->frame_len==n&&same(g->frame,p+5,n)?NE_OK:NE_REBIND;
 g->frame_token=token;g->frame_len=(uint8_t)n;for(i=0;i<n;++i)g->frame[i]=p[5+i];return NE_OK;
}
int nx_bind_anchor(struct nx_guard *g,const struct ne_message *m,uint32_t now)
{
 const uint8_t *p=m->payload;uint64_t id=ne_get64(p);int e=nx_check_session(g,&m->h);if(e)return e;
 if(!id||ne_get32(p+8)!=g->info_ms||ne_get32(p+12)>100u||
    (uint32_t)(now-g->info_ms)>1000u||!g->info_cookie)return NE_STALE;
 if(id<=g->anchors[0].id)return NE_STALE;
 g->anchors[1]=g->anchors[0];g->anchors[0].id=id;g->anchors[0].expires_ms=now+30000u;return NE_OK;
}
int nx_anchor_valid(const struct nx_guard *g,uint64_t id,uint32_t now)
{
 unsigned i;
 for(i=0;i<2;++i){
  if(id&&id==g->anchors[i].id&&(int32_t)(g->anchors[i].expires_ms-now)>0)return 1;
 }
 return 0;
}
int nx_accept_pose(struct nx_guard *g,const struct ne_message *m,uint32_t now)
{
 const uint8_t *p=m->payload;uint64_t age=ne_get64(p+36),seq=ne_get64(p+16);
 int64_t stamp=(int64_t)ne_get64(p+24);uint32_t clock=ne_get32(p+32);
 uint32_t deadline=ne_get32(p+44),budget;int e=nx_check_session(g,&m->h);if(e)return e;
 if(!g->frame_token||ne_get32(p+12)!=g->frame_token)return NE_REBIND;
 if(!m->h.localization_epoch||!seq||ne_get32(p+56)!=3u||age>250000000u||
    !nx_anchor_valid(g,ne_get64(p+48),now))return NE_POSE_STALE;
 if(g->pose_localization==m->h.localization_epoch&&seq<=g->pose_seq)return NE_STALE;
 if(!clock||clock>2u||stamp<0)return NE_POSE_STALE;
 if(g->pose_seq&&g->pose_localization==m->h.localization_epoch){
  if(clock!=g->pose_source_clock||stamp<g->pose_source_stamp_ns)return NE_REBIND;
  if(stamp==g->pose_source_stamp_ns)return NE_STALE;
 }
 budget=(250000000u-(uint32_t)age)/1000000u;
 if((int32_t)(deadline-now)<0||(uint32_t)(deadline-now)>budget)return NE_POSE_STALE;
 g->pose_seq=seq;g->pose_localization=m->h.localization_epoch;g->pose_source_stamp_ns=stamp;
 g->pose_source_clock=clock;g->pose_deadline=deadline;g->live|=NX_LIVE_POSE;return NE_OK;
}
int nx_accept_permit(struct nx_guard *g,const struct ne_message *m,uint32_t now,uint64_t nav_epoch)
{
 const uint8_t *p=m->payload;
 uint64_t auth=ne_get64(p+12),phone=ne_get64(p+20),permit=ne_get64(p+28);
 uint32_t pd=ne_get32(p+36),ad=ne_get32(p+40);int e=nx_check_session(g,&m->h);if(e)return e;
 if(ne_get32(p)!=NE_OWNER_NAV||!nav_epoch||ne_get64(p+4)!=nav_epoch)return NE_IDENTITY;
 if(!nx_anchor_valid(g,ne_get64(p+44),now))return NE_NO_PERMIT;
 if((int32_t)(pd-now)<0||(uint32_t)(pd-now)>500u||
    (int32_t)(ad-now)<=0||(uint32_t)(ad-now)>30000u)return NE_NO_PERMIT;
 if(!auth||!phone||!permit||auth<g->auth_epoch||permit<=g->permit_seq)return NE_STALE;
 if(auth==g->auth_epoch){
  if(phone<g->phone_seq)return NE_STALE;
  if(phone==g->phone_seq&&(int32_t)(ad-g->auth_deadline)>0)return NE_NO_PERMIT;
 }
 g->nav_epoch=nav_epoch;g->auth_epoch=auth;g->phone_seq=phone;g->permit_seq=permit;
 g->permit_deadline=pd;g->auth_deadline=ad;g->live|=NX_LIVE_MOTION|NX_LIVE_AUTH;return NE_OK;
}
int nx_motion_valid(const struct nx_guard *g,uint32_t now)
{return (g->live&NX_LIVE_MOTION)&&(int32_t)(g->permit_deadline-now)>=0;}
int nx_authorized(const struct nx_guard *g,uint32_t now)
{return (g->live&NX_LIVE_AUTH)&&(int32_t)(g->auth_deadline-now)>0;}
int nx_pose_valid(const struct nx_guard *g,uint32_t now)
{return (g->live&NX_LIVE_POSE)&&(int32_t)(g->pose_deadline-now)>=0;}
void nx_revoke(struct nx_guard *g)
{
 /* Preserve source replay floors; a late old permit cannot re-acquire a run.
  * The executor/hardware must also increment nav_epoch and clear START. */
 g->nav_epoch=0;g->live=0;g->permit_deadline=0;g->auth_deadline=0;g->pose_deadline=0;
}
unsigned nx_expire(struct nx_guard *g,uint32_t now)
{
 unsigned expired=0;
 if((g->live&NX_LIVE_MOTION)&&!nx_motion_valid(g,now))expired|=NX_LIVE_MOTION;
 if((g->live&NX_LIVE_AUTH)&&!nx_authorized(g,now))expired|=NX_LIVE_AUTH;
 if((g->live&NX_LIVE_POSE)&&!nx_pose_valid(g,now))expired|=NX_LIVE_POSE;
 g->live&=(uint8_t)~expired;return expired;
}
