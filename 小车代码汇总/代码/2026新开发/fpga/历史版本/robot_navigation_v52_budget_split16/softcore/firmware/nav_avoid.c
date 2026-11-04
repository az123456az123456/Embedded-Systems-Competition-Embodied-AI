#include "nav_avoid.h"
#include "nav_math.h"
#include "nav_obstacle.h"
#include <limits.h>
static uint32_t abs32(int32_t v){return v<0?0u-(uint32_t)v:(uint32_t)v;}
static int newer(uint32_t a,uint32_t b){return a!=b&&(uint32_t)(a-b)<UINT32_C(0x80000000);}
void nav_avoid_init(struct nav_avoid *a)
{uint8_t *p=(uint8_t*)a;unsigned i;for(i=0;i<sizeof(*a);++i)p[i]=0;}
static void command(struct nav_exec *e,uint32_t now,int32_t v,int32_t w)
{
 struct nv_intent i;i.nav_epoch=e->nav_epoch;i.run=e->task.run;i.plan=e->task.plan;
 i.pose_seq=e->guard.pose_seq;i.permit_seq=e->guard.permit_seq;i.linear_mmps=v;i.angular_mradps=w;
 nav_exec_submit_intent(e,&i,now);
}
/* L5 owns a separate source ledger because an older non-identical sequence
 * must not re-seed the existing KF and then masquerade as a new clear echo. */
static unsigned observe(struct nav_avoid *a,const struct lc_input *in,const struct lc_state *c,uint32_t now)
{
 unsigned i,usable=0;
 for(i=0;i<3;++i){
  uint32_t seq=in->sonar_sequence[i],stamp=in->sonar_stamp_ms[i];uint16_t z=in->sonar_mm[i];unsigned bit=1u<<i;
  if((in->sonar_online&bit)&&seq&&z>=30u&&z!=UINT16_MAX&&(uint32_t)(now-stamp)<=LC_SONAR_TIMEOUT_MS&&
     (!(a->known&bit)||(newer(seq,a->source_seq[i])&&newer(stamp,a->source_ms[i])))){
   a->source_seq[i]=seq;a->source_ms[i]=stamp;a->raw_mm[i]=z>4500u?4500u:z;a->known|=(uint8_t)bit;
   a->filtered&=(uint8_t)~bit;a->filtered_mm[i]=0;
  }
  /* The production loop observes RAW before lc_update. A later decision may
   * attach only that exact accepted sample's KF, without touching its age. */
  if((a->known&bit)&&c->sonar[i].valid&&c->sonar[i].sequence==a->source_seq[i]&&c->sonar[i].stamp_ms==a->source_ms[i]){
   uint32_t v=(uint32_t)(c->sonar[i].x_q8/256);a->filtered_mm[i]=(uint16_t)(v>4500u?4500u:v);a->filtered|=(uint8_t)bit;
  }
  if((a->known&bit)&&(uint32_t)(now-a->source_ms[i])<=LC_SONAR_TIMEOUT_MS)usable|=bit;
 }
 return usable;
}
static unsigned range_mm(const struct nav_avoid *a,unsigned i)
{return a->filtered_mm[i]&&a->filtered_mm[i]<a->raw_mm[i]?a->filtered_mm[i]:a->raw_mm[i];}
static uint32_t front_stop(int32_t v,uint32_t age)
{
 uint32_t n=lc_front_stop_mm(v,age);return n+(NA_NEAR_MM-LC_FRONT_STOP_BASE_MM);
}
static int near(const struct nav_exec *e,const struct nav_avoid *a,const struct lc_state *c,uint32_t now)
{
 int32_t v=lc_forward_component_mmps(c);if(a->speed_mmps>v)v=a->speed_mmps;if(e->intent.linear_mmps>v)v=e->intent.linear_mmps;
 return range_mm(a,0)<=front_stop(v,now-a->source_ms[0])||range_mm(a,1)<=NA_NEAR_MM||range_mm(a,2)<=NA_NEAR_MM;
}
static void begin_stop(struct nav_exec *e,struct nav_avoid *a,struct nv_stop *s,uint32_t now,int finishing)
{
 nav_exec_set_avoid_phase(e,NV_STOP);a->stop_ms=now;a->finishing=(uint8_t)finishing;
 a->speed_mmps=0;a->crossed=0;a->clear_count=0;a->reason=finishing?NA_FINISH_STOP:NA_STOP_WAIT;
 nv_stop_begin(s,now);command(e,now,0,0);
}
static int after_stop(const struct nav_avoid *a,unsigned usable)
{
 unsigned i;if(usable!=7u)return 0;
 for(i=0;i<3;++i)if((int32_t)(a->source_ms[i]-a->stop_ms)<=0)return 0;
 return 1;
}
/* ceil(full swept radius)297mm minus probe projection, with the independent
 *200mm selected-side floor. All distances remain sensor-face distances.
 * CAD nominal extents:rear167.25,left137,+80margin; probes145.417/112.87. */
static int rotation_clear(const struct nav_avoid *a,int side)
{
 return range_mm(a,0)>50u&&range_mm(a,1)>185u&&range_mm(a,2)>185u&&
        range_mm(a,side>0?1u:2u)>NA_SIDE_RELEASE_MM;
}
static int choose(const struct nav_avoid *a)
{
 unsigned l=range_mm(a,1),r=range_mm(a,2);
 if(l<=NA_NEAR_MM&&r<=NA_NEAR_MM)return 0;
 if(l<=NA_NEAR_MM)return -1;
 if(r<=NA_NEAR_MM)return 1;
 return a->direction?a->direction:l>r?1:-1;
}
static int clear_observation(struct nav_avoid *a,uint32_t now,unsigned usable)
{
 unsigned i;uint32_t required=front_stop(NA_CLEAR_REFERENCE_MMPS,now-a->source_ms[0])+40u;
 if(required<NA_FRONT_CLEAR_MIN_MM)required=NA_FRONT_CLEAR_MIN_MM;
 if(!a->crossed)return 0;
 if(usable!=7u||range_mm(a,0)<=required||range_mm(a,1)<=NA_SIDE_RELEASE_MM||range_mm(a,2)<=NA_SIDE_RELEASE_MM)return -1;
 if(a->filtered!=7u)return 0;
 for(i=0;i<3;++i){
  if((int32_t)(a->source_ms[i]-LC_STOP_SENSOR_LATENCY_MS-a->cross_ms)<=0||
     !newer(a->source_seq[i],a->clear_seq[i]))return 0;
 }
 for(i=0;i<3;++i)a->clear_seq[i]=a->source_seq[i];
 return 1;
}
int nav_avoid_update(struct nav_exec *e,struct nav_avoid *a,struct nv_stop *stop,
 const struct nv_stop_sample *sample,const struct lc_input *raw,const struct lc_state *control)
{
 uint32_t now,usable;int side;unsigned i;
 if(!e||!a||!stop||!sample||!raw||!control)return NV_AVOID_INACTIVE;
 now=sample->now_ms;nav_exec_poll(e,now);
 if(e->state!=NV_FOLLOWING&&e->state!=NV_AVOIDING){nav_avoid_init(a);return NV_AVOID_INACTIVE;}
 if(a->nav_epoch!=e->nav_epoch){nav_avoid_init(a);a->nav_epoch=e->nav_epoch;}
 usable=observe(a,raw,control,now);
 if(e->state==NV_FOLLOWING){
  if(usable==7u&&!near(e,a,control,now))return NV_AVOID_INACTIVE;
  /* Mark only accepted finite near echoes once on entry. Stale/missing
   * echoes still stop, but cannot invent an obstacle or a clear cell. */
  for(i=0;i<3;++i)if((usable&(1u<<i))&&a->raw_mm[i]<=
      (i?NA_NEAR_MM:front_stop(e->intent.linear_mmps,now-a->source_ms[i])))
   (void)nav_obstacle_mark(e,i,a->raw_mm[i]);
  nav_exec_begin_avoid(e);a->active=1;a->started_ms=now;a->scan_urad=0;a->direction=0;
  begin_stop(e,a,stop,now,0);return NV_AVOID_ACTIVE;
 }
 if(!a->active){a->active=1;a->started_ms=now;begin_stop(e,a,stop,now,0);}
 if(!nav_exec_motion_allowed(e,now)){command(e,now,0,0);return NV_AVOID_ACTIVE;}
 if(a->scan_urad>=NA_SCAN_LIMIT_URAD||(uint32_t)(now-a->started_ms)>=NA_BUDGET_MS){
  a->reason=NA_SCAN_LIMIT;a->speed_mmps=0;command(e,now,0,0);return NV_AVOID_ACTIVE;
 }
 if(usable!=7u){
  if(e->avoid_phase!=NV_STOP)begin_stop(e,a,stop,now,0);
  a->reason=NA_SOURCE_WAIT;command(e,now,0,0);return NV_AVOID_ACTIVE;
 }
 if(e->avoid_phase==NV_STOP){
  command(e,now,0,0);
  if(!nv_stop_update(stop,sample)){a->reason=NA_STOP_WAIT;return NV_AVOID_ACTIVE;}
  if(!after_stop(a,usable)){a->reason=NA_SOURCE_WAIT;return NV_AVOID_ACTIVE;}
  if(a->finishing){
   /* A newly observed close obstacle during braking cancels clear completion.
    * Keep the existing source/stop proof; never replan from stale clear data. */
   if(near(e,a,control,now)){begin_stop(e,a,stop,now,0);return NV_AVOID_ACTIVE;}
   if(nav_exec_finish_avoid(e,now)==NE_OK){nav_avoid_init(a);return NV_AVOID_REPLAN;}
   return NV_AVOID_ACTIVE;
  }
  nav_exec_set_avoid_phase(e,NV_SELECT);
 }
 if(e->avoid_phase==NV_SELECT){
  /* A previous stop proof cannot license movement after somebody moves the
   * waiting vehicle. Keep consuming actual source evidence while selecting. */
  if(!nv_stop_update(stop,sample)){a->reason=NA_STOP_WAIT;command(e,now,0,0);return NV_AVOID_ACTIVE;}
  side=choose(a);
  if(!side){a->reason=NA_SELECT_WAIT;command(e,now,0,0);return NV_AVOID_ACTIVE;}
  if(!rotation_clear(a,side)){side=-side;if(!rotation_clear(a,side)){a->reason=NA_SELECT_WAIT;command(e,now,0,0);return NV_AVOID_ACTIVE;}}
  a->direction=(int8_t)side;a->turn_start_yaw=e->pose.yaw_urad;a->last_yaw=e->pose.yaw_urad;a->pose_seq=e->guard.pose_seq;
  a->crossed=0;a->clear_count=0;for(i=0;i<3;++i)a->clear_seq[i]=a->source_seq[i];
  a->last_ms=now-20u;nav_exec_set_avoid_phase(e,NV_TURN_CLEAR);
 }
 if(e->avoid_phase==NV_TURN_CLEAR){
  int left=range_mm(a,1)<=NA_NEAR_MM,right=range_mm(a,2)<=NA_NEAR_MM;
  if(e->guard.pose_seq!=a->pose_seq){
   uint32_t travel=abs32(nm_wrap(e->pose.yaw_urad-a->last_yaw));
   if(UINT32_MAX-a->scan_urad<travel)a->scan_urad=UINT32_MAX;else a->scan_urad+=travel;
   a->last_yaw=e->pose.yaw_urad;a->pose_seq=e->guard.pose_seq;
  }
  if(a->scan_urad>=NA_SCAN_LIMIT_URAD){a->reason=NA_SCAN_LIMIT;a->speed_mmps=0;command(e,now,0,0);return NV_AVOID_ACTIVE;}
  if((left!=right&&a->direction!=(left?-1:1))||!rotation_clear(a,a->direction)){
   a->direction=(int8_t)-a->direction;begin_stop(e,a,stop,now,0);return NV_AVOID_ACTIVE;
  }
  if(!a->crossed&&a->direction*nm_wrap(e->pose.yaw_urad-a->turn_start_yaw)>=NA_MIN_TURN_URAD){a->crossed=1;a->cross_ms=now;}
  {int clear=clear_observation(a,now,usable);if(clear>0){if(a->clear_count<2)++a->clear_count;}else if(clear<0)a->clear_count=0;}
  if(a->clear_count>=2){
   /* No blind departure distance: clear -> measured stop -> same-goal A*. */
   begin_stop(e,a,stop,now,1);return NV_AVOID_ACTIVE;
  }else{
   a->reason=a->clear_count?NA_CLEAR_CONFIRM:NA_TURNING;
   if((uint32_t)(now-a->last_ms)>=20u||e->intent.pose_seq!=e->guard.pose_seq||e->intent.permit_seq!=e->guard.permit_seq){
    if((uint32_t)(now-a->last_ms)>=20u)a->last_ms=now;
    command(e,now,0,a->direction*NA_TURN_MRADPS);
   }return NV_AVOID_ACTIVE;
  }
 }

 command(e,now,0,0);return NV_AVOID_ACTIVE;
}
