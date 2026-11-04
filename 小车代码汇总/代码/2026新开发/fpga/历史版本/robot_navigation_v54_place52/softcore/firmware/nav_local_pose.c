#include "nav_local_pose.h"
#include "nav_math.h"
#include <limits.h>
static int32_t radians(int32_t mdeg){return (mdeg/1000)*17453+((mdeg%1000)*17453)/1000;}
static int32_t difference(int32_t a,int32_t b){uint32_t x=(uint32_t)a-(uint32_t)b;return x<=INT32_MAX?(int32_t)x:-1-(int32_t)(UINT32_MAX-x);}
static int valid(const struct lc_input *in,const struct lc_state *c){
 return in->local_valid&&in->sample_id&&!in->encoder_illegal&&c->calibrated&&c->imu_reason==LC_IMU_READY&&
  (uint32_t)(in->now_ms-in->imu_effective_ms)<=250u;
}
void nlp_reset(struct nav_local_pose *p){unsigned i;uint8_t *b=(uint8_t*)p;for(i=0;i<sizeof(*p);++i)b[i]=0;}
int nlp_seed(struct nav_local_pose *p,const struct nav_exec *e,const struct lc_input *in,const struct lc_state *c){
 uint16_t cell;unsigned i;
 if(!valid(in,c)||!nav_exec_cell(&e->active,e->pose.x_mm,e->pose.y_mm,&cell)||!nx_pose_valid(&e->guard,in->now_ms))return 0;
 nlp_reset(p);p->x_um=(int32_t)((uint32_t)e->pose.x_mm-(uint32_t)e->active.origin_x_mm)*1000;
 p->y_um=(int32_t)((uint32_t)e->pose.y_mm-(uint32_t)e->active.origin_y_mm)*1000;
 p->yaw_offset=nm_wrap(e->pose.yaw_urad-radians(c->yaw_mdeg));p->previous_yaw=e->pose.yaw_urad;
 for(i=0;i<4;++i)p->counts[i]=in->counts[i];
 p->sample=in->sample_id;p->stamp=in->now_ms;p->imu_stamp=c->imu_stamp_ms;p->last_publish_ms=in->now_ms-20u;p->seeded=1;return 1;
}
int nlp_update(struct nav_local_pose *p,const struct nav_exec *e,const struct lc_input *in,const struct lc_state *c){
 int32_t ds=0,d,yaw,dyaw,sn,cs,um,qx,qy;uint32_t dt;unsigned i;
 (void)e;
 if(!p->seeded||p->fault)return -1;
 if(!valid(in,c))goto bad;
 if(in->sample_id==p->sample)return 0;
 dt=in->now_ms-p->stamp;
 if((int32_t)(in->sample_id-p->sample)<=0||!dt||dt>100u)goto bad;
 yaw=nm_wrap(radians(c->yaw_mdeg)+p->yaw_offset);dyaw=nm_wrap(yaw-p->previous_yaw);
 if(dyaw> (int32_t)(dt*3000u+10000u)||dyaw< -(int32_t)(dt*3000u+10000u))goto bad;
 for(i=0;i<4;++i){
  d=difference(in->counts[i],p->counts[i]);
  if(d> (int32_t)(dt*450u+32u)||d< -(int32_t)(dt*450u+32u))goto bad;
  ds+=(i==0||i==2)?-d:d;
 }
 /* <=100ms source interval bounds every signed product below2^31. Keep
  * sub-micrometre and Q12 projection remainders instead of dropping motion. */
 d=ds*LC_TICK_NM+p->nm_remainder;um=d/4000;p->nm_remainder=d%4000;
 nm_sincos(nm_wrap(p->previous_yaw+dyaw/2),&sn,&cs);
 qx=um*(cs/8)+p->x_remainder;qy=um*(sn/8)+p->y_remainder;
 p->x_um+=qx/4096;p->y_um+=qy/4096;p->x_remainder=qx%4096;p->y_remainder=qy%4096;
 if(p->x_um<0||p->x_um>=6400000||p->y_um<0||p->y_um>=6400000)goto bad;
 for(i=0;i<4;++i)p->counts[i]=in->counts[i];
 p->sample=in->sample_id;p->stamp=in->now_ms;p->imu_stamp=c->imu_stamp_ms;p->previous_yaw=yaw;return 1;
 bad:p->fault=1;return -1;
}
int nlp_publish(struct nav_local_pose *p,struct nav_exec *e,const struct lc_input *in,uint32_t now){
 uint32_t age=now-in->imu_effective_ms;
 if(!p->seeded||p->fault||!in->local_valid||in->sample_id!=p->sample||p->published_sample==p->sample||age>250u||e->guard.pose_seq==UINT64_MAX||!nx_pose_valid(&e->guard,now))return 0;
 e->pose.x_mm=e->active.origin_x_mm+p->x_um/1000;e->pose.y_mm=e->active.origin_y_mm+p->y_um/1000;e->pose.yaw_urad=p->previous_yaw;
 ++e->guard.pose_seq;e->guard.pose_deadline=in->imu_effective_ms+250u;e->guard.live|=NX_LIVE_POSE;
 e->pose.source_age_ms=age;e->pose.accepted_ms=now;p->last_publish_ms=now;p->published_sample=p->sample;return 1;
}
