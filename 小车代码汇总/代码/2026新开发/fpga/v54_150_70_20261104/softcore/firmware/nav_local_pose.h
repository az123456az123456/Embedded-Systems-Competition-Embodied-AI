#ifndef NAV_LOCAL_POSE_H
#define NAV_LOCAL_POSE_H
#include "nav_exec.h"
#include "layered_control.h"
struct nav_local_pose {
 int32_t x_um,y_um,yaw_offset,previous_yaw,nm_remainder,x_remainder,y_remainder;
 int32_t counts[4];
 uint32_t sample,published_sample,stamp,imu_stamp,last_publish_ms;
 uint8_t seeded,fault;
};
void nlp_reset(struct nav_local_pose *p);
/*1=new source integrated,0=duplicate,-1=invalid/expired. Never renew from a
 * repeated sample. Coordinates stay local to the committed6.4m window. */
int nlp_seed(struct nav_local_pose *p,const struct nav_exec *e,const struct lc_input *in,const struct lc_state *c);
int nlp_update(struct nav_local_pose *p,const struct nav_exec *e,const struct lc_input *in,const struct lc_state *c);
int nlp_publish(struct nav_local_pose *p,struct nav_exec *e,const struct lc_input *in,uint32_t now);
#endif
