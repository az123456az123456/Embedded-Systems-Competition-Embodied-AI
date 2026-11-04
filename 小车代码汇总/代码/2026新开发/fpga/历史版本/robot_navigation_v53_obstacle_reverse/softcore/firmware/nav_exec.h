#ifndef NAV_EXEC_H
#define NAV_EXEC_H
#include "nav_protocol.h"
#include "nav_exec_protocol.h"

/* Production state machine. It owns no second path array: the READY planner
 * accessors are the only path publication API. Tracking/avoidance algorithms
 * supply explicit feedback through the interfaces below (L4/L5). */
enum nv_state { NV_IDLE=0,NV_PLANNING=1,NV_FOLLOWING=2,NV_AVOIDING=3,NV_ARRIVED=4,NV_CANCELLED=5 };
enum nv_avoid_phase { NV_STOP=0,NV_SELECT=1,NV_TURN_CLEAR=2,NV_REVERSE=3,NV_ADVANCE=NV_REVERSE };
enum nv_reason {
 NV_R_NONE=0,NV_R_AWAIT_PLAN=1,NV_R_AWAIT_START=2,NV_R_NO_ROUTE=3,
 NV_R_CANCEL=4,NV_R_MANUAL=5,NV_R_MOTION_EXPIRED=6,NV_R_AUTH_EXPIRED=7,
 NV_R_POSE_EXPIRED=8,NV_R_REFERENCE_RESET=9,NV_R_NEW_MAP=10,
 NV_R_OBSTACLE=11,NV_R_AVOID_COMPLETE=12,NV_R_ARRIVED=13,
 NV_R_OUTSIDE=14,NV_R_INTERNAL=15,NV_R_UPLOAD_WAIT=16
};
struct nv_map {
 uint64_t epoch,revision,transaction;
 int32_t origin_x_mm,origin_y_mm;
 uint32_t frame_token,crc32;
 uint8_t valid;
};
struct nv_pose {
 int32_t x_mm,y_mm,yaw_urad;
 uint64_t map_epoch,map_revision,localization_epoch;
 uint32_t accepted_ms,source_age_ms;
};
struct nv_intent {
 uint64_t nav_epoch,run,plan,pose_seq,permit_seq;
 int32_t linear_mmps,angular_mradps;
};
struct nav_exec {
 struct nav_server *server;
 struct nx_guard guard;
 struct nx_identity task;
 struct nv_map active,upload;
 struct nv_pose pose;
 uint64_t nav_epoch,run_floor,plan_counter,state_seq,start_pose_seq;
 uint64_t goal_request,start_request,start_permit_seq,start_auth_epoch;
 uint32_t remaining_mm,planned_revision;
 int32_t goal_x_mm,goal_y_mm;
 struct nv_intent intent;
 uint16_t cursor,path_count,start_cell,goal_cell;
 uint16_t upload_chunks,reason;
 uint64_t rear_seq,rear_source_stamp;
 uint32_t rear_deadline,rear_mm;
 uint8_t rear_valid;
 uint8_t state,avoid_phase,plan_ready,resume,plan_due,owner,rebind_required,external_quiet,map_quiet;
};
void nav_exec_init(struct nav_exec *e,struct nav_server *server);
/* All messages must pass ne_decode first. This function rechecks shape and
 * runs the real L1 admission guard; it does not generate motor authorization.
 * INFO/BIND/FRAME/ANCHOR and MAP/POSE/task/PERMIT all use this same entry point.
 * Return a fixed ne_status; caller builds the ACK with unchanged wire ABI. */
int nav_exec_handle(struct nav_exec *e,const struct ne_message *m,uint32_t now_ms,uint32_t cycles);
/* Main calls between polls, never from a cooperative callback. A* may call the
 * installed service; cancel/map/pose/permit handling stays available there.
 * Returns NE_BUSY if no new plan is due, otherwise the actual result. */
int nav_exec_plan(struct nav_exec *e,uint32_t now_ms);
void nav_exec_poll(struct nav_exec *e,uint32_t now_ms);
/* A legal external MOT, including zero, revokes this executor only. Old ROS
 * CANCEL/zero cannot write or revoke the manual controller's output. */
void nav_exec_manual_takeover(struct nav_exec *e);
/* Fresh MMIO owner/effective-motion observation only, default false. This is
 * not permission or a software claim; BIND/first START cannot infer external
 * quiescence from this executor's own zero output. */
void nav_exec_set_map_quiet(struct nav_exec *e,int quiet);
void nav_exec_set_external_quiet(struct nav_exec *e,int quiet);
int nav_exec_motion_allowed(const struct nav_exec *e,uint32_t now_ms);
/* Token-bound result submission: stale calculations cannot commit output. */
int nav_exec_submit_intent(struct nav_exec *e,const struct nv_intent *intent,uint32_t now_ms);
void nav_exec_begin_avoid(struct nav_exec *e);
void nav_exec_set_avoid_phase(struct nav_exec *e,enum nv_avoid_phase phase);
/* Caller requires two fresh clear observations AND real measured stop evidence. */
int nav_exec_finish_avoid(struct nav_exec *e,uint32_t now_ms);
/* L4 calls only after 80 mm geometry AND stop evidence. */
int nav_exec_arrive(struct nav_exec *e,uint32_t now_ms);
int nav_exec_cell(const struct nv_map *map,int32_t x_mm,int32_t y_mm,uint16_t *cell);
/* Paged reads perform identity + revision + READY checks on every access. */
int nav_exec_page(struct nav_exec *e,const struct ne_message *m,uint16_t *cells,unsigned capacity,unsigned *count);
#endif
