#ifndef NAV_EXEC_PROTOCOL_H
#define NAV_EXEC_PROTOCOL_H
#include "../../common/nav_exec_wire.h"

/* The actual admission state used by the executor, not a test authority.
 * It does not enable motors or generate permission. Hardware checks these
 * conditions independently before applying a PWM commit. */
struct nx_identity {
 uint64_t session,run,map_epoch,map_revision,localization_epoch,plan;
 uint8_t goal[16];
};
#define NX_ID_SESSION 1u
#define NX_ID_RUN 2u
#define NX_ID_GOAL 4u
#define NX_ID_MAP_EPOCH 8u
#define NX_ID_MAP_REVISION 16u
#define NX_ID_LOCALIZATION 32u
#define NX_ID_PLAN 64u
#define NX_ID_TASK (NX_ID_SESSION|NX_ID_RUN|NX_ID_GOAL)
#define NX_ID_ALL 127u
#define NX_LANES 5u
#define NX_LIVE_MOTION 1u
#define NX_LIVE_AUTH 2u
#define NX_LIVE_POSE 4u
struct nx_anchor { uint64_t id; uint32_t expires_ms; };
struct nx_guard {
 uint64_t session,info_cookie,frame_token;
 uint64_t last_transport[NX_LANES];
 struct nx_anchor anchors[2];
 uint64_t pose_seq,pose_localization;
 int64_t pose_source_stamp_ns;
 uint64_t nav_epoch,auth_epoch,phone_seq,permit_seq;
 uint32_t info_ms,pose_deadline,permit_deadline,auth_deadline,pose_source_clock;
 uint8_t frame_len,live;
 uint8_t frame[63];
};
void nx_guard_init(struct nx_guard *g);
void nx_identity_from_header(struct nx_identity *id,const struct ne_header *h);
int nx_identity_check(const struct nx_identity *id,const struct ne_header *h,unsigned mask);
uint64_t nx_info_cookie(struct nx_guard *g,uint64_t request_id,uint32_t now_ms,uint32_t cycles);
int nx_bind_session(struct nx_guard *g,const struct ne_message *m,uint32_t now_ms,int owner_quiet);
int nx_check_session(const struct nx_guard *g,const struct ne_header *h);
/* Call only after the operation's semantic/identity checks. Failed requests
 * must not consume a sequence or overwrite a valid pending operation. */
int nx_accept_transport(struct nx_guard *g,const struct ne_header *h);
int nx_bind_frame(struct nx_guard *g,const struct ne_message *m);
int nx_bind_anchor(struct nx_guard *g,const struct ne_message *m,uint32_t now_ms);
int nx_anchor_valid(const struct nx_guard *g,uint64_t id,uint32_t now_ms);
int nx_accept_pose(struct nx_guard *g,const struct ne_message *m,uint32_t now_ms);
int nx_accept_permit(struct nx_guard *g,const struct ne_message *m,uint32_t now_ms,uint64_t expected_nav_epoch);
/* Remaining time returns zero at the exact expiry; validity is independent.
 * Motion/pose deadline == now is valid; auth deadline == now is invalid. */
int nx_motion_valid(const struct nx_guard *g,uint32_t now_ms);
int nx_authorized(const struct nx_guard *g,uint32_t now_ms);
int nx_pose_valid(const struct nx_guard *g,uint32_t now_ms);
void nx_revoke(struct nx_guard *g);
/* Called by the real-time poll; clears expired qualifications permanently
 * rather than allowing an old absolute deadline to revive after tick wrap. */
unsigned nx_expire(struct nx_guard *g,uint32_t now_ms);
#endif
