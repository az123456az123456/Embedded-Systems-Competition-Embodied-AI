#ifndef NAV_AVOID_H
#define NAV_AVOID_H
#include "nav_exec.h"
#include "nav_stop.h"
#include "layered_control.h"
/* Original v46 state machine:300mm avoidance entry;50mm raw hard stop.
 * Clearing the front and measured stop still precede replan. No new reverse state. */
#define NA_NEAR_MM 300u
#define NA_SIDE_RELEASE_MM 320u
#define NA_FRONT_CLEAR_MIN_MM 350u
#define NA_CLEAR_REFERENCE_MMPS 220 /* preserve conservative clearance threshold */
#define NA_TURN_MRADPS 350
#define NA_MIN_TURN_URAD 139626
#define NA_SCAN_LIMIT_URAD 12566372u
#define NA_BUDGET_MS 120000u

enum nv_avoid_result { NV_AVOID_INACTIVE=0,NV_AVOID_ACTIVE=1,NV_AVOID_REPLAN=2 };
enum nv_avoid_reason { NA_NONE=0,NA_NEAR=1,NA_SOURCE_WAIT=2,NA_STOP_WAIT=3,
 NA_SELECT_WAIT=4,NA_TURNING=5,NA_CLEAR_CONFIRM=6,NA_ADVANCING=7,
 NA_FINISH_STOP=8,NA_SCAN_LIMIT=9,NA_BAD_SOURCE=10 };
struct nav_avoid {
 uint64_t nav_epoch,pose_seq;
 uint32_t source_seq[3],source_ms[3],clear_seq[3];
 uint16_t raw_mm[3],filtered_mm[3];
 uint32_t started_ms,stop_ms,last_ms,cross_ms,scan_urad;
 int32_t turn_start_yaw,last_yaw;
 int16_t speed_mmps;
 int8_t direction;
 uint8_t known,filtered,active,finishing,crossed,clear_count,reason;
};
void nav_avoid_init(struct nav_avoid *a);
/* Call BEFORE nav_track_update in FOLLOWING and exclusively in AVOIDING.
 * A genuine near/expired beam invokes nav_exec_begin_avoid and an immediate
 * token-bound zero in this same call. If result!=INACTIVE or state changed
 * from FOLLOWING, caller MUST NOT call track in that cycle: stop is shared.
 * Hardware snapshots are real lc_input/lc_state plus nv_stop_sample, not a
 * fabricated boolean obstacle/stop or a second authorization backend.
 * Local decisions never inspect active/pending bitmap or path contents.
 * AVOID finish uses nav_exec_finish_avoid; main later calls nav_exec_plan.
 */
int nav_avoid_update(struct nav_exec *e,struct nav_avoid *a,struct nv_stop *stop,
 const struct nv_stop_sample *sample,const struct lc_input *raw,const struct lc_state *control);
#endif
