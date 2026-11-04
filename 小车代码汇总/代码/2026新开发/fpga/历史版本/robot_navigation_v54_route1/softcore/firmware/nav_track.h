#ifndef NAV_TRACK_H
#define NAV_TRACK_H
#include "nav_exec.h"
#include "nav_stop.h"
/* Keep one fresh250ms pose interval below half a100mm planning cell. This
 * bounds requested speed, not a measured physical stopping-distance claim. */
#define NT_MAX_LINEAR_MMPS 180
#define NT_STRAIGHT_PASS_CROSS_MM 40
enum nv_track_result { NV_TRACK_WAIT=0,NV_TRACK_COMMAND=1,NV_TRACK_STOPPING=2,NV_TRACK_ARRIVED=3,
 NV_TRACK_BLOCKED=-1,NV_TRACK_INVALID=-2,NV_TRACK_OUTSIDE=-3 };
struct nav_track {
 uint64_t nav_epoch,plan;
 uint32_t last_ms;
 int16_t speed_mmps;
 uint16_t turn_cursor;
 int8_t turn_direction;
 uint8_t initialized,final_leg,stopping,turn_phase;
};
void nav_track_init(struct nav_track *t);
/* Shared stop evidence can be reused by track/avoid because main states are
 * mutually exclusive. No false boolean stop input and no second path copy.
 * Call at >=50Hz; the speed trajectory advances at most once per20ms. A new
 * POSE/PERMIT identity recomputes geometry before the next PI release without
 * advancing that trajectory. Invalidation/arrival zero remains immediate.
 * The sample is the actual sensor/MMIO
 * snapshot. nav_exec owns pose, path cursor and token-bound command output. */
int nav_track_update(struct nav_exec *e,struct nav_track *t,struct nv_stop *stop,
                     const struct nv_stop_sample *sample);
/* True supercover traversal in local millimetres, not just grid endpoints. */
int nav_track_line_clear(const struct astar_context *planner,const struct nv_map *map,
                         int32_t x0,int32_t y0,int32_t x1,int32_t y1);
#endif
