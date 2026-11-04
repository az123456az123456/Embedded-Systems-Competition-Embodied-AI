#ifndef NAV_OBSTACLE_H
#define NAV_OBSTACLE_H
#include "nav_exec.h"
/* Existing firmware CAD nominal probe projections: front145.417mm,
 * side112.87mm. Rounding and +50mm uncertainty are explicit, not a claim of
 * measured probe extrinsics. Current body extents are user-measured.
 * Only new finite/fresh near returns may add occupied cells. Never clear. */
unsigned nav_obstacle_mark(struct nav_exec *e,unsigned sensor,uint16_t range_mm);
#endif
