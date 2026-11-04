#ifndef NAV_PROTOCOL_H
#define NAV_PROTOCOL_H
#include "astar.h"
/* Internal map/planner storage shared by NAV3. Filename retained for includes,
 * not a wire-protocol compatibility implementation. */
struct nav_server {
 struct astar_context planner;
 uint8_t staging[ASTAR_BITMAP_BYTES];
 uint32_t generation,path_revision,plan_cycles;
 uint16_t path_length;
 uint8_t map_valid,upload_active,path_valid;
};
uint32_t nav_crc32(const uint8_t *p,unsigned len);
void nav_server_init(struct nav_server *s);
#endif
