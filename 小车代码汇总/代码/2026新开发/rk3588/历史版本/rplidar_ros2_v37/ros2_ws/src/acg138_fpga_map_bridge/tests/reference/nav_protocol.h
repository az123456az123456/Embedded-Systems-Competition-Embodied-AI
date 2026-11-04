#ifndef NAV_PROTOCOL_H
#define NAV_PROTOCOL_H
#include "astar.h"
#define NAV_MESSAGE_BYTES 64u
#define NAV_MAGIC UINT32_C(0x4e415632)
enum nav_operation { NAV_BEGIN=1, NAV_CHUNK=2, NAV_COMMIT=3, NAV_PLAN=4, NAV_PAGE=5, NAV_INFO=6 };
enum nav_error { NAV_OK=0, NAV_BAD_REQUEST=1, NAV_NO_MAP=2, NAV_GENERATION=3,
 NAV_INCOMPLETE=4, NAV_MAP_CRC=5, NAV_NO_ROUTE=6, NAV_BLOCKED=7,
 NAV_STALE=8, NAV_BUSY=9, NAV_INTERNAL=10 };
struct nav_server {
 struct astar_context planner;
 uint8_t staging[ASTAR_BITMAP_BYTES];
 uint8_t previous_request[64], previous_reply[64];
 uint32_t generation, staging_generation, expected_crc, previous_id, plan_id;
 uint32_t plan_cycles;
 uint16_t chunks, path_length;
 uint8_t map_valid, upload_active, have_previous, path_valid;
};
uint32_t nav_get32(const uint8_t *p);
void nav_put32(uint8_t *p,uint32_t v);
uint32_t nav_crc32(const uint8_t *p,unsigned len);
void nav_server_init(struct nav_server *s);
/* Returns1 for a reply. Invalid magic/version/reply/status/reserved is dropped.
 * cycles callback may be NULL for deterministic host tests. */
int nav_process(struct nav_server *s,const uint8_t request[64],uint8_t reply[64],
                uint32_t (*cycles)(void));
#endif
