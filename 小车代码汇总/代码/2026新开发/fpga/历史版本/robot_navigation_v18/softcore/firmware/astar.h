#ifndef ROBOT_ASTAR_H
#define ROBOT_ASTAR_H
#include <stdint.h>

#define ASTAR_SIDE 64u
#define ASTAR_CELLS (ASTAR_SIDE * ASTAR_SIDE)
#define ASTAR_BITMAP_BYTES (ASTAR_CELLS / 8u)
#define ASTAR_NONE UINT16_MAX

enum astar_error {
    ASTAR_BAD_COORDINATE = -1,
    ASTAR_BLOCKED_ENDPOINT = -2,
    ASTAR_NO_ROUTE = -3,
    ASTAR_INTERNAL_ERROR = -4
};

/* No allocation or recursion: the complete working set lives in this object.
 * path[] contains start through goal, including both endpoints. */
struct astar_context {
    uint8_t occupied[ASTAR_BITMAP_BYTES];
    uint8_t closed[ASTAR_BITMAP_BYTES];
    uint16_t g[ASTAR_CELLS];
    uint16_t parent[ASTAR_CELLS];
    uint16_t heap[ASTAR_CELLS];
    uint16_t position[ASTAR_CELLS];
    uint16_t path[ASTAR_CELLS];
    uint16_t heap_count;
    uint16_t goal;
    uint32_t expanded;
};

void astar_clear_map(struct astar_context *ctx);
void astar_set_occupied(struct astar_context *ctx, unsigned x, unsigned y, int value);
int astar_is_occupied(const struct astar_context *ctx, unsigned x, unsigned y);
/* Returns the number of cells in path[] or a negative astar_error.
 * Coordinates outside [0,63] and occupied endpoints are rejected. */
int astar_plan(struct astar_context *ctx, unsigned sx, unsigned sy,
               unsigned gx, unsigned gy);
#endif
