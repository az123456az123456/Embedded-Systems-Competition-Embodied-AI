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
    ASTAR_INTERNAL_ERROR = -4,
    ASTAR_CANCELLED = -5,
    ASTAR_BUSY = -6
};

enum astar_phase {
    ASTAR_PHASE_IDLE = 0,
    ASTAR_PHASE_INIT = 1,
    ASTAR_PHASE_SEARCH = 2,
    ASTAR_PHASE_TRACE = 3,
    ASTAR_PHASE_REVERSE = 4,
    ASTAR_PHASE_READY = 5,
    ASTAR_PHASE_FAILED = 6,
    ASTAR_PHASE_CANCELLED = 7
};

/* No allocation or recursion: the complete working set lives in this object.
 * Initialize with astar_init() or all-zero initialization before any other API.
 * workspace is private algorithm storage: never read it from a callback or
 * consumer. Access the complete start-through-goal path only through the
 * guarded accessors below. Heap lifetime ends before TRACE starts.
 */
struct astar_context {
    uint8_t occupied[ASTAR_BITMAP_BYTES];
    uint8_t closed[ASTAR_BITMAP_BYTES];
    uint16_t g[ASTAR_CELLS];
    /* Exact four-neighbour back-link: child->parent is only -64,-1,+1,+64.
     * Zero means unvisited/none; start never reads this field. */
    int8_t parent[ASTAR_CELLS];
    union {
        uint16_t heap[ASTAR_CELLS];
        uint16_t path[ASTAR_CELLS];
    } workspace;
    uint16_t position[ASTAR_CELLS];
    uint16_t heap_count;
    uint16_t goal;
    uint32_t expanded;
    uint16_t path_length;
    uint8_t phase, cancel_requested;
    uint32_t path_revision;
};

/* Begin a new context lifetime. Never invoke during an operation. An embedded
 * owner (nav_server) must be reinitialized as a whole, including replay state;
 * independently reinitializing its planner would reuse publication identities.
 */
void astar_init(struct astar_context *ctx);
/* Map mutations are no-ops while busy; otherwise they invalidate the path. */
void astar_clear_map(struct astar_context *ctx);
void astar_set_occupied(struct astar_context *ctx, unsigned x, unsigned y, int value);
int astar_is_occupied(const struct astar_context *ctx, unsigned x, unsigned y);
/* Returns the number of cells in path[] or a negative astar_error.
 * Coordinates outside [0,63] and occupied endpoints are rejected. */
int astar_plan(struct astar_context *ctx, unsigned sx, unsigned sy,
               unsigned gx, unsigned gy);
enum astar_phase astar_get_phase(const struct astar_context *ctx);
int astar_busy(const struct astar_context *ctx);
unsigned astar_path_length(const struct astar_context *ctx);
uint32_t astar_path_revision(const struct astar_context *ctx);
/* Returns 1 only for a READY path and in-range index. On refusal, *cell is
 * unchanged and the overlaid workspace is not read. NULL cell is refused. */
int astar_path_get(const struct astar_context *ctx, unsigned index, uint16_t *cell);
/* Only busy operations accept cancellation. A request never publishes a path;
 * it is consumed at the next bounded cooperative checkpoint. */
int astar_cancel(struct astar_context *ctx);
/* Busy: request cancellation. Otherwise: invalidate and advance path revision.
 * Revision saturates at UINT32_MAX. No further plan is accepted after exhaustion
 * until a new context/owner lifetime is initialized; identities never wrap.
 */
void astar_invalidate_path(struct astar_context *ctx);
/* Cooperative high-priority service. Called at bounded work intervals during
 * initialization, every node expansion and path extraction. NULL disables it.
 * The callback may request cancellation/query accessors. Reentrant astar_plan
 * (including another context) returns ASTAR_BUSY without changing that target.
 * This is a single-CPU cooperative guard, not a thread-safety contract.
 * The callback must not access workspace or
 * initialize the context; map mutations are refused until the operation ends.
 */
void astar_set_service(void (*service)(void));
#endif
