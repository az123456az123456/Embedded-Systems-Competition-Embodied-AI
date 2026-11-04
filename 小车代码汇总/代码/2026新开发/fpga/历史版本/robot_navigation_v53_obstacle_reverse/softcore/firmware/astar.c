#include "astar.h"
static void (*control_service)(void);
/* One bare-metal cooperative scheduler owns the process-global service hook.
 * Refuse callback reentry even when it supplies a different planner context. */
static struct astar_context *active_context;
void astar_set_service(void (*service)(void)) { control_service = service; }
static void service(void) { if (control_service) control_service(); }

enum astar_phase astar_get_phase(const struct astar_context *ctx)
{
    return (enum astar_phase)ctx->phase;
}

int astar_busy(const struct astar_context *ctx)
{
    return ctx->phase >= ASTAR_PHASE_INIT && ctx->phase <= ASTAR_PHASE_REVERSE;
}

unsigned astar_path_length(const struct astar_context *ctx)
{
    return ctx->phase == ASTAR_PHASE_READY ? ctx->path_length : 0;
}

uint32_t astar_path_revision(const struct astar_context *ctx)
{
    return ctx->path_revision;
}

int astar_path_get(const struct astar_context *ctx, unsigned index, uint16_t *cell)
{
    if (ctx->phase != ASTAR_PHASE_READY || index >= ctx->path_length || !cell)
        return 0;
    *cell = ctx->workspace.path[index];
    return 1;
}

int astar_cancel(struct astar_context *ctx)
{
    if (!astar_busy(ctx)) return 0;
    ctx->cancel_requested = 1;
    return 1;
}

void astar_invalidate_path(struct astar_context *ctx)
{
    if (astar_cancel(ctx)) return;
    if (ctx->path_revision != UINT32_MAX) {
        ++ctx->path_revision;
        ctx->phase = ASTAR_PHASE_IDLE;
    } else ctx->phase = ASTAR_PHASE_FAILED;
    ctx->path_length = 0;
    ctx->cancel_requested = 0;
}

static int finish(struct astar_context *ctx, int result)
{
    active_context = 0;
    ctx->path_length = 0;
    ctx->heap_count = 0;
    ctx->phase = result == ASTAR_CANCELLED ? ASTAR_PHASE_CANCELLED : ASTAR_PHASE_FAILED;
    ctx->cancel_requested = 0;
    return result;
}

static int checkpoint(struct astar_context *ctx)
{
    service();
    return ctx->cancel_requested != 0;
}

static int bit_get(const uint8_t *bitmap, unsigned cell)
{
    return (bitmap[cell >> 3] >> (cell & 7u)) & 1u;
}

static void bit_set(uint8_t *bitmap, unsigned cell, int value)
{
    uint8_t mask = (uint8_t)(1u << (cell & 7u));
    if (value) bitmap[cell >> 3] |= mask;
    else bitmap[cell >> 3] &= (uint8_t)~mask;
}

void astar_init(struct astar_context *ctx)
{
    unsigned i;
    ctx->phase = ASTAR_PHASE_IDLE;
    ctx->cancel_requested = 0;
    ctx->path_length = 0;
    ctx->heap_count = 0;
    ctx->goal = 0;
    ctx->expanded = 0;
    ctx->path_revision = 0;
    for (i = 0; i < ASTAR_BITMAP_BYTES; ++i) ctx->occupied[i] = 0;
    for (i = 0; i < ASTAR_BITMAP_BYTES; ++i) ctx->dynamic[i] = 0;
}

void astar_clear_map(struct astar_context *ctx)
{
    unsigned i;
    if (astar_busy(ctx)) return;
    astar_invalidate_path(ctx);
    ctx->heap_count = 0;
    ctx->goal = 0;
    ctx->expanded = 0;
    for (i = 0; i < ASTAR_BITMAP_BYTES; ++i) ctx->occupied[i] = 0;
    for (i = 0; i < ASTAR_BITMAP_BYTES; ++i) ctx->dynamic[i] = 0;
}

void astar_set_occupied(struct astar_context *ctx, unsigned x, unsigned y, int value)
{
    if (!astar_busy(ctx) && x < ASTAR_SIDE && y < ASTAR_SIDE) {
        astar_invalidate_path(ctx);
        bit_set(ctx->occupied, y * ASTAR_SIDE + x, value);
    }
}

int astar_is_occupied(const struct astar_context *ctx, unsigned x, unsigned y)
{
    if (x >= ASTAR_SIDE || y >= ASTAR_SIDE) return 1;
    return bit_get(ctx->occupied, y * ASTAR_SIDE + x)||bit_get(ctx->dynamic,y*ASTAR_SIDE+x);
}

void astar_clear_dynamic(struct astar_context *ctx)
{
    unsigned i;if(astar_busy(ctx))return;
    astar_invalidate_path(ctx);
    for(i=0;i<ASTAR_BITMAP_BYTES;++i)ctx->dynamic[i]=0;
}
int astar_mark_dynamic(struct astar_context *ctx,unsigned x,unsigned y)
{
    unsigned cell=y*64u+x;
    if(astar_busy(ctx)||x>=64u||y>=64u||bit_get(ctx->dynamic,cell))return 0;
    bit_set(ctx->dynamic,cell,1);astar_invalidate_path(ctx);return 1;
}

static unsigned heuristic(unsigned a, unsigned b)
{
    unsigned ax = a & 63u, ay = a >> 6;
    unsigned bx = b & 63u, by = b >> 6;
    return (ax > bx ? ax - bx : bx - ax) + (ay > by ? ay - by : by - ay);
}

static int better(const struct astar_context *ctx, uint16_t a, uint16_t b)
{
    unsigned ha = heuristic(a, ctx->goal), hb = heuristic(b, ctx->goal);
    unsigned fa = ctx->g[a] + ha, fb = ctx->g[b] + hb;
    /* Deterministic tie breaking toward the goal reduces open-map expansions. */
    if (fa != fb) return fa < fb;
    if (ha != hb) return ha < hb;
    return a < b;
}

static void heap_swap(struct astar_context *ctx, unsigned a, unsigned b)
{
    uint16_t tmp = ctx->workspace.heap[a];
    ctx->workspace.heap[a] = ctx->workspace.heap[b];
    ctx->workspace.heap[b] = tmp;
    ctx->position[ctx->workspace.heap[a]] = (uint16_t)a;
    ctx->position[ctx->workspace.heap[b]] = (uint16_t)b;
}

static void heap_up(struct astar_context *ctx, unsigned i)
{
    while (i) {
        unsigned parent = (i - 1u) >> 1;
        if (!better(ctx, ctx->workspace.heap[i], ctx->workspace.heap[parent])) break;
        heap_swap(ctx, i, parent);
        i = parent;
    }
}

static uint16_t heap_pop(struct astar_context *ctx)
{
    uint16_t result = ctx->workspace.heap[0];
    unsigned i = 0;
    --ctx->heap_count;
    ctx->position[result] = ASTAR_NONE;
    if (!ctx->heap_count) return result;
    ctx->workspace.heap[0] = ctx->workspace.heap[ctx->heap_count];
    ctx->position[ctx->workspace.heap[0]] = 0;
    for (;;) {
        unsigned left = i * 2u + 1u, best = i;
        if (left >= ctx->heap_count) break;
        if (better(ctx, ctx->workspace.heap[left], ctx->workspace.heap[best])) best = left;
        if (left + 1u < ctx->heap_count &&
            better(ctx, ctx->workspace.heap[left + 1u], ctx->workspace.heap[best])) best = left + 1u;
        if (best == i) break;
        heap_swap(ctx, i, best);
        i = best;
    }
    return result;
}

static int visit(struct astar_context *ctx, uint16_t from, uint16_t to)
{
    unsigned tentative;
    uint16_t pos;
    if (astar_is_occupied(ctx,to%64u,to/64u) || bit_get(ctx->closed, to)) return 0;
    tentative = (unsigned)ctx->g[from] + 1u;
    if (tentative >= ctx->g[to]) return 0;
    ctx->g[to] = (uint16_t)tentative;
    ctx->parent[to] = (int8_t)((int)from - (int)to);
    pos = ctx->position[to];
    if (pos == ASTAR_NONE) {
        if (ctx->heap_count >= ASTAR_CELLS) return ASTAR_INTERNAL_ERROR;
        pos = ctx->heap_count++;
        ctx->workspace.heap[pos] = to;
        ctx->position[to] = pos;
    }
    heap_up(ctx, pos);
    return 0;
}

int astar_plan(struct astar_context *ctx, unsigned sx, unsigned sy,
               unsigned gx, unsigned gy)
{
    unsigned i;
    uint16_t start, goal;
    if (active_context || astar_busy(ctx)) return ASTAR_BUSY;
    if (ctx->path_revision == UINT32_MAX) return finish(ctx, ASTAR_INTERNAL_ERROR);
    active_context = ctx;
    ++ctx->path_revision;
    ctx->phase = ASTAR_PHASE_INIT;
    ctx->path_length = 0;
    ctx->cancel_requested = 0;
    ctx->expanded = 0;
    ctx->heap_count = 0;
    if (sx >= ASTAR_SIDE || sy >= ASTAR_SIDE || gx >= ASTAR_SIDE || gy >= ASTAR_SIDE)
        return finish(ctx, ASTAR_BAD_COORDINATE);
    start = (uint16_t)(sy * ASTAR_SIDE + sx);
    goal = (uint16_t)(gy * ASTAR_SIDE + gx);
    if (astar_is_occupied(ctx,start%64u,start/64u) || astar_is_occupied(ctx,goal%64u,goal/64u))
        return finish(ctx, ASTAR_BLOCKED_ENDPOINT);
    for (i = 0; i < ASTAR_BITMAP_BYTES; ++i) {
        ctx->closed[i] = 0;
        if (!(i & 31u) && checkpoint(ctx)) return finish(ctx, ASTAR_CANCELLED);
    }
    for (i = 0; i < ASTAR_CELLS; ++i) {
        ctx->g[i] = ASTAR_NONE;
        ctx->parent[i] = 0;
        ctx->position[i] = ASTAR_NONE;
        if (!(i & 31u) && checkpoint(ctx)) return finish(ctx, ASTAR_CANCELLED);
    }
    ctx->goal = goal;
    ctx->g[start] = 0;
    ctx->workspace.heap[0] = start;
    ctx->position[start] = 0;
    ctx->heap_count = 1;
    ctx->phase = ASTAR_PHASE_SEARCH;
    while (ctx->heap_count) {
        if (checkpoint(ctx)) return finish(ctx, ASTAR_CANCELLED);
        uint16_t current = heap_pop(ctx);
        unsigned x = current & 63u, y = current >> 6;
        ++ctx->expanded;
        if (current == goal) {
            unsigned count = 0, a, b;
            uint16_t n = goal;
            /* No subsequent heap operation is possible. The overlay becomes
             * private path construction storage until READY is published. */
            ctx->phase = ASTAR_PHASE_TRACE;
            if (checkpoint(ctx)) return finish(ctx, ASTAR_CANCELLED);
            for (;;) {
                if (count >= ASTAR_CELLS) return finish(ctx, ASTAR_INTERNAL_ERROR);
                ctx->workspace.path[count++] = n;
                if (!(count & 31u) && checkpoint(ctx)) return finish(ctx, ASTAR_CANCELLED);
                if (n == start) break;
                { /* Reject corrupt/none/wrapping links before indexing g[]. */
                    int delta = ctx->parent[n];
                    uint16_t previous;
                    if ((delta != -64 && delta != -1 && delta != 1 && delta != 64) ||
                        (delta == -64 && n < ASTAR_SIDE) ||
                        (delta == 64 && n >= ASTAR_CELLS - ASTAR_SIDE) ||
                        (delta == -1 && !(n & 63u)) ||
                        (delta == 1 && (n & 63u) == 63u))
                        return finish(ctx, ASTAR_INTERNAL_ERROR);
                    previous = (uint16_t)((int)n + delta);
                    if (ctx->g[previous] == ASTAR_NONE ||
                        (unsigned)ctx->g[previous] + 1u != ctx->g[n])
                        return finish(ctx, ASTAR_INTERNAL_ERROR);
                    n = previous;
                }
            }
            ctx->phase = ASTAR_PHASE_REVERSE;
            if (checkpoint(ctx)) return finish(ctx, ASTAR_CANCELLED);
            for (a = 0, b = count - 1u; a < b; ++a, --b) {
                uint16_t tmp = ctx->workspace.path[a];
                ctx->workspace.path[a] = ctx->workspace.path[b];
                ctx->workspace.path[b] = tmp;
                if (!(a & 31u) && checkpoint(ctx)) return finish(ctx, ASTAR_CANCELLED);
            }
            ctx->path_length = (uint16_t)count;
            ctx->heap_count = 0;
            ctx->phase = ASTAR_PHASE_READY;
            active_context = 0;
            return (int)count;
        }
        bit_set(ctx->closed, current, 1);
        if (x && visit(ctx, current, (uint16_t)(current - 1u))) return finish(ctx, ASTAR_INTERNAL_ERROR);
        if (x + 1u < ASTAR_SIDE && visit(ctx, current, (uint16_t)(current + 1u))) return finish(ctx, ASTAR_INTERNAL_ERROR);
        if (y && visit(ctx, current, (uint16_t)(current - ASTAR_SIDE))) return finish(ctx, ASTAR_INTERNAL_ERROR);
        if (y + 1u < ASTAR_SIDE && visit(ctx, current, (uint16_t)(current + ASTAR_SIDE))) return finish(ctx, ASTAR_INTERNAL_ERROR);
    }
    return finish(ctx, ASTAR_NO_ROUTE);
}
