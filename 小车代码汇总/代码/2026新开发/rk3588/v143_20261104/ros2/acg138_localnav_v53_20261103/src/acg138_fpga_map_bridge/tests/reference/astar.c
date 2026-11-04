#include "astar.h"
static void (*control_service)(void);
void astar_set_service(void (*service)(void)) { control_service = service; }
static void service(void) { if (control_service) control_service(); }

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

void astar_clear_map(struct astar_context *ctx)
{
    unsigned i;
    for (i = 0; i < ASTAR_BITMAP_BYTES; ++i) ctx->occupied[i] = 0;
}

void astar_set_occupied(struct astar_context *ctx, unsigned x, unsigned y, int value)
{
    if (x < ASTAR_SIDE && y < ASTAR_SIDE)
        bit_set(ctx->occupied, y * ASTAR_SIDE + x, value);
}

int astar_is_occupied(const struct astar_context *ctx, unsigned x, unsigned y)
{
    if (x >= ASTAR_SIDE || y >= ASTAR_SIDE) return 1;
    return bit_get(ctx->occupied, y * ASTAR_SIDE + x);
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
    uint16_t tmp = ctx->heap[a];
    ctx->heap[a] = ctx->heap[b];
    ctx->heap[b] = tmp;
    ctx->position[ctx->heap[a]] = (uint16_t)a;
    ctx->position[ctx->heap[b]] = (uint16_t)b;
}

static void heap_up(struct astar_context *ctx, unsigned i)
{
    while (i) {
        unsigned parent = (i - 1u) >> 1;
        if (!better(ctx, ctx->heap[i], ctx->heap[parent])) break;
        heap_swap(ctx, i, parent);
        i = parent;
    }
}

static uint16_t heap_pop(struct astar_context *ctx)
{
    uint16_t result = ctx->heap[0];
    unsigned i = 0;
    --ctx->heap_count;
    ctx->position[result] = ASTAR_NONE;
    if (!ctx->heap_count) return result;
    ctx->heap[0] = ctx->heap[ctx->heap_count];
    ctx->position[ctx->heap[0]] = 0;
    for (;;) {
        unsigned left = i * 2u + 1u, best = i;
        if (left >= ctx->heap_count) break;
        if (better(ctx, ctx->heap[left], ctx->heap[best])) best = left;
        if (left + 1u < ctx->heap_count &&
            better(ctx, ctx->heap[left + 1u], ctx->heap[best])) best = left + 1u;
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
    if (bit_get(ctx->occupied, to) || bit_get(ctx->closed, to)) return 0;
    tentative = (unsigned)ctx->g[from] + 1u;
    if (tentative >= ctx->g[to]) return 0;
    ctx->g[to] = (uint16_t)tentative;
    ctx->parent[to] = from;
    pos = ctx->position[to];
    if (pos == ASTAR_NONE) {
        if (ctx->heap_count >= ASTAR_CELLS) return ASTAR_INTERNAL_ERROR;
        pos = ctx->heap_count++;
        ctx->heap[pos] = to;
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
    ctx->expanded = 0;
    ctx->heap_count = 0;
    if (sx >= ASTAR_SIDE || sy >= ASTAR_SIDE || gx >= ASTAR_SIDE || gy >= ASTAR_SIDE)
        return ASTAR_BAD_COORDINATE;
    start = (uint16_t)(sy * ASTAR_SIDE + sx);
    goal = (uint16_t)(gy * ASTAR_SIDE + gx);
    if (bit_get(ctx->occupied, start) || bit_get(ctx->occupied, goal))
        return ASTAR_BLOCKED_ENDPOINT;
    for (i = 0; i < ASTAR_BITMAP_BYTES; ++i) {
        ctx->closed[i] = 0;
        if (!(i & 31u)) service();
    }
    for (i = 0; i < ASTAR_CELLS; ++i) {
        ctx->g[i] = ASTAR_NONE;
        ctx->parent[i] = ASTAR_NONE;
        ctx->position[i] = ASTAR_NONE;
        if (!(i & 31u)) service();
    }
    ctx->goal = goal;
    ctx->g[start] = 0;
    ctx->heap[0] = start;
    ctx->position[start] = 0;
    ctx->heap_count = 1;
    while (ctx->heap_count) {
        service();
        uint16_t current = heap_pop(ctx);
        unsigned x = current & 63u, y = current >> 6;
        ++ctx->expanded;
        if (current == goal) {
            unsigned count = 0, a, b;
            uint16_t n = goal;
            for (;;) {
                if (count >= ASTAR_CELLS) return ASTAR_INTERNAL_ERROR;
                ctx->path[count++] = n;
                if (!(count & 31u)) service();
                if (n == start) break;
                n = ctx->parent[n];
                if (n == ASTAR_NONE) return ASTAR_INTERNAL_ERROR;
            }
            for (a = 0, b = count - 1u; a < b; ++a, --b) {
                uint16_t tmp = ctx->path[a];
                ctx->path[a] = ctx->path[b];
                ctx->path[b] = tmp;
                if (!(a & 31u)) service();
            }
            return (int)count;
        }
        bit_set(ctx->closed, current, 1);
        if (x && visit(ctx, current, (uint16_t)(current - 1u))) return ASTAR_INTERNAL_ERROR;
        if (x + 1u < ASTAR_SIDE && visit(ctx, current, (uint16_t)(current + 1u))) return ASTAR_INTERNAL_ERROR;
        if (y && visit(ctx, current, (uint16_t)(current - ASTAR_SIDE))) return ASTAR_INTERNAL_ERROR;
        if (y + 1u < ASTAR_SIDE && visit(ctx, current, (uint16_t)(current + ASTAR_SIDE))) return ASTAR_INTERNAL_ERROR;
    }
    return ASTAR_NO_ROUTE;
}
