#include "../firmware/astar.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static struct astar_context ctx;
static int16_t distance_to[ASTAR_CELLS];
static uint16_t bfs_queue[ASTAR_CELLS];
static uint8_t seen_path[ASTAR_CELLS];
static uint32_t random_state = UINT32_C(0x39a71e04);
static unsigned cases;

static uint32_t random_value(void)
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state;
}

/* Independent FIFO BFS: uses neither the A* heap, heuristic nor parent logic. */
static int blocked(unsigned cell)
{
    return !!(ctx.occupied[cell / 8u] & (1u << (cell % 8u)));
}

static int bfs(unsigned start, unsigned goal)
{
    unsigned head = 0, tail = 0, i;
    if (blocked(start) || blocked(goal)) return ASTAR_BLOCKED_ENDPOINT;
    for (i = 0; i < ASTAR_CELLS; ++i) distance_to[i] = -1;
    distance_to[start] = 0;
    bfs_queue[tail++] = (uint16_t)start;
    while (head < tail) {
        unsigned n = bfs_queue[head++];
        unsigned x = n % ASTAR_SIDE, y = n / ASTAR_SIDE;
        unsigned neighbors[4], count = 0;
        if (n == goal) return distance_to[n] + 1;
        if (x) neighbors[count++] = n - 1;
        if (x < ASTAR_SIDE - 1) neighbors[count++] = n + 1;
        if (y) neighbors[count++] = n - ASTAR_SIDE;
        if (y < ASTAR_SIDE - 1) neighbors[count++] = n + ASTAR_SIDE;
        for (i = 0; i < count; ++i) {
            unsigned next = neighbors[i];
            if (blocked(next) || distance_to[next] >= 0) continue;
            assert(tail < ASTAR_CELLS);
            distance_to[next] = distance_to[n] + 1;
            bfs_queue[tail++] = (uint16_t)next;
        }
    }
    return ASTAR_NO_ROUTE;
}

static int check(unsigned start, unsigned goal)
{
    int expected = bfs(start, goal);
    int actual = astar_plan(&ctx, start % ASTAR_SIDE, start / ASTAR_SIDE,
                           goal % ASTAR_SIDE, goal / ASTAR_SIDE);
    unsigned i;
    ++cases;
    if (actual != expected) {
        fprintf(stderr, "case %u start=%u goal=%u expected=%d actual=%d\n",
                cases, start, goal, expected, actual);
        assert(actual == expected);
    }
    if (actual < 0) return actual;
    assert(ctx.path[0] == start);
    assert(ctx.path[actual - 1] == goal);
    memset(seen_path, 0, sizeof(seen_path));
    for (i = 0; i < (unsigned)actual; ++i) {
        unsigned cell = ctx.path[i];
        assert(cell < ASTAR_CELLS);
        assert(!blocked(cell));
        assert(!seen_path[cell]);
        seen_path[cell] = 1;
        if (i) {
            unsigned prev = ctx.path[i - 1];
            int dx = (int)(cell % ASTAR_SIDE) - (int)(prev % ASTAR_SIDE);
            int dy = (int)(cell / ASTAR_SIDE) - (int)(prev / ASTAR_SIDE);
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            assert(dx + dy == 1);
        }
    }
    return actual;
}

int main(void)
{
    unsigned x, y, round;
    astar_clear_map(&ctx);
    assert(check(0, 4095) == 127);
    assert(check(4095, 0) == 127);
    assert(check(1023, 1023) == 1);
    astar_set_occupied(&ctx, 63, 63, 1);
    assert(check(0, 4095) == ASTAR_BLOCKED_ENDPOINT);
    assert(check(4095, 4095) == ASTAR_BLOCKED_ENDPOINT);
    assert(check(4095, 0) == ASTAR_BLOCKED_ENDPOINT);
    assert(astar_plan(&ctx, 64, 0, 0, 0) == ASTAR_BAD_COORDINATE);
    assert(astar_plan(&ctx, 0, 0, 0, 64) == ASTAR_BAD_COORDINATE);
    astar_clear_map(&ctx);
    for (x = 0; x < ASTAR_SIDE; ++x) astar_set_occupied(&ctx, x, 32, 1);
    assert(check(0, 4095) == ASTAR_NO_ROUTE);
    astar_set_occupied(&ctx, 31, 32, 0);
    assert(check(0, 4095) == 127);

    /* A long winding corridor catches truncated scores and parent chains. */
    astar_clear_map(&ctx);
    for (y = 1; y < ASTAR_SIDE; y += 2) {
        unsigned opening = ((y / 2) & 1u) ? 0u : 63u;
        for (x = 0; x < ASTAR_SIDE; ++x)
            astar_set_occupied(&ctx, x, y, x != opening);
    }
    assert(check(0, 62u * ASTAR_SIDE) == 2079);

    /* Repeat calls reuse the same context, so stale closed/heap state is tested. */
    for (round = 0; round < 320; ++round) {
        unsigned density = (round % 8u) * 10u;
        unsigned start, goal;
        astar_clear_map(&ctx);
        for (y = 0; y < ASTAR_SIDE; ++y)
            for (x = 0; x < ASTAR_SIDE; ++x)
                astar_set_occupied(&ctx, x, y, random_value() % 100u < density);
        start = random_value() % ASTAR_CELLS;
        goal = random_value() % ASTAR_CELLS;
        /* Include blocked-endpoint cases, then force clear and compare again. */
        check(start, goal);
        astar_set_occupied(&ctx, start % ASTAR_SIDE, start / ASTAR_SIDE, 0);
        astar_set_occupied(&ctx, goal % ASTAR_SIDE, goal / ASTAR_SIDE, 0);
        check(start, goal);
    }
    printf("PASS A*: %u BFS comparisons, 320 deterministic random maps, workspace=%zu bytes\n",
           cases, sizeof(ctx));
    return 0;
}
