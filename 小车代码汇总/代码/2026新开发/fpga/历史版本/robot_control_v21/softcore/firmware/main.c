#include <stdint.h>
#include "astar.h"
#include "mmio.h"
#include "nav_protocol.h"

static struct nav_server server;
#define planner server.planner
/* These remain visible in the ELF for a debugger. There is no motor-write API. */
volatile uint32_t observed_imu_words[16];
volatile uint32_t observed_motor_words[4];
volatile uint32_t selftest_passes;

static uint32_t run_selftests(void)
{
    uint32_t began, elapsed, expanded;
    int count;
    unsigned x;
    astar_clear_map(&planner);
    began = NAV_CYCLES;
    count = astar_plan(&planner, 0, 0, 63, 63);
    elapsed = NAV_CYCLES - began;
    expanded = planner.expanded;
    if (count != 127 || planner.path[0] != 0 || planner.path[126] != 4095)
        return UINT32_C(0x1001);
    NAV_PATH_LENGTH = (uint32_t)count;
    NAV_ASTAR_CYCLES = elapsed;
    NAV_EXPANDED = expanded;

    astar_set_occupied(&planner, 63, 63, 1);
    if (astar_plan(&planner, 0, 0, 63, 63) != ASTAR_BLOCKED_ENDPOINT)
        return UINT32_C(0x1002);
    astar_clear_map(&planner);
    for (x = 0; x < ASTAR_SIDE; ++x) astar_set_occupied(&planner, x, 32, 1);
    if (astar_plan(&planner, 0, 0, 63, 63) != ASTAR_NO_ROUTE)
        return UINT32_C(0x1003);
    astar_clear_map(&planner);
    if (astar_plan(&planner, 17, 29, 17, 29) != 1 ||
        planner.path[0] != 29u * ASTAR_SIDE + 17u)
        return UINT32_C(0x1004);
    ++selftest_passes;
    return 0;
}

static void read_snapshot(void)
{
    uint32_t capabilities = NAV_CAPABILITIES;
    unsigned i;
    NAV_SNAPSHOT = 1;
    /* Ordered volatile accesses: hardware captures both buses on this command. */
    if (capabilities & 1u) {
        for (i = 0; i < 16; ++i) observed_imu_words[i] = NAV_IMU_WORD(i);
        if (observed_imu_words[0] == UINT32_C(0x494d5539))
            NAV_IMU_SEQUENCE = observed_imu_words[2];
    }
    if (capabilities & 2u)
        for (i = 0; i < 4; ++i) observed_motor_words[i] = NAV_MOTOR_WORD(i);
}

static uint32_t cycle_reader(void) { return NAV_CYCLES; }

int main(void)
{
    uint32_t last_beat, error;
    uint8_t request[64], reply[64];
    unsigned i;
    nav_server_init(&server);
    NAV_HEARTBEAT = 1;
    NAV_STATUS = NAV_READY;
    NAV_FW_ERROR = 0;
    NAV_PATH_LENGTH = 0;
    NAV_ASTAR_CYCLES = 0;
    NAV_EXPANDED = 0;
    NAV_IMU_SEQUENCE = 0;
    if (NAV_BOARD_ID != NAV_EXPECTED_ID) error = 1;
    else error = run_selftests();
    NAV_FW_ERROR = error;
    NAV_STATUS = NAV_READY | (error ? NAV_FW_FAILED : NAV_SELFTEST_OK);
    read_snapshot();
    last_beat = NAV_UPTIME_MS;
    for (;;) {
        uint32_t now = NAV_UPTIME_MS;
        if ((uint32_t)(now - last_beat) >= 100u) {
            last_beat = now;
            NAV_HEARTBEAT = NAV_HEARTBEAT + 1u;
            read_snapshot();
        }
        if (!error && NAV_RX_PENDING && NAV_TX_READY) {
            for (i=0;i<16;++i) nav_put32(request+i*4u,NAV_RX_WORD(i));
            NAV_RX_RELEASE=1;
            if (nav_process(&server,request,reply,cycle_reader)) {
                for(i=0;i<16;++i) NAV_TX_WORD(i)=nav_get32(reply+i*4u);
                NAV_TX_SEND=1;
                NAV_PATH_LENGTH=server.path_valid?server.path_length:0;
                NAV_ASTAR_CYCLES=server.plan_cycles;
                NAV_EXPANDED=planner.expanded;
            }
        }
    }
}
