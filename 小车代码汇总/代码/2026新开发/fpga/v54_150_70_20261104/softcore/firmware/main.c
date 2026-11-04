#include <stdint.h>
#include "astar.h"
#include "mmio.h"
#include "nav_protocol.h"
#include "layered_mmio.h"
#include "nav_runtime.h"

static struct nav_server server;
struct nav_runtime navigation_runtime;
#define planner server.planner
/* These remain visible in the ELF for a debugger. Motor writes use the
 * independently authorized MOT4 MMIO commit; the CPU cannot arm hardware. */
volatile uint32_t observed_imu_words[16];
volatile uint32_t observed_motor_words[4];
volatile uint32_t selftest_passes;

static uint32_t run_selftests(void)
{
    uint32_t began, elapsed, expanded;
    int count;
    uint16_t first, last;
    unsigned x;
    astar_clear_map(&planner);
    began = NAV_CYCLES;
    count = astar_plan(&planner, 0, 0, 63, 63);
    elapsed = NAV_CYCLES - began;
    expanded = planner.expanded;
    if (count != 127 || !astar_path_get(&planner, 0, &first) || first != 0 ||
        !astar_path_get(&planner, 126, &last) || last != 4095)
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
        !astar_path_get(&planner, 0, &first) || first != 29u * ASTAR_SIDE + 17u)
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

static void cooperative_service(void) { nav_runtime_service(&navigation_runtime); }

int main(void)
{
    uint32_t last_beat,error;
    layered_service_init();
    nav_server_init(&server);
    nav_runtime_init(&navigation_runtime,&server);
    astar_set_service(cooperative_service);
    NAV_HEARTBEAT=1;NAV_STATUS=NAV_READY;NAV_FW_ERROR=0;
    NAV_PATH_LENGTH=0;NAV_ASTAR_CYCLES=0;NAV_EXPANDED=0;NAV_IMU_SEQUENCE=0;
    if(NAV_BOARD_ID!=NAV_EXPECTED_ID || nav_mmio_read(NM_ID)!=UINT32_C(0x4e584d31) || nav_mmio_read(NM_ABI)!=UINT32_C(0x00010000))error=1;
    else error=run_selftests();
    NAV_FW_ERROR=error;
    NAV_STATUS=NAV_READY|(error?NAV_FW_FAILED:NAV_SELFTEST_OK);
    navigation_runtime.ready=(uint8_t)(error==0);
    read_snapshot();last_beat=NAV_UPTIME_MS;
    for(;;){
        uint32_t now;
        nav_runtime_service(&navigation_runtime);
        if(!error)nav_runtime_plan(&navigation_runtime);
        now=NAV_UPTIME_MS;
        if((uint32_t)(now-last_beat)>=100u){
            last_beat=now;NAV_HEARTBEAT=NAV_HEARTBEAT+1u;read_snapshot();
        }
    }
}
