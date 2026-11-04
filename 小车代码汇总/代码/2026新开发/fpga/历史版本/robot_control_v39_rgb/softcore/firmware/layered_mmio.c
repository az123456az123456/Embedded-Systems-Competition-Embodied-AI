#include "layered_control.h"
#include "mmio.h"
/* Debugger-visible production state; host and hardware use the same controller. */
struct lc_state layered_state;
static struct lc_input input;
static struct lc_output output;
static uint32_t heartbeat;
static uint32_t last_poll;
volatile uint32_t control_service_calls;
volatile uint32_t control_service_max_gap_cycles;
static uint32_t previous_service_cycle;

void layered_service_init(void)
{
    lc_init(&layered_state);
    heartbeat=0; last_poll=UINT32_MAX-2u;
}
void layered_service(void)
{
    uint32_t now=NAV_UPTIME_MS, cycles=NAV_CYCLES, gap; unsigned i;
    gap=cycles-previous_service_cycle;
    if(previous_service_cycle && gap>control_service_max_gap_cycles)control_service_max_gap_cycles=gap;
    previous_service_cycle=cycles; ++control_service_calls;
    if((uint32_t)(now-last_poll)<2u)return;
    last_poll=now;
    NAV_REG(0x200)=1;
    input.now_ms=NAV_REG(0x204);
    input.epoch=NAV_REG(0x208);
    input.flags=NAV_REG(0x20c);
    input.command_sequence=NAV_REG(0x210);
    {
        uint32_t command=NAV_REG(0x214),masks=NAV_REG(0x218);
        input.linear_mmps=(int16_t)(command&65535u);
        input.yaw_rate_mradps=(int16_t)(command>>16);
        input.left_mask=(uint8_t)(masks&15u);
    }
    for(i=0;i<4;++i)input.counts[i]=(int32_t)NAV_REG(0x220+i*4u);
    input.encoder_illegal=(uint8_t)NAV_REG(0x230);
    input.imu_arrival_ms=NAV_REG(0x234);
    for(i=0;i<16;++i)input.imu_words[i]=NAV_REG(0x240+i*4u);
    for(i=0;i<3;++i) {
        input.sonar_mm[i]=(uint16_t)NAV_REG(0x280+i*4u);
        input.sonar_stamp_ms[i]=NAV_REG(0x290+i*4u);
        input.sonar_sequence[i]=NAV_REG(0x2a4+i*4u);
    }
    input.sonar_online=(uint8_t)NAV_REG(0x2a0);
    if(!lc_update(&layered_state,&input,&output))return;
    if(NAV_STATUS&NAV_FW_FAILED) {
        output.flags&=~1u;
        for(i=0;i<4;++i)output.duty[i]=0;
    }
    for(i=0;i<4;++i)NAV_REG(0x300+i*4u)=(uint32_t)(int32_t)output.duty[i];
    for(i=0;i<3;++i) {
        NAV_REG(0x310+i*4u)=output.sonar_mm[i];
        NAV_REG(0x320+i*4u)=output.sonar_sequence[i];
    }
    NAV_REG(0x31c)=output.flags;
    NAV_REG(0x330)=output.yaw_mdeg;
    NAV_REG(0x334)=output.bias_mdps;
    NAV_REG(0x338)=output.calibration_progress;
    NAV_REG(0x33c)=output.late_count;
    NAV_REG(0x32c)=++heartbeat;
}
