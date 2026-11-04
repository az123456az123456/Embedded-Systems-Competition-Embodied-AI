#include "layered_mmio.h"
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
int layered_snapshot(void)
{
    uint32_t now=NAV_UPTIME_MS,cycles=NAV_CYCLES,gap;unsigned i;
    gap=cycles-previous_service_cycle;
    if(previous_service_cycle&&gap>control_service_max_gap_cycles)control_service_max_gap_cycles=gap;
    previous_service_cycle=cycles;++control_service_calls;
    if((uint32_t)(now-last_poll)<2u)return 0;
    last_poll=now;NAV_REG(0x200)=1;
    input.now_ms=NAV_REG(0x204);input.epoch=NAV_REG(0x208);input.flags=NAV_REG(0x20c);input.command_sequence=NAV_REG(0x210);
    {uint32_t c=NAV_REG(0x214),m=NAV_REG(0x218);input.linear_mmps=(int16_t)c;input.yaw_rate_mradps=(int16_t)(c>>16);input.left_mask=(uint8_t)(m&15u);}
    for(i=0;i<4;++i)input.counts[i]=(int32_t)NAV_REG(0x220+i*4u);
    input.encoder_illegal=(uint8_t)NAV_REG(0x230);input.imu_arrival_ms=NAV_REG(0x234);
    for(i=0;i<16;++i)input.imu_words[i]=NAV_REG(0x240+i*4u);
    for(i=0;i<3;++i){input.sonar_mm[i]=(uint16_t)NAV_REG(0x280+i*4u);input.sonar_stamp_ms[i]=NAV_REG(0x290+i*4u);input.sonar_sequence[i]=NAV_REG(0x2a4+i*4u);}
    input.sonar_online=(uint8_t)NAV_REG(0x2a0);return 1;
}
const struct lc_input *layered_raw(void){return &input;}
const struct lc_output *layered_output(void){return &output;}
int layered_step(int nav,int32_t v,int32_t w,uint32_t epoch)
{
    uint32_t old_flags=input.flags,old_epoch=input.epoch;int16_t old_v=input.linear_mmps,old_w=input.yaw_rate_mradps;uint8_t old_mask=input.left_mask;int commit;unsigned i;
    if(nav){input.flags=LC_PROTOCOL_MOT4|LC_CONFIGURED|LC_ARMED;input.epoch=epoch;input.left_mask=10;
        input.linear_mmps=layered_state.calibrated?(int16_t)v:0;input.yaw_rate_mradps=layered_state.calibrated?(int16_t)w:0;}
    commit=lc_update(&layered_state,&input,&output);
    input.flags=old_flags;input.epoch=old_epoch;input.linear_mmps=old_v;input.yaw_rate_mradps=old_w;input.left_mask=old_mask;
    if(NAV_STATUS&NAV_FW_FAILED){output.flags&=~1u;for(i=0;i<4;++i)output.duty[i]=0;}
    return commit;
}
void layered_publish_legacy(int nav)
{
    unsigned i;
    /* A NAV calculation must never become a fallback manual duty. The legacy
     * heartbeat still publishes filter/IMU diagnostics but keeps duties zero. */
    for(i=0;i<4;++i)NAV_REG(0x300+i*4u)=nav?0u:(uint32_t)(int32_t)output.duty[i];
    for(i=0;i<3;++i){NAV_REG(0x310+i*4u)=output.sonar_mm[i];NAV_REG(0x320+i*4u)=output.sonar_sequence[i];}
    NAV_REG(0x31c)=output.flags;NAV_REG(0x330)=output.yaw_mdeg;NAV_REG(0x334)=output.bias_mdps;NAV_REG(0x338)=output.calibration_progress;NAV_REG(0x33c)=output.late_count;NAV_REG(0x32c)=++heartbeat;
}
void layered_stop_sample(struct nv_stop_sample *s)
{
    unsigned i;s->now_ms=input.now_ms;s->encoder_ms=input.now_ms;s->encoder_seq=input.now_ms;
    for(i=0;i<4;++i)s->counts[i]=input.counts[i];
    s->encoder_valid=(uint8_t)(input.encoder_illegal==0);s->imu_ms=layered_state.imu_stamp_ms;s->imu_seq=layered_state.imu_sequence;s->yaw_mdeg=layered_state.yaw_mdeg;s->gyro_mdps=layered_state.gyro_mdps;
    s->imu_valid=(uint8_t)(layered_state.imu_seen&&layered_state.calibrated&&layered_state.imu_reason==LC_IMU_READY);
}
void layered_service(void){if(layered_snapshot()&&layered_step(0,0,0,0))layered_publish_legacy(0);}
