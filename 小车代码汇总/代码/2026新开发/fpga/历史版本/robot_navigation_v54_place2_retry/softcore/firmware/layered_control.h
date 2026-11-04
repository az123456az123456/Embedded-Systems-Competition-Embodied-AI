#ifndef ROBOT_LAYERED_CONTROL_H
#define ROBOT_LAYERED_CONTROL_H
#include <stdint.h>
#define LC_PERIOD_MS 20u
#define LC_SONAR_TIMEOUT_MS 1000u
#define LC_MAX_DUTY 600
/* v48: user-selected50mm minimum, from the probe face. Forward source-age,
 * reaction and braking terms remain additive, not a fixed50mm stop promise. */
#define LC_FRONT_STOP_BASE_MM 50u
#define LC_SIDE_STOP_HARD_MM 50u
#define LC_STOP_REACTION_MS 850u
#define LC_STOP_SENSOR_LATENCY_MS 300u
#define LC_STOP_BRAKE_MMPS2 400u
#define LC_CIRCUMFERENCE_UM 205070u
#define LC_TRACK_UM 180635u
#define LC_CPR_X10 605277u
/* round(circumference_um * 10000 / CPR_x10), <0.001% quantization error. */
#define LC_TICK_NM 3388
#define LC_PROTOCOL_MOT4 1u
#define LC_CONFIGURED 2u
#define LC_ARMED 4u
#define LC_WATCHDOG 8u
#define LC_CPU_TRAP 16u
/* 0x31c upper diagnostic bits; existing low ready/filter bits stay unchanged. */
#define LC_REASON_SHIFT 16u
#define LC_MOUNT_VALID (UINT32_C(1)<<24)
#define LC_MOUNT_Z_DOWN (UINT32_C(1)<<25)
#define LC_SOURCE_RESTART_SEEN (UINT32_C(1)<<26)
enum lc_imu_readiness_reason {
    LC_IMU_READY=0, LC_IMU_WAIT_PACKET=1, LC_IMU_BAD_FORMAT=2,
    LC_IMU_RECEIVE_STALE=3, LC_IMU_SOURCE_STALE=4, LC_IMU_SOURCE_RESTART=5,
    LC_CAL_REQUESTED_MOTION=6, LC_CAL_ENCODER_MOVING=7, LC_CAL_GRAVITY_RANGE=8,
    LC_CAL_GYRO_OFFSET_RANGE=9, LC_CAL_GYRO_UNSTABLE=10,
    LC_CAL_COLLECTING=11, LC_IMU_SENSOR_NOT_INITIALIZED=12
};

struct lc_input {
    uint32_t now_ms, epoch, flags, command_sequence;
    uint32_t sample_id,imu_effective_ms;
    uint8_t local_valid;
    int16_t linear_mmps, yaw_rate_mradps;
    uint8_t left_mask, encoder_illegal;
    int32_t counts[4];
    uint32_t imu_words[16], imu_arrival_ms;
    uint16_t sonar_mm[3];
    uint32_t sonar_sequence[3], sonar_stamp_ms[3];
    uint8_t sonar_online;
};
struct lc_output {
    int16_t duty[4];
    uint16_t sonar_mm[3];
    uint32_t sonar_sequence[3];
    uint32_t flags, yaw_mdeg, bias_mdps, calibration_progress, late_count;
};
struct lc_kalman {
    int32_t x_q8;
    uint32_t variance, sequence, stamp_ms;
    uint16_t far_mm;
    uint8_t seen, valid, far_pending;
};
struct lc_state {
    struct lc_kalman sonar[3];
    int32_t count_previous[4], integral_q8[4], wheel_mmps[4];
    int32_t yaw_mdeg, target_yaw_mdeg, gyro_mdps, yaw_remainder, target_remainder;
    int32_t gyro_bias_mdps, calibration_sum[3], calibration_min[3], calibration_max[3];
    int32_t gyro_bias_vector_mdps[3], gravity_q12[3], calibration_accel_sum[3];
    uint32_t imu_sequence, imu_stamp_ms, calibration_started_ms;
    uint32_t last_step_ms, last_poll_ms, epoch, late_count, steps, max_step_ms;
    uint16_t calibration_count;
    uint8_t initialized, imu_seen, calibrated, active_previous, command_seen;
    uint8_t imu_reason, source_reset_seen;
    uint32_t command_sequence;
};
void lc_init(struct lc_state *state);
void lc_observe(struct lc_state *state,const struct lc_input *input);
/* Signed centre-line component; opposing wheel speeds in a pure turn cancel. */
int32_t lc_forward_component_mmps(const struct lc_state *state);
/* ceil(base + v*(source_age + profile latency/reaction) + v^2/(2*brake)).
 * Expired age or out-of-bound physical speed returns UINT16_MAX (blocked). */
uint32_t lc_front_stop_mm(int32_t forward_mmps, uint32_t source_age_ms);
/* Called on each 2ms snapshot; PI/PD runs at 20ms, sensor updates only on new
 * source sequence. Returns 1 when the output needs an atomic hardware commit. */
int lc_update(struct lc_state *state, const struct lc_input *input, struct lc_output *output);
#endif
