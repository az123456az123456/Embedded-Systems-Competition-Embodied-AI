#ifndef NAV_STOP_H
#define NAV_STOP_H
#include <stdint.h>

/* Source identities/ticks come from the actual encoder/IMU snapshot. Repeated
 * observations cannot accumulate a stop window just by changing now_ms. */
struct nv_stop_sample {
    uint32_t now_ms, encoder_ms, encoder_seq, imu_ms, imu_seq;
    int32_t counts[4], yaw_mdeg, gyro_mdps;
    uint8_t encoder_valid, imu_valid;
};
struct nv_stop_slice {
    uint32_t start_ms, end_ms, ticks[4];
    int32_t yaw_min, yaw_max;
};
struct nv_stop {
    struct nv_stop_slice slices[6];
    uint32_t requested_ms, encoder_ms, encoder_seq, imu_ms, imu_seq, imu_start_ms;
    uint32_t slice_start_ms, pending_ticks[4];
    int32_t previous_counts[4], yaw_origin, yaw_min, yaw_max;
    uint8_t active, seen, count, next, imu_samples, reason;
};
enum nv_stop_reason {
    NV_STOP_WAIT=0, NV_STOP_NO_SOURCE=1, NV_STOP_SOURCE_AGE=2,
    NV_STOP_BEFORE_REQUEST=3, NV_STOP_SOURCE_ORDER=4, NV_STOP_GAP=5,
    NV_STOP_WHEELS_MOVING=6, NV_STOP_IMU_MOVING=7, NV_STOP_CONFIRMED=8
};
void nv_stop_begin(struct nv_stop *stop, uint32_t now_ms);
/* 80ms measured window, each wheel <=15mm/s, |gyro|<=80mrad/s and
 * yaw spread <=80mrad/s * window +1mrad. Both source ages <=250ms.
 * All four wheel paths accumulate absolute differences, not net counts.
 * No sensor data, old pre-stop data or command-zero is proof of stopping. */
int nv_stop_update(struct nv_stop *stop, const struct nv_stop_sample *sample);
#endif
