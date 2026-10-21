#ifndef IMU_DATA_H
#define IMU_DATA_H
#include <stdint.h>
#include <stddef.h>
#define IMU_G 9.80665
#define IMU_PI 3.14159265358979323846
#define IMU_CAL_MAX_AXIS_DPS 6.0
enum imu_calibration_status {
    IMU_CAL_WAITING, IMU_CAL_COLLECTING, IMU_CAL_MOTION, IMU_CAL_NOISE
};
struct imu_sample {
    uint16_t flags, ag_age, mag_age;
    uint32_t sequence, timestamp, uptime, errors, drops;
    double accel[3], gyro[3], temperature;
};
struct imu_state {
    struct imu_sample sample;
    double q[4], bias[3], sum[3], yaw_zero;
    double roll, pitch, yaw;
    unsigned calibration, calibrating;
    int bias_valid;
    unsigned calibration_samples, calibration_ms;
    enum imu_calibration_status calibration_status;
    double calibration_gyro_m2[3], calibration_accel_mean[3], calibration_accel_m2[3];
    double calibration_gyro_min[3], calibration_gyro_max[3];
    double calibration_accel_min[3], calibration_accel_max[3];
    double gyro_deadband;
    uint32_t sample_interval_ms;
    uint32_t previous_sequence, previous_timestamp, previous_uptime;
    uint64_t received_ms;
    int have_packet, initialized, have_sequence;
};
int imu_decode(const unsigned char *, size_t, struct imu_sample *);
void imu_init(struct imu_state *);
int imu_update(struct imu_state *, const struct imu_sample *, uint64_t);
int imu_fresh(const struct imu_state *, uint64_t);
void imu_calibrate(struct imu_state *);
void imu_zero_yaw(struct imu_state *);
int imu_open_socket(const char *interface_name, char name[16]);
uint64_t imu_clock_ms(void);
#endif
