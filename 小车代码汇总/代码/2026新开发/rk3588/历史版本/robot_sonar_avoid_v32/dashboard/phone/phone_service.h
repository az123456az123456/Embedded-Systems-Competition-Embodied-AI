#ifndef ROBOT_PHONE_SERVICE_H
#define ROBOT_PHONE_SERVICE_H
#include "vision.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "imu_data.h"
#include "sensor_data.h"
struct phone_service;
struct phone_options {
    const char *bind_address, *pair_token_file, *interface_name, *model;
    unsigned port, left_mask, invert_mask, stream_fps; /* stream_fps: 0=15, otherwise 1..30 */
    int demo, camera_width, camera_height;
    const char *camera_view;
};
struct phone_status {
    int connected, armed;
    unsigned port;
    double left, right;
    char pair_code[129], address[128];
};
struct phone_imu_diagnostics {
    const char *interface_name;
    int socket_open,last_errno;
    uint64_t received,decoded,accepted;
    unsigned last_frame_bytes;
    unsigned sensor_flags,who_am_i,i2c_address;
};
/* Shares snapshots only: never opens a camera, NPU or IMU receive socket. */
struct phone_service *phone_start(const struct phone_options *, char *error, unsigned error_size);
void phone_publish(struct phone_service *, const struct vision_snapshot *, const struct imu_state *, uint64_t now_ms);
void phone_update_imu_network(struct phone_service *, const struct phone_imu_diagnostics *);
void phone_update_sensors(struct phone_service *, const struct sensor_state *, uint64_t now_ms);
void phone_reset_imu_reference(struct phone_service *, uint64_t now_ms);
/* Main thread owns the sole IMU filter. 1 = zero heading, 2 = calibrate. */
unsigned phone_take_imu_action(struct phone_service *);
void phone_disarm(struct phone_service *);
void phone_get_status(struct phone_service *, struct phone_status *);
void phone_stop(struct phone_service *);
#ifdef __cplusplus
}
#endif
#endif
