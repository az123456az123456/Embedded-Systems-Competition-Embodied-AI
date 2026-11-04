#ifndef PERCEPTION_DASHBOARD_H
#define PERCEPTION_DASHBOARD_H
#include "imu_data.h"
#include "vision.h"
#include "sensor_data.h"
#include "navigation_map.h"
#include "navigation_viewport.h"
#include "display_transform.h"
#define VIEW_W 1280
#define VIEW_H 760
struct dashboard {
    uint32_t pixels[VIEW_W * VIEW_H];
    float history[120][3];
    int head, count, demo;
    int phone_enabled, phone_connected, motors_armed;
    unsigned phone_port;
    char phone_pair_code[129], phone_address[128];
    double motor_left, motor_right;
    struct vision_snapshot vision;
    struct sensor_state sensors;
    int navigation_selected;
    struct navigation_map navigation;
    struct navigation_viewport navigation_viewport;
};
void dashboard_init(struct dashboard *);
void dashboard_navigation_native(const struct dashboard *,const struct display_transform *,uint32_t *,uint64_t);
void dashboard_history(struct dashboard *, const struct imu_state *, uint64_t);
void dashboard_render(struct dashboard *, const struct imu_state *, uint64_t, const char *);
/* 4 drags window margins; 5 camera tab; 6 navigation tab. No motor actions. */
int dashboard_action(double x, double y);
int dashboard_ppm(const struct dashboard *, const char *);
#endif
