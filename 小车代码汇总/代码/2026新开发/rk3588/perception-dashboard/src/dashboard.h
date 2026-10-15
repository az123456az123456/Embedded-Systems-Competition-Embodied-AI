#ifndef PERCEPTION_DASHBOARD_H
#define PERCEPTION_DASHBOARD_H
#include "imu_data.h"
#include "vision.h"
#define VIEW_W 1280
#define VIEW_H 760
struct dashboard {
    uint32_t pixels[VIEW_W * VIEW_H];
    float history[120][3];
    int head, count, demo;
    struct vision_snapshot vision;
};
void dashboard_init(struct dashboard *);
void dashboard_history(struct dashboard *, const struct imu_state *, uint64_t);
void dashboard_render(struct dashboard *, const struct imu_state *, uint64_t, const char *);
/* 1: zero relative heading; 2: calibrate; 3: close; 4: drag title bar. */
int dashboard_action(double x, double y);
int dashboard_ppm(const struct dashboard *, const char *);
#endif
