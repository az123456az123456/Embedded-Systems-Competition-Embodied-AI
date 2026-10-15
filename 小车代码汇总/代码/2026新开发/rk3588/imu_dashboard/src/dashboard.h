#ifndef DASHBOARD_H
#define DASHBOARD_H
#include "imu_data.h"
#define VIEW_W 1000
#define VIEW_H 680
struct dashboard {
    uint32_t pixels[VIEW_W*VIEW_H];
    float history[120][3];
    int head, count, demo;
};
void dashboard_init(struct dashboard *);
void dashboard_history(struct dashboard *, const struct imu_state *, uint64_t);
void dashboard_render(struct dashboard *, const struct imu_state *, uint64_t, const char *);
// Returns 1=zero heading, 2=calibrate gyro, 3=close, 4=drag title bar.
int dashboard_action(double x, double y);
int dashboard_ppm(const struct dashboard *,const char *path);
#endif
