#pragma once
#include <string>

namespace robotphone {
// Source coordinates refer to each 960x1080 sensor ROI, before preview scaling.
// Positive angles and lateral offsets point to the robot's right.
struct BearingEye {
    bool calibrated = false;
    double fx_px = 0;
    double cx_px = 480;
    double yaw_right_deg = 0;
    double lateral_right_m = 0;
    double min_u_px = 0;
    double max_u_px = 959;
    double max_error_deg = 0;
};
struct BearingConfig {
    BearingEye eyes[2];
    double lookahead_m = 1.0;
    bool enabled = false;
    BearingConfig() { eyes[0].lateral_right_m = -.060; eyes[1].lateral_right_m = -.020; }
};
struct BearingResult {
    bool valid = false;
    double camera_deg = 0; // capture-time optical ray, including mounting yaw
    double turn_deg = 0;   // current-time ray, adjusted to the virtual lookahead plane
    double u_px = 0;
    std::string reason;
};
// Errors reset output to the disabled default configuration.
bool loadBearingConfig(const std::string &path, BearingConfig &config, std::string &error);
bool validateBearingConfig(const BearingConfig &config, std::string &error);
BearingResult targetBearing(const BearingConfig &config, int eye, double source_u_px,
                            double yaw_change_left_deg);
}
