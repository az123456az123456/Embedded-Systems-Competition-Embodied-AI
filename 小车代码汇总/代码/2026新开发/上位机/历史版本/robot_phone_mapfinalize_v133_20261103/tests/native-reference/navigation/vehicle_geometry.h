#pragma once
#include <array>
#include <cmath>

// User-approved fixed v22 profile. Distances are metres, angles radians.
// CAD nominal dimensions are not an assertion of measured acoustic origins,
// load-dependent wheel circumference, or effective skid-steer track width.
namespace vehicle_profile {
inline constexpr char id[] = "acg138-v22-user-fixed-20261025";
inline constexpr double pi = 3.14159265358979323846;
inline constexpr double wheel_diameter_m = .065276;
inline constexpr double wheel_circumference_m = .20507;
inline constexpr double encoder_ticks_per_turn = 60527.7;
// Signal order M1, M2, M3, M4; not a claim about physical left/right wiring.
inline constexpr std::array<int,4> encoder_signs{{-1,1,-1,1}};
inline constexpr double track_m = .180635;
inline constexpr double wheelbase_m = .1719;
inline constexpr double body_front_m = .145417;
inline constexpr double body_rear_m = .16725;
inline constexpr double body_left_m = .137;  // CAD .115 + user-confirmed .022 board
inline constexpr double body_right_m = .115;
inline constexpr double body_width_m = body_left_m + body_right_m;
inline constexpr double body_length_m = body_front_m + body_rear_m;
inline constexpr double body_height_m = .27265;
// Conservative outer rectangle sweep, including left extension for full length.
// sqrt(.16725^2 + .137^2), precomputed as the nearest double so the
// serialized fixed profile is identical on host and target libm versions.
inline constexpr double robot_radius_m = .2161979706195227;
inline constexpr double imu_mount_x_m = .081417;
inline constexpr double imu_mount_y_m = 0.;
inline constexpr double sonar_half_angle_rad = 30. * pi / 180.;
inline constexpr std::array<std::array<double,3>,3> sonar_mounts{{
    {{.145417, 0., 0.}},
    {{.097417, .057, 30. * pi / 180.}},
    {{.097417,-.057,-30. * pi / 180.}}
}};
} // namespace vehicle_profile
