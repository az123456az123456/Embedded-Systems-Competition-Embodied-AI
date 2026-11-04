#include "../navcore.h"
#include "../vehicle_geometry.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <type_traits>

int main(){
    using namespace robot_nav;
    static_assert(std::is_same<decltype(Config{}.encoder_ticks_per_turn),double>::value,
                  "fractional user CPR must remain double throughout odometry");
    Config c;
    assert(c.encoder_ticks_per_turn==60527.7);
    assert((c.encoder_signs==std::array<int,4>{{-1,1,-1,1}}));
    assert(std::abs(c.body_front_m+c.body_rear_m-.312667)<1e-12);
    assert(std::abs(c.body_left_m+c.body_right_m-.252)<1e-12);
    assert(c.body_left_m==.137&&c.body_right_m==.115);
    assert(c.robot_radius_m==std::hypot(.16725,.137));
    assert(c.robot_radius_m>.216&&c.robot_radius_m<.217);
    assert(c.sonar_mounts[0][0]==.145417&&c.sonar_mounts[0][1]==0);
    assert(c.sonar_mounts[1][0]==.097417&&c.sonar_mounts[1][1]==.057);
    assert(c.sonar_mounts[2][0]==.097417&&c.sonar_mounts[2][1]==-.057);
    assert(std::abs(c.sonar_mounts[1][2]-vehicle_profile::pi/6)<1e-15);
    assert(std::abs(c.sonar_mounts[2][2]+vehicle_profile::pi/6)<1e-15);
    assert(c.imu_mount_x_m==.081417&&c.imu_mount_y_m==0);
    c.configured=true;c.drive_mps_per_full_pwm=1.;c.sonar_latency_s=0;
    assert(c.errors().empty());
    Navigator nav(c);
    SensorFrame frame;frame.timestamp_s=1;frame.sequence=1;frame.imu_valid=true;
    frame.sonar_m={{2.,2.,2.}};
    nav.ingest(frame);
    frame.timestamp_s=1.1;frame.sequence=2;
    frame.counts={{uint32_t(-6053),6053,uint32_t(-6053),6053}};
    assert(nav.ingest(frame));
    const double expected=6053*.20507/60527.7;
    assert(std::abs(nav.pose().x-expected)<1e-13);
    assert(std::abs(nav.pose().y)<1e-15);
    // A truncating uint32_t conversion would differ by ~2.4e-7 m here.
    assert(std::abs(nav.pose().x-6053*.20507/60527.)>1e-7);
    frame.timestamp_s=1.2;frame.sequence=3;frame.counts={{0,0,0,0}};
    assert(nav.ingest(frame));assert(std::abs(nav.pose().x)<1e-13);
    std::cout<<"PASS fixed vehicle extents, conservative sweep, sonar geometry, fractional CPR and four-sign forward/reverse odometry\n";
}
