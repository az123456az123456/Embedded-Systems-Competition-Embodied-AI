#include "../imu_alignment.h"
#include "../navcore.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace robot_nav;
constexpr double pi=3.14159265358979323846;
double delta(double v){return std::atan2(std::sin(v),std::cos(v));}
int main(){
    unsigned checks=0;auto check=[&](bool v){assert(v);++checks;};
    ImuAlignment a;check(a.lookup(0).status==ImuLookupStatus::NeedFuture);
    check(a.push({1000,10,170*pi/180})==ImuPushStatus::Accepted);
    check(a.lookup(999).status==ImuLookupStatus::BeforeHistory);
    check(a.lookup(1001).status==ImuLookupStatus::NeedFuture);
    check(a.push({1000,10,170*pi/180})==ImuPushStatus::Duplicate);
    check(a.size()==1);check(a.push({990,9,0})==ImuPushStatus::IgnoredOld);
    check(a.push({1100,20,-178*pi/180})==ImuPushStatus::Accepted);
    for(unsigned t=1000;t<=1100;++t){auto value=a.lookup(t);check(value.status==ImuLookupStatus::Ready);check(std::abs(delta(value.yaw_rad-(170+(t-1000)*.12)*pi/180))<1e-9);}
    check(a.lookup(1101).status==ImuLookupStatus::NeedFuture);
    check(a.push({1100,20,-177*pi/180})==ImuPushStatus::Fault);check(a.fault_reason()=="imu_source_snapshot_conflict");
    const auto fault=a.diagnostics();check(a.push({1200,30,-170*pi/180})==ImuPushStatus::Fault);check(a.diagnostics().after_sequence==fault.after_sequence);
    check(a.lookup(1100).status==ImuLookupStatus::Fault);a.reset();check(!a.faulted()&&a.size()==0);
    check(a.push({0xfffffff0u,0xffffffffu,3.13})==ImuPushStatus::Accepted);
    check(a.push({0x20u,0u,-3.13})==ImuPushStatus::Accepted);
    auto v=a.lookup(0);check(v.status==ImuLookupStatus::Ready&&v.source_unwrapped_ms==int64_t(0x100000000ULL));
    check(std::abs(std::abs(v.yaw_rad)-pi)<.02);
    a.reset();check(a.push({100,1,0})==ImuPushStatus::Accepted);
    check(a.push({110,2,.5})==ImuPushStatus::Fault);check(a.fault_reason()=="imu_source_yaw_jump");
    check(a.diagnostics().dt_s==.01&&a.diagnostics().dyaw_rad==.5);
    a.reset();a.push({100,1,0});check(a.push({351,2,0})==ImuPushStatus::Fault);check(a.fault_reason()=="imu_source_gap");
    a.reset();a.push({100,1,0});check(a.push({99,2,0})==ImuPushStatus::Fault);check(a.fault_reason()=="imu_source_timestamp_not_forward");
    a.reset();check(a.push({0,1,std::numeric_limits<double>::quiet_NaN()})==ImuPushStatus::Fault);
    a.reset();for(unsigned i=0;i<80;++i)check(a.push({i*50,i,delta(i*.05)})==ImuPushStatus::Accepted);
    check(a.size()<=42);check(a.lookup(0).status==ImuLookupStatus::BeforeHistory);check(a.lookup(3900).status==ImuLookupStatus::Ready);
    // Reproduce the real event-driven SNS problem: two fresh encoder frames
    // one millisecond apart are joined to different reused HTTP yaw snapshots.
    // The physical motion is only 1 rad/s, and the yaw source itself is valid.
    Config c;c.configured=true;c.drive_mps_per_full_pwm=.6;
    auto frame=[](double stamp,double yaw){SensorFrame f;f.timestamp_s=stamp;f.imu_valid=true;f.yaw_rad=yaw;f.sonar_m={2.,2.,2.};return f;};
    auto before=frame(.099,0),after=frame(.100,.1);
    // Differential wheel motion agrees with the real 1 rad/s CCW rotation.
    const double metres_per_tick=c.wheel_circumference_m/c.encoder_ticks_per_turn;
    for(unsigned i=0;i<4;++i){double wheel=(c.left_wheel_mask&(1U<<i))?-.5*c.track_m:.5*c.track_m;
        after.counts[i]=uint32_t(int64_t(std::llround(wheel*.001/metres_per_tick*c.encoder_signs[i])));}
    Navigator old(c);old.ingest(before);check(!old.ingest(after));check(old.reason()=="imu_yaw_jump");
    ImuAlignment aligned;aligned.push({0,1,0});aligned.push({100,11,.1});
    auto y99=aligned.lookup(99),y100=aligned.lookup(100);check(y99.status==ImuLookupStatus::Ready&&y100.status==ImuLookupStatus::Ready);
    before.yaw_rad=y99.yaw_rad;after.yaw_rad=y100.yaw_rad;
    Navigator fixed(c);fixed.ingest(before);check(fixed.ingest(after));check(fixed.localization_valid(.100));
    // A genuine source jump is rejected before interpolation can smooth it.
    check(aligned.push({110,12,1.0})==ImuPushStatus::Fault);check(aligned.fault_reason()=="imu_source_yaw_jump");
    std::cout<<"PASS "<<checks<<" IMU alignment checks: source dedup, strict bracketing/no extrapolation, +/-pi and uint32 wrap, actual jump/gap latch, bounded history, asynchronous 1ms SNS false-jump reproduction\n";
}
