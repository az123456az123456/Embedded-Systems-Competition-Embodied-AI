#include "../navcore.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace robot_nav;
namespace {
unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x))throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+" " #x);}while(0)
bool close(double a,double b){return std::abs(a-b)<1e-10;}
Config config(){Config c;c.configured=true;c.drive_mps_per_full_pwm=.6;c.left_wheel_mask=12;c.sonar_latency_s=0;return c;}
SensorFrame sample(double host,std::optional<double> device={},double yaw=0,int32_t count=0){
    SensorFrame f;f.timestamp_s=host;f.device_timestamp_s=device;f.imu_valid=true;f.yaw_rad=yaw;
    f.sonar_m={{2.,2.,2.}};f.sequence=uint32_t(std::llround(host*1000));
    for(unsigned i=0;i<4;++i)f.counts[i]=uint32_t(count*vehicle_profile::encoder_signs[i]);
    return f;
}
void ready(Navigator& n,bool device=false){n.ingest(sample(1,device?std::optional<double>(10):std::nullopt));CHECK(n.ingest(sample(1.05,device?std::optional<double>(10.05):std::nullopt)));CHECK(n.localization_valid(1.050001));}
void test_encoder_diagnostic_and_first_reason(){
    auto c=config();Navigator n(c);ready(n);auto f=sample(1.10,{},.6,100000);f.sequence=91;
    CHECK(!n.ingest(f));CHECK(n.reason()=="encoder_jump");const auto first=n.localization_diagnostics();
    CHECK(first.latched&&first.first_reason=="encoder_jump");CHECK(first.jump_wheel_mask==15&&first.yaw_jump);
    CHECK(close(first.dt_s,.05)&&close(first.host_dt_s,.05));CHECK(close(first.dyaw_rad,.6));
    CHECK(close(first.yaw_limit_rad,c.max_measured_turn_rps*.05+.06));CHECK(close(first.wheel_limit_m,c.max_measured_wheel_mps*.05+.005));
    CHECK(first.motion_limits_evaluated&&!first.using_device_time&&first.imu_valid);
    CHECK(first.sequence==91&&first.previous_sequence==1050);
    for(double d:first.wheel_delta_m)CHECK(close(d,100000*c.wheel_circumference_m/c.encoder_ticks_per_turn));
    CHECK(n.localization_problem(1.10)=="encoder_jump");CHECK(!n.localization_valid(1.10));
    n.stop("operator_stop");CHECK(n.localization_diagnostics().first_reason=="encoder_jump");
    n.set_goal(1,0);CHECK(!n.start(1.10));CHECK(n.localization_diagnostics().sequence==91);
    n.invalidate_localization("later_source_fault",1.11);CHECK(n.localization_diagnostics().first_reason=="encoder_jump");
    CHECK(n.localization_diagnostics().timestamp_s==first.timestamp_s&&n.localization_diagnostics().dyaw_rad==first.dyaw_rad);
    auto later=sample(1.15,{},.6,100000);CHECK(n.ingest(later));CHECK(!n.localization_valid(1.150001));
    CHECK(n.localization_problem(1.150001)=="encoder_jump");
    n.reset_localization();CHECK(!n.localization_diagnostics().latched&&n.localization_diagnostics().first_reason.empty());ready(n);
}
void test_yaw_diagnostic(){
    Navigator n(config());ready(n);CHECK(!n.ingest(sample(1.1,{},.6)));CHECK(n.reason()=="imu_yaw_jump");
    const auto& d=n.localization_diagnostics();CHECK(d.first_reason=="imu_yaw_jump"&&d.yaw_jump&&d.jump_wheel_mask==0);
    for(double m:d.wheel_delta_m)CHECK(m==0);
    CHECK(close(d.dyaw_rad,.6));
}
void test_original_limits_unchanged(){
    auto c=config();double dt=1.1-1.05,yaw_limit=c.max_measured_turn_rps*dt+.06;
    for(double sign:{-1.,1.}){
        Navigator okay(c);ready(okay);CHECK(okay.ingest(sample(1.1,{},sign*(yaw_limit-1e-6))));
        Navigator bad(c);ready(bad);CHECK(!bad.ingest(sample(1.1,{},sign*(yaw_limit+1e-6))));CHECK(bad.reason()=="imu_yaw_jump");
    }
    double scale=c.wheel_circumference_m/c.encoder_ticks_per_turn,limit=c.max_measured_wheel_mps*dt+.005;
    int32_t low=int32_t(std::floor(limit/scale));
    Navigator okay(c);ready(okay);CHECK(okay.ingest(sample(1.1,{},0,low)));
    Navigator bad(c);ready(bad);CHECK(!bad.ingest(sample(1.1,{},0,low+1)));CHECK(bad.reason()=="encoder_jump");
}
void test_device_time_removes_receipt_burst_jump(){
    Navigator n(config());n.ingest(sample(100,10));CHECK(n.ingest(sample(100.001,10.05,.1,2000)));
    CHECK(n.localization_valid(100.001001));CHECK(close(n.pose().x,2000*.20507/60527.7*std::cos(.05)));
    CHECK(!n.localization_diagnostics().latched);
    // The same physical increment has always been invalid if it truly took
    // only one device millisecond. A host pause cannot make that jump safe.
    Navigator fast(config());fast.ingest(sample(100,10));CHECK(!fast.ingest(sample(100.05,10.001,.1,2000)));
    CHECK(fast.reason()=="encoder_jump");const auto& d=fast.localization_diagnostics();
    CHECK(d.using_device_time&&close(d.dt_s,.001)&&close(d.host_dt_s,.05));CHECK(d.yaw_jump);
    CHECK(close(d.yaw_limit_rad,.065)&&close(d.wheel_limit_m,.0065));
}
void test_device_short_interval_with_delayed_host_is_valid(){
    Navigator n(config());n.ingest(sample(100,10));CHECK(n.ingest(sample(100.05,10.001,.004,100)));
    CHECK(n.localization_valid(100.050001));CHECK(!n.localization_diagnostics().latched);
    CHECK(!n.ingest(sample(100.10,10.002,.104,100)));CHECK(n.reason()=="imu_yaw_jump");
    CHECK(close(n.localization_diagnostics().dt_s,.001)&&close(n.localization_diagnostics().host_dt_s,.05));
}
void test_device_jump_on_long_host_interval_still_rejected(){
    Navigator n(config());n.ingest(sample(100,10));CHECK(n.ingest(sample(100.05,10.001,0,100)));
    CHECK(!n.ingest(sample(100.10,10.002,0,3100)));CHECK(n.reason()=="encoder_jump");
}
void test_legacy_host_time_and_freshness(){
    Navigator n(config());ready(n);CHECK(n.ingest(sample(1.1,{},.1,2000)));CHECK(n.localization_valid(1.100001));
    CHECK(n.localization_problem(1.36)=="telemetry_stale");CHECK(!n.localization_diagnostics().latched);
    auto f=sample(1.15);f.imu_age_s=.31;CHECK(n.ingest(f));CHECK(n.localization_problem(1.150001)=="imu_invalid_or_stale");
}
void test_host_gap_remains_fail_closed(){
    Navigator n(config());ready(n,true);CHECK(!n.ingest(sample(1.4,10.1)));CHECK(n.reason()=="odometry_gap");
    const auto& d=n.localization_diagnostics();CHECK(d.latched&&d.first_reason=="odometry_gap");
    CHECK(close(d.host_dt_s,.35)&&close(d.dt_s,.05)&&d.using_device_time&&!d.motion_limits_evaluated);
}
void test_device_gap_rejected(){
    Navigator n(config());ready(n,true);CHECK(!n.ingest(sample(1.1,10.4)));CHECK(n.reason()=="sensor_device_time_gap");
    CHECK(close(n.localization_diagnostics().dt_s,.35));
}
void test_timebase_presence_change(){
    for(bool initially_device:{false,true}){
        Navigator n(config());ready(n,initially_device);
        CHECK(!n.ingest(sample(1.1,initially_device?std::nullopt:std::optional<double>(10.1))));
        CHECK(n.reason()=="sensor_timebase_changed");const auto& d=n.localization_diagnostics();
        CHECK(d.previous_has_device_time==initially_device&&d.sample_has_device_time!=initially_device);
        CHECK(d.latched&&!d.using_device_time);
    }
}
void test_device_duplicate_and_reversal(){
    for(double t:{10.05,10.04}){
        Navigator n(config());ready(n,true);CHECK(!n.ingest(sample(1.1,t)));CHECK(n.reason()=="sensor_device_timestamp_nonmonotonic");
        CHECK(n.localization_diagnostics().latched&&n.localization_diagnostics().dt_s<=0);
    }
}
void test_nonfinite_device_time(){
    for(double t:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){
        Navigator startup(config());CHECK(!startup.ingest(sample(1,t)));CHECK(!startup.localization_diagnostics().latched);
        Navigator n(config());ready(n,true);CHECK(!n.ingest(sample(1.1,t)));CHECK(n.reason()=="sensor_device_timestamp_invalid");
        CHECK(n.localization_diagnostics().latched);
    }
}
void test_invalid_sensor_protection_and_immutable_snapshot(){
    for(bool encoder:{false,true}){
        Navigator n(config());ready(n);auto f=sample(1.1);if(encoder)f.encoder_ok[2]=false;else f.imu_valid=false;
        CHECK(!n.ingest(f));CHECK(n.reason()=="invalid_odometry_frame");const auto first=n.localization_diagnostics();
        CHECK(first.latched&&first.encoder_invalid_mask==(encoder?4:0)&&first.imu_valid==encoder);
        CHECK(n.ingest(sample(1.15)));CHECK(!n.localization_valid(1.150001));
        CHECK(n.localization_problem(1.150001)=="invalid_odometry_frame");
    }
}
void test_external_source_invalidations(){
    Navigator startup(config());startup.invalidate_localization("waiting_source",0);
    CHECK(!startup.localization_diagnostics().latched);CHECK(startup.localization_problem(0)=="odometry_not_ready");ready(startup);
    Navigator n(config());ready(n,true);auto stop=n.invalidate_localization("imu_source_timestamp_reversal",1.06);
    CHECK(stop.left==0&&stop.right==0&&!n.enabled());const auto first=n.localization_diagnostics();
    CHECK(first.latched&&first.external_invalidation&&first.first_reason=="imu_source_timestamp_reversal");
    CHECK(n.localization_problem(1.06)=="imu_source_timestamp_reversal");
    n.stop("phone_stop");n.set_goal(1,0);CHECK(!n.start(1.06));CHECK(n.localization_diagnostics().first_reason==first.first_reason);
    n.invalidate_localization("another_source_fault",1.07);CHECK(n.localization_diagnostics().timestamp_s==first.timestamp_s);
    n.reset_localization();CHECK(!n.localization_diagnostics().latched);
}
void test_wheel_disagreement_protection_keeps_first_evidence(){
    auto c=config();Navigator n(c);n.ingest(sample(1,10));
    for(int i=1;i<=16;++i){auto f=sample(1+i*.05,10+i*.05);f.counts[2]=uint32_t(-i*200);n.ingest(f);}
    CHECK(n.localization_diagnostics().first_reason=="encoder_wheel_disagreement");
    CHECK(!n.localization_valid(1.800001));CHECK(n.localization_diagnostics().motion_limits_evaluated);
    CHECK(n.localization_diagnostics().wheel_comparison_window_s>=.25);
}
}
int main(){
    test_encoder_diagnostic_and_first_reason();test_yaw_diagnostic();test_original_limits_unchanged();
    test_device_time_removes_receipt_burst_jump();test_device_short_interval_with_delayed_host_is_valid();
    test_device_jump_on_long_host_interval_still_rejected();test_legacy_host_time_and_freshness();
    test_host_gap_remains_fail_closed();test_device_gap_rejected();test_timebase_presence_change();
    test_device_duplicate_and_reversal();test_nonfinite_device_time();test_invalid_sensor_protection_and_immutable_snapshot();
    test_external_source_invalidations();test_wheel_disagreement_protection_keeps_first_evidence();
    std::cout<<"PASS "<<checks<<" production localization diagnostics, immutable first faults, unchanged motion limits and device/host clock checks\n";
}
