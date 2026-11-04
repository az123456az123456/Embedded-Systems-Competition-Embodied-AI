#include "../sensor_bridge.h"
#include "../vendor/picojson.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace robot_nav;
constexpr double base=100.;
Config config(){Config c;c.configured=true;c.drive_mps_per_full_pwm=.6;c.sonar_latency_s=0;return c;}
robotnav::GatewayTelemetry imu(uint32_t device,uint32_t seq,double yaw_rad,double age_ms=5){
    robotnav::GatewayTelemetry t;t.available=true;t.imu_valid=t.imu_fresh=t.imu_bias_valid=t.imu_timestamp_valid=true;
    t.imu_calibrating=t.imu_action_pending=false;t.imu_timestamp_ms=device;t.imu_sequence=seq;t.yaw_deg=yaw_rad*180/vehicle_profile::pi;
    t.imu_sample_age_ms=age_ms;t.request_started_ms=uint64_t(std::llround(base*1000+device+age_ms));t.received_ms=t.request_started_ms+3;return t;
}
navwire::Sensors sns(uint32_t device,uint32_t seq){navwire::Sensors s;s.uptime_ms=device;s.sequence=seq;s.valid=7;s.distance_mm={2000,2000,2000};s.age_ms={0,0,0};return s;}
void establish(NavigationInputs& input,Navigator& nav){
    input.update_imu(imu(0,1,0),base+.008);input.update_imu(imu(100,11,.1),base+.108);
    input.enqueue(sns(50,1),base+.051);input.enqueue(sns(100,2),base+.101);input.drain(nav,base+.108);
    assert(nav.localization_valid(base+.108)&&input.fault_reason().empty());
}
int main(){unsigned checks=0;
#define check(x) do { ++checks; if (!(x)) { std::cerr << "sensor bridge failed line=" << __LINE__ << " check=" << checks << "\n"; std::abort(); } } while (0)
    {
        NavigationInputs input(config());Navigator nav(config());robotnav::GatewayTelemetry absent;
        input.update_imu(absent,base);input.enqueue(sns(1,1),base+.001);input.drain(nav,base+.002);
        check(!nav.frame());check(input.fault_reason().empty());
        auto uncal=imu(0,1,0);uncal.imu_calibrating=true;input.update_imu(uncal,base+.008);input.drain(nav,base+.008);
        check(!nav.localization_diagnostics().latched&&input.fault_reason().empty());
        establish(input,nav);check(nav.frame()->device_timestamp_s&&std::abs(*nav.frame()->device_timestamp_s-.1)<1e-9);
    }
    {
        NavigationInputs input(config());Navigator nav(config());input.update_imu(imu(0,1,0),base+.008);
        input.enqueue(sns(99,1),base+.0995);input.enqueue(sns(100,2),base+.1005);input.drain(nav,base+.101);
        check(!nav.frame()&&input.wait_reason()=="waiting_future_imu_bracket");
        input.update_imu(imu(100,11,.1),base+.108);input.drain(nav,base+.108);
        check(nav.localization_valid(base+.108));check(std::abs(nav.pose().yaw-.1)<1e-9);check(!nav.localization_diagnostics().latched);
        check(std::abs(nav.frame()->imu_age_s-.0025)<1e-8); // source bound includes 2ms conservative margin
    }
    {
        // Kernel delivery can bunch two snapshots: motion timing remains the
        // device's 50ms, while freshness still uses the actual host receipt.
        NavigationInputs input(config());Navigator nav(config());input.update_imu(imu(0,1,0),base+.008);input.update_imu(imu(50,6,.15),base+.058);
        auto a=sns(0,1),b=sns(50,2);double scale=config().wheel_circumference_m/config().encoder_ticks_per_turn;
        for(unsigned i=0;i<4;++i){double velocity=(config().left_wheel_mask&(1U<<i))?-.5*config().track_m*3:.5*config().track_m*3;b.counts[i]=int32_t(std::llround(velocity*.05/scale*config().encoder_signs[i]));}
        input.enqueue(a,base+.055);input.enqueue(b,base+.056);input.drain(nav,base+.058);
        check(nav.localization_valid(base+.058));check(nav.frame()->device_timestamp_s&&*nav.frame()->device_timestamp_s==.05);
    }
    {
        NavigationInputs input(config());Navigator nav(config());establish(input,nav);
        auto missing=imu(100,11,.1);missing.available=false;input.update_imu(missing,base+.15);input.enqueue(sns(150,3),base+.151);input.drain(nav,base+.16);
        check(input.fault_reason().empty()&&!nav.localization_diagnostics().latched);check(nav.frame()->sequence==2);
        input.update_imu(imu(200,21,.2),base+.208);input.drain(nav,base+.208);
        check(input.fault_reason().empty()&&nav.localization_valid(base+.208));check(nav.frame()->sequence==3);
    }
    {
        NavigationInputs input(config());Navigator nav(config());establish(input,nav);
        auto invalid=imu(150,16,.15);invalid.imu_valid=false;input.update_imu(invalid,base+.158);input.drain(nav,base+.158);
        check(nav.localization_diagnostics().latched&&input.fault_reason()=="imu_source_invalid");
        input.update_imu(imu(200,21,.2),base+.208);input.drain(nav,base+.208);check(!nav.localization_valid(base+.208));
        input.reset();nav.reset_localization();establish(input,nav);check(nav.localization_valid(base+.108));
    }
    {
        NavigationInputs input(config());Navigator nav(config());establish(input,nav);
        input.update_imu(imu(110,12,1.),base+.118);input.drain(nav,base+.118);
        check(input.fault_reason()=="imu_source_yaw_jump"&&nav.localization_diagnostics().latched);
        picojson::value diagnostic;check(picojson::parse(diagnostic,input.diagnostics_json()).empty());
        check(diagnostic.get("before_timestamp_ms").get<double>()==100&&diagnostic.get("after_timestamp_ms").get<double>()==110);
        check(std::abs(diagnostic.get("source_dt_s").get<double>()-.01)<1e-9);
    }
    {
        NavigationInputs input(config());Navigator nav(config());establish(input,nav);
        input.enqueue(sns(250,3),base+.251);input.drain(nav,base+.252);check(input.wait_reason()=="waiting_future_imu_bracket");
        input.drain(nav,base+.51);check(nav.localization_diagnostics().latched&&!input.fault_reason().empty());
    }
    {
        NavigationInputs input(config());Navigator nav(config());establish(input,nav);
        auto duplicate=imu(100,11,.1,205);input.update_imu(duplicate,base+.308);input.drain(nav,base+.308);
        check(input.fault_reason().empty()); // latest actual source age .210 < .25
        duplicate=imu(100,11,.1,305);input.update_imu(duplicate,base+.408);input.drain(nav,base+.408);
        check(input.fault_reason()=="imu_source_stale"&&nav.localization_diagnostics().latched);
    }
    {
        NavigationInputs input(config());Navigator nav(config());establish(input,nav);
        for(unsigned i=0;i<257;++i)input.enqueue(sns(101+i,3+i),base+.11);
        input.drain(nav,base+.12);check(input.fault_reason()=="sensor_alignment_queue_overflow"&&nav.localization_diagnostics().latched);
    }
    {
        NavigationInputs input(config());Navigator nav(config());establish(input,nav);
        auto bad=sns(100,3);bad.illegal=1;input.enqueue(bad,base+.11);input.drain(nav,base+.12);
        check(nav.localization_diagnostics().latched&&nav.localization_diagnostics().first_reason=="invalid_odometry_frame");
    }
    {
        NavigationInputs input(config());Navigator nav(config());input.update_imu(imu(100,11,.1),base+.108);
        input.enqueue(sns(50,1),base+.051);input.enqueue(sns(100,2),base+.101);input.drain(nav,base+.108);
        check(input.fault_reason().empty()&&nav.frame()&&nav.frame()->sequence==2);
        input.update_imu(imu(200,21,.2),base+.208);input.enqueue(sns(150,3),base+.151);input.drain(nav,base+.208);check(nav.localization_valid(base+.208));
        input.enqueue(sns(50,4),base+.21);input.drain(nav,base+.211);check(input.fault_reason()=="imu_history_missing"&&nav.localization_diagnostics().latched);
    }
    {
        NavigationInputs input(config());Navigator nav(config());input.update_imu(imu(0,1,0),base+.008);
        input.enqueue(sns(0,1),base+.001);input.drain(nav,base+.008);check(nav.frame().has_value());
        auto invalid=imu(50,6,0);invalid.imu_calibrating=true;input.update_imu(invalid,base+.058);
        input.update_imu(imu(100,11,1.),base+.108);input.update_imu(imu(150,16,1.05),base+.158);
        input.enqueue(sns(100,2),base+.101);input.enqueue(sns(150,3),base+.151);input.drain(nav,base+.158);
        check(input.fault_reason().empty()&&!nav.localization_diagnostics().latched);check(nav.localization_valid(base+.158));
        check(std::abs(nav.pose().yaw-.05)<1e-9);
    }
    {
        // Two full turns plus: wrapped source clocks and yaw, event SNS half
        // way between asynchronous HTTP source snapshots, and real wheel counts.
        NavigationInputs input(config());Navigator nav(config());const uint32_t origin=0xfffffff0u;uint32_t packet=0;
        for(unsigned i=0;i<=140;++i){
            uint32_t elapsed=i*50;auto t=imu(origin+elapsed,0xfffffff0u+i*5,0.);
            t.yaw_deg=std::atan2(std::sin(2.96+i*.1),std::cos(2.96+i*.1))*180/vehicle_profile::pi;
            t.request_started_ms=uint64_t(base*1000)+elapsed+5;t.received_ms=t.request_started_ms+3;
            input.update_imu(t,base+elapsed/1000.+.008);
            for(unsigned e:{i?elapsed-25:0u,elapsed}){
                if(i==0&&packet)continue;
                auto s=sns(origin+e,++packet);double scale=config().wheel_circumference_m/config().encoder_ticks_per_turn;
                for(unsigned k=0;k<4;++k){double velocity=(config().left_wheel_mask&(1U<<k))?-config().track_m:config().track_m;
                    s.counts[k]=int32_t(std::llround(velocity*(e/1000.)/scale*config().encoder_signs[k]));}
                input.enqueue(s,base+e/1000.+.001);
            }
            input.drain(nav,base+elapsed/1000.+.008);check(input.fault_reason().empty());
            if(i)check(nav.localization_valid(base+elapsed/1000.+.008));
        }
        check(std::abs(std::atan2(std::sin(nav.pose().yaw-14.),std::cos(nav.pose().yaw-14.)))<1e-8);
    }
    for(uint16_t raw:{uint16_t(4501),uint16_t(4559),uint16_t(4600),uint16_t(4799),uint16_t(5936),uint16_t(6001),uint16_t(32000),uint16_t(65534)}){
        NavigationInputs input(config());Navigator nav(config());establish(input,nav);
        auto s=sns(150,3);s.distance_mm[0]=raw;s.age_ms[0]=17;
        input.update_imu(imu(150,16,.15),base+.158);input.enqueue(s,base+.151);input.drain(nav,base+.158);
        check(nav.frame()->sonar_m[0]&&*nav.frame()->sonar_m[0]==4.5);
        check(nav.frame()->sonar_age_s[0]==.017&&nav.frame()->sonar_observation_id[0]==133);
        check(nav.frame()->timestamp_s==base+.151&&nav.start_problem(base+.158).empty());
        check(s.distance_mm[0]==raw&&s.age_ms[0]==17); // input contract is immutable
        check(nav.start_problem(base+.41)=="telemetry_stale");
    }
    for(unsigned invalid=0;invalid<5;++invalid){
        NavigationInputs input(config());Navigator nav(config());establish(input,nav);
        auto s=sns(150,3);s.distance_mm[0]=4799;
        if(invalid==0)s.distance_mm[0]=0;
        if(invalid==1)s.distance_mm[0]=UINT16_MAX;
        if(invalid==2){s.valid=6;s.fault=1;}
        if(invalid==3)s.age_ms[0]=900;
        if(invalid==4)s.age_ms[0]=UINT16_MAX;
        input.update_imu(imu(150,16,.15),base+.158);input.enqueue(s,base+.151);input.drain(nav,base+.158);
        check(!nav.motion_gate_diagnostics(base+.158).source_usable);
        if(invalid<3)check(!nav.frame()->sonar_m[0]);
        else check(nav.frame()->sonar_age_s[0]>=.9);
        check(nav.start_straight(base+.158));
        const auto stopped=nav.tick(base+.158);
        check(stopped.linear_mps==0&&stopped.angular_rps==0&&nav.state()=="SENSOR_WAIT");
    }
    std::cout<<"PASS "<<checks<<" sensor bridge checks: bracket queue, physical source dt, conservative age, brief HTTP failure, explicit faults, bounded queue, calibration startup, immutable yaw snapshots and reset\n";
}
