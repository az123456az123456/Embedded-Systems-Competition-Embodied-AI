#include "../navcore.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace robot_nav;
namespace {
unsigned checks=0;void check(bool v,const char*s){++checks;if(!v)throw std::runtime_error(s);}
Config config(){Config c;c.configured=true;c.drive_mps_per_full_pwm=.6;c.left_wheel_mask=12;c.sonar_latency_s=0;return c;}
SensorFrame sample(const Config& c,double t,const std::array<double,4>& distance){
    SensorFrame f;f.timestamp_s=t;f.device_timestamp_s=t;f.imu_valid=true;f.sonar_m={2.,2.,2.};
    for(unsigned i=0;i<4;++i)f.counts[i]=uint32_t(int64_t(std::llround(distance[i]*c.encoder_ticks_per_turn/c.wheel_circumference_m))*c.encoder_signs[i]);
    return f;
}
void stationary_edges(){
    auto c=config();Navigator n(c);std::array<double,4>d{};n.ingest(sample(c,1,d));
    const double edge=c.wheel_circumference_m/c.encoder_ticks_per_turn;
    for(unsigned i=1;i<=2500;++i){d[2]=(i%2?10:-10)*edge;auto f=sample(c,1+i*.001,d);check(n.ingest(f),"1ms stationary edge noise is valid odometry");}
    check(!n.localization_diagnostics().latched,"low-net-count stationary noise cannot become relative-speed fault");
}
void physical_floor(){
    auto c=config();const double scale=c.wheel_circumference_m/c.encoder_ticks_per_turn;
    const int below=int(std::floor(c.encoder_min_compare_mps*c.encoder_compare_window_s/scale));
    for(int ticks:{below,below+1}){
        Navigator n(c);std::array<double,4>d{};n.ingest(sample(c,1,d));
        for(unsigned i=1;i<=25;++i){d[2]=i*ticks/5.*scale;n.ingest(sample(c,1+i*.05,d));}
        const bool expected=ticks>below;
        check(n.localization_diagnostics().latched==expected,"physical .01m/s floor uses actual 250ms speed, not per-packet ratios");
        if(expected){const auto& e=n.localization_diagnostics();check(e.first_reason=="encoder_wheel_disagreement","above-floor stuck partner remains protected");
            check(e.wheel_comparison_window_s>=.25&&e.window_wheel_mps[2]>=c.encoder_min_compare_mps,"fault records aggregate measured speed/window");}
    }
}
void actual_failures_and_first_reason(){
    auto c=config();for(bool reverse:{false,true}){
        Navigator n(c);std::array<double,4>d{};n.ingest(sample(c,1,d));
        for(unsigned i=1;i<=20;++i){const double t=i*.05;d={t*.05,t*.05,t*.05,reverse?-t*.05:0};n.ingest(sample(c,1+t,d));}
        const auto first=n.localization_diagnostics();check(first.latched&&first.first_reason=="encoder_wheel_disagreement","real stuck/reversed same-side wheel still faults");
        check(first.wheel_comparison_window_s>=.25,"fault includes a full comparison window");
        n.stop();n.invalidate_localization("later_fault",2.01);check(n.localization_diagnostics().first_reason==first.first_reason,"later stop/fault cannot swallow first cause");
        check(!n.enabled(),"fault never automatically rearms");
    }
}
void legitimate_differential_and_event_bursts(){
    auto c=config();Navigator n(c);std::array<double,4>d{};double t=1;n.ingest(sample(c,t,d));
    for(unsigned i=0;i<100;++i)for(double dt:{.001,.049}){
        t+=dt;for(unsigned w=0;w<4;++w)d[w]+=(c.left_wheel_mask&(1U<<w)?.03:.09)*dt;
        check(n.ingest(sample(c,t,d)),"true differential speeds with 1ms event snapshots remain valid");
    }
    check(!n.localization_diagnostics().latched,"compare within acknowledged side, never between left/right turns");
}
}
int main(){stationary_edges();physical_floor();actual_failures_and_first_reason();legitimate_differential_and_event_bursts();std::cout<<"PASS encoder fixed-window "<<checks<<" checks\n";}
