// Public production Navigator replay. No replacement mismatch implementation.
// --expect-legacy-leak is only for proving the frozen v27.1 counterexample.
#ifndef ENCODER_CORE_HEADER
#define ENCODER_CORE_HEADER "../navcore.h"
#endif
#include ENCODER_CORE_HEADER
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
using namespace robot_nav;
namespace {
unsigned checks=0;
void ck(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
Config configured(){Config c;c.configured=true;c.drive_mps_per_full_pwm=.5;return c;}
struct Replay {
    Config config=configured();Navigator nav{config};double now=0,yaw=0;
    std::array<double,4> counts{};uint32_t sequence=0;
    Replay(){sample();for(unsigned i=0;i<20;++i)step({0,0,0,0});}
    bool sample(){
        SensorFrame frame;frame.timestamp_s=now;frame.device_timestamp_s=now;
        frame.sequence=++sequence;frame.imu_valid=true;frame.yaw_rad=wrap(yaw);frame.sonar_m={4.5,4.5,4.5};
        for(unsigned i=0;i<4;++i)frame.counts[i]=uint32_t(int64_t(std::llround(counts[i])));
        return nav.ingest(frame,now);
    }
    bool step(const std::array<double,4>& physical_mps,double dt=.05,double yaw_rps=0){
        const double ticks_per_m=config.encoder_ticks_per_turn/config.wheel_circumference_m;
        for(unsigned i=0;i<4;++i)counts[i]+=physical_mps[i]*dt*ticks_per_m*config.encoder_signs[i];
        yaw=wrap(yaw+yaw_rps*dt);now+=dt;return sample();
    }
    bool window(const std::array<double,4>& rates,double yaw_rps=0){
        for(unsigned i=0;i<5;++i)if(!step(rates,.05,yaw_rps))return false;
        return true;
    }
    void toggle(){nav.stop("test_manual_stop");ck(nav.command().left==0&&nav.command().right==0,"explicit stop stays zero");
        ck(nav.start_straight(now),"unlatched source can accept a new straight intent");}
};
const std::array<double,4> good{{.12,.12,.12,.12}};
const std::array<double,4> right_rear_stopped{{.12,.12,0,.12}};
void interrupted_bad_windows(bool expect_legacy){
    Replay replay;replay.toggle();
    // No contiguous abnormal run exceeds .50s, below the .60s confirmation
    // interval. A genuinely matching .25s window separates the two episodes.
    // Legacy leaky scores were .25,.50,.25,.50,.75 and latched on window5.
    const bool bad[]={true,true,false,true,true};bool healthy=true;
    unsigned stopped_at=0;
    for(unsigned window=0;window<5;++window){
        healthy=replay.window(bad[window]?right_rear_stopped:good);
        std::cout<<"encoder_episode window="<<window+1<<" bad="<<bad[window]
                 <<" t_s="<<replay.now<<" accepted="<<healthy<<" reason="<<replay.nav.reason()<<'\n';
        if(!healthy){stopped_at=window+1;break;}
        replay.toggle();
    }
    if(expect_legacy){
        ck(!healthy&&stopped_at==5,"frozen legacy core reproduces accumulated non-contiguous bad-window fault");
        ck(replay.nav.localization_diagnostics().first_reason=="encoder_wheel_disagreement","counterexample specifically reaches mismatch latch");
    }else{
        ck(healthy&&replay.nav.localization_valid(replay.now),"matching window ends the prior unconfirmed mismatch streak");
        ck(!replay.nav.localization_diagnostics().latched,"interrupted short episodes do not latch localization");
        for(unsigned i=0;i<5;++i)ck(replay.window(good),"subsequent healthy physical rates remain accepted");
    }
}
void persistent_faults_and_real_report(){
    const std::array<double,4> user_rates{{-.40392159953211504,-.00024393856036161954,
                                        -.000027104284484624395,-.10738717512808185}};
    for(unsigned kind=0;kind<3;++kind){
        Replay replay;replay.toggle();auto rates=right_rear_stopped;double yaw_rps=0;
        if(kind==1)rates={.12,.12,-.12,.12};
        if(kind==2){rates=user_rates;yaw_rps=-.07992630589752947/.042;}
        bool detected=false;unsigned windows=0;
        for(;windows<5;++windows){if(!replay.window(rates,yaw_rps)){detected=true;break;}
            replay.toggle(); // Repeated Start/Stop must not hide a real fault.
        }
        ck(detected&&windows==2,"three contiguous .25s bad windows still detect genuine disagreement");
        ck(replay.nav.localization_diagnostics().latched&&replay.nav.localization_diagnostics().first_reason=="encoder_wheel_disagreement","genuine fault remains latched with original cause");
        ck(!replay.nav.enabled()&&replay.nav.command().left==0&&replay.nav.command().right==0,"genuine fault commands zero and disables navigation");
        const auto& diagnostics=replay.nav.localization_diagnostics();
        if(kind==2){
            for(unsigned i=0;i<4;++i)ck(std::abs(diagnostics.window_wheel_mps[i]-user_rates[i])<.00005,"real reported four-rate pattern retained under encoder quantization");
            std::cout<<"real_report_replay detected_t_s="<<replay.now<<" rates=";
            for(double speed:diagnostics.window_wheel_mps)std::cout<<speed<<',';
            std::cout<<" reason="<<diagnostics.first_reason<<'\n';
        }
        replay.nav.stop("operator_stop_after_real_fault");
        ck(!replay.nav.start_straight(replay.now),"Stop/Start cannot silently clear a true localization fault");
        for(unsigned i=0;i<20;++i)replay.step(good);
        ck(replay.nav.localization_diagnostics().latched&&!replay.nav.start_straight(replay.now),"later matching rates cannot auto-unlatch old pose");
    }
}
void healthy_phase_and_no_fake_reset(){
    Replay replay;replay.toggle();
    // Small, alternating same-side speed differences do not cross the ratio
    // threshold. Repeated explicit starts cannot change the measured pose or
    // discard source encoder reference/counts.
    for(unsigned i=0;i<80;++i){const auto pose=replay.nav.pose();const auto source=*replay.nav.frame();
        replay.toggle();ck(replay.nav.pose().x==pose.x&&replay.nav.pose().y==pose.y&&replay.nav.pose().yaw==pose.yaw,"intent toggles do not silently reset odometry");
        ck(replay.nav.frame()->counts==source.counts,"intent toggles retain actual source count reference");
        ck(replay.window(i%2?std::array<double,4>{.12,.12,.11,.12}:std::array<double,4>{.11,.12,.12,.12}),"normal small front/rear differences remain healthy");
    }
    ck(!replay.nav.localization_diagnostics().latched,"normal speed variation never latches");
}
}
int main(int argc,char**argv){try{
    bool legacy=false;if(argc==2&&std::string(argv[1])=="--expect-legacy-leak")legacy=true;
    else if(argc!=1)throw std::runtime_error("usage: test-encoder-lifecycle [--expect-legacy-leak]");
    interrupted_bad_windows(legacy);persistent_faults_and_real_report();healthy_phase_and_no_fake_reset();
    std::cout<<"PASS encoder lifecycle "<<checks<<" checks mode="<<(legacy?"legacy-counterexample-confirmed":"contiguous-streak")<<'\n';
}catch(const std::exception& error){std::cerr<<"FAIL encoder lifecycle: "<<error.what()<<" checks="<<checks<<'\n';return 1;}}
