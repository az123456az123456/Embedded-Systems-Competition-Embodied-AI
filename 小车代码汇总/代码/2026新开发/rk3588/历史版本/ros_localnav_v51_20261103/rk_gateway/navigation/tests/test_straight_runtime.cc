// Exercise actual Shared command/status wiring; no sockets or physical I/O.
#define main navigation_main_not_executed
#include "../main.cc"
#undef main

namespace {
unsigned checks=0;
void check(bool value,const char* why){++checks;if(!value)throw std::runtime_error(why);}
Config configured(){Config c;applyFixedProfile(c);c.configured=true;c.drive_mps_per_full_pwm=.5;return c;}
void feed(Shared& s,double t,uint8_t valid=7,double front=2.){
    auto& raw=s.sensors;raw.sequence++;raw.uptime_ms+=20;raw.valid=valid;raw.fault=7U^valid;
    raw.age_ms={0,0,0};raw.distance_mm={uint16_t(front*1000),2000,2000};raw.window_ms=50;
    s.have_sensors=true;s.received_s=t;
    robot_nav::SensorFrame f;f.sequence=raw.sequence;f.timestamp_s=t;f.device_timestamp_s=t;
    f.imu_valid=true;f.yaw_rad=.4;
    for(unsigned i=0;i<3;++i)if(valid&(1U<<i))f.sonar_m[i]=raw.distance_mm[i]/1000.;
    const bool first=!s.nav.frame();
    const bool accepted=s.nav.ingest(f,t);
    check(accepted||(first&&s.nav.frame()&&s.nav.frame()->sequence==f.sequence&&
                    !s.nav.localization_diagnostics().latched),
          "production first-frame baseline or established odometry accepted");
}
void prime(Shared& s){s.have_wheel_layout=true;const auto t=navwire::monotonicSeconds();feed(s,t-.02);feed(s,t);}
}
int main(){try{
    std::string error;
    {
        auto c=configured();c.configured=false;Shared s(c,true);prime(s);
        check(!s.command("straight",0,0,error),"uncalibrated start rejected");
        check(!s.nav.enabled(),"uncalibrated rejection cannot enable motion");
    }
    {
        Shared s(configured(),false);prime(s);
        check(!s.command("straight",0,0,error)&&error=="observe_only","observe-only runtime rejects motion");
    }
    {
        Shared s(configured(),true);
        check(!s.command("straight",0,0,error)&&error=="verified_wheel_layout_required","verified ACK layout required");
        s.have_wheel_layout=true;
        check(!s.command("straight",0,0,error),"source data required");
        prime(s);s.reset_required=true;
        check(!s.command("straight",0,0,error),"reset barrier retained");
    }
    {
        Shared s(configured(),true);prime(s);
        const auto observed=s.nav.grid().observed_s();const auto values=s.nav.grid().values();
        check(s.command("straight",0,0,error),"actual Shared accepts straight intent");
        check(s.nav.enabled()&&s.nav.straight_assist_active(),"distinct straight mode is enabled");
        check(s.nav.grid().observed_s()==observed&&s.nav.grid().values()==values,"button never paints free map evidence");
        picojson::value status;check(picojson::parse(status,s.status()).empty(),"actual runtime status JSON");
        check(status.get("straight_assist_active").get<bool>(),"status exposes actual straight activity");
        check(status.get("start_mode").get<std::string>()=="straight","status distinguishes straight from exploration");
        check(status.get("straight_requested_speed_mps").get<double>()==.30,"status publishes requested 0.3m/s");
        check(status.get("straight_available_speed_mps").get<double>()==.30,"calibrated 0.5m/s-per-duty supports 0.3 with 0.6 ceiling");
        double t=navwire::monotonicSeconds();bool advanced=false;
        for(unsigned i=0;i<60;++i){t+=.02;feed(s,t);const auto c=s.nav.tick(t);
            check(c.linear_mps<=.300001&&c.linear_mps>=0,"straight test speed bounded");
            check(s.nav.enabled(),"healthy straight observation does not disarm");advanced=advanced||c.linear_mps>0;
        }
        check(advanced,"sensor-gated straight can advance before whole map exists");
        t+=.02;feed(s,t,7,.20);const auto blocked=s.nav.tick(t);
        check(blocked.left==0&&blocked.right==0,"near obstacle first commands zero");
        check(s.nav.enabled(),"obstacle stop retains explicit assisted intent");
        t+=.02;feed(s,t,6,.20);const auto fault=s.nav.tick(t);
        check(fault.left==0&&fault.right==0&&s.nav.enabled()&&s.nav.state()=="SENSOR_WAIT","transient invalid front sonar keeps zero authorized wait");
        check(s.nav.straight_assist_active(),"transient range wait preserves assisted intent");
        for(unsigned i=0;i<260&&s.nav.enabled();++i){t+=.02;feed(s,t,6,.20);const auto waiting=s.nav.tick(t);check(waiting.linear_mps==0&&waiting.angular_rps==0,"persistent source wait remains exactly zero");}
        check(s.nav.enabled()&&s.nav.straight_assist_active()&&s.nav.state()=="SENSOR_WAIT","continuous source failure retains stopped active indicator");
    }
    {
        Shared s(configured(),true);prime(s);check(s.command("straight",0,0,error),"start for cancellation");
        s.cancel("manual_takeover");check(!s.nav.enabled()&&!s.nav.straight_assist_active(),"manual/STOP cancellation clears mode");
        check(s.nav.command().left==0&&s.nav.command().right==0,"cancellation clears output");
    }
    {
        Shared s(configured(),true);prime(s);
        // Deliberately preserve the existing sensor frame through the first
        // control tick: no post-start sonar observation or cleared map is fed.
        std::vector<int8_t> occupied(4096,1);
        check(s.nav.grid().import_observations(occupied,s.nav.grid().origin(),navwire::monotonicSeconds()),
              "historical occupied map fixture for actual Shared direct start");
        const auto before=s.nav.grid().values();
        const auto start_lower=navwire::monotonicSeconds();
        check(s.command("straight",0,0,error),"actual Shared direct start accepts fresh existing measurements");
        check(s.nav.command().left==0&&s.nav.command().right==0,"intent alone does not bypass zero control handoff");
        const auto tick_at=navwire::monotonicSeconds()+.02;
        const auto first=s.nav.tick(tick_at);
        check(first.linear_mps>0&&first.left>0&&first.right>0,
              "first actual control tick advances without after-start observations or historical map clearance");
        check(first.linear_mps<=s.nav.config().accel_mps2*(tick_at-start_lower)+1e-9,
              "first control tick still follows acceleration ramp over actual elapsed time");
        check(s.nav.grid().values()==before,"direct start never erases occupied map evidence");
        s.cancel("manual_takeover");
        check(!s.nav.enabled()&&s.nav.command().left==0&&s.nav.command().right==0,
              "manual cancellation still stops immediate-start mode");
    }
    std::printf("PASS %u actual Shared straight start/status/strict-source/cancellation checks; no hardware\n",checks);
    return 0;
}catch(const std::exception& e){std::fprintf(stderr,"FAIL straight runtime: %s\n",e.what());return 1;}}
