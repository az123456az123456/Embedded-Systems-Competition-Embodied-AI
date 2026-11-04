// Replay actual SNS1 wire packets through the production parser, time bridge,
// localization core, and production Shared reset barriers. No sockets or motors.
#define main navigation_production_main_not_executed
#include "../main.cc"
#undef main
#include <cstdlib>
#include <limits>

namespace {
unsigned checks=0;
void check(bool ok){++checks;if(!ok){std::cerr<<"Replay assertion "<<checks<<" failed\n";std::abort();}}
constexpr double pi=3.14159265358979323846;
const uint8_t host[6]={2,4,6,8,10,12};
robotnav::GatewayTelemetry imu(uint32_t ms,uint32_t seq,double yaw,double host_s){
    robotnav::GatewayTelemetry t;
    t.available=t.imu_valid=t.imu_fresh=t.imu_bias_valid=t.imu_timestamp_valid=true;
    t.imu_calibrating=t.imu_action_pending=false;
    t.imu_timestamp_ms=ms;t.imu_sequence=seq;t.yaw_deg=yaw*180/pi;
    t.received_ms=uint64_t(std::llround(host_s*1000));t.request_started_ms=t.received_ms-1;t.imu_sample_age_ms=0;
    return t;
}
std::vector<uint8_t> packet(uint32_t ms,uint32_t seq,const Config& c,double yaw_delta){
    std::vector<uint8_t>b(80);std::copy(host,host+6,b.begin());
    const uint8_t fpga[6]={2,0xac,0x13,0x80,0,1};std::copy(fpga,fpga+6,b.begin()+6);
    b[12]=0x88;b[13]=0xb8;auto*p=b.data()+14;std::memcpy(p,"SNS1",4);p[4]=1;p[5]=7;
    auto put32=[&](unsigned i,uint32_t v){for(unsigned j=0;j<4;++j)p[i+j]=uint8_t(v>>(24-j*8));};
    auto put16=[&](unsigned i,unsigned v){p[i]=v>>8;p[i+1]=v;};
    put32(8,seq);put32(12,ms);
    const double elapsed=(ms-1000)/1000., turn=yaw_delta/.05;
    for(unsigned i=0;i<4;++i){
        const bool left=c.left_wheel_mask&(1U<<i);
        const double distance=(.2+(left?-1:1)*turn*c.track_m/2)*elapsed;
        const int32_t ticks=int32_t(std::llround(distance*c.encoder_ticks_per_turn/c.wheel_circumference_m))*c.encoder_signs[i];
        put32(16+4*i,uint32_t(ticks));
    }
    for(unsigned i=0;i<3;++i){put16(48+2*i,2000);put16(54+2*i,0);}
    // Event snapshots have zero window/delta but carry current absolute counts.
    put16(60,ms%50?0:50);const auto crc=navwire::crc16(p,64);b[78]=crc>>8;b[79]=crc;
    return b;
}
void enqueue(robot_nav::NavigationInputs& bridge,const std::vector<uint8_t>& b,double host_s){
    navwire::Sensors s;check(navwire::sensorFrame(b.data(),b.size(),host,s));bridge.enqueue(s,host_s);
}
}
int main(){
    Config c;applyFixedProfile(c);c.configured=true;c.drive_mps_per_full_pwm=.6;c.left_wheel_mask=12;
    for(double turn:{2*pi/180,.10}){
        Navigator nav(c);robot_nav::NavigationInputs bridge(c);
        bridge.update_imu(imu(1000,1,179*pi/180,101.000),101.000);
        bridge.update_imu(imu(1050,2,robot_nav::wrap(179*pi/180+turn),101.050),101.050);
        enqueue(bridge,packet(1000,1,c,turn),101.0500);
        enqueue(bridge,packet(1001,2,c,turn),101.0501);
        enqueue(bridge,packet(1002,3,c,turn),101.0502);
        enqueue(bridge,packet(1050,4,c,turn),101.0510);
        bridge.drain(nav,101.052);
        check(bridge.fault_reason().empty());check(nav.localization_valid(101.052));
        check(!nav.localization_diagnostics().latched);check(!nav.enabled());
        check(std::abs(nav.pose().yaw-turn)<1e-8);
        check(nav.frame()&&std::abs(*nav.frame()->device_timestamp_s-1.05)<1e-9);
        check(nav.frame()->sequence==4);
        // A brief HTTP timeout waits without inventing an invalid sensor sample
        // or re-arming navigation when the source subsequently recovers.
        robotnav::GatewayTelemetry missing;bridge.update_imu(missing,101.080);bridge.drain(nav,101.080);
        check(bridge.fault_reason().empty());check(nav.localization_valid(101.080));check(!nav.enabled());
        bridge.update_imu(imu(1100,3,robot_nav::wrap(179*pi/180+2*turn),101.100),101.100);
        enqueue(bridge,packet(1100,5,c,turn),101.101);bridge.drain(nav,101.102);
        check(bridge.fault_reason().empty());check(nav.localization_valid(101.102));check(!nav.enabled());
        // Real source loss remains latched across transport recovery and stop.
        bridge.update_imu(missing,101.400);bridge.drain(nav,101.400);
        check(bridge.fault_reason()=="imu_source_stale");check(nav.localization_diagnostics().latched);
        bridge.update_imu(imu(1400,4,0,101.400),101.400);bridge.drain(nav,101.401);nav.stop();
        check(nav.localization_problem(101.401)=="imu_source_stale");check(!nav.enabled());
        auto bad=packet(1100,5,c,turn);bad[35]^=1;navwire::Sensors rejected;
        check(!navwire::sensorFrame(bad.data(),bad.size(),host,rejected));
    }
    // Exercise the actual methods called by main while its Shared mutex is held;
    // old HTTP requests and pre-reset kernel timestamps cannot seed a new origin.
    Shared shared(c,true);robotnav::GatewayTelemetry ack;
    ack.available=ack.link_ok=true;ack.motor_ack_age_ms=10;ack.left_mask=12;
    check(shared.synchronize_wheel_layout(ack));
    auto barrier=shared.input_request_barrier_ms;
    check(!shared.input_request_allowed(barrier-1));check(shared.input_request_allowed(barrier));
    check(!shared.input_sensor_allowed(double(barrier-1)/1000.));
    check(shared.input_sensor_allowed(double(barrier+1)/1000.));
    check(!shared.input_sensor_allowed(std::numeric_limits<double>::quiet_NaN()));
    check(!shared.input_sensor_allowed(std::numeric_limits<double>::infinity()));
    check(!shared.input_sensor_allowed(-std::numeric_limits<double>::infinity()));
    std::string error;const auto epoch=shared.origin_epoch;
    check(shared.command("reset-map",0,0,error));check(shared.origin_epoch==epoch+1);
    check(shared.input_request_barrier_ms>=barrier);check(!shared.nav.enabled());check(!shared.nav.frame());
    check(shared.inputs.wait_reason()=="waiting_imu_source");check(shared.inputs.fault_reason().empty());
    check(!shared.command("start",0,0,error)&&error=="waiting_imu_source");
    ack.left_mask=10;check(shared.synchronize_wheel_layout(ack));
    check(shared.reset_required&&!shared.nav.enabled()&&!shared.nav.frame());
    check(shared.inputs.wait_reason()=="waiting_imu_source");
    std::cout<<"PASS production SNS1 replay + reset barriers: "<<checks<<" checks\n";
}
