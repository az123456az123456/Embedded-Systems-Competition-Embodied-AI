// Host integration simulation: actual Shared/Navigator/Gateway/Controller,
// source-clock IMU alignment, actual SNS1 + MOT4/ACK4 codecs and loopback HTTP.
// Only the physical chassis, FPGA wire endpoint and phone heartbeat are models.
// This never opens AF_PACKET or sends a command to a physical device.
#ifndef NAVIGATION_MAIN_PATH
#define NAVIGATION_MAIN_PATH "../main.cc"
#endif
#define main navigation_main_not_run_in_simulation
#include NAVIGATION_MAIN_PATH
#undef main
#include "../../dashboard/phone/controller.h"
#include "../../dashboard/phone/motor_link.h"
#include "../vendor/http_server.h"
#include <arpa/inet.h>
#include <chrono>
#include <cstdlib>
#include <future>

namespace {
using namespace std::chrono;
std::atomic<unsigned> checks{0};
uint64_t ms(){return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();}
void check(bool good,const std::string& label){++checks;if(!good)throw std::runtime_error(label);}
unsigned port(){int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"socket");sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);check(bind(fd,(sockaddr*)&a,sizeof a)==0,"bind");socklen_t n=sizeof a;check(getsockname(fd,(sockaddr*)&a,&n)==0,"getsockname");close(fd);return ntohs(a.sin_port);}
struct Reply {int status=0;picojson::value json;};
Reply http(unsigned p,const std::string& path,const std::string& body="",bool get=false){
    int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"HTTP socket");timeval t{2,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&t,sizeof t);
    sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(p);
    check(connect(fd,(sockaddr*)&a,sizeof a)==0,"HTTP connect");
    const auto request=std::string(get?"GET ":"POST ")+path+" HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer 012345\r\nContent-Length: "+std::to_string(body.size())+"\r\nConnection: close\r\n\r\n"+body;
    check(robotphone::writeAll(fd,request.data(),request.size()),"HTTP write");
    std::string data;char b[8192];ssize_t n;while((n=recv(fd,b,sizeof b,0))>0)data.append(b,n);close(fd);
    const auto end=data.find("\r\n\r\n");check(end!=std::string::npos,"HTTP header");Reply r;std::istringstream h(data);std::string protocol;h>>protocol>>r.status;
    check(picojson::parse(r.json,data.substr(end+4)).empty(),"HTTP JSON");return r;
}
void put16(uint8_t* p,uint16_t x){p[0]=x>>8;p[1]=x;}
void put32(uint8_t* p,uint32_t x){for(int i=0;i<4;++i)p[i]=x>>(24-i*8);}
uint16_t get16(const uint8_t*p){return uint16_t(p[0])<<8|p[1];}
int signed16(const uint8_t*p){const auto n=get16(p);return n<32768?n:int(n)-65536;}
uint32_t get32(const uint8_t*p){return uint32_t(p[0])<<24|uint32_t(p[1])<<16|uint32_t(p[2])<<8|p[3];}
constexpr uint8_t host_mac[6]={2,0,0,0,0,7},fpga_mac[6]={2,0xac,0x13,0x80,0,1};

// Backend request semantics intentionally match phone_service.cc, but all motor
// state and readiness decisions execute the unmodified production Controller.
struct Backend {
    robotphone::HttpServer server;
    mutable std::mutex mutex;
    robotphone::Controller controller;
    robotphone::MotorAckWindow acknowledgements;
    robotphone::SoftcoreMonitor softcore;
    uint32_t motor_sequence=0,source_sequence=0,source_ms=0;
    uint64_t sequence=0,last_ack_ms=0;
    std::string navigation_ticket;
    double left_pwm=0,right_pwm=0,yaw=0;
    std::array<double,4> counts{};
    bool ack_enabled=true;
    bool core_ready=true;unsigned core_flags=3;
    std::atomic<unsigned> delay_twist_ms{0},twist_entered{0};
    unsigned last_protocol=1;
    unsigned valid_mask=7;
    std::array<double,3> ranges{2,2,2};
    unsigned control_rejections=0;
    static robotphone::Config config(){robotphone::Config c;c.left_motor_mask=10;return c;}
    Backend():controller(config()){softcore.socketState(true);}
    std::string state(bool okay=true) {
        const auto& s=controller.state();const auto now=ms();
        std::ostringstream o;o<<std::setprecision(12)<<"{\"ok\":"<<(okay?"true":"false")<<",\"session\":\"session\",\"armed\":"<<(s.armed?"true":"false")
          <<",\"reason\":"<<quoted(s.reason)<<",\"imu_action_pending\":false,\"motor\":{\"link_ok\":"<<(acknowledgements.fresh(now)?"true":"false")
          <<",\"left_mask\":10,\"invert_mask\":15,\"left\":"<<s.left<<",\"right\":"<<s.right<<",\"ack_left\":"<<left_pwm<<",\"ack_right\":"<<right_pwm
          <<",\"velocity_protocol_supported\":true,\"softcore_ready\":"<<(core_ready?"true":"false")<<",\"softcore_reason\":"<<quoted(softcore.reason(now,core_ready))<<",\"softcore_diagnostics\":"<<softcore.json(now,core_ready)<<",\"ack_protocol\":"<<last_protocol<<",\"ack_age_ms\":"<<(last_ack_ms?now-last_ack_ms:9999)<<"},\"imu\":{\"valid\":true,\"fresh\":true,\"bias_valid\":true,\"calibrating\":false,\"yaw\":"<<yaw*180/PI
          <<",\"roll\":0,\"pitch\":0,\"timestamp_ms\":"<<source_ms<<",\"sequence\":"<<source_sequence<<",\"sample_age_ms\":0}}";return o.str();
    }
    void start(unsigned p){std::string error;check(server.start("127.0.0.1",p,[&](int fd,const robotphone::HttpRequest&r){
        if(r.path=="/api/navigation/twist"){++twist_entered;std::this_thread::sleep_for(milliseconds(delay_twist_ms.load()));}
        std::lock_guard<std::mutex> g(mutex);
        if(r.path=="/api/connect"||r.path=="/api/state"){robotphone::httpReply(fd,200,state());return;}
        picojson::value v;check(picojson::parse(v,r.body).empty(),"backend parse");
        const auto op=v.get("op").get<std::string>();const auto seq=uint64_t(v.get("seq").get<double>());const auto now=ms();bool okay=true;
        const bool nav=r.path!="/api/control";
        if(seq<=sequence&&op!="stop"&&op!="disarm")okay=false;else{
            sequence=std::max(sequence,seq);if(!nav)navigation_ticket.clear();
            if(r.path=="/api/navigation/claim"&&op=="claim"){
                okay=controller.state().armed&&controller.state().mode==robotphone::Mode::Idle;
                if(okay){okay=controller.driveVelocity(seq,0,0,now);if(okay)navigation_ticket="loopback-private-ticket";}
            }else if(r.path=="/api/navigation/twist"&&op=="twist"){
                okay=!navigation_ticket.empty()&&v.get("navigation_ticket").is<std::string>()&&v.get("navigation_ticket").get<std::string>()==navigation_ticket;
                if(okay)okay=controller.driveVelocity(seq,v.get("linear_mps").get<double>(),v.get("angular_rps").get<double>(),now);
            }else if(op=="arm")okay=controller.arm(seq,now);
            else if(op=="drive")okay=controller.drive(seq,v.get("throttle").get<double>(),v.get("steer").get<double>(),false,now);
            else if(op=="idle"){
                const auto mode=controller.state().mode;
                if(mode!=robotphone::Mode::Disarmed&&mode!=robotphone::Mode::Idle)okay=controller.drive(seq,0,0,false,now);
            }else if(op=="stop")controller.stop(now);
            else if(op=="disarm")controller.disarm(now);
            else okay=false;
        }
        if(!okay){++control_rejections;if(nav||op=="drive")controller.disarm(now);}
        auto output=state(okay);if(okay&&op=="claim")output.insert(output.size()-1,",\"navigation_ticket\":\"loopback-private-ticket\"");
        robotphone::httpReply(fd,okay?200:409,output);
    },error),"backend start "+error);}
    void physics(double dt,uint32_t device_ms){
        std::lock_guard<std::mutex> g(mutex);const auto now=ms();source_ms=device_ms;++source_sequence;
        const double vl=left_pwm*.6,vr=right_pwm*.6;
        for(unsigned i=0;i<4;++i)counts[i]+=((10&(1U<<i))?vl:vr)*dt*vehicle_profile::encoder_ticks_per_turn/vehicle_profile::wheel_circumference_m*vehicle_profile::encoder_signs[i];
        yaw+=(vr-vl)/vehicle_profile::track_m*dt;
        controller.updateImu({now,true,0,0,yaw*180/PI});controller.updateArcProtocol(true);controller.updateAngleProtocol(true);
        // Decode the real independent NAV1 wire format. These packets do not
        // enter the MotorAckWindow and cannot renew its command lifetime.
        std::vector<uint8_t> nav(60);std::memset(nav.data(),255,6);std::copy(fpga_mac,fpga_mac+6,nav.begin()+6);nav[12]=0x88;nav[13]=0xb7;
        auto*np=nav.data()+14;std::memcpy(np,"NAV1",4);np[4]=1;np[5]=core_flags;put32(np+8,device_ms/100+1);put32(np+20,source_sequence);put16(np+26,robotphone::motorCrc16(np,26));
        check(softcore.accept(nav.data(),nav.size(),host_mac,now),"real NAV1 decoder");
        controller.updateVelocityProtocol(true,core_ready);controller.updateMotorLink(now,acknowledgements.fresh(now));const auto& state=controller.tick(now);
        // Real production MOT1 encoder -> simulated FPGA -> real ACK1 decoder
        // and replay/freshness window. The plant adopts only the encoded output.
        const auto mot=state.twist_mode?robotphone::velocityMotorFrame(host_mac,++motor_sequence,state.armed,std::lround(state.linear_mps*1000),std::lround(state.angular_rps*1000),10,15):robotphone::motorFrame(host_mac,++motor_sequence,state.armed,std::lround(state.left*1000),std::lround(state.right*1000),10,15);
        last_protocol=state.twist_mode?4:1;acknowledgements.noteSent(motor_sequence,now,10,15,last_protocol);
        if(ack_enabled){
            const auto* payload=mot.data()+14;check(robotphone::motorCrc16(payload,16)==get16(payload+16),"MOT1 CRC");
            if(state.twist_mode){
                const double v=(payload[5]&1)?signed16(payload+10)/1000.:0,w=(payload[5]&1)?signed16(payload+12)/1000.:0;
                left_pwm=(v-w*vehicle_profile::track_m*.5)/.6;right_pwm=(v+w*vehicle_profile::track_m*.5)/.6;
                if(!core_ready)left_pwm=right_pwm=0;
            }else {left_pwm=(payload[5]&1)?signed16(payload+10)/1000.:0;right_pwm=(payload[5]&1)?signed16(payload+12)/1000.:0;}
            std::vector<uint8_t> raw(60);std::copy(host_mac,host_mac+6,raw.begin());std::copy(fpga_mac,fpga_mac+6,raw.begin()+6);raw[12]=0x88;raw[13]=0xb6;
            auto* p=raw.data()+14;std::memcpy(p,last_protocol==4?"ACK4":"ACK1",4);p[4]=last_protocol;p[5]=(payload[5]&1)|4|(last_protocol==4&&core_ready?8:0);put32(p+6,get32(payload+6));put16(p+10,uint16_t(std::lround(left_pwm*1000)));put16(p+12,uint16_t(std::lround(right_pwm*1000)));p[14]=10;p[15]=15;put16(p+16,robotphone::motorCrc16(p,16));
            robotphone::MotorAck ack;check(robotphone::decodeMotorAck(raw.data(),raw.size(),host_mac,ack),"ACK1 decode");check(acknowledgements.accept(ack,now),"ACK1 replay window");last_ack_ms=now;
            controller.updateMotorLink(now,acknowledgements.fresh(now));
        }else if(!acknowledgements.fresh(now)){left_pwm=right_pwm=0;}
    }
    navwire::Sensors sensors(uint32_t seq,uint32_t clock){
        std::lock_guard<std::mutex> g(mutex);std::vector<uint8_t> raw(80);std::copy(host_mac,host_mac+6,raw.begin());std::copy(fpga_mac,fpga_mac+6,raw.begin()+6);raw[12]=0x88;raw[13]=0xb8;
        auto* p=raw.data()+14;std::memcpy(p,"SNS1",4);p[4]=1;p[5]=valid_mask;p[6]=7^valid_mask;put32(p+8,seq);put32(p+12,clock);
        for(unsigned i=0;i<4;++i)put32(p+16+i*4,uint32_t(int64_t(std::llround(counts[i]))));
        for(unsigned i=0;i<3;++i){put16(p+48+2*i,uint16_t(std::lround(ranges[i]*1000)));put16(p+54+2*i,uint16_t(clock%100));}
        put16(p+60,50);put16(raw.data()+78,navwire::crc16(p,64));navwire::Sensors result;
        check(navwire::sensorFrame(raw.data(),raw.size(),host_mac,result),"SNS1 real decoder");return result;
    }
    robotphone::State snapshot(){std::lock_guard<std::mutex> g(mutex);return controller.state();}
    void setRanges(std::array<double,3> next,unsigned valid=7){std::lock_guard<std::mutex>g(mutex);ranges=next;valid_mask=valid;}
};

struct Simulation {
    Backend backend;const unsigned backend_port=port(),phone_port=port();
    const std::string token="/tmp/controller-navigation-simulation-"+std::to_string(getpid())+"-"+std::to_string(phone_port);
    Config config;std::unique_ptr<Shared> shared;std::unique_ptr<robotnav::Gateway> gateway;
    std::atomic<bool> running{true},heartbeat{true};std::thread runtime,mobile;
    std::atomic<unsigned> transitions{0},unsafe_pairs{0},runtime_errors{0};
    std::mutex phone_mutex;uint64_t mobile_seq=0,nav_seq=0;const uint64_t started=ms();
    std::string last_state;std::atomic<bool> started_navigation{false};
    Simulation(){
        applyFixedProfile(config);config.velocity_control=true;config.configured=false;config.drive_mps_per_full_pwm=0;
        shared=std::make_unique<Shared>(config,true);backend.start(backend_port);{std::ofstream f(token);f<<"012345\n";}
        robotnav::GatewayCallbacks callbacks;
        callbacks.navigation_layout=[&](const robotnav::GatewayTelemetry&t){shared->synchronize_wheel_layout(t);};
        callbacks.navigation_command=[&](const std::string&o,double x,double y,std::string&e){return shared->command(o,x,y,e);};
        callbacks.navigation_cancel=[&](const std::string&r){shared->cancel(r);};
        callbacks.navigation_state=[&]{return shared->status();};
        robotnav::GatewayOptions options;options.bind_address="127.0.0.1";options.port=phone_port;options.backend_port=backend_port;options.token_file=token;
        gateway=std::make_unique<robotnav::Gateway>(options,callbacks);std::string error;check(gateway->start(error),"gateway start "+error);
        // Establish first real ACK while Controller is safely disarmed.
        backend.physics(.02,0);backend.physics(.02,20);
        check(http(phone_port,"/api/connect","{}").status==200,"phone connect");
        runtime=std::thread([&]{run();});
        mobile=std::thread([&]{while(running){if(heartbeat){const auto reply=control("idle");if(reply.status!=200)++runtime_errors;}std::this_thread::sleep_for(milliseconds(100));}});
        wait([&]{std::lock_guard<std::mutex>g(shared->mutex);return shared->start_problem(navwire::monotonicSeconds()).empty();},2500,"aligned production inputs ready");
    }
    ~Simulation(){running=false;if(mobile.joinable())mobile.join();if(runtime.joinable())runtime.join();gateway->stop();backend.server.stop();unlink(token.c_str());}
    template<class Predicate>void wait(Predicate pred,unsigned duration,const std::string& label){const auto until=ms()+duration;while(!pred()&&ms()<until)std::this_thread::sleep_for(milliseconds(10));check(pred(),label);}
    Reply control(const std::string& op,const std::string& fields=""){
        std::lock_guard<std::mutex>g(phone_mutex);return http(phone_port,"/api/control","{\"session\":\"session\",\"seq\":"+std::to_string(++mobile_seq)+",\"op\":\""+op+"\""+fields+"}");
    }
    void start(){check(control("arm").status==200,"explicit phone arm");check(control("drive",",\"throttle\":0,\"steer\":0").status==200,"explicit zero intent");
        std::lock_guard<std::mutex>g(phone_mutex);const auto reply=http(phone_port,"/api/navigation","{\"session\":\"session\",\"op\":\"explore\",\"nav_seq\":"+std::to_string(++nav_seq)+",\"intent_seq\":"+std::to_string(mobile_seq)+"}");
        check(reply.status==200,"navigation start accepted: "+reply.json.serialize());started_navigation=true;}
    bool enabled(){std::lock_guard<std::mutex>g(shared->mutex);return shared->nav.enabled();}
    std::string state(){std::lock_guard<std::mutex>g(shared->mutex);return shared->nav.state();}
    void trace(const std::string& event){
        std::string state,reason;bool enabled;double x=0;
        {std::lock_guard<std::mutex>g(shared->mutex);state=shared->nav.state();reason=shared->nav.reason();enabled=shared->nav.enabled();x=shared->nav.pose().x;}
        const auto controller=backend.snapshot();
        std::cout<<"SIM t="<<double(ms()-started)/1000<<" event="<<event<<" core="<<state<<" reason="<<reason<<" core_enabled="<<enabled<<" gateway_active="<<gateway->navigationActive()
          <<" armed="<<controller.armed<<" pwm=("<<controller.left<<","<<controller.right<<") controller_reason="<<controller.reason<<" x_m="<<x<<'\n'<<std::flush;
    }
    void run(){try{
        uint64_t previous=ms(),next_sensor=previous,next_poll=previous,next_trace=previous+1000;uint32_t seq=0;
        while(running){const auto begin=ms();const auto device=uint32_t(begin-started);backend.physics(double(begin-previous)/1000,device);previous=begin;
            auto telemetry=gateway->telemetry();if(begin>=next_poll){telemetry=gateway->pollState();next_poll=ms()+50;}
            const auto now=navwire::monotonicSeconds();std::optional<navwire::Sensors> packet;
            if(begin>=next_sensor){packet=backend.sensors(++seq,device);next_sensor=begin+50;}
            {std::lock_guard<std::mutex> g(shared->mutex);
                if(shared->input_request_allowed(telemetry.request_started_ms))shared->inputs.update_imu(telemetry,now);
                if(packet){shared->sensors=*packet;shared->have_sensors=true;shared->received_s=now;if(shared->have_wheel_layout&&shared->input_sensor_allowed(now))shared->inputs.enqueue(*packet,now);}
                if(shared->have_wheel_layout&&!shared->reset_required)shared->inputs.drain(shared->nav,now);
            }
            bool active=false,core_enabled=false;uint64_t ticket=0;robot_nav::Control command;
            gateway->visitNavigation([&](bool a,uint64_t gen){active=a;ticket=gen;std::lock_guard<std::mutex>g(shared->mutex);command=shared->nav.tick(navwire::monotonicSeconds());
                if(!active&&shared->nav.enabled()){++unsafe_pairs;shared->nav.stop("phone_or_manual_cancel");shared->cancel_plan=true;}
                core_enabled=shared->nav.enabled();});
            if(active){if(!core_enabled)gateway->cancelNavigation(command.reason,ticket);else gateway->applyNav(command.linear_mps,command.angular_rps,ticket);}
            const auto current=state();if(current!=last_state){last_state=current;++transitions;trace("state_change");}
            if(begin>=next_trace){trace("heartbeat_1s");next_trace=begin+1000;}
            std::this_thread::sleep_until(steady_clock::time_point(milliseconds(begin+20)));
        }
    }catch(const std::exception&e){++runtime_errors;std::cerr<<"SIM RUNTIME ERROR "<<e.what()<<'\n';running=false;}} 
};

void wait_armed_zero(){
    std::cout<<"SCENARIO start_obstacle_AVOID_WAIT_10s_zero_keep_armed\n";
    Simulation sim;sim.backend.setRanges({.20,.20,.20});std::this_thread::sleep_for(milliseconds(250));sim.start();
    sim.wait([&]{return sim.state()=="AVOID_WAIT";},2500,"entered local obstacle WAIT");
    for(unsigned i=0;i<100;++i){std::this_thread::sleep_for(milliseconds(100));const auto state=sim.backend.snapshot();check(state.armed,"WAIT must keep real Controller armed");check(state.left==0&&state.right==0,"WAIT has exact zero PWM");check(sim.enabled()&&sim.gateway->navigationActive(),"WAIT retains real core and gateway");}
    sim.trace("wait_10s_pass");check(sim.unsafe_pairs==0,"no inactive/enabled core race");check(sim.runtime_errors==0,"HTTP runtime no rejection");
    check(sim.control("stop").status==200,"explicit STOP accepted");sim.wait([&]{return !sim.enabled()&&!sim.gateway->navigationActive()&&!sim.backend.snapshot().armed;},600,"STOP disarms all layers");sim.trace("explicit_stop_terminated");
}
void active_invalid_recovery_requires_new_start(){
    std::cout<<"SCENARIO transient_sonar_zero_session_then_persistent_timeout\n";
    Simulation sim;sim.start();sim.wait([&]{const auto s=sim.backend.snapshot();return s.twist_mode&&s.linear_mps>0;},2500,"real chassis advances through Controller");
    sim.trace("actual_encoder_motion");sim.backend.setRanges({2,2,2},6);
    sim.wait([&]{const auto s=sim.backend.snapshot();return sim.state()=="SENSOR_WAIT"&&s.armed&&s.linear_mps==0&&s.angular_rps==0&&sim.gateway->navigationActive();},1000,"transient invalid front sonar keeps authorized zero session");
    for(unsigned i=0;i<10;++i){std::this_thread::sleep_for(milliseconds(100));const auto s=sim.backend.snapshot();check(s.armed&&s.linear_mps==0&&s.angular_rps==0&&sim.enabled()&&sim.gateway->navigationActive(),"fresh phone owner retains zero range wait");}
    sim.backend.setRanges({2,2,2},7);sim.wait([&]{const auto s=sim.backend.snapshot();return s.armed&&s.twist_mode&&s.linear_mps>0;},2500,"new post-stop sonar resumes same authorized task");sim.trace("source_restored_same_session");
    sim.backend.setRanges({2,2,2},6);
    sim.wait([&]{return sim.state()=="SENSOR_WAIT";},1000,"persistent source timeout begins in zero WAIT");
    sim.wait([&]{const auto s=sim.backend.snapshot();return !s.armed&&s.left==0&&s.right==0&&!sim.gateway->navigationActive();},6200,"continuous five-second source failure ends session");sim.trace("source_5s_timeout_terminated");
    sim.backend.setRanges({2,2,2},7);std::this_thread::sleep_for(milliseconds(1000));check(!sim.enabled()&&!sim.backend.snapshot().armed,"fresh sonar alone cannot restart terminated task");
    sim.start();sim.wait([&]{const auto s=sim.backend.snapshot();return s.twist_mode&&s.linear_mps>0;},2500,"new explicit arm and Start recover persistent timeout");
    check(sim.runtime_errors==0,"recovery HTTP no unexpected errors");check(sim.unsafe_pairs==0,"recovery has no startup race");check(sim.control("stop").status==200,"final STOP");
}
void missing_owner_stops(){
    std::cout<<"SCENARIO owner_heartbeat_loss_350ms_disarm\n";
    Simulation sim;sim.start();sim.wait([&]{const auto s=sim.backend.snapshot();return s.twist_mode&&s.linear_mps>0;},2500,"motion before lease loss");sim.heartbeat=false;
    sim.wait([&]{return !sim.backend.snapshot().armed&&!sim.gateway->navigationActive();},750,"true phone lease expiry stops");sim.trace("owner_lease_terminated");
    sim.heartbeat=true;std::this_thread::sleep_for(milliseconds(800));check(!sim.backend.snapshot().armed&&!sim.enabled(),"heartbeats returning cannot rearm or restart");sim.trace("owner_recovered_still_disarmed");
}
void missing_ack_stops(){
    std::cout<<"SCENARIO actual_ACK4_loss_300ms_disarm\n";
    Simulation sim;sim.start();sim.wait([&]{const auto s=sim.backend.snapshot();return s.twist_mode&&s.linear_mps>0;},2500,"motion before ACK loss");
    {std::lock_guard<std::mutex>g(sim.backend.mutex);sim.backend.ack_enabled=false;}
    sim.wait([&]{return !sim.backend.snapshot().armed&&!sim.gateway->navigationActive();},900,"actual ACK freshness expires");sim.trace("motor_ACK_terminated");
    {std::lock_guard<std::mutex>g(sim.backend.mutex);sim.backend.ack_enabled=true;}
    std::this_thread::sleep_for(milliseconds(600));check(!sim.backend.snapshot().armed&&!sim.enabled(),"ACK return cannot rearm");
}
Reply prepare_start(Simulation &sim){
    check(sim.control("arm").status==200,"explicit arm before preparation");
    check(sim.control("drive",",\"throttle\":0,\"steer\":0").status==200,"zero intent before preparation");
    std::lock_guard<std::mutex>g(sim.phone_mutex);return http(sim.phone_port,"/api/navigation","{\"session\":\"session\",\"op\":\"straight\",\"nav_seq\":"+std::to_string(++sim.nav_seq)+",\"intent_seq\":"+std::to_string(sim.mobile_seq)+"}");
}
void local_ready(Simulation &sim,bool ready,unsigned flags=3){std::lock_guard<std::mutex>g(sim.backend.mutex);sim.backend.core_ready=ready;sim.backend.core_flags=flags;}
void nav1_classification_and_explicit_recovery(){
    std::cout<<"SCENARIO NAV1_CPU_trap_and_calibration_classification_ACK4_authority\n";
    Simulation sim;
    for(const auto& fault:std::array<std::pair<unsigned,const char*>,2>{{{7,"softcore_cpu_trap"},{11,"softcore_firmware_fault"}}}){
        local_ready(sim,false,fault.first);std::this_thread::sleep_for(milliseconds(160));
        const auto began=ms();const auto reply=prepare_start(sim);
        check(reply.status==409&&reply.json.get("error").get<std::string>()==fault.second,"actual NAV1 failure classification");
        check(ms()-began<200,"confirmed CPU fault returns inside200ms, not the APK1200ms timeout");
        check(!sim.gateway->navigationActive()&&!sim.gateway->navigationPreparing()&&!sim.enabled()&&!sim.backend.snapshot().armed,"confirmed CPU fault has no motion authority");
        const auto telemetry=sim.gateway->telemetry();picojson::value diagnostic;
        check(picojson::parse(diagnostic,telemetry.softcore_diagnostics_json).empty(),"passive NAV1 telemetry JSON");
        check(diagnostic.get("flags").get<double>()==fault.first&&diagnostic.get("observed_imu_sequence").get<double>()>0,"NAV1 flags and observed IMU sequence retained");
        local_ready(sim,true);
        std::this_thread::sleep_for(milliseconds(220));check(!sim.enabled()&&!sim.backend.snapshot().armed,"NAV1/ACK4 recovery cannot rearm or restart");
    }
    sim.start();sim.wait([&]{return sim.backend.snapshot().linear_mps>0;},2500,"new explicit Start after recovery works");
    check(sim.control("stop").status==200,"STOP after diagnosis recovery");check(sim.runtime_errors==0,"NAV1 classification lifecycle HTTP healthy");
}
void asynchronous_preparation(){
    std::cout<<"SCENARIO asynchronous_2s_calibration_old_APK_compatible_zero_session\n";
    Simulation sim;local_ready(sim,false);std::this_thread::sleep_for(milliseconds(150));
    const auto epoch=sim.shared->origin_epoch;const auto began=ms();const auto reply=prepare_start(sim);
    check(reply.status==200&&ms()-began<200,"preparation accepted without synchronous calibration wait");
    const auto nav=reply.json.get("navigation");check(nav.get("active").get<bool>()&&nav.get("preparing").get<bool>()&&!nav.get("motion_active").get<bool>(),"legacy APK sees an accepted active session, never false motion");
    check(nav.get("state").get<std::string>()=="PREPARING"&&nav.get("reason").get<std::string>()=="waiting_softcore_calibration","preparation status honest");
    const auto first=sim.gateway->telemetry().preparation_started_ms;
    for(unsigned i=0;i<22;++i){std::this_thread::sleep_for(milliseconds(100));const auto s=sim.backend.snapshot();check(s.armed&&s.twist_mode&&s.linear_mps==0&&s.angular_rps==0,"2s calibration retains armed ZERO MOT4");check(!sim.enabled()&&!sim.gateway->navigationActive()&&sim.gateway->navigationPreparing(),"internal core never starts early");}
    {std::lock_guard<std::mutex>g(sim.phone_mutex);const auto repeated=http(sim.phone_port,"/api/navigation","{\"session\":\"session\",\"op\":\"straight\",\"nav_seq\":"+std::to_string(sim.nav_seq)+",\"intent_seq\":"+std::to_string(sim.mobile_seq)+"}");check(repeated.status==409,"replayed start cannot replace preparation");}
    check(sim.gateway->telemetry().preparation_started_ms==first,"replay cannot refresh15s preparation budget");
    local_ready(sim,true);sim.wait([&]{return sim.gateway->navigationActive()&&sim.enabled()&&!sim.gateway->navigationPreparing();},1000,"fresh ACK4 ready atomically activates actual core");
    check(sim.shared->origin_epoch==epoch,"preparation never resets localization/map");check(sim.control("stop").status==200,"STOP after prepared start");
}
void preparation_cancellation(){
    for(const std::string action:{"stop","manual","lease","late_zero","sonar"}){
        std::cout<<"SCENARIO preparation_cancel_"<<action<<'\n';Simulation sim;local_ready(sim,false);std::this_thread::sleep_for(milliseconds(120));const auto epoch=sim.shared->origin_epoch;
        check(prepare_start(sim).status==200&&sim.gateway->navigationPreparing(),"entered preparation");
        if(action=="manual")check(sim.control("drive",",\"throttle\":0.1,\"steer\":0.2").status==200,"manual takeover admitted during preparation");
        else if(action=="lease"){sim.heartbeat=false;sim.wait([&]{return !sim.gateway->navigationPreparing()&&!sim.backend.snapshot().armed;},800,"350ms phone lease cancels preparation");sim.heartbeat=true;}
        else if(action=="sonar"){sim.backend.setRanges({2,2,2},6);std::this_thread::sleep_for(milliseconds(200));local_ready(sim,true);sim.wait([&]{const auto c=sim.backend.snapshot();return !sim.gateway->navigationPreparing()&&sim.gateway->navigationActive()&&sim.state()=="SENSOR_WAIT"&&c.armed&&c.linear_mps==0&&c.angular_rps==0;},1500,"post-calibration transient sonar starts zero WAIT only");check(sim.control("stop").status==200,"STOP cancels newly waiting session");}
        else if(action=="late_zero"){
            sim.backend.delay_twist_ms=100;const auto count=sim.backend.twist_entered.load();sim.wait([&]{return sim.backend.twist_entered>count;},500,"zero preparation command delayed in actual HTTP");
            local_ready(sim,true);check(sim.control("stop").status==200,"STOP admitted before delayed zero reply");sim.backend.delay_twist_ms=0;
        }else check(sim.control("stop").status==200,"explicit STOP cancels preparation");
        local_ready(sim,true);std::this_thread::sleep_for(milliseconds(300));check(!sim.gateway->navigationPreparing()&&!sim.gateway->navigationActive()&&!sim.enabled(),"late ready cannot revive cancelled preparation");
        check(sim.shared->origin_epoch==epoch,"cancellation/failure never clears localization or map");
        check(sim.control("stop").status==200,"final cancellation STOP");
    }
}
void preparation_timeout(){
    std::cout<<"SCENARIO preparation_actual_15s_timeout_no_auto_restart\n";
    Simulation sim;local_ready(sim,false);std::this_thread::sleep_for(milliseconds(120));const auto began=ms();const auto epoch=sim.shared->origin_epoch;
    check(prepare_start(sim).status==200,"15s preparation accepted");
    sim.wait([&]{return !sim.gateway->navigationPreparing()&&!sim.backend.snapshot().armed;},16000,"actual15s timeout cancels zero-only preparation");
    check(ms()-began>=15000&&ms()-began<15800,"actual wall clock respects15s preparation budget");
    const auto state=http(sim.phone_port,"/api/state","",true);check(state.json.get("navigation").get("cancel_reason").get<std::string>()=="softcore_preparation_timeout","timeout has explicit reason");
    local_ready(sim,true);std::this_thread::sleep_for(milliseconds(250));check(!sim.gateway->navigationActive()&&!sim.enabled()&&!sim.backend.snapshot().armed,"CPU recovery after timeout cannot rearm");
    check(sim.shared->origin_epoch==epoch,"timeout never resets map or localization");
}
}
int main(){try{wait_armed_zero();active_invalid_recovery_requires_new_start();missing_owner_stops();missing_ack_stops();nav1_classification_and_explicit_recovery();asynchronous_preparation();preparation_cancellation();preparation_timeout();std::cout<<"PASS controller/navigation lifecycle "<<checks<<" checks; real Controller, MOT4/ACK4/NAV1/SNS1 codecs, source alignment, Shared and Gateway; no physical IO\n";return 0;}catch(const std::exception&e){std::cerr<<"FAIL controller/navigation lifecycle: "<<e.what()<<'\n';return 1;}}
