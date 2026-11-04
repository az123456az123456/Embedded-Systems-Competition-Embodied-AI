// Exercise the actual production Shared command/tick state, the actual Gateway,
// and real loopback HTTP together. No camera, FPGA or physical motors are used.
#define main navigation_production_main_not_executed
#include "../main.cc"
#undef main
#include "../vendor/http_server.h"
#include <arpa/inet.h>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <sys/socket.h>

using namespace std::chrono;
namespace {
unsigned checks=0;
void checkAt(bool result,unsigned line) { ++checks;if(!result){std::fprintf(stderr,"FAIL handoff line=%u check=%u\n",line,checks);std::abort();} }
#define check(value) checkAt((value),__LINE__)
uint64_t test_ms() { return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count(); }
unsigned unused_port() {
    int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0);
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    check(bind(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0);
    socklen_t size=sizeof(address);check(getsockname(fd,reinterpret_cast<sockaddr*>(&address),&size)==0);
    close(fd);return ntohs(address.sin_port);
}
struct Response { int code=0; picojson::value json; };
Response http(unsigned port,const std::string& path,const std::string& body) {
    int fd=socket(AF_INET,SOCK_STREAM,0);assert(fd>=0);
    timeval timeout{2,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address.sin_port=htons(port);
    assert(connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0);
    const auto bytes="POST "+path+" HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer 012345\r\nContent-Length: "+std::to_string(body.size())+"\r\nConnection: close\r\n\r\n"+body;
    assert(robotphone::writeAll(fd,bytes.data(),bytes.size()));
    std::string data;char buffer[4096];ssize_t size;
    while((size=recv(fd,buffer,sizeof(buffer),0))>0)data.append(buffer,size);
    close(fd);const auto offset=data.find("\r\n\r\n");assert(offset!=std::string::npos);
    std::istringstream head(data);std::string protocol;Response result;head>>protocol>>result.code;
    assert(picojson::parse(result.json,data.substr(offset+4)).empty());return result;
}
template<class Predicate> void wait_for(Predicate predicate) {
    const auto limit=test_ms()+1500;
    while(!predicate()&&test_ms()<limit)std::this_thread::sleep_for(milliseconds(1));
    check(predicate());
}

struct Backend {
    robotphone::HttpServer server;
    std::mutex mutex;
    bool armed=false;
    uint64_t sequence=0;
    std::atomic<unsigned> delay_zero{0},delay_claim{0},zero_entered{0},claim_entered{0};
    std::atomic<bool> reject_zero{false},reject_claim{false};
    std::string state(bool okay=true)const {
        return std::string("{\"ok\":")+(okay?"true":"false")+
          ",\"session\":\"session\",\"armed\":"+(armed?"true":"false")+
          ",\"imu_action_pending\":false,\"reason\":\"fixture\",\"motor\":{\"velocity_protocol_supported\":true,\"softcore_ready\":true,\"ack_protocol\":4,\"link_ok\":true,\"left_mask\":10,\"invert_mask\":15,\"left\":0,\"right\":0,\"ack_age_ms\":1},\"imu\":{\"valid\":true,\"fresh\":true,\"bias_valid\":true,\"calibrating\":false,\"yaw\":0,\"roll\":0,\"pitch\":0,\"timestamp_ms\":100,\"sequence\":10,\"sample_age_ms\":1}}";
    }
    void start(unsigned port) {
        std::string error;
        check(server.start("127.0.0.1",port,[&](int fd,const robotphone::HttpRequest& request){
            if(request.path=="/api/state"||request.path=="/api/connect") {
                std::lock_guard<std::mutex> lock(mutex);robotphone::httpReply(fd,200,state());return;
            }
            picojson::value value;assert(picojson::parse(value,request.body).empty());
            const auto op=value.get("op").get<std::string>();
            if(op=="drive") { ++zero_entered;std::this_thread::sleep_for(milliseconds(delay_zero.load())); }
            if(op=="claim") { ++claim_entered;std::this_thread::sleep_for(milliseconds(delay_claim.load())); }
            std::lock_guard<std::mutex> lock(mutex);int status=200;
            const auto received=uint64_t(value.get("seq").get<double>());
            if(received<=sequence)status=409;
            else {
                sequence=received;
                if(op=="arm")armed=true;
                else if(op=="stop"||op=="disarm")armed=false;
                else if((op=="drive"&&reject_zero)||(op=="claim"&&reject_claim))status=409;
                else if((op=="drive"||op=="claim"||op=="twist")&&!armed)status=409;
            }
            auto output=state(status==200);
            if(op=="claim"&&status==200)output.insert(output.size()-1,",\"navigation_ticket\":\"private-capability\"");
            robotphone::httpReply(fd,status,output);
        },error));
    }
    bool is_armed() { std::lock_guard<std::mutex> lock(mutex);return armed; }
};

struct Fixture {
    Backend backend;
    const unsigned backend_port=unused_port(),phone_port=unused_port();
    const std::string token="/tmp/gateway-core-handoff-"+std::to_string(getpid())+"-"+std::to_string(phone_port);
    Config config;
    std::unique_ptr<Shared> shared;
    std::unique_ptr<robotnav::Gateway> gateway;
    std::thread runtime;
    std::atomic<bool> running{true},valid_sonar{true};
    std::atomic<unsigned> attempts{0},successful_starts{0},visits{0},bad_pairs{0};
    uint64_t mobile_sequence=0,navigation_sequence=0;

    Fixture() {
        backend.start(backend_port);{std::ofstream file(token);file<<"012345\n";}
        applyFixedProfile(config);config.configured=true;config.drive_mps_per_full_pwm=.6;
        shared=std::make_unique<Shared>(config,true);
        robotnav::GatewayCallbacks callbacks;
        callbacks.navigation_layout=[&](const robotnav::GatewayTelemetry& value){shared->synchronize_wheel_layout(value);};
        callbacks.navigation_command=[&](const std::string& op,double x,double y,std::string& error){
            if(op=="start"||op=="explore")++attempts;
            const bool okay=shared->command(op,x,y,error);
            if(okay&&(op=="start"||op=="explore"))++successful_starts;
            return okay;
        };
        callbacks.navigation_cancel=[&](const std::string& reason){shared->cancel(reason);};
        callbacks.navigation_state=[&]{return shared->status();};
        callbacks.navigation_goto=[&](double x,double y,const std::string& session,uint32_t revision,std::string& error){
            ++attempts;const bool okay=shared->go_to(x,y,session,revision,error);if(okay)++successful_starts;return okay;
        };
        robotnav::GatewayOptions options;options.backend_port=backend_port;options.port=phone_port;
        options.bind_address="127.0.0.1";options.token_file=token;
        gateway=std::make_unique<robotnav::Gateway>(options,callbacks);
        std::string error;check(gateway->start(error));check(http(phone_port,"/api/connect","{}").code==200);
        check(control("arm").code==200);gateway->visitNavigation([](bool,uint64_t){});
        // Feed actual production odometry/range frames; use the exact production
        // lock ordering and cancellation check from the navigation runtime.
        feed(navwire::monotonicSeconds()-.02);feed(navwire::monotonicSeconds());
        runtime=std::thread([&]{
            while(running) {
                feed(navwire::monotonicSeconds());
                bool active=false,enabled=false;uint64_t ticket=0;robot_nav::Control command;
                gateway->visitNavigation([&](bool current,uint64_t generation){
                    active=current;ticket=generation;std::lock_guard<std::mutex> lock(shared->mutex);
                    command=shared->nav.tick(navwire::monotonicSeconds());
                    if(!active&&shared->nav.enabled()) { ++bad_pairs;shared->nav.stop("phone_or_manual_cancel");shared->cancel_plan=true; }
                    enabled=shared->nav.enabled();++visits;
                });
                if(active&&!enabled)gateway->cancelNavigation(command.reason,ticket);
                std::this_thread::sleep_for(milliseconds(1));
            }
        });
    }
    ~Fixture() {
        running=false;if(runtime.joinable())runtime.join();gateway->stop();backend.server.stop();unlink(token.c_str());
    }
    void feed(double now) {
        std::lock_guard<std::mutex> lock(shared->mutex);
        robot_nav::SensorFrame frame;frame.timestamp_s=now;frame.imu_valid=true;
        if(valid_sonar)frame.sonar_m={2.,2.,2.};
        shared->nav.ingest(frame);
    }
    Response control(const std::string& op,const std::string& fields="") {
        return http(phone_port,"/api/control","{\"session\":\"session\",\"seq\":"+std::to_string(++mobile_sequence)+",\"op\":\""+op+"\""+fields+"}");
    }
    void prepare() {
        backend.delay_zero=0;backend.delay_claim=0;valid_sonar=true;
        check(control("arm").code==200);
        check(control("drive",",\"throttle\":0,\"steer\":0").code==200);
        feed(navwire::monotonicSeconds());
    }
    Response start_http(const std::string& op="explore",const std::string& fields="") {
        return http(phone_port,"/api/navigation","{\"session\":\"session\",\"op\":\""+op+"\",\"nav_seq\":"+std::to_string(++navigation_sequence)+",\"intent_seq\":"+std::to_string(mobile_sequence)+fields+"}");
    }
    bool core_enabled() {std::lock_guard<std::mutex> lock(shared->mutex);return shared->nav.enabled();}
};
}

#ifndef GATEWAY_HANDOFF_NO_MAIN
int main() {
    {
        Fixture fixture;
        // Run both phone Start spellings repeatedly. Each 80ms socket wait is
        // inside the real 150ms timeout and true 350ms owner lease.
        for(const auto& op:{"start","explore","start","explore"}) {
            fixture.prepare();fixture.backend.delay_zero=80;fixture.backend.delay_claim=80;
            const auto zero_before=fixture.backend.zero_entered.load(),claim_before=fixture.backend.claim_entered.load();
            const auto visits_before=fixture.visits.load(),starts_before=fixture.successful_starts.load();
            Response response;std::thread start([&]{response=fixture.start_http(op);});
            wait_for([&]{return fixture.backend.zero_entered>zero_before;});
            check(!fixture.core_enabled());check(!fixture.gateway->navigationActive());
            wait_for([&]{return fixture.backend.claim_entered>claim_before;});
            check(!fixture.core_enabled());check(!fixture.gateway->navigationActive());
            start.join();check(response.code==200);check(fixture.gateway->navigationActive());check(fixture.core_enabled());
            check(fixture.backend.is_armed());check(fixture.successful_starts==starts_before+1);
            check(fixture.visits>visits_before+20);check(fixture.bad_pairs==0);
            check(fixture.control("stop").code==200);check(!fixture.gateway->navigationActive());check(!fixture.core_enabled());check(!fixture.backend.is_armed());
        }
        // Local foreground console Start follows the same atomic handoff.
        fixture.prepare();fixture.backend.delay_zero=80;fixture.backend.delay_claim=80;
        std::string error;check(fixture.gateway->requestNavigation("explore",0,0,error));
        check(fixture.gateway->navigationActive()&&fixture.core_enabled()&&fixture.backend.is_armed());
        check(fixture.bad_pairs==0);check(fixture.control("stop").code==200);

        // Backend rejection at either stage cannot enable the actual core.
        for(bool reject_zero:{true,false}) {
            fixture.prepare();fixture.backend.reject_zero=reject_zero;fixture.backend.reject_claim=!reject_zero;
            const auto before=fixture.attempts.load();const auto result=fixture.start_http();
            check(result.code==409);check(fixture.attempts==before);
            check(!fixture.gateway->navigationActive()&&!fixture.core_enabled()&&fixture.backend.is_armed());
            check(result.json.get("error").get<std::string>()==(reject_zero?"navigation_zero_rejected":"navigation_capability_rejected"));
            fixture.backend.reject_zero=false;fixture.backend.reject_claim=false;
        }

        // A transient range gap arriving during claim accepts only a zero
        // waiting session. Current IMU/odom/ACK/owner guards still apply.
        fixture.prepare();fixture.backend.delay_claim=100;
        const auto claims_before=fixture.backend.claim_entered.load(),attempts_before=fixture.attempts.load();
        Response rejected;std::thread start([&]{rejected=fixture.start_http();});
        wait_for([&]{return fixture.backend.claim_entered>claims_before;});
        fixture.valid_sonar=false;fixture.feed(navwire::monotonicSeconds());
        start.join();check(rejected.code==200);
        check(fixture.attempts==attempts_before+1);
        check(fixture.gateway->navigationActive()&&fixture.core_enabled()&&fixture.backend.is_armed());
        wait_for([&]{std::lock_guard<std::mutex> lock(fixture.shared->mutex);return fixture.shared->nav.state()=="SENSOR_WAIT";});
        {std::lock_guard<std::mutex> lock(fixture.shared->mutex);check(fixture.shared->nav.state()=="SENSOR_WAIT");check(fixture.shared->nav.command().linear_mps==0&&fixture.shared->nav.command().angular_rps==0);}
        for(unsigned i=0;i<3;++i){check(fixture.control("idle").code==200);std::this_thread::sleep_for(milliseconds(80));check(fixture.gateway->navigationActive()&&fixture.core_enabled());}
        fixture.valid_sonar=true;
        for(unsigned i=0;i<12;++i){check(fixture.control("idle").code==200);std::this_thread::sleep_for(milliseconds(40));}
        check(fixture.gateway->navigationActive()&&fixture.core_enabled()&&fixture.backend.is_armed());
        check(fixture.control("stop").code==200);check(!fixture.gateway->navigationActive()&&!fixture.core_enabled()&&!fixture.backend.is_armed());
        check(fixture.bad_pairs==0);
    }
    // STOP admission revokes a pending zero/claim generation immediately; the
    // old Start is never allowed to start the core after the stop barrier.
    for(bool stop_during_zero:{true,false}) {
        Fixture fixture;fixture.prepare();fixture.backend.delay_zero=100;fixture.backend.delay_claim=100;
        const auto zero_before=fixture.backend.zero_entered.load(),claim_before=fixture.backend.claim_entered.load();
        Response started,stopped;std::thread start([&]{started=fixture.start_http();});
        wait_for([&]{return stop_during_zero?fixture.backend.zero_entered>zero_before:fixture.backend.claim_entered>claim_before;});
        std::thread stop([&]{stopped=fixture.control("stop");});start.join();stop.join();
        check(started.code==409&&stopped.code==200);check(fixture.attempts==0);
        check(!fixture.gateway->navigationActive()&&!fixture.core_enabled()&&!fixture.backend.is_armed());
        check(fixture.bad_pairs==0);
    }
    // Map-bound goto already uses the same begin-then-core order; run the real
    // production map cache and goal checks so it cannot regress independently.
    {
        Fixture fixture;fixture.prepare();std::string session;uint32_t revision=0;
        {
            std::lock_guard<std::mutex> lock(fixture.shared->mutex);const auto now=navwire::monotonicSeconds();
            check(fixture.shared->nav.grid().import_observations(std::vector<int8_t>(4096,0),{-3.2,-3.2},now));
            session=fixture.shared->map_session+"-"+std::to_string(fixture.shared->origin_epoch);
            revision=fixture.shared->nav.grid().revision();
            fixture.shared->map_cache.publish(robot_nav::map_snapshot_json(fixture.shared->nav,now,0,false,session),now);
        }
        fixture.backend.delay_zero=80;fixture.backend.delay_claim=80;
        const auto response=fixture.start_http("goto",",\"x\":0.55,\"y\":0.05,\"map_session\":\""+session+"\",\"map_revision\":"+std::to_string(revision));
        check(response.code==200);check(fixture.gateway->navigationActive()&&fixture.core_enabled());check(fixture.bad_pairs==0);
        check(fixture.control("stop").code==200);
    }
    std::printf("PASS %u real Shared/Gateway handoff checks: delayed zero+claim, phone/local repeat starts, backend/core rejection, STOP barriers, map-bound goto; no inactive/enabled core window\n",checks);
}

#endif // GATEWAY_HANDOFF_NO_MAIN
