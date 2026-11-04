// Real production Shared + Gateway + asynchronous wrong.log writer. The HTTP
// peer is loopback only; this executable never opens FPGA/camera/motor devices.
#define main navigation_main_not_executed
#include "../main.cc"
#undef main
#include "../vendor/http_server.h"
#include <arpa/inet.h>
#include <cstdlib>
#include <sys/socket.h>

namespace {
unsigned checks=0;
void check(bool value,const char* why){++checks;if(!value)throw std::runtime_error(why);}
unsigned free_port(){int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"socket");sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);check(!bind(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a)),"bind");socklen_t n=sizeof(a);check(!getsockname(fd,reinterpret_cast<sockaddr*>(&a),&n),"getsockname");close(fd);return ntohs(a.sin_port);}
struct Response{int code=0;picojson::value data;};
Response post(unsigned port,const std::string& path,const std::string& body){
    int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"http socket");timeval timeout{2,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);check(!connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a)),"http connect");
    auto request="POST "+path+" HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer TOKEN_NEVER_LOG\r\nContent-Length: "+std::to_string(body.size())+"\r\nConnection: close\r\n\r\n"+body;
    check(robotphone::writeAll(fd,request.data(),request.size()),"http write");std::string bytes;char buffer[4096];ssize_t n;while((n=recv(fd,buffer,sizeof(buffer),0))>0)bytes.append(buffer,n);close(fd);
    const auto end=bytes.find("\r\n\r\n");check(end!=std::string::npos,"http response");Response r;std::string protocol;std::istringstream header(bytes);header>>protocol>>r.code;check(picojson::parse(r.data,bytes.substr(end+4)).empty(),"http json");return r;
}
Config config(){Config c;applyFixedProfile(c);c.configured=true;c.drive_mps_per_full_pwm=.6;c.sonar_wait_timeout_s=.08;return c;} // shortened persistent-source timeout fixture; production is 5s
void feed(Shared& shared,double now,unsigned valid=7,double distance=2.,double yaw=0){
    std::lock_guard<std::mutex> lock(shared.mutex);auto& raw=shared.sensors;raw.sequence++;raw.uptime_ms+=20;raw.valid=valid;raw.fault=7U^valid;raw.counts={{-12345,23456,-34567,45678}};raw.delta={{0,0,0,0}};raw.age_ms={{10,20,30}};raw.distance_mm={{uint16_t(distance*1000),uint16_t(distance*1000),uint16_t(distance*1000)}};raw.window_ms=50;shared.have_sensors=true;shared.received_s=navwire::monotonicSeconds();
    robot_nav::SensorFrame f;f.timestamp_s=now;f.sequence=raw.sequence;f.device_timestamp_s=now;f.imu_valid=true;f.imu_age_s=.001;f.yaw_rad=yaw;for(unsigned i=0;i<4;++i)f.counts[i]=uint32_t(raw.counts[i]);for(unsigned i=0;i<3;++i)if(valid&(1U<<i))f.sonar_m[i]=distance;
    shared.nav.ingest(f,now);shared.observe_diagnostics_locked();
}
std::vector<picojson::value> read_log(const std::string& path){std::ifstream in(path);check(bool(in),"wrong.log written");std::vector<picojson::value> records;std::string line;while(std::getline(in,line)){picojson::value value;check(picojson::parse(value,line).empty(),"each line valid JSON");records.push_back(value);}return records;}
bool text(const picojson::value& value,const char* key,const std::string& expected){return value.get(key).is<std::string>()&&value.get(key).get<std::string>()==expected;}
}
int main(){
    const std::string dir="/tmp/wronglog-navigation-"+std::to_string(getpid());mkdir(dir.c_str(),0700);const auto path=dir+"/wrong.log";
    setenv("ROBOT_WRONG_LOG_PATH",path.c_str(),1);setenv("ROBOT_WRONG_LOG_MAX_BYTES","4194304",1);
    try{
        // Inspection modes must not start a logger or create the default file.
        char app[]="robot-navigation",version[]="--version";char* argv[]={app,version,nullptr};check(navigation_main_not_executed(2,argv)==0,"version succeeds");check(access(path.c_str(),F_OK)!=0,"--version creates no wrong.log");
        wrong_log_start("navigation-test","v26.3-test");
        picojson::value settings;check(picojson::parse(settings,runtime_log_config(config(),1,true)).empty(),"safe runtime configuration JSON");
        check(settings.get("runtime_limits").get("sonar_timeout_s").get<double>()==.75,"actual sonar timeout logged");check(settings.get("runtime_limits").get("max_pwm").get<double>()==.45,"actual duty limit logged");check(settings.get("encoder_ticks_per_turn").get<double>()==60527.7,"actual CPR logged");wrong_log_snapshot("navigation_config",settings.serialize().c_str());
        // BOOTSTRAP_WAIT is an enabled zero-PWM diagnostic, not a fault. Use
        // genuine production ticks and range observations, without map imports.
        {
            Shared wait(config(),true);wait.have_wheel_layout=true;double t=navwire::monotonicSeconds();feed(wait,t-.02,7,2.);feed(wait,t,7,2.);std::string error;check(wait.command("start",0,0,error),"start with valid near obstacle");
            for(unsigned i=0;i<80&&wait.nav.state()!="BOOTSTRAP_WAIT";++i){t+=.02;feed(wait,t,7,2.,.12);std::lock_guard<std::mutex> lock(wait.mutex);wait.nav.tick(t);wait.observe_diagnostics_locked();}
            check(wait.nav.state()=="BOOTSTRAP_WAIT"&&wait.nav.enabled(),"wait remains enabled");check(wait.nav.command().left==0&&wait.nav.command().right==0,"wait zero duty");
            wrong_log_snapshot("navigation",wait.status().c_str());
        }
        robotphone::HttpServer backend;const unsigned backend_port=free_port(),phone_port=free_port();std::mutex backend_mutex;bool armed=false;uint64_t ack=0;
        auto backend_json=[&]{return std::string("{\"ok\":true,\"session\":\"SESSION_NEVER_LOG\",\"armed\":")+(armed?"true":"false")+",\"reason\":\"fixture\",\"imu_action_pending\":false,\"motor\":{\"velocity_protocol_supported\":true,\"softcore_ready\":true,\"ack_protocol\":4,\"link_ok\":true,\"left_mask\":10,\"invert_mask\":15,\"left\":0,\"right\":0,\"ack_left\":0,\"ack_right\":0,\"ack_sequence\":"+std::to_string(ack)+",\"ack_age_ms\":2},\"imu\":{\"valid\":true,\"fresh\":true,\"bias_valid\":true,\"calibrating\":false,\"yaw\":0,\"roll\":0,\"pitch\":0,\"sequence\":4321,\"timestamp_ms\":123456,\"sample_age_ms\":1}}";};
        std::string error;check(backend.start("127.0.0.1",backend_port,[&](int fd,const robotphone::HttpRequest& request){std::lock_guard<std::mutex> lock(backend_mutex);std::string op;if(request.path!="/api/state"&&request.path!="/api/connect"){picojson::value v;check(picojson::parse(v,request.body).empty(),"backend input");op=v.get("op").get<std::string>();ack=uint64_t(v.get("seq").get<double>());if(op=="arm")armed=true;if(op=="stop"||op=="disarm")armed=false;}auto out=backend_json();if(op=="claim")out.insert(out.size()-1,",\"navigation_ticket\":\"CAPABILITY_NEVER_LOG\"");robotphone::httpReply(fd,200,out);},error),"backend start");
        const auto token=dir+"/token";{std::ofstream file(token);file<<"TOKEN_NEVER_LOG\n";}
        Shared shared(config(),true);robotnav::GatewayCallbacks cb;cb.navigation_layout=[&](const robotnav::GatewayTelemetry& t){shared.synchronize_wheel_layout(t);};cb.navigation_command=[&](const std::string& op,double x,double y,std::string& e){return shared.command(op,x,y,e);};cb.navigation_cancel=[&](const std::string& why){shared.cancel(why);};cb.navigation_state=[&]{return shared.status();};
        robotnav::GatewayOptions options;options.bind_address="127.0.0.1";options.port=phone_port;options.backend_port=backend_port;options.token_file=token;
        {
            robotnav::Gateway gateway(options,cb);check(gateway.start(error),"gateway start");check(post(phone_port,"/api/connect","{}").code==200,"connect");
            auto control=[&](unsigned seq,const std::string& op,const std::string& fields=""){return post(phone_port,"/api/control","{\"session\":\"SESSION_NEVER_LOG\",\"seq\":"+std::to_string(seq)+",\"op\":\""+op+"\""+fields+"}");};
            check(control(1,"arm").code==200,"arm");double t=navwire::monotonicSeconds();feed(shared,t-.02);feed(shared,t);gateway.visitNavigation([](bool,uint64_t){});wrong_log_snapshot("navigation",shared.status().c_str());
            check(control(2,"drive",",\"throttle\":0,\"steer\":0").code==200,"zero");
            check(post(phone_port,"/api/navigation","{\"session\":\"SESSION_NEVER_LOG\",\"op\":\"start\",\"nav_seq\":1,\"intent_seq\":2}").code==200,"start");check(gateway.navigationActive(),"gateway active");
            // Short invalid is a logged zero wait; only persistent invalid
            // becomes a fault, retained before gateway cancellation.
            t=navwire::monotonicSeconds();feed(shared,t,6);
            robot_nav::Control command;uint64_t generation=0;gateway.visitNavigation([&](bool active,uint64_t ticket){check(active,"active before failure cancellation");generation=ticket;std::lock_guard<std::mutex> lock(shared.mutex);command=shared.nav.tick(navwire::monotonicSeconds());shared.observe_diagnostics_locked();});
            check(shared.nav.enabled()&&shared.nav.state()=="SENSOR_WAIT","transient sonar wait retains authorization");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));t=navwire::monotonicSeconds();feed(shared,t,6);
            gateway.visitNavigation([&](bool active,uint64_t ticket){check(active,"active before persistent source cancellation");generation=ticket;std::lock_guard<std::mutex> lock(shared.mutex);command=shared.nav.tick(navwire::monotonicSeconds());shared.observe_diagnostics_locked();});
            check(!shared.nav.enabled()&&shared.nav.reason()=="sonar_unavailable_timeout","genuine persistent-source timeout fault");gateway.cancelNavigation(command.reason,generation);check(!gateway.navigationActive(),"cancel disarms");
            check(post(phone_port,"/api/navigation","{\"session\":\"SESSION_NEVER_LOG\",\"op\":\"start\",\"nav_seq\":2,\"intent_seq\":2}").code==409,"start rejection logged");
            check(control(3,"stop",",\"client_reason\":\"app_background\",\"client_mono_ms\":456789,\"client_state_age_ms\":99,\"client_lease_age_ms\":-1,\"client_diag_id\":77,\"client_secret\":\"CLIENT_SECRET_NEVER_LOG\"").code==200,"phone diagnostic STOP unchanged");
            check(control(4,"drive",",\"throttle\":0,\"steer\":0,\"client_reason\":\"joystick_release\",\"client_mono_ms\":456799,\"client_diag_id\":78").code==200,"normal release diagnostic unchanged");
            gateway.stop();
        }
        backend.stop();wrong_log_shutdown();
        const auto records=read_log(path);bool wait=false,core_fault=false,gateway_cancel=false,disarm=false,rejected=false,history=false,ack_seen=false,client_admitted=false,client_executed=false,release=false;size_t fault_index=records.size(),cancel_index=records.size();std::string all;
        for(size_t i=0;i<records.size();++i){const auto& r=records[i];all+=r.serialize();const auto& d=r.get("data");history=history||text(r,"phase","before");
            if(text(r,"component","navigation")&&text(d,"operation","bootstrap_wait")){wait=true;check(text(d,"classification","waiting_not_fault"),"WAIT classified diagnostic");check(d.get("navigation_enabled").get<bool>(),"WAIT log says enabled");}
            if(text(r,"component","navigation")&&text(r,"reason","sonar_unavailable_timeout")&&text(d,"classification","fault")){core_fault=true;fault_index=std::min(i,fault_index);check(text(d,"state","FAULT"),"original FAULT state retained");check(d.get("sensor_packet").get("valid").get<double>()==6,"invalid channel retained");check(d.get("sensor_packet").get("fault").get<double>()==1,"fault mask retained");check(d.get("sensor_packet").get("sequence").get<double>()>0,"SNS sequence retained");check(d.get("sonar_health").get("channels").get<picojson::array>()[0].get("last_accepted_mm").get<double>()==2000,"last accepted range retained");check(d.get("map_evidence").get("cells").get<std::string>().size()==4096,"observed grid retained within budget");check(d.get("map_evidence").get("computed_at_mono_ms").is<double>(),"map view time retained");check(!d.get("invalid_json").is<bool>(),"diagnostic fits payload budget");}
            if(text(r,"component","gateway")&&text(d,"operation","navigation_cancel")){gateway_cancel=true;cancel_index=std::min(i,cancel_index);check(d.get("active").get<bool>(),"cancel log before invalidate");check(d.get("backend_sequence").get<double>()>=4,"causal backend seq");check(d.get("mobile_sequence").get<double>()==2,"causal phone seq");check(d.get("navigation_sequence").get<double>()==1,"causal nav seq");check(d.get("imu_sequence").get<double>()==4321,"IMU seq");ack_seen=d.get("ack_sequence").get<double>()>=4;}
            disarm=disarm||text(r,"reason","disarm_confirmed");rejected=rejected||(text(r,"component","gateway")&&text(d,"operation","navigation_request")&&d.get("http_status").is<double>()&&d.get("http_status").get<double>()==409);
            if(text(r,"component","phone_client")&&text(d,"client_reason","app_background")){
                check(text(d,"classification","client_reported_unverified"),"phone reports marked unverified");check(text(d,"clock_domain","phone_monotonic_not_RK_clock"),"phone clock distinguished");check(d.get("client_diag_id").get<double>()==77&&d.get("mobile_sequence").get<double>()==3,"phone intent correlations retained");check(d.get("client_state_age_ms").get<double>()==99,"phone source age retained");check(!d.get("client_lease_age_ms").is<double>(),"unknown negative age omitted");client_admitted=client_admitted||text(d,"phase","admitted");client_executed=client_executed||text(d,"phase","executed");
            }
            if(text(r,"component","phone_client")&&text(d,"client_reason","joystick_release")){release=true;check(text(r,"event_kind","snapshot"),"normal zero release never triggers fault incident");}
        }
        check(wait&&core_fault&&gateway_cancel&&disarm&&rejected&&history&&ack_seen&&client_admitted&&client_executed&&release,"all diagnostic pathways recorded");check(fault_index<cancel_index,"original core fault precedes cancellation");check(all.find("TOKEN_NEVER_LOG")==std::string::npos&&all.find("SESSION_NEVER_LOG")==std::string::npos&&all.find("CAPABILITY_NEVER_LOG")==std::string::npos&&all.find("CLIENT_SECRET_NEVER_LOG")==std::string::npos,"no secrets logged");
        std::printf("PASS %u wrong.log navigation checks: real BOOTSTRAP_WAIT, real HTTP start, original SNS fault before cancel/disarm, rejection, cross-process sequence keys, prior context, no secrets; log=%s\n",checks,path.c_str());return 0;
    }catch(const std::exception& e){wrong_log_shutdown();std::fprintf(stderr,"FAIL wronglog navigation: %s\n",e.what());return 1;}
}
