#include "phone_service.h"
#include "stream_rate.h"
#include "controller.h"
#include "http_server.h"
#include "json_flat.h"
#include "motor_link.h"
#include "follow_sensors.h"
#include "jpeg_codec.h"
#include "control_audit.h"
#include "fpga_navigation.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <ifaddrs.h>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <linux/sockios.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>
using namespace robotphone;
namespace {
uint64_t clockMs(){return imu_clock_ms();}
std::vector<unsigned char> randomBytes(size_t count){
 std::vector<unsigned char> bytes(count);int fd=open("/dev/urandom",O_RDONLY|O_CLOEXEC);
 if(fd<0)throw std::runtime_error("Cannot open random source");
 size_t at=0;while(at<count){ssize_t n=read(fd,bytes.data()+at,count-at);if(n<0&&errno==EINTR)continue;if(n<=0){close(fd);throw std::runtime_error("Random source failed");}at+=n;}close(fd);return bytes;
}
std::string randomHex(size_t count){auto bytes=randomBytes(count);static const char hex[]="0123456789abcdef";std::string out;for(auto b:bytes){out+=hex[b>>4];out+=hex[b&15];}return out;}
std::string loadToken(const std::string &path){
 std::ifstream file(path);std::string token;
 if(file){std::getline(file,token);if(!token.empty()&&token.back()=='\r')token.pop_back();}
 else {
  // Rejection sampling avoids modulo bias. Existing long pairing keys are retained.
  while(token.size()<6){auto b=randomBytes(1)[0];if(b<250)token+=char('0'+b%10);}
  int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);
  if(fd<0)throw std::runtime_error("Cannot create pairing code file: "+path);
  std::string bytes=token+'\n';size_t at=0;
  while(at<bytes.size()){ssize_t n=write(fd,bytes.data()+at,bytes.size()-at);if(n<0&&errno==EINTR)continue;if(n<=0){close(fd);throw std::runtime_error("Cannot write pairing code file");}at+=n;}close(fd);
 }
 const bool short_code=token.size()==6&&token.find_first_not_of("0123456789")==std::string::npos;
 if((!short_code&&(token.size()<8||token.size()>128))||token.find_first_of(" \t\r\n\0",0,5)!=std::string::npos)throw std::runtime_error("Pair code must be 6 digits or an existing 8..128 character key");
 return token;
}
bool secretEqual(const std::string &a,const std::string &b){unsigned diff=a.size()^b.size();for(size_t i=0;i<std::max(a.size(),b.size());i++)diff|=(i<a.size()?static_cast<unsigned char>(a[i]):0)^(i<b.size()?static_cast<unsigned char>(b[i]):0);return diff==0;}
std::string base64(const std::string &s){static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";std::string out;for(size_t i=0;i<s.size();i+=3){unsigned n=static_cast<unsigned char>(s[i])<<16;if(i+1<s.size())n|=static_cast<unsigned char>(s[i+1])<<8;if(i+2<s.size())n|=static_cast<unsigned char>(s[i+2]);out+=alphabet[n>>18];out+=alphabet[(n>>12)&63];out+=i+1<s.size()?alphabet[(n>>6)&63]:'=';out+=i+2<s.size()?alphabet[n&63]:'=';}return out;}
struct PublishedFrame {uint64_t id=0,capture_ms=0,detection_ms=0;std::vector<uint8_t> jpeg;std::string metadata_prefix;};
struct FrameAlias {uint64_t preview_id,detection_id,time_ms;};
}

// Standalone service probes do not link vision.cc; an attached production
// application provides these strong definitions. Never create a second camera.
extern "C" void vision_snapshot_copy(vision_worker *,vision_snapshot *) __attribute__((weak));
extern "C" uint64_t vision_snapshot_generation(vision_worker *) __attribute__((weak));
extern "C" void vision_observation_copy(vision_worker *,vision_snapshot *) __attribute__((weak));

struct phone_service {
 phone_options options;std::string bind_,token_path_,interface_,imu_interface_,model_,token_,session_,motor_error_,address_;
 phone_imu_diagnostics imu_network={};
 Controller controller;MotorLink motor;HttpServer http;
 FpgaNavigationAdapter fpga_navigation;
 ControlAudit audit;
 std::mutex state_mutex,snapshot_mutex,frame_mutex,diagnostic_mutex;
 std::string pending_control_report;
 std::condition_variable snapshot_ready,frame_ready;
 std::unique_ptr<vision_snapshot> pending{new vision_snapshot{}};
 imu_state imu={};vision_snapshot metrics={};
 std::shared_ptr<PublishedFrame> frame;std::deque<FrameAlias> aliases;std::deque<VisionFrame> detection_history;
 std::atomic<bool> stopped{true},force_tx{false};std::atomic<unsigned> imu_action{0};std::atomic<int> streams{0};
 std::atomic<uint64_t> encodes{0},encode_us{0},snapshot_copies{0},pending_drops{0},sent_frames{0};
 std::atomic<uint64_t> stream_stale_drops{0},stream_slow_disconnects{0},stream_send_ms{0},stream_source_age_ms{0},stream_backlog_bytes{0};
 std::atomic<double> target_fps{0},phone_result_fps{0},window_encode_fps{0},window_send_fps{0},window_encode_ms{0};
 std::thread control_thread,publisher,observer;std::mutex observer_mutex;
 bool motor_available=false,have_snapshot=false;
 uint64_t sequence=0,owner_control_ms=0,last_client_ms=0,last_detection_id=0,last_preview_id=0;
 uint64_t auth_window=0,auth_block_until=0;unsigned auth_failures=0;
 static Config config(const phone_options &o){
  Config c;c.left_motor_mask=o.left_mask;c.vision_fresh_ms=VISION_BOX_MAX_AGE_MS;
  const char *camera_source=std::getenv("PERCEPTION_CAMERA_SOURCE");
  const bool prepared_rgb=camera_source&&!std::strcmp(camera_source,"fpga-rgb");
  c.follow_stereo_view=((o.camera_width==1920&&o.camera_height==1080)||prepared_rgb)&&o.camera_view&&!std::strcmp(o.camera_view,"both");
  auto tuning=[](const char *name,double fallback,double lo,double hi,bool exclusive_lo){
   const char *text=std::getenv(name);if(!text)return fallback;
   char *end=nullptr;errno=0;const double value=std::strtod(text,&end);
   if(end==text||*end||errno==ERANGE||!std::isfinite(value)||
      (exclusive_lo?value<=lo:value<lo)||value>hi)
    throw std::runtime_error(std::string("Invalid ")+name+": expected finite value "+
      (exclusive_lo?">":">=")+std::to_string(lo)+" and <="+std::to_string(hi));
   return value;
  };
  c.turn_inner_ratio=tuning("PERCEPTION_TURN_INNER_RATIO",c.turn_inner_ratio,.10,.80,false);
  c.turn_full_deg=tuning("PERCEPTION_TURN_FULL_DEG",c.turn_full_deg,15.,90.,true);
  const char *path=std::getenv("FOLLOW_GEOMETRY_FILE");std::string error;
  if(!loadBearingConfig(path&&*path?path:"/root/robot-navigation/follow_geometry.json",c.bearing,error))
   std::fprintf(stderr,"FOLLOW geometry unavailable (%s); image servo uses pixel centering\n",error.c_str());
  return c;
 }
 std::string navigation_ticket_;
 uint64_t navigation_ticket_sequence_=0;
 uint64_t softcore_report_ms_=0;
 std::string softcore_reason_;
 bool softcoreRequested()const{return controller.state().armed&&(controller.state().twist_mode||!navigation_ticket_.empty());}
 void observeSoftcore(uint64_t now) noexcept {
  try {
  const auto reason=motor.softcoreReason(now,softcoreRequested());const bool changed=reason!=softcore_reason_;
  if(changed||!softcore_report_ms_||now<softcore_report_ms_||now-softcore_report_ms_>=100){
   const auto json=motor.softcore().json(now,motor.softcoreReady(now),softcoreRequested());
   wrong_log_snapshot("softcore",json.c_str());
   if(changed&&motor.softcore().status().seen&&reason!="ready"&&reason!="softcore_idle_protocol_probe")wrong_log_event("softcore",reason.c_str(),json.c_str());
   softcore_report_ms_=now;softcore_reason_=reason;
  }
  }catch(...){ /* A diagnostic allocation failure cannot interrupt control. */ }
 }
 void reportSoftcore(uint64_t now) noexcept {
  try {const auto json=motor.softcore().json(now,motor.softcoreReady(now),softcoreRequested());std::fprintf(stderr,"SOFTCORE_NAV %s\n",json.c_str());}
  catch(...){ /* Preserve the original actuation and timing decisions. */ }
 }
 void auditState(uint64_t now) noexcept {audit.observe(now,controller.state(),motor.diagnostics(),motor.ack(),options.left_mask,options.invert_mask,owner_control_ms,sequence,&motor.softcore(),motor.softcoreReady(now));}
 explicit phone_service(const phone_options &o):options(o),bind_(o.bind_address?o.bind_address:"0.0.0.0"),token_path_(o.pair_token_file?o.pair_token_file:"pair-token.txt"),interface_(o.interface_name?o.interface_name:""),model_(o.model?o.model:""),controller(config(o)){
  token_=loadToken(token_path_);imu_init(&imu);
  address_="http://<RK3588-WiFi-IP>:"+std::to_string(options.port);
  ifaddrs *addresses=nullptr;if(getifaddrs(&addresses)==0){
   for(auto *a=addresses;a;a=a->ifa_next)if(a->ifa_addr&&a->ifa_addr->sa_family==AF_INET&&std::strcmp(a->ifa_name,"lo")&&interface_!=a->ifa_name){char ip[INET_ADDRSTRLEN];auto *addr=reinterpret_cast<sockaddr_in*>(a->ifa_addr);if(inet_ntop(AF_INET,&addr->sin_addr,ip,sizeof(ip)))address_="http://"+std::string(ip)+':'+std::to_string(options.port);}
   freeifaddrs(addresses);
  }
 }
 ~phone_service(){stop();}
 bool start(std::string &error){
  if(!options.demo&&!interface_.empty()){motor_available=motor.open(interface_,motor_error_);if(!motor_available){std::fprintf(stderr,"Phone motor link: %s\n",motor_error_.c_str());wrong_log_event("motor_network","socket_open_failed","{\"category\":\"network_failure\"}");}}
  const char *nav_source=std::getenv("PERCEPTION_NAVIGATION_SOURCE");
  if(nav_source&&!std::strcmp(nav_source,"fpga")){
   const bool available=fpga_navigation.open(interface_);
   std::fprintf(stderr,"FPGA_NAV adapter=%s autonomous_execution=FPGA NVA_sender=dashboard startup_enable=no\n",available?"ready":"unavailable");
  }
  std::fprintf(stderr,"MOTOR_NET interface=%s socket=%s error=%s left_mask=%u invert_mask=%u; ACK required before arm\n",interface_.empty()?"none":interface_.c_str(),motor_available?"open":options.demo?"demo":"closed",motor_error_.empty()?"none":motor_error_.c_str(),options.left_mask,options.invert_mask);
  const auto &manual=controller.config();
  std::fprintf(stderr,"MANUAL_ARC straight_cone_deg=%.1f inner_min_ratio=%.3f full_deflection_deg=%.1f radial_outer_max=%.3f sharp_wire=MOT1-exact-wheels compatible_fpga=v21\n",manual.straight_cone_deg,manual.turn_inner_ratio,manual.turn_full_deg,manual.max_duty);
  stopped=false;
  if(!http.start(bind_,options.port,[this](int fd,const HttpRequest &r){request(fd,r);},error)){stopped=true;wrong_log_event("phone_service","http_start_failed","{}");return false;}
  try{control_thread=std::thread(&phone_service::controlLoop,this);publisher=std::thread(&phone_service::publishLoop,this);}catch(...){stop();throw;}
  std::fprintf(stderr,"PHONE unified service on %s:%u; pairing code saved in %s; shared camera/NPU/IMU\n",bind_.c_str(),options.port,token_path_.c_str());return true;
 }
 void stop(){
  if(stopped.exchange(true))return;
  wrong_log_event("phone_service","service_stopping","{\"category\":\"lifecycle_stop\"}");
  snapshot_ready.notify_all();frame_ready.notify_all();
  {std::lock_guard<std::mutex> lock(observer_mutex);if(observer.joinable())observer.join();}
  http.stop();
  if(publisher.joinable())publisher.join();
  if(control_thread.joinable())control_thread.join();
  for(int i=0;i<3;i++)if(motor_available)motor.send(false,0,0,options.left_mask,options.invert_mask,clockMs());
  motor.close();
 }
 bool attach(vision_worker *source){
  std::lock_guard<std::mutex> guard(observer_mutex);
  if(!source||stopped||observer.joinable()||!vision_snapshot_copy||!vision_snapshot_generation||!vision_observation_copy)return false;
  try{observer=std::thread([this,source]{
   try{
    auto snapshot=std::make_unique<vision_snapshot>();uint64_t previous=UINT64_MAX;bool had_clients=false;
    while(!stopped){
     const auto generation=vision_snapshot_generation(source);const bool clients=streams.load()>0;
     if(generation!=previous||(clients&&!had_clients)){
      if(clients)vision_snapshot_copy(source,snapshot.get());
      else{vision_observation_copy(source,snapshot.get());snapshot->have_frame=0;} // no pixels copied; never publish a black first frame if a client races in
      publish(snapshot.get(),nullptr,clockMs());previous=generation;
     }
     had_clients=clients;std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
   }catch(const std::exception &e){wrong_log_event("phone_observer","snapshot_failed",e.what());}
  });}catch(...){return false;}
  return true;
 }
 void controlLoop(){uint64_t sent_at=0,probe_at=0,report=clockMs();unsigned probe_version=4;while(!stopped){{
  std::lock_guard<std::mutex> lock(state_mutex);
  // HTTP/state readers also tick the controller under this mutex. Read the
  // control time only after acquiring it: an earlier timestamp held while
  // waiting could otherwise falsely look like a monotonic-clock reversal.
  const uint64_t now=clockMs();
  motor.receive(now);
  fpga_navigation.owner(session_,sequence,owner_control_ms,controller.state().armed,now);
  fpga_navigation.tick(now);
  observeSoftcore(now);
  bool ready=fpga_navigation.quiet()?fpga_navigation.ready(now):options.demo||(motor_available&&motor.fresh(now)&&motor.ack().left_mask==options.left_mask&&motor.ack().invert_mask==options.invert_mask);
  controller.updateMotorLink(now,ready);
  controller.updateAngleProtocol(options.demo||motor.angleSupported());
  controller.updateArcProtocol(options.demo||motor.arcSupported());
  controller.updateVelocityProtocol(options.demo||motor.velocitySupported(),options.demo||motor.softcoreReady(now));
  ImuSample sample;sample.time_ms=imu.received_ms>=imu.sample.ag_age?imu.received_ms-imu.sample.ag_age:0;sample.calibrated=imu_fresh(&imu,now)&&imu.initialized&&!imu.calibrating;if(!sample.calibrated)sample.time_ms=now;sample.roll_deg=imu.roll;sample.pitch_deg=imu.pitch;sample.yaw_deg=imu.yaw;controller.updateImu(sample);
  // Logical authorization expires in Controller after 30 s; short link waits only zero motion.
  const State &state=controller.tick(now);
  if(motor_available&&!fpga_navigation.quiet()&&(now-sent_at>=50||force_tx.exchange(false))){
   if(state.armed&&state.twist_mode)motor.sendVelocity(true,state.linear_mps,state.angular_rps,options.left_mask,options.invert_mask,now);
   else if(state.armed&&state.arc_mode)motor.sendArc(true,state.arc_outer_permille,state.arc_curve_q8,options.left_mask,options.invert_mask,now);
   else if(state.armed&&state.angle_mode)motor.sendAngle(true,state.turn_angle_deg,state.follow_forward,options.left_mask,options.invert_mask,now);
   else if((!state.armed||state.motion_paused||state.mode==Mode::Idle)&&now-probe_at>=75){
    // A short ACK/softcore wait retains logical authority. Reconfigure only
    // with physical DISARM+ZERO, then probe armed ZERO; never replay speed.
    const bool probe_armed=state.armed&&!motor.configurationNeeded()&&motor.ack().configured;
    if(probe_version==4)motor.sendVelocity(probe_armed,0,0,options.left_mask,options.invert_mask,now);
    else if(probe_version==3)motor.sendArc(probe_armed,0,0,options.left_mask,options.invert_mask,now);
    else motor.sendAngle(probe_armed,0,0,options.left_mask,options.invert_mask,now);
    probe_version=probe_version==4?3:probe_version==3?2:4;probe_at=now;
   }
   else motor.send(state.armed,state.left,state.right,options.left_mask,options.invert_mask,now);
   sent_at=now;
  }
  auditState(now);
  if(now-report>=2000){std::string report_text;char report_line[8192];
  if(state.target_id){const auto b=controller.selectedBearing(now);std::snprintf(report_line,sizeof(report_line),"FOLLOW target=%llu mode=%s u_px=%.2f camera_deg=%.3f turn_deg=%.3f forward=%.3f valid=%d reason=%s wire=%s\n",(unsigned long long)state.target_id,modeName(state.mode),b.u_px,b.camera_deg,b.turn_deg,state.follow_forward,b.valid,b.reason.c_str(),state.angle_mode?"MOT2":"MOT1");report_text+=report_line;}
  {const auto &d=motor.diagnostics();const auto &a=motor.ack();std::snprintf(report_line,sizeof(report_line),"PHONE_CTRL interface=%s armed=%d mode=%s reason=%s out=(%.3f,%.3f) ack=(%.3f,%.3f) ack_armed=%d link_ok=%d ack_seq=%u ack_age_ms=%lld tx=%llu tx_errors=%llu rx=%llu ack_ok=%llu rejected=%llu arc_protocol=%d wire=%s wire_purpose=%s\n",interface_.empty()?"none":interface_.c_str(),state.armed,modeName(state.mode),state.reason.c_str(),state.left,state.right,a.left/1000.,a.right/1000.,a.armed,state.motor_link_fresh,a.sequence,d.last_ack_ms?(long long)(now-d.last_ack_ms):-1,(unsigned long long)d.tx_frames,(unsigned long long)d.tx_errors,(unsigned long long)d.rx_frames,(unsigned long long)d.ack_frames,(unsigned long long)d.rx_rejected,options.demo||motor.arcSupported(),state.twist_mode?"MOT4":state.arc_mode?"MOT3":state.angle_mode?"MOT2":"MOT1",state.twist_mode?"closed-loop-velocity":state.arc_mode?"representable-arc":state.angle_mode?"follow-angle":state.mode==Mode::Manual&&state.reason=="manual"?"exact-manual-sharp-or-transition":"physical-wheels");report_text+=report_line;std::snprintf(report_line,sizeof(report_line),"SOFTCORE_CTRL wire=%s supported=%d ready=%d linear_mmps=%ld yaw_rate_mradps=%ld ack_protocol=%u ack_ready=%d\n",state.twist_mode?"MOT4":"legacy",options.demo||motor.velocitySupported(),options.demo||motor.softcoreReady(now),std::lround(state.linear_mps*1000),std::lround(state.angular_rps*1000),a.protocol,a.softcore_ready);report_text+=report_line;
    try{report_text+="SOFTCORE_NAV "+motor.softcore().json(now,motor.softcoreReady(now),softcoreRequested())+"\n";}catch(...){}
    {std::lock_guard<std::mutex> diagnostic_lock(diagnostic_mutex);pending_control_report=report_text.substr(0,16384);}report=now;}}

 }std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
 static void jpegWrite(void *context,void *data,int size){auto *out=static_cast<std::vector<uint8_t>*>(context);auto *bytes=static_cast<uint8_t*>(data);out->insert(out->end(),bytes,bytes+size);}
 void publish(const vision_snapshot *v,const imu_state *i,uint64_t now){
  {std::lock_guard<std::mutex> lock(state_mutex);if(i)imu=*i;
   // Metrics contain no image copy. The JPEG encoder owns the latest image slot.
   std::memcpy(reinterpret_cast<char*>(&metrics)+offsetof(vision_snapshot,timestamp_ms),reinterpret_cast<const char*>(v)+offsetof(vision_snapshot,timestamp_ms),sizeof(*v)-offsetof(vision_snapshot,timestamp_ms));
   const bool modern=v->pair_status!=0;
   const auto control_id=modern?v->control_frame_id:v->detection_frame_id;
   const auto control_time=modern?v->control_timestamp_ms:v->detection_timestamp_ms;
   const bool control_ready=modern?v->control_available:v->detection_available;
   if(control_id&&control_id!=last_detection_id&&control_ready&&now>=control_time&&now-control_time<=VISION_BOX_MAX_AGE_MS){
    VisionFrame vf;vf.frame_id=control_id;vf.time_ms=control_time;
    vf.complete_mask=modern?v->control_complete_mask:3;vf.requested_mask=modern?v->eye_requested_mask:3;vf.control_eligible=control_ready;
    vf.geometry={v->image_geometry_kind,v->tensor_eye_width,v->tensor_eye_height,v->physical_eye_width,v->physical_eye_height,
                 v->tensor_content_x,v->tensor_content_y,v->tensor_content_width,v->tensor_content_height};
    if(v->image_geometry_kind==1&&v->preview_content_height>0){vf.geometry.viewport_y=double(v->preview_content_y)/VISION_H;vf.geometry.viewport_height=double(v->preview_content_height)/VISION_H;}
    const bool have_source=v->source_count>0;
    const int source_count=modern?v->control_count:have_source?v->source_count:v->count;
    for(int n=0;n<std::min(source_count,VISION_OBJECTS);n++){const auto &o=modern?v->control_objects[n]:have_source?v->source_objects[n]:v->objects[n];Detection d;d.class_id=o.class_id;d.name=o.name;d.score=o.score;d.eye=o.eye;d.x1=o.left;d.y1=o.top;d.x2=o.right;d.y2=o.bottom;d.source_index=n;vf.boxes.push_back(std::move(d));}
    detection_history.push_back(controller.updateVision(std::move(vf)));
    while(detection_history.size()>16)detection_history.pop_front();
    last_detection_id=control_id;
   }
   phone_result_fps=v->fps;
   auditState(now);
  }
  if(!v->have_frame||streams.load()==0)return;
  {std::lock_guard<std::mutex> lock(snapshot_mutex);if(v->frame_id==last_preview_id)return;if(have_snapshot)pending_drops++;*pending=*v;snapshot_copies++;have_snapshot=true;last_preview_id=v->frame_id;}snapshot_ready.notify_one();
 }
 void publishLoop(){auto snapshot=std::unique_ptr<vision_snapshot>(new vision_snapshot{});
  auto last_encode_start=std::chrono::steady_clock::time_point{};
  const auto tid=static_cast<id_t>(syscall(SYS_gettid));
  (void)setpriority(PRIO_PROCESS,tid,5);errno=0;int encoder_nice=getpriority(PRIO_PROCESS,tid);
  uint64_t report=clockMs(),previous_encodes=0,previous_us=0,previous_sent=0;
  while(!stopped){
   std::string control_report;{std::lock_guard<std::mutex> guard(diagnostic_mutex);control_report.swap(pending_control_report);}
   // UART/tee backpressure may delay this best-effort publisher, never the
   // motor sender or a mutex needed by the control HTTP path. Latest only.
   if(!control_report.empty())std::fwrite(control_report.data(),1,control_report.size(),stderr);
   auto now=clockMs();if(streams.load()==0)target_fps=0;
   if(now-report>=2000){
    const auto n=encodes.load(),us=encode_us.load(),sent=sent_frames.load(),jobs=n-previous_encodes;
    window_encode_fps=jobs*1000./(now-report);window_send_fps=(sent-previous_sent)*1000./(now-report);window_encode_ms=jobs?(us-previous_us)/1000./jobs:0.;
    std::fprintf(stderr,"PHONE_PERF clients=%d target_fps=%.2f result_fps=%.2f encode_fps=%.2f send_fps=%.2f send_scope=all-clients encode_ms=%.2f fps_mode=%s fps_cap=%u encodes=%llu frame_copies=%llu send_frames=%llu superseded=%llu codec=%s nice=%d stale_drops=%llu slow_disconnects=%llu send_ms=%llu source_age_ms=%llu tcp_backlog_bytes=%llu latency_policy=latest-only-250ms\n",streams.load(),target_fps.load(),phone_result_fps.load(),window_encode_fps.load(),window_send_fps.load(),window_encode_ms.load(),options.stream_fps?"capped":"auto",options.stream_fps,(unsigned long long)n,(unsigned long long)snapshot_copies.load(),(unsigned long long)sent,(unsigned long long)pending_drops.load(),jpeg_codec_name(),encoder_nice,(unsigned long long)stream_stale_drops.load(),(unsigned long long)stream_slow_disconnects.load(),(unsigned long long)stream_send_ms.load(),(unsigned long long)stream_source_age_ms.load(),(unsigned long long)stream_backlog_bytes.load());
    report=now;previous_encodes=n;previous_us=us;previous_sent=sent;
   }
   {std::unique_lock<std::mutex> lock(snapshot_mutex);snapshot_ready.wait_for(lock,std::chrono::milliseconds(100),[this]{return stopped||have_snapshot;});if(stopped)return;if(!have_snapshot)continue;
    if(streams.load()==0){have_snapshot=false;last_encode_start={};target_fps=0;continue;}
    // Re-evaluate on a newer pending frame so auto can rise from startup 1fps
    // to real throughput immediately, rather than waiting an obsolete period.
    while(!stopped&&streams.load()>0){
     const double rate=streamTargetFps(pending->detection_enabled,pending->fps,pending->preview_fps,pending->capture_fps,options.stream_fps);
     const auto due=last_encode_start+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1./rate));
     if(last_encode_start==std::chrono::steady_clock::time_point{}||std::chrono::steady_clock::now()>=due)break;
     snapshot_ready.wait_until(lock,due);
    }
    if(stopped)return;
    if(streams.load()==0){have_snapshot=false;last_encode_start={};target_fps=0;continue;}
    // Producer overwrites this one slot while throttling/encoding. Always take
    // the latest source after the wait, never accumulate JPEG work.
    *snapshot=*pending;snapshot_copies++;have_snapshot=false;
    const double target=streamTargetFps(snapshot->detection_enabled,snapshot->fps,snapshot->preview_fps,snapshot->capture_fps,options.stream_fps);
    target_fps=target;phone_result_fps=snapshot->fps;
    last_encode_start=std::chrono::steady_clock::now();
   }
   now=clockMs(); // pacing can wait; expiry/aliases must use post-wait time.
   auto next=std::make_shared<PublishedFrame>();next->id=snapshot->frame_id;next->capture_ms=snapshot->timestamp_ms;next->detection_ms=snapshot->pair_status?snapshot->control_timestamp_ms:snapshot->detection_timestamp_ms;
   {std::lock_guard<std::mutex> lock(state_mutex);
    const bool modern=snapshot->pair_status!=0;
    const auto detection_id=modern?snapshot->control_frame_id:snapshot->detection_frame_id;
    const auto detection_time=modern?snapshot->control_timestamp_ms:snapshot->detection_timestamp_ms;
    const bool available=modern?snapshot->control_available:snapshot->detection_available;
    VisionFrame vf;for(const auto &saved:detection_history)if(saved.frame_id==detection_id)vf=saved;
    auto boxes=vf.boxes;uint64_t selected=controller.state().target_id;
    // Keep the original APK's selectable layer complete-only. It has its own
    // frame/time; newer partial observations are a separate explicitly-labelled
    // channel and can never clear a selected ID or extend control freshness.
    if(detection_id==snapshot->detection_frame_id)for(auto &box:boxes){const int n=box.source_index;
     if(n>=0&&n<snapshot->count&&n<VISION_OBJECTS&&snapshot->motion_valid[n]){
      const auto &o=snapshot->objects[n];
      if(o.class_id==box.class_id&&o.eye==box.eye&&std::isfinite(o.left)&&std::isfinite(o.top)&&std::isfinite(o.right)&&std::isfinite(o.bottom)&&o.right>o.left&&o.bottom>o.top){
       box.x1=o.left;box.y1=o.top;box.x2=o.right;box.y2=o.bottom;
      }
     }
    }
    const bool fresh=vf.frame_id&&available&&now>=detection_time&&now-detection_time<=VISION_BOX_MAX_AGE_MS;
    if(!fresh)boxes.clear();else aliases.push_back({next->id,vf.frame_id,next->capture_ms});
    while(!aliases.empty()&&(aliases.size()>32||now<aliases.front().time_ms||now-aliases.front().time_ms>500))aliases.pop_front();
    std::stable_sort(boxes.begin(),boxes.end(),[selected](const Detection &a,const Detection &b){if((a.track_id==selected)!=(b.track_id==selected))return a.track_id==selected;return a.score>b.score;});
    std::ostringstream meta;meta<<std::fixed<<std::setprecision(3)<<"{\"frame_id\":"<<next->id<<",\"detections_frame_id\":"<<detection_id<<",\"detection_timestamp_ms\":"<<detection_time
     <<",\"update_id\":"<<(modern?snapshot->control_update_id:snapshot->detection_update_id)<<",\"detections_pair_status\":"<<quoteJson(detection_id?"PAIR_COMPLETE":"WAITING_COMPLETE")
     <<",\"complete_mask\":"<<(modern?snapshot->control_complete_mask:snapshot->eye_complete_mask)<<",\"control_eligible\":"<<(fresh?"true":"false")
     <<",\"source_sequence\":"<<(modern?snapshot->control_source_sequence:snapshot->detection_source_sequence)<<",\"source_epoch\":"<<(modern?snapshot->control_source_epoch:snapshot->detection_source_epoch)
     <<",\"box_max_age_ms\":"<<VISION_BOX_MAX_AGE_MS<<",\"width\":"<<VISION_W<<",\"height\":"<<VISION_H<<",\"demo\":"<<(options.demo?"true":"false")<<",\"selected_id\":"<<selected<<",\"detections\":[";
    for(size_t n=0;n<std::min<size_t>(32,boxes.size());n++){if(n)meta<<',';const auto &d=boxes[n];meta<<"{\"id\":"<<d.track_id<<",\"selectable\":true,\"label\":"<<quoteJson(d.name)<<",\"score\":"<<d.score<<",\"eye\":"<<d.eye<<",\"box\":["<<d.x1<<','<<d.y1<<','<<d.x2<<','<<d.y2<<"]}";}
    meta<<"],\"preview_pair_status\":"<<quoteJson(snapshot->pair_status==VISION_PAIR_PARTIAL?"PAIR_PARTIAL":"PAIR_COMPLETE")
     <<",\"partial_frame_id\":"<<snapshot->detection_frame_id<<",\"partial_update_id\":"<<snapshot->detection_update_id<<",\"partial_timestamp_ms\":"<<snapshot->detection_timestamp_ms
     <<",\"partial_complete_mask\":"<<snapshot->eye_complete_mask<<",\"partial_control_eligible\":false,\"counterpart_timeout\":"<<(snapshot->counterpart_timeout?"true":"false")
     <<",\"partial_source_sequence\":"<<snapshot->detection_source_sequence<<",\"partial_source_epoch\":"<<snapshot->detection_source_epoch
     <<",\"partial_eye_frame_id\":["<<snapshot->eye_frame_id[0]<<','<<snapshot->eye_frame_id[1]<<"],\"partial_eye_source_timestamp_ms\":["<<snapshot->eye_source_timestamp_ms[0]<<','<<snapshot->eye_source_timestamp_ms[1]<<"],\"partial_detections\":[";
    if(snapshot->pair_status==VISION_PAIR_PARTIAL&&snapshot->detection_available){for(int n=0;n<std::min(snapshot->count,32);++n){if(n)meta<<',';const auto &d=snapshot->objects[n];meta<<"{\"id\":0,\"selectable\":false,\"label\":"<<quoteJson(d.name)<<",\"score\":"<<d.score<<",\"eye\":"<<d.eye<<",\"box\":["<<d.left<<','<<d.top<<','<<d.right<<','<<d.bottom<<"]}";}}
    meta<<']';next->metadata_prefix=meta.str();
   }
   const auto encode_start=std::chrono::steady_clock::now();
   int encoded=jpeg_from_argb(snapshot->pixels,VISION_W,VISION_H,72,jpegWrite,&next->jpeg);
   encode_us+=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-encode_start).count();encodes++;
   const auto completed=clockMs();
   if(completed<next->capture_ms||completed-next->capture_ms>250){stream_stale_drops++;continue;}
   if(encoded&&next->jpeg.size()<1024*1024){std::lock_guard<std::mutex> lock(frame_mutex);frame=std::move(next);frame_ready.notify_all();}
  }
 }
 std::string stateJson(bool okay=true,const std::string &error="",bool include_session=false){
  const uint64_t now=clockMs();const State &s=controller.tick(now);
  const bool sensor_fresh=imu_fresh(&imu,now),valid=sensor_fresh&&imu.initialized;
  const auto &d=motor.diagnostics();const auto &ack=motor.ack();
  const auto bearing=controller.selectedBearing(now);const auto *target=controller.selectedTarget();
  const auto &geometry=controller.config().bearing;
  const int64_t received_age=imu.received_ms&&now>=imu.received_ms?int64_t(now-imu.received_ms):-1;
  const int64_t sample_age=received_age>=0?received_age+imu.sample.ag_age:-1;
  const bool fresh=sensor_fresh&&sample_age>=0&&uint64_t(sample_age)<=controller.config().imu_fresh_ms;
  const int64_t ack_age=options.demo?0:d.last_ack_ms&&now>=d.last_ack_ms?int64_t(now-d.last_ack_ms):-1;
  const char *motor_status=options.demo?"simulation_only":interface_.empty()?"interface_not_selected":!motor_available?"socket_open_failed":!d.tx_frames&&d.tx_errors?"transmit_failed":!d.ack_frames?"waiting_for_remote_fpga_ack":ack_age>300?"fpga_ack_stale":!ack.configured?"fpga_unconfigured":ack.watchdog?"fpga_watchdog":s.motor_link_fresh?"fpga_ack_verified":"waiting_for_matching_configuration";
  const char *imu_status=options.demo?"simulation_only":!imu_network.socket_open?"socket_open_failed":!imu_network.received?"waiting_for_imu_frame":!imu_network.decoded?"frame_decode_rejected":!(imu.sample.flags&1)?"sensor_data_invalid":!imu.initialized?"waiting_for_gravity":!fresh?"stale":imu.calibrating?"calibrating":"ready";
  std::ostringstream out;out<<std::fixed<<std::setprecision(4)
   <<"{\"ok\":"<<(okay?"true":"false")<<",\"connected\":true,\"protocol\":1,\"unified\":true,\"demo\":"<<(options.demo?"true":"false")
   <<",\"logical_armed\":"<<(s.armed?"true":"false")<<",\"armed\":"<<(s.armed?"true":"false")<<",\"mode\":"<<quoteJson(modeName(s.mode))<<",\"reason\":"<<quoteJson(s.reason)
   <<",\"fpga_navigation_adapter\":"<<(fpga_navigation.active()?"true":"false")<<",\"fpga_MOT_quiet\":"<<(fpga_navigation.quiet()?"true":"false")
   <<",\"heading_error_deg\":"<<s.heading_error_deg<<",\"selected_id\":"<<s.target_id<<",\"control_lease_ms\":"<<controller.config().lease_ms<<",\"motion_ttl_ms\":"<<controller.config().motion_ttl_ms<<",\"control_sequence\":"<<sequence<<",\"motion_paused\":"<<(s.motion_paused?"true":"false")<<",\"pause_reason\":"<<quoteJson(s.pause_reason)<<",\"authorization_mono_ms\":"<<s.authorization_ms<<",\"imu_action_pending\":"<<(imu_action.load()?"true":"false")
   <<",\"follow_geometry\":{\"configured\":"<<(geometry.enabled?"true":"false")<<",\"status\":"<<quoteJson(geometry.enabled?"calibrated_virtual_lookahead":"image_centering")<<",\"lookahead_m\":"<<geometry.lookahead_m<<",\"image_servo_supported\":true,\"association_mode\":"<<quoteJson(geometry.enabled?"calibrated_rays":"center_then_frontal_lidar")<<"}"
   <<",\"follow\":{\"active\":"<<(s.mode==Mode::Follow?"true":"false")<<",\"angle_mode\":"<<(s.angle_mode?"true":"false")<<",\"turn_angle_deg\":"<<s.turn_angle_deg<<",\"forward_duty\":"<<s.follow_forward
   <<",\"desired_distance_m\":"<<controller.config().follow_distance_m<<",\"distance_valid\":"<<(controller.followObservationFresh(now)&&controller.followObservation().matched?"true":"false")
   <<",\"distance_m\":"<<controller.followObservation().distance_m<<",\"bearing_right_deg\":"<<controller.followObservation().bearing_right_deg
   <<",\"target_x_body_m\":"<<controller.followObservation().x_body_m<<",\"target_y_body_m\":"<<controller.followObservation().y_body_m
   <<",\"scan_mono_ms\":"<<controller.followObservation().scan_ms<<",\"status\":"<<quoteJson(s.reason)<<"}"
   <<",\"selected_target\":";
  if(target){
   double ray_left=0,ray_right=0;const bool rays=controller.selectedRays(now,ray_left,ray_right);
   double image_error=0,image_delta=0;const bool image_valid=controller.selectedImage(now,image_error,image_delta);
   const double lateral=target->eye>=0&&target->eye<2?geometry.eyes[target->eye].lateral_right_m:0;
   out<<"{\"id\":"<<target->track_id<<",\"eye\":"<<target->eye<<",\"u_px\":"<<bearing.u_px<<",\"v_px\":"<<bearing.v_px<<",\"camera_angle_deg\":"<<bearing.camera_deg<<",\"turn_angle_deg\":"<<bearing.turn_deg<<",\"angle_valid\":"<<(bearing.valid?"true":"false")<<",\"geometry_reason\":"<<quoteJson(bearing.reason)
      <<",\"selection_mono_ms\":"<<s.selection_ms<<",\"vision_mono_ms\":"<<controller.vision().time_ms<<",\"ray_bounds_valid\":"<<(rays?"true":"false")
      <<",\"ray_left_deg\":"<<ray_left<<",\"ray_right_deg\":"<<ray_right<<",\"camera_lateral_right_m\":"<<lateral<<",\"image_valid\":"<<(image_valid?"true":"false")<<",\"image_error\":"<<image_error<<",\"yaw_since_capture_deg\":"<<image_delta<<",\"label\":"<<quoteJson(target->name)<<"}";
  }
  else out<<"null";
  out
   <<",\"motor\":{\"enabled\":"<<(motor_available?"true":"false")<<",\"link_ok\":"<<(s.motor_link_fresh?"true":"false")<<",\"configured\":"<<(s.motor_link_fresh?"true":"false")
   <<",\"interface\":"<<quoteJson(interface_)<<",\"left_mask\":"<<options.left_mask<<",\"invert_mask\":"<<options.invert_mask<<",\"left\":"<<s.left<<",\"right\":"<<s.right
   <<",\"ack_left\":"<<ack.left/1000.<<",\"ack_right\":"<<ack.right/1000.<<",\"ack_armed\":"<<(ack.armed?"true":"false")<<",\"ack_configured\":"<<(ack.configured?"true":"false")<<",\"ack_watchdog\":"<<(ack.watchdog?"true":"false")<<",\"configuration_needed\":"<<(motor.configurationNeeded()?"true":"false")<<",\"configuration_hints\":"<<d.configuration_hints<<",\"configuration_seeds\":"<<d.configuration_seeds<<",\"last_seed_sequence\":"<<d.last_seed_sequence
   <<",\"ack_protocol\":"<<(options.demo?4:ack.protocol)<<",\"angle_protocol_supported\":"<<((options.demo||motor.angleSupported())?"true":"false")
   <<",\"arc_protocol_supported\":"<<((options.demo||motor.arcSupported())?"true":"false")
   <<",\"velocity_protocol_supported\":"<<((options.demo||motor.velocitySupported())?"true":"false")
   <<",\"softcore_ready\":"<<((options.demo||motor.softcoreReady(now))?"true":"false")
   <<",\"softcore_reason\":"<<quoteJson(options.demo?"simulation_only":motor.softcoreReason(now,softcoreRequested()))
   <<",\"softcore_diagnostics\":"<<motor.softcore().json(now,motor.softcoreReady(now),softcoreRequested())
   <<",\"control_backend\":\"picorv32-v28\",\"twist_mode\":"<<(s.twist_mode?"true":"false")<<",\"linear_mps\":"<<s.linear_mps<<",\"angular_rps\":"<<s.angular_rps
   <<",\"turn_inner_ratio\":"<<controller.config().turn_inner_ratio<<",\"turn_full_deg\":"<<controller.config().turn_full_deg<<",\"straight_cone_deg\":"<<controller.config().straight_cone_deg
   <<",\"arc_mode\":"<<(s.arc_mode?"true":"false")<<",\"arc_outer_permille\":"<<s.arc_outer_permille<<",\"arc_curve_q8\":"<<s.arc_curve_q8
   <<",\"ack_sequence\":"<<ack.sequence<<",\"ack_age_ms\":"<<ack_age<<",\"tx_frames\":"<<d.tx_frames<<",\"tx_errors\":"<<d.tx_errors<<",\"rx_frames\":"<<d.rx_frames<<",\"ack_frames\":"<<d.ack_frames<<",\"rx_rejected\":"<<d.rx_rejected<<",\"last_errno\":"<<d.last_errno
   <<",\"status\":"<<quoteJson(motor_status)<<",\"open_error\":"<<quoteJson(motor_error_)<<"}"
   <<",\"imu\":{\"valid\":"<<(valid?"true":"false")<<",\"fresh\":"<<(fresh?"true":"false")<<",\"packet_received\":"<<(imu.have_packet?"true":"false")<<",\"initialized\":"<<(imu.initialized?"true":"false")<<",\"bias_valid\":"<<(imu.bias_valid?"true":"false")
   <<",\"interface\":"<<quoteJson(imu_interface_)<<",\"socket_open\":"<<(imu_network.socket_open?"true":"false")<<",\"last_errno\":"<<imu_network.last_errno
   <<",\"rx_frames\":"<<imu_network.received<<",\"decoded_frames\":"<<imu_network.decoded<<",\"accepted_frames\":"<<imu_network.accepted<<",\"last_frame_bytes\":"<<imu_network.last_frame_bytes
   <<",\"flags\":"<<imu.sample.flags<<",\"sensor_flags\":"<<imu_network.sensor_flags<<",\"who_am_i\":"<<imu_network.who_am_i<<",\"i2c_address\":"<<imu_network.i2c_address
   <<",\"sequence\":"<<imu.sample.sequence<<",\"timestamp_ms\":"<<imu.sample.timestamp<<",\"sensor_age_ms\":"<<imu.sample.ag_age<<",\"received_age_ms\":"<<received_age<<",\"sample_age_ms\":"<<sample_age<<",\"errors\":"<<imu.sample.errors<<",\"drops\":"<<imu.sample.drops
   <<",\"roll\":"<<imu.roll<<",\"pitch\":"<<imu.pitch<<",\"yaw\":"<<imu.yaw<<",\"accel\":["<<imu.sample.accel[0]*IMU_G<<','<<imu.sample.accel[1]*IMU_G<<','<<imu.sample.accel[2]*IMU_G<<"]"
   <<",\"calibrating\":"<<(imu.calibrating?"true":"false")<<",\"calibration_percent\":"<<imu.calibration/2<<",\"status\":"<<quoteJson(imu_status)<<"}"
   <<",\"vision\":{\"valid\":"<<(s.vision_fresh?"true":"false")<<",\"fps\":"<<metrics.fps<<",\"result_fps\":"<<metrics.fps<<",\"capture_fps\":"<<metrics.capture_fps<<",\"preview_fps\":"<<metrics.preview_fps<<",\"inference_ms\":"<<metrics.inference_ms
   <<",\"npu_workers\":"<<metrics.npu_workers<<",\"stereo\":"<<quoteJson(metrics.npu_workers==3?"pipeline3":metrics.npu_workers==2?"parallel":"serial")<<",\"detection_enabled\":"<<(metrics.detection_enabled?"true":"false")<<",\"phone_fps_limit\":"<<options.stream_fps
   <<",\"preview_pair_status\":"<<quoteJson(metrics.pair_status==VISION_PAIR_PARTIAL?"PAIR_PARTIAL":"PAIR_COMPLETE")<<",\"detections_pair_status\":"<<quoteJson(metrics.control_frame_id?"PAIR_COMPLETE":"WAITING_COMPLETE")<<",\"complete_mask\":"<<(metrics.pair_status?metrics.control_complete_mask:metrics.eye_complete_mask)<<",\"control_eligible\":"<<(metrics.pair_status?metrics.control_available?"true":"false":metrics.control_eligible?"true":"false")<<",\"update_id\":"<<(metrics.pair_status?metrics.control_update_id:metrics.detection_update_id)<<",\"source_sequence\":"<<(metrics.pair_status?metrics.control_source_sequence:metrics.detection_source_sequence)<<",\"source_epoch\":"<<(metrics.pair_status?metrics.control_source_epoch:metrics.detection_source_epoch)<<",\"partial_complete_mask\":"<<metrics.eye_complete_mask<<",\"partial_update_id\":"<<metrics.detection_update_id<<",\"model\":"<<quoteJson(model_.substr(model_.find_last_of('/')+1))<<",\"width\":"<<options.camera_width<<",\"height\":"<<options.camera_height<<",\"frame_id\":"<<metrics.frame_id<<",\"detections_frame_id\":"<<(metrics.pair_status?metrics.control_frame_id:metrics.detection_frame_id)<<",\"partial_frame_id\":"<<metrics.detection_frame_id<<",\"status\":"<<quoteJson(metrics.status)<<"}"
   <<",\"phone\":{\"video_clients\":"<<streams.load()<<",\"target_fps\":"<<target_fps.load()<<",\"result_fps\":"<<phone_result_fps.load()<<",\"encode_fps\":"<<window_encode_fps.load()<<",\"send_fps\":"<<window_send_fps.load()<<",\"send_scope\":\"all-clients\",\"encode_ms\":"<<window_encode_ms.load()<<",\"fps_mode\":"<<quoteJson(options.stream_fps?"capped":"auto")<<",\"fps_cap\":"<<options.stream_fps<<",\"encodes\":"<<encodes.load()<<",\"snapshot_copies\":"<<snapshot_copies.load()<<",\"sent_frames\":"<<sent_frames.load()<<",\"superseded\":"<<pending_drops.load()<<",\"encode_total_ms\":"<<encode_us.load()/1000.<<",\"stale_drops\":"<<stream_stale_drops.load()<<",\"slow_disconnects\":"<<stream_slow_disconnects.load()<<",\"send_ms\":"<<stream_send_ms.load()<<",\"source_age_ms\":"<<stream_source_age_ms.load()<<",\"tcp_backlog_bytes\":"<<stream_backlog_bytes.load()<<",\"codec\":"<<quoteJson(jpeg_codec_name())<<"}"
   <<",\"navigation\":{\"ready\":false,\"reason\":\"encoder/range_not_connected\"}";
  if(include_session)out<<",\"session\":"<<quoteJson(session_);
  if(!error.empty())out<<",\"error\":"<<quoteJson(error);
  out<<'}';return out.str();
 }
 void abandonSlowVideo(int fd){
  stream_slow_disconnects++;
  // Abort only this video socket. A graceful FIN would still flush queued
  // historical bytes after returning from this handler.
  linger reset{1,0};(void)setsockopt(fd,SOL_SOCKET,SO_LINGER,&reset,sizeof(reset));
 }
 void video(int fd){
  // One shared encoder serves phone, semantic search and PPE voice.
  if(streams.fetch_add(1)>=3){streams--;httpReply(fd,429,"{\"ok\":false,\"error\":\"video_client_limit\"}");return;}
  struct Guard{std::atomic<int> &n;~Guard(){n--;}}guard{streams};
  // The old default autotuned send buffer held multiple seconds of MJPEG even
  // though the user-space publisher had just one slot. Bound both the buffer
  // and the complete-frame send time. A video reconnect never touches arm.
  // Linux doubles SO_SNDBUF. 128 KiB permits a single detailed JPEG without
  // delayed-ACK stalls, while the deadline still prevents a frame FIFO.
  int capacity=131072,one=1;
  (void)setsockopt(fd,SOL_SOCKET,SO_SNDBUF,&capacity,sizeof(capacity));
  (void)setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
#ifdef TCP_NOTSENT_LOWAT
  unsigned lowat=1024;(void)setsockopt(fd,IPPROTO_TCP,TCP_NOTSENT_LOWAT,&lowat,sizeof(lowat));
#endif
#ifdef TCP_USER_TIMEOUT
  unsigned timeout=500;(void)setsockopt(fd,IPPROTO_TCP,TCP_USER_TIMEOUT,&timeout,sizeof(timeout));
#endif
  const std::string head="HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=robotframe\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";if(!writeAll(fd,head.data(),head.size()))return;
  uint64_t last=0;while(!stopped){std::shared_ptr<PublishedFrame> image;{std::unique_lock<std::mutex> lock(frame_mutex);frame_ready.wait_for(lock,std::chrono::milliseconds(200),[this,last]{return stopped||(frame&&frame->id!=last);});if(stopped)return;if(!frame||frame->id==last)continue;image=frame;}
   uint64_t now=clockMs();last=image->id;
   if(now<image->capture_ms||now-image->capture_ms>250){stream_stale_drops++;continue;}
   int queued=0;if(ioctl(fd,SIOCOUTQ,&queued)==0)stream_backlog_bytes=std::max(0,queued);
   if(queued>262144){abandonSlowVideo(fd);return;}
   stream_source_age_ms=now-image->capture_ms;
   std::string metadata=image->metadata_prefix+",\"capture_mono_ms\":"+std::to_string(image->capture_ms)+",\"server_send_mono_ms\":"+std::to_string(now)+",\"max_transport_lag_ms\":250,\"age_ms\":"+std::to_string(now-image->capture_ms)+",\"box_age_ms\":"+std::to_string(image->detection_ms&&now>=image->detection_ms?now-image->detection_ms:999999)+'}';std::ostringstream h;h<<"--robotframe\r\nContent-Type: image/jpeg\r\nContent-Length: "<<image->jpeg.size()<<"\r\nX-Frame-Id: "<<image->id<<"\r\nX-Frame-Meta: "<<base64(metadata)<<"\r\n\r\n";auto header=h.str();const auto deadline=std::min(now+200,image->capture_ms+300);
   if(!writeAllBefore(fd,header.data(),header.size(),deadline)||!writeAllBefore(fd,image->jpeg.data(),image->jpeg.size(),deadline)||!writeAllBefore(fd,"\r\n",2,deadline)){abandonSlowVideo(fd);return;}
   stream_send_ms=clockMs()-now;sent_frames++;
  }
 }
 void request(int fd,const HttpRequest &r){
  sockaddr_in peer={};socklen_t peer_size=sizeof(peer);
  const bool local_peer=getpeername(fd,reinterpret_cast<sockaddr*>(&peer),&peer_size)==0&&peer.sin_family==AF_INET&&(ntohl(peer.sin_addr.s_addr)>>24)==127;
  const bool private_path=r.path.compare(0,16,"/api/navigation/")==0;
  if(private_path&&!local_peer){httpReply(fd,403,"{\"ok\":false,\"error\":\"navigation_internal_only\"}");return;}
  auto auth=r.headers.find("authorization");int gate=0;{std::lock_guard<std::mutex> lock(state_mutex);uint64_t now=clockMs();if(now<auth_block_until)gate=429;else if(auth==r.headers.end()||!secretEqual(auth->second,"Bearer "+token_)){if(now-auth_window>60000){auth_window=now;auth_failures=0;}if(++auth_failures>=10){auth_block_until=now+30000;auth_failures=0;}gate=401;}else last_client_ms=now;}
  if(gate){{std::lock_guard<std::mutex> lock(state_mutex);audit.command(clockMs(),"authentication",0,gate,gate==401?"authentication_rejected":"authentication_rate_limited",controller.state().armed,controller.state());}httpReply(fd,gate,"{\"ok\":false,\"error\":\"pairing_token_required\"}");return;}
  if(r.method=="GET"&&r.path=="/api/video.mjpg"){video(fd);return;}
  std::string response;int code=200;{std::lock_guard<std::mutex> lock(state_mutex);uint64_t now=clockMs();
   const bool before_armed=controller.state().armed;const std::string before_reason=controller.state().reason;std::string audit_op="unknown",audit_cause;uint64_t audit_sequence=0;bool audit_request=r.method=="POST",audit_internal=false;ClientReportedDiagnostic client_reported;
   if(r.method=="POST"&&r.path=="/api/navigation/follow-observation"){
    JsonObject input;std::string error;FollowObservation o;bool okay=parseFlatJson(r.body,input,error);
    auto stamp=[&](const char*key,uint64_t&v){double n=0;if(!jsonNumber(input,key,n)||n<1||n>9007199254740991.||std::floor(n)!=n)return false;v=uint64_t(n);return true;};
    okay=okay&&stamp("target_id",o.target_id)&&stamp("selection_ms",o.selection_ms)&&stamp("vision_ms",o.vision_ms)&&stamp("scan_ms",o.scan_ms)&&stamp("pose_ms",o.pose_ms)&&
        jsonBool(input,"matched",o.matched)&&jsonBool(input,"forward_clear",o.forward_clear)&&jsonBool(input,"reverse_clear",o.reverse_clear)&&jsonBool(input,"turn_clear",o.turn_clear)&&
        jsonNumber(input,"distance_m",o.distance_m)&&jsonNumber(input,"bearing_right_deg",o.bearing_right_deg)&&
        jsonNumber(input,"x_body_m",o.x_body_m)&&jsonNumber(input,"y_body_m",o.y_body_m)&&controller.updateFollowObservation(o,now);
    code=okay?200:409;response=okay?"{\"ok\":true}":"{\"ok\":false,\"error\":\"follow_observation_rejected\"}";
    // Observing is read-only with respect to owner, sequence and motion lease.
    audit_request=false;
   }else if(r.path.compare(0,21,"/api/navigation/fpga/")==0){
    const bool stopped_mode=controller.state().mode==Mode::Idle||controller.state().mode==Mode::Disarmed;
    // Quiet preparation after End grants no motion. Claim/START still require
    // the current phone owner to explicitly ARM and obtain a real NVA ACK.
    const bool was_quiet=fpga_navigation.quiet();
    if(!fpga_navigation.request(r.method,r.path,r.body,session_,stopped_mode,now,code,response)){
     code=503;response="{\"ok\":false,\"error\":\"fpga_adapter_not_compiled\"}";
    }else if(code==200&&!was_quiet&&fpga_navigation.quiet()){
     // Complete a physical DISARM+ZERO once before suppressing periodic probes.
     // Otherwise quiet could overtake controlLoop's pending force_tx. The FPGA
     // still requires its own measured standstill and COMMIT acknowledgement.
     if(!motor_available||!motor.send(false,0,0,options.left_mask,options.invert_mask,now)){
      fpga_navigation.takeover();force_tx=true;code=503;
      response="{\"ok\":false,\"error\":\"quiet_entry_stop_send_failed\"}";
     }else std::fprintf(stderr,"FPGA_QUIET entered disarm_zero_sent=1 repeated_MOT_probes=0\n");
    }
   }else if(r.method=="GET"&&r.path=="/api/state")response=stateJson();
   else if(r.method=="POST"&&r.path=="/api/connect"){
    audit_op="connect";
    if(!session_.empty()&&controller.tick(now).armed&&now>=owner_control_ms&&now-owner_control_ms<controller.config().lease_ms){code=409;response=stateJson(false,"controller_busy");}
    else{fpga_navigation.takeover();controller=Controller(config(options));last_detection_id=0;aliases.clear();detection_history.clear();session_=randomHex(16);navigation_ticket_.clear();sequence=0;owner_control_ms=now;force_tx=true;response=stateJson(true,"",true);}
   }else if(r.method=="POST"&&(r.path=="/api/control"||r.path=="/api/navigation/wheels"||r.path=="/api/navigation/claim"||r.path=="/api/navigation/twist"||r.path=="/api/navigation/lease"||r.path=="/api/navigation/pause")){
    const bool navigation_path=r.path!="/api/control";
    audit_internal=navigation_path;
    JsonObject input;std::string error,session,op;double seq_value=0;
    if(!parseFlatJson(r.body,input,error)||!jsonString(input,"session",session)||!jsonString(input,"op",op)||!jsonNumber(input,"seq",seq_value)||seq_value<1||seq_value>9007199254740991.||std::floor(seq_value)!=seq_value){code=400;audit_cause="invalid_control_request";response=stateJson(false,"invalid_control_request");}
    else if(session_.empty()||!secretEqual(session,session_)){code=403;audit_cause="not_control_owner";audit_op=ControlAudit::operation(op);audit_sequence=seq_value;response=stateJson(false,"not_control_owner");}
    else{client_reported=ControlAudit::clientReported(input);uint64_t seq=seq_value;audit_op=ControlAudit::operation(op);audit_sequence=seq;bool stop_op=op=="stop"||op=="disarm"||op=="disconnect";if(seq<=sequence&&!stop_op){code=409;audit_cause="stale_sequence";response=stateJson(false,"stale_sequence");}
     else{sequence=std::max(sequence,seq);bool okay=true;
      double source_value=double(now);const bool trusted_source=local_peer&&input.count("owner_command_ms");
      if(trusted_source&&(!jsonNumber(input,"owner_command_ms",source_value)||source_value<0||source_value>double(now)||std::floor(source_value)!=source_value))okay=false;
      const uint64_t source_ms=okay?uint64_t(source_value):0;
      controller.commandSourceTime(trusted_source?source_ms:UINT64_MAX);
      struct SourceGuard {Controller& controller;~SourceGuard(){controller.commandSourceTime();}} source_guard{controller};
      if(trusted_source&&(!stop_op&&op!="pause")&&now-source_ms>=controller.config().lease_ms)okay=false;
      if(!navigation_path)navigation_ticket_.clear();
      if(!okay)error="invalid_owner_command_time";
      else if(navigation_path){
       if(r.path=="/api/navigation/claim"&&op=="claim"&&controller.state().armed&&controller.state().mode==Mode::Idle){
        okay=(options.demo||motor.velocitySupported())&&controller.driveVelocity(seq,0,0,now,source_ms);
        if(!okay)error="fpga_v28_velocity_required";
        if(okay){navigation_ticket_=randomHex(24);navigation_ticket_sequence_=seq;}
       } else if(r.path=="/api/navigation/twist"&&op=="twist"){
        double linear=0,angular=0;std::string ticket;
        okay=!navigation_ticket_.empty()&&jsonString(input,"navigation_ticket",ticket)&&secretEqual(ticket,navigation_ticket_)&&seq>navigation_ticket_sequence_&&jsonNumber(input,"linear_mps",linear)&&jsonNumber(input,"angular_rps",angular);
        if(okay){okay=controller.driveVelocity(seq,linear,angular,now,source_ms);navigation_ticket_sequence_=seq;}
        if(!okay)error=controller.state().reason=="softcore_not_ready"?"softcore_not_ready":"navigation_velocity_or_ticket_invalid";
       } else if(r.path=="/api/navigation/lease"&&op=="lease"){
        okay=trusted_source&&controller.lease(seq,now,source_ms);
      } else if(r.path=="/api/navigation/pause"&&op=="pause"){
        std::string cause;jsonString(input,"reason",cause);controller.pause(cause.empty()?"navigation_paused":cause);navigation_ticket_.clear();
      } else if(r.path=="/api/navigation/wheels"&&op=="wheels"){
        // Old private endpoint cannot silently bypass the physical-unit contract.
        okay=false;error="fpga_v28_velocity_required";
       } else okay=false;
      }
      else if(op=="arm")okay=controller.arm(seq,now);
      else if(op=="drive"){double throttle=0,steer=0;bool hold=false;if(!jsonNumber(input,"throttle",throttle)||!jsonNumber(input,"steer",steer)||std::abs(throttle)>1||std::abs(steer)>1)okay=false;else{if(input.count("hold")&&!jsonBool(input,"hold",hold))okay=false;if(okay)okay=controller.drive(seq,throttle,steer,hold,now);}}
      else if(op=="rotate"){double speed=0;std::string direction;okay=jsonNumber(input,"speed",speed)&&jsonString(input,"direction",direction)&&(direction=="cw"||direction=="ccw")&&speed>=0&&speed<=1;if(okay)okay=controller.rotate(seq,direction=="cw",speed,now);}
      else if(op=="idle"){okay=controller.idle(seq,now,source_ms);}
      else if(op=="select"){double display=0,target=0;okay=jsonNumber(input,"frame_id",display)&&jsonNumber(input,"target_id",target)&&display>=1&&target>=1&&display<=9007199254740991.&&target<=9007199254740991.&&std::floor(display)==display&&std::floor(target)==target;uint64_t detection=0;if(okay)for(const auto &a:aliases)if(a.preview_id==uint64_t(display)&&now>=a.time_ms&&now-a.time_ms<=500)detection=a.detection_id;if(okay)okay=detection&&controller.select(detection,target,now);}
      else if(op=="follow")okay=controller.follow(seq,now);
      else if(op=="stop"){bool emergency=false;if(input.count("emergency")&&!jsonBool(input,"emergency",emergency))okay=false;if(okay)controller.stop(now,emergency);}
      else if(op=="disarm")controller.disarm(now);
      else if(op=="disconnect"){controller.ownerDisconnected(now);navigation_ticket_.clear();session_.clear();}
      else if(op=="zero"||op=="calibrate"){controller.pause(op=="zero"?"imu_zero":"imu_calibrate");if(op=="zero"&&(!imu_fresh(&imu,now)||!imu.initialized))okay=false;else imu_action.fetch_or(op=="zero"?1u:2u);}
      else if(op=="configure"){double mask=0,invert=0;okay=jsonNumber(input,"left_mask",mask)&&jsonNumber(input,"invert_mask",invert)&&mask>=1&&mask<=14&&invert>=0&&invert<=15&&std::floor(mask)==mask&&std::floor(invert)==invert;if(okay){unsigned bits=0;for(unsigned k=mask;k;k>>=1)bits+=k&1;okay=bits==2;}if(okay){bool changed=options.left_mask!=unsigned(mask)||options.invert_mask!=unsigned(invert);options.left_mask=mask;options.invert_mask=invert;okay=controller.configure(config(options));if(changed)controller.updateMotorLink(now,false);last_detection_id=0;aliases.clear();detection_history.clear();}}
      else{okay=false;error=op=="navigate"||op=="explore"?"navigation_hardware_not_connected":"unknown_operation";}
      if(!okay){audit_cause=controller.state().reason;
       if(audit_cause==before_reason||audit_cause=="disarmed"){
        if(audit_op=="unknown")audit_cause="unknown_operation";
        else if(navigation_path)audit_cause=op=="claim"?"navigation_claim_requires_armed_idle":"navigation_ticket_or_wheels_invalid";
        else if(op=="arm")audit_cause=!controller.state().foreground?"background":!controller.state().motor_link_fresh?"motor_link_not_ready":"arm_rejected";
        else if((op=="drive"||op=="rotate"||op=="follow")&&!controller.state().armed)audit_cause="motor_not_armed";
        else if(op=="drive")audit_cause="invalid_drive_parameters";
        else if(op=="rotate")audit_cause="invalid_rotate_parameters";
        else if(op=="select")audit_cause="selection_frame_or_target_invalid";
        else if(op=="configure")audit_cause="invalid_wheel_configuration";
        else if(op=="zero")audit_cause="imu_not_ready_for_zero";
        else audit_cause="command_rejected";
       }
       if(op=="follow")error=controller.state().reason;
       if(op=="drive"&&controller.state().reason=="imu_stale")error="imu_not_ready_for_heading_hold";
       if(op=="drive"&&(controller.state().reason=="arc_protocol_waiting"||controller.state().reason=="arc_protocol_lost"))error=controller.state().reason;
       if(op=="drive"||op=="rotate"||navigation_path)controller.pause(error.empty()?"command_rejected":error);
       code=409;response=stateJson(false,error.empty()?"command_rejected_"+op:error);
      }else {
       if(!navigation_path&&(op=="drive"||op=="rotate"||op=="follow"||op=="stop"||op=="disarm"||op=="disconnect"||op=="configure"||op=="zero"||op=="calibrate"))fpga_navigation.takeover();
       // Only an accepted owner command renews authorization. Internal navigation
       // repeats carry the original phone-arrival timestamp and cannot extend it.
       if(op!="pause"&&!stop_op){
        if(!trusted_source||source_ms>owner_control_ms)owner_control_ms=source_ms;
        if(controller.state().armed)controller.renewAuthorization(source_ms,now);
       }
       fpga_navigation.owner(session_,sequence,owner_control_ms,controller.state().armed,now);
       response=stateJson();
       if(navigation_path&&op=="claim")response.insert(response.size()-1,",\"navigation_ticket\":"+quoteJson(navigation_ticket_));
      }force_tx=true;
     }
    }
   }else{code=404;audit_cause="endpoint_not_found";response="{\"ok\":false,\"error\":\"not_found\"}";}
   if(audit_request){if(code>=400&&audit_cause.empty())audit_cause=code==409&&audit_op=="connect"?"controller_busy":"request_rejected";audit.command(now,audit_op,audit_sequence,code,audit_cause,before_armed,controller.state(),audit_internal,client_reported);}
   auditState(now);
  }httpReply(fd,code,response);
 }
};

extern "C" phone_service *phone_start(const phone_options *o,char *error,unsigned length){try{if(!o||o->port<1024||o->port>65535||o->left_mask<1||o->left_mask>14||o->invert_mask>15||o->stream_fps>60)throw std::runtime_error("Invalid phone options");unsigned bits=0;for(unsigned m=o->left_mask;m;m>>=1)bits+=m&1;if(bits!=2)throw std::runtime_error("Left mask must select exactly two wheels");auto s=std::unique_ptr<phone_service>(new phone_service(*o));std::string e;if(!s->start(e))throw std::runtime_error(e);return s.release();}catch(const std::exception &e){if(error&&length)std::snprintf(error,length,"%s",e.what());return nullptr;}}
extern "C" int phone_attach_vision(phone_service *s,vision_worker *v){return s&&s->attach(v);}
extern "C" void phone_update_imu(phone_service *s,const imu_state *i){if(s&&i){std::lock_guard<std::mutex> lock(s->state_mutex);s->imu=*i;}}
extern "C" void phone_publish(phone_service *s,const vision_snapshot *v,const imu_state *i,uint64_t now){if(s&&v&&i)s->publish(v,i,now);}
extern "C" void phone_update_sensors(phone_service *s,const sensor_state *sensors,uint64_t now){
 if(!s||!sensors)return;
 const auto ranges=followRanges(*sensors,now);
 std::lock_guard<std::mutex> lock(s->state_mutex);s->controller.updateRanges(ranges);
}
extern "C" void phone_update_imu_network(phone_service *s,const phone_imu_diagnostics *d){if(s&&d){std::lock_guard<std::mutex> lock(s->state_mutex);s->imu_network=*d;s->imu_interface_=d->interface_name?d->interface_name:"";s->imu_network.interface_name=nullptr;}}
extern "C" unsigned phone_take_imu_action(phone_service *s){return s?s->imu_action.exchange(0):0;}
extern "C" void phone_reset_imu_reference(phone_service *s,uint64_t now){if(s){std::lock_guard<std::mutex> lock(s->state_mutex);s->controller.resetYawReference(now);s->auditState(now);s->force_tx=true;}}
extern "C" void phone_disarm(phone_service *s){if(s){std::lock_guard<std::mutex> lock(s->state_mutex);const auto now=clockMs();const bool before=s->controller.state().armed;s->fpga_navigation.takeover();s->controller.disarm(now);s->audit.command(now,"disarm",s->sequence,200,"local_runtime_disarm",before,s->controller.state());s->auditState(now);s->force_tx=true;}}
extern "C" void phone_get_status(phone_service *s,phone_status *status){if(!status)return;*status={};if(!s)return;std::lock_guard<std::mutex> lock(s->state_mutex);const auto now=clockMs();const auto &v=s->controller.tick(now);s->auditState(now);status->connected=!s->session_.empty()&&now>=s->last_client_ms&&(now-s->last_client_ms<30000||s->streams.load()>0);status->armed=v.armed;status->port=s->options.port;status->left=v.left;status->right=v.right;std::snprintf(status->pair_code,sizeof(status->pair_code),"%s",s->token_.c_str());std::snprintf(status->address,sizeof(status->address),"%s",s->address_.c_str());}
extern "C" void phone_stop(phone_service *s){delete s;}
