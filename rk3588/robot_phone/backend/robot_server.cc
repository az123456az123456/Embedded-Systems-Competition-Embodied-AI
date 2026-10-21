#include "controller.h"
#include "http_server.h"
#include "json_flat.h"
#include "motor_link.h"
#include "jpeg_codec.h"
#include "perception/vision.h"
extern "C" {
#include "perception/imu_data.h"
}
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <ifaddrs.h>
#include <iostream>
#include <linux/if_packet.h>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

using namespace robotphone;
static volatile sig_atomic_t interrupted=0;
static void interruptHandler(int){interrupted=1;}
static uint64_t clockMs(){return imu_clock_ms();}
static double wrapDegrees(double x){return std::remainder(x,360.);}
static std::string randomHex(size_t count){
 std::vector<unsigned char> bytes(count);int fd=open("/dev/urandom",O_RDONLY|O_CLOEXEC);if(fd<0)throw std::runtime_error("Cannot open random source");
 size_t at=0;while(at<count){ssize_t n=read(fd,bytes.data()+at,count-at);if(n<0&&errno==EINTR)continue;if(n<=0){close(fd);throw std::runtime_error("Random source failed");}at+=n;}close(fd);
 static const char hex[]="0123456789abcdef";std::string out;for(auto b:bytes){out+=hex[b>>4];out+=hex[b&15];}return out;
}
static std::string loadToken(const std::string &path){
 std::ifstream file(path);std::string token;
 if(file){std::getline(file,token);if(!token.empty()&&token.back()=='\r')token.pop_back();}
 else {
  token=randomHex(12);int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);if(fd<0)throw std::runtime_error("Cannot create pairing token file: "+path);
  std::string bytes=token+'\n';ssize_t n=write(fd,bytes.data(),bytes.size());int saved=errno;close(fd);if(n!=static_cast<ssize_t>(bytes.size()))throw std::runtime_error("Cannot write token file: "+std::string(strerror(saved)));
 }
 if(token.size()<8||token.size()>128||token.find_first_of(" \t\r\n\0",0,5)!=std::string::npos)throw std::runtime_error("Pairing token must be 8..128 non-space characters");
 return token;
}
static bool secretEqual(const std::string &a,const std::string &b){unsigned diff=a.size()^b.size();size_t n=std::max(a.size(),b.size());for(size_t i=0;i<n;i++)diff|=(i<a.size()?static_cast<unsigned char>(a[i]):0)^(i<b.size()?static_cast<unsigned char>(b[i]):0);return diff==0;}
static std::string base64(const std::string &s){
 static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";std::string out;
 for(size_t i=0;i<s.size();i+=3){unsigned n=static_cast<unsigned char>(s[i])<<16;if(i+1<s.size())n|=static_cast<unsigned char>(s[i+1])<<8;if(i+2<s.size())n|=static_cast<unsigned char>(s[i+2]);out+=alphabet[n>>18];out+=alphabet[(n>>12)&63];out+=i+1<s.size()?alphabet[(n>>6)&63]:'=';out+=i+2<s.size()?alphabet[n&63]:'=';}return out;
}
struct Options {
 std::string model="model/model.rknn",labels="model/coco_80_labels_list.txt",device="/dev/video41",view="both",interface,bind="0.0.0.0",token_file="pair-token.txt";
 unsigned port=8080,left_mask=3,invert_mask=12;int width=1920,height=1080,seconds=0;float confidence=.35f,nms=.45f;bool demo=false;
};
struct PublishedFrame {
 uint64_t id=0,capture_ms=0;std::vector<uint8_t> jpeg;std::string metadata_prefix;
};
class RobotApp {
public:
 explicit RobotApp(Options options):options_(std::move(options)),controller_(makeConfig(options_)){
  imu_init(&imu_);token_=loadToken(options_.token_file);
 }
 ~RobotApp(){stop();}
 bool start(std::string &error){
  if(!options_.demo){
   char name[16]={};imu_fd_=imu_open_socket(options_.interface.empty()?nullptr:options_.interface.c_str(),name);interface_=name;
   if(imu_fd_<0){std::cerr<<"IMU network unavailable: "<<strerror(errno)<<"; check root and --interface\n";}
   if(!interface_.empty()){motor_available_=motor_.open(interface_,motor_error_);if(!motor_available_)std::cerr<<"Motor link: "<<motor_error_<<'\n';}
   vision_options vo={options_.model.c_str(),options_.device.c_str(),options_.labels.c_str(),options_.view.c_str(),options_.width,options_.height,options_.confidence,options_.nms};
   vision_=vision_start(&vo);if(!vision_){error="Cannot start vision worker";return false;}
  }
  if(!http_.start(options_.bind,options_.port,[this](int fd,const HttpRequest &r){request(fd,r);},error))return false;
  stopped_=false;started_=clockMs();control_thread_=std::thread(&RobotApp::controlLoop,this);publisher_=std::thread(&RobotApp::publishLoop,this);
  std::cout<<"Robot phone server port "<<options_.port<<" / "<<(options_.demo?"DEMO: no physical motors":"FPGA interface "+interface_)<<"\nPairing token: "<<token_<<"\n";
  ifaddrs *addresses=nullptr;if(getifaddrs(&addresses)==0){for(auto *a=addresses;a;a=a->ifa_next)if(a->ifa_addr&&a->ifa_addr->sa_family==AF_INET&&std::strcmp(a->ifa_name,"lo")){char ip[INET_ADDRSTRLEN];auto *addr=reinterpret_cast<sockaddr_in*>(a->ifa_addr);if(inet_ntop(AF_INET,&addr->sin_addr,ip,sizeof(ip)))std::cout<<a->ifa_name<<": http://"<<ip<<':'<<options_.port<<"\n";}freeifaddrs(addresses);}
  std::cout.flush();return true;
 }
 void stop(){
  if(stopped_.exchange(true)){if(vision_){vision_stop(vision_);vision_=nullptr;}if(imu_fd_>=0){close(imu_fd_);imu_fd_=-1;}return;}
  frame_ready_.notify_all();http_.stop();if(publisher_.joinable())publisher_.join();if(control_thread_.joinable())control_thread_.join();
  for(int i=0;i<3;i++)if(motor_available_)motor_.send(false,0,0,options_.left_mask,options_.invert_mask,clockMs());
  motor_.close();if(vision_){vision_stop(vision_);vision_=nullptr;}if(imu_fd_>=0){close(imu_fd_);imu_fd_=-1;}
 }
private:
 Options options_;Controller controller_;imu_state imu_={};MotorLink motor_;HttpServer http_;vision_worker *vision_=nullptr;
 std::string token_,session_,interface_,motor_error_,vision_error_="Starting camera...";uint64_t sequence_=0,owner_control_ms_=0,started_=0,frame_number_=0,last_capture_=0;
 int imu_fd_=-1;bool motor_available_=false;double reference_[3]={};float fps_=0;
 std::atomic<bool> stopped_{true},force_tx_{false};std::atomic<int> streams_{0};std::thread control_thread_,publisher_;
 std::mutex state_mutex_,frame_mutex_;std::condition_variable frame_ready_;std::shared_ptr<PublishedFrame> frame_;
 static Config makeConfig(const Options &o){Config c;c.left_motor_mask=o.left_mask;return c;}
 void receiveImu(){
  if(imu_fd_<0)return;
  for(int k=0;k<128;k++){
   uint8_t bytes[2048];sockaddr_ll from={};socklen_t size=sizeof(from);ssize_t n=recvfrom(imu_fd_,bytes,sizeof(bytes),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&from),&size);
   if(n<0)break;
   if(from.sll_pkttype==PACKET_OUTGOING)continue;
   imu_sample sample;
   if(imu_decode(bytes,n,&sample)){std::lock_guard<std::mutex> lock(state_mutex_);imu_update(&imu_,&sample,clockMs());}
  }
 }
 void demoImu(uint64_t now){
  double t=(now-started_)/1000.;imu_sample s={};s.flags=15;s.ag_age=0;s.temperature=26.4;s.sequence=(now-started_)/10+1;s.timestamp=now-started_;s.uptime=now-started_;s.accel[0]=.02*sin(t);s.accel[1]=.03*cos(t);s.accel[2]=sqrt(1-s.accel[0]*s.accel[0]-s.accel[1]*s.accel[1]);
  imu_update(&imu_,&s,now);imu_.calibrating=0;
 }
 void controlLoop(){
  uint64_t sent_at=0;
  while(!stopped_){
   uint64_t now=clockMs();receiveImu();motor_.receive(now);
   {std::lock_guard<std::mutex> lock(state_mutex_);
    if(options_.demo)demoImu(now);
    bool ready=options_.demo||(motor_available_&&motor_.fresh(now)&&motor_.ack().left_mask==options_.left_mask&&motor_.ack().invert_mask==options_.invert_mask);
    controller_.updateMotorLink(now,ready);
    ImuSample sample;sample.time_ms=imu_.received_ms>=imu_.sample.ag_age?imu_.received_ms-imu_.sample.ag_age:0;sample.calibrated=imu_fresh(&imu_,now)&&imu_.initialized&&!imu_.calibrating;if(!sample.calibrated)sample.time_ms=now;sample.roll_deg=imu_.roll;sample.pitch_deg=imu_.pitch;sample.yaw_deg=imu_.yaw;controller_.updateImu(sample);
    if(!session_.empty()&&owner_control_ms_&&now-owner_control_ms_>1500&&controller_.state().armed){controller_.ownerDisconnected(now);force_tx_=true;}
    const State &state=controller_.tick(now);
    if(motor_available_&&(now-sent_at>=50||force_tx_.exchange(false))){motor_.send(state.armed,state.left,state.right,options_.left_mask,options_.invert_mask,now);sent_at=now;}
   }
   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
 }
 static void jpegWrite(void *context,void *data,int size){auto *out=static_cast<std::vector<uint8_t>*>(context);auto *bytes=static_cast<uint8_t*>(data);out->insert(out->end(),bytes,bytes+size);}
 void publishLoop(){
  auto snapshot=std::unique_ptr<vision_snapshot>(new vision_snapshot{});
  while(!stopped_){
   uint64_t now=clockMs();if(options_.demo)vision_demo(snapshot.get(),now-started_);else vision_snapshot_copy(vision_,snapshot.get());
   if(!snapshot->have_frame||snapshot->timestamp_ms==last_capture_){
    {std::lock_guard<std::mutex> lock(state_mutex_);vision_error_=snapshot->status;}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));continue;
   }
   last_capture_=snapshot->timestamp_ms;auto next=std::make_shared<PublishedFrame>();next->capture_ms=snapshot->timestamp_ms;
   {std::lock_guard<std::mutex> lock(state_mutex_);
    VisionFrame vf;vf.frame_id=++frame_number_;vf.time_ms=next->capture_ms;
    for(int i=0;i<std::min(snapshot->count,VISION_OBJECTS);i++){const auto &o=snapshot->objects[i];Detection d;d.class_id=o.class_id;d.name=o.name;d.score=o.score;d.eye=o.eye;d.x1=o.left;d.y1=o.top;d.x2=o.right;d.y2=o.bottom;vf.boxes.push_back(d);}
    vf=controller_.updateVision(std::move(vf));fps_=snapshot->fps;vision_error_.clear();next->id=vf.frame_id;
    auto boxes=vf.boxes;uint64_t selected=controller_.state().target_id;
    std::stable_sort(boxes.begin(),boxes.end(),[selected](const Detection &a,const Detection &b){if((a.track_id==selected)!=(b.track_id==selected))return a.track_id==selected;return a.score>b.score;});
    std::ostringstream meta;meta<<std::fixed<<std::setprecision(3)<<"{\"frame_id\":"<<next->id<<",\"width\":"<<VISION_W<<",\"height\":"<<VISION_H<<",\"demo\":"<<(options_.demo?"true":"false")<<",\"selected_id\":"<<selected<<",\"detections\":[";
    for(size_t i=0;i<std::min<size_t>(32,boxes.size());i++){if(i)meta<<',';const auto &d=boxes[i];meta<<"{\"id\":"<<d.track_id<<",\"label\":"<<quoteJson(d.name)<<",\"score\":"<<d.score<<",\"eye\":"<<d.eye<<",\"box\":["<<d.x1<<','<<d.y1<<','<<d.x2<<','<<d.y2<<"]}";}
    meta<<']';next->metadata_prefix=meta.str();
   }
   if(jpeg_from_argb(snapshot->pixels,VISION_W,VISION_H,72,jpegWrite,&next->jpeg)&&next->jpeg.size()<1024*1024){std::lock_guard<std::mutex> lock(frame_mutex_);frame_=std::move(next);frame_ready_.notify_all();}
   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
 }
 std::string stateJson(bool okay=true,const std::string &error="",bool include_session=false){
  uint64_t now=clockMs();const State &s=controller_.tick(now);bool valid=imu_fresh(&imu_,now)&&imu_.initialized;
  std::ostringstream out;out<<std::fixed<<std::setprecision(4)<<"{\"ok\":"<<(okay?"true":"false")<<",\"connected\":true,\"protocol\":1,\"demo\":"<<(options_.demo?"true":"false")<<",\"armed\":"<<(s.armed?"true":"false")<<",\"mode\":"<<quoteJson(modeName(s.mode))<<",\"reason\":"<<quoteJson(s.reason)<<",\"heading_error_deg\":"<<s.heading_error_deg<<",\"selected_id\":"<<s.target_id<<",\"control_lease_ms\":350,\"motor\":{\"enabled\":"<<(motor_available_?"true":"false")<<",\"link_ok\":"<<(s.motor_link_fresh?"true":"false")<<",\"left_mask\":"<<options_.left_mask<<",\"invert_mask\":"<<options_.invert_mask<<",\"left\":"<<s.left<<",\"right\":"<<s.right<<",\"status\":"<<quoteJson(options_.demo?"simulation_only":s.motor_link_fresh?"fpga_ack_verified":"waiting_for_remote_fpga_ack")<<"},\"imu\":{\"valid\":"<<(valid?"true":"false")<<",\"roll\":"<<wrapDegrees(imu_.roll-reference_[0])<<",\"pitch\":"<<wrapDegrees(imu_.pitch-reference_[1])<<",\"yaw\":"<<wrapDegrees(imu_.yaw-reference_[2])<<",\"accel\":["<<imu_.sample.accel[0]*IMU_G<<','<<imu_.sample.accel[1]*IMU_G<<','<<imu_.sample.accel[2]*IMU_G<<"],\"calibrating\":"<<(imu_.calibrating?"true":"false")<<",\"calibration_percent\":"<<imu_.calibration/2<<"},\"vision\":{\"valid\":"<<(s.vision_fresh?"true":"false")<<",\"fps\":"<<fps_<<",\"frame_id\":"<<controller_.vision().frame_id<<",\"status\":"<<quoteJson(vision_error_)<<'}';
  if(include_session)out<<",\"session\":"<<quoteJson(session_);
  if(!error.empty())out<<",\"error\":"<<quoteJson(error);
  out<<'}';return out.str();
 }
 void video(int fd){
  if(streams_.fetch_add(1)>=2){streams_--;httpReply(fd,429,"{\"ok\":false,\"error\":\"video_client_limit\"}");return;}
  struct Guard {std::atomic<int> &n;~Guard(){n--;}} guard{streams_};
  const std::string head="HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=robotframe\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
  if(!writeAll(fd,head.data(),head.size()))return;
  uint64_t last=0;
  while(!stopped_){
   std::shared_ptr<PublishedFrame> frame;
   {std::unique_lock<std::mutex> lock(frame_mutex_);frame_ready_.wait_for(lock,std::chrono::milliseconds(200),[this,last]{return stopped_||(frame_&&frame_->id!=last);});if(stopped_)return;if(!frame_||frame_->id==last)continue;frame=frame_;}
   uint64_t now=clockMs();std::string metadata=frame->metadata_prefix+",\"age_ms\":"+std::to_string(now>=frame->capture_ms?now-frame->capture_ms:0)+'}';
   std::ostringstream h;h<<"--robotframe\r\nContent-Type: image/jpeg\r\nContent-Length: "<<frame->jpeg.size()<<"\r\nX-Frame-Id: "<<frame->id<<"\r\nX-Frame-Meta: "<<base64(metadata)<<"\r\n\r\n";
   std::string header=h.str();if(!writeAll(fd,header.data(),header.size())||!writeAll(fd,frame->jpeg.data(),frame->jpeg.size())||!writeAll(fd,"\r\n",2))return;last=frame->id;
  }
 }
 void request(int fd,const HttpRequest &r){
  auto auth=r.headers.find("authorization");
  if(auth==r.headers.end()||!secretEqual(auth->second,"Bearer "+token_)){httpReply(fd,401,"{\"ok\":false,\"error\":\"pairing_token_required\"}");return;}
  if(r.method=="GET"&&r.path=="/api/video.mjpg"){video(fd);return;}
  std::string response;int code=200;
  {std::lock_guard<std::mutex> lock(state_mutex_);uint64_t now=clockMs();
   if(r.method=="GET"&&r.path=="/api/state")response=stateJson();
   else if(r.method=="POST"&&r.path=="/api/connect"){
    if(!session_.empty()&&controller_.tick(now).armed&&now>=owner_control_ms_&&now-owner_control_ms_<1500){code=409;response=stateJson(false,"controller_busy");}
    else {controller_=Controller(makeConfig(options_));session_=randomHex(16);sequence_=0;owner_control_ms_=now;force_tx_=true;response=stateJson(true,"",true);}
   }else if(r.method=="POST"&&r.path=="/api/control"){
    JsonObject input;std::string error,session,op;double seq_value=0;
    if(!parseFlatJson(r.body,input,error)||!jsonString(input,"session",session)||!jsonString(input,"op",op)||!jsonNumber(input,"seq",seq_value)||seq_value<1||seq_value>9007199254740991.||std::floor(seq_value)!=seq_value){code=400;response=stateJson(false,"invalid_control_request");}
    else if(session_.empty()||!secretEqual(session,session_)){code=403;response=stateJson(false,"not_control_owner");}
    else {
     uint64_t seq=seq_value;bool stop_op=op=="stop"||op=="disarm";
     if(seq<=sequence_&&!stop_op){code=409;response=stateJson(false,"stale_sequence");}
     else {
      sequence_=std::max(sequence_,seq);owner_control_ms_=now;bool okay=true;
      if(op=="arm")okay=controller_.arm(seq,now);
      else if(op=="drive"){
       double throttle=0,steer=0;bool hold=false;
       if(!jsonNumber(input,"throttle",throttle)||!jsonNumber(input,"steer",steer)||std::abs(throttle)>1||std::abs(steer)>1)okay=false;
       else {if(input.count("hold")&&!jsonBool(input,"hold",hold))okay=false;if(okay)okay=controller_.drive(seq,throttle,steer,hold,now);}
      }else if(op=="idle"){
       // Foreground ownership keepalive must neither resume motion nor clear a
       // selected target. It never renews an existing motion command's lease.
       auto mode=controller_.state().mode;
       if(mode==Mode::Manual||mode==Mode::HeadingHold||mode==Mode::Follow)
           okay=controller_.drive(seq,0,0,false,now);
      }else if(op=="select"){
       double frame=0,target=0;okay=jsonNumber(input,"frame_id",frame)&&jsonNumber(input,"target_id",target)&&frame>=1&&target>=1&&frame<=9007199254740991.&&target<=9007199254740991.&&std::floor(frame)==frame&&std::floor(target)==target;
       if(okay)okay=controller_.select(frame,target,now);
      }else if(op=="follow")okay=controller_.follow(seq,now);
      else if(op=="stop"){bool emergency=false;if(input.count("emergency")&&!jsonBool(input,"emergency",emergency))okay=false;if(okay)controller_.stop(now,emergency);}
      else if(op=="disarm")controller_.disarm(now);
      else if(op=="zero"||op=="calibrate"){
       controller_.disarm(now);
       if(op=="zero"){if(imu_fresh(&imu_,now)&&imu_.initialized){reference_[0]=imu_.roll;reference_[1]=imu_.pitch;reference_[2]=imu_.yaw;}else okay=false;}
       else {imu_calibrate(&imu_);ImuSample sample;sample.time_ms=now;sample.calibrated=false;controller_.updateImu(sample);std::fill(std::begin(reference_),std::end(reference_),0.);}
      }else if(op=="configure"){
       double mask=0,invert=12;okay=jsonNumber(input,"left_mask",mask)&&jsonNumber(input,"invert_mask",invert)&&mask>=1&&mask<=14&&invert>=0&&invert<=15&&std::floor(mask)==mask&&std::floor(invert)==invert;
       if(okay){unsigned m=mask;unsigned bits=0;for(unsigned k=m;k;k>>=1)bits+=k&1;okay=bits==2;}
       if(okay){options_.left_mask=mask;options_.invert_mask=invert;okay=controller_.configure(makeConfig(options_));}
      }else {okay=false;error="unknown_operation";}
      if(!okay){code=409;response=stateJson(false,error.empty()?"command_rejected_"+op:error);}else response=stateJson();force_tx_=true;
     }
    }
   }else {code=404;response="{\"ok\":false,\"error\":\"not_found\"}";}
  }
  httpReply(fd,code,response);
 }
};
static void usage(const char *name){std::cout<<"Usage: "<<name<<" [--model FILE] [--labels FILE] [--camera /dev/video41] [--interface FPGA_NET]\n  [--port 8080] [--bind 0.0.0.0] [--token-file pair-token.txt] [--left-mask 3] [--invert-mask 12]\n  [--view both|left|right|single] [--demo] [--seconds N]\n";}
int main(int argc,char **argv){
 Options options;
 try{
  for(int i=1;i<argc;i++){
   std::string key=argv[i];if(key=="--help"){usage(argv[0]);return 0;}if(key=="--demo"){options.demo=true;continue;}if(i+1==argc)throw std::runtime_error("Missing option value: "+key);std::string value=argv[++i];
   auto integer=[&](int lo,int hi){size_t consumed=0;long n=std::stol(value,&consumed);if(consumed!=value.size()||n<lo||n>hi)throw std::runtime_error("Invalid "+key);return int(n);};
   auto fraction=[&]{size_t consumed=0;float n=std::stof(value,&consumed);if(consumed!=value.size()||!std::isfinite(n)||n<=0||n>=1)throw std::runtime_error("Invalid "+key);return n;};
   if(key=="--model")options.model=value;else if(key=="--labels")options.labels=value;else if(key=="--camera")options.device=value;else if(key=="--interface")options.interface=value;else if(key=="--bind")options.bind=value;else if(key=="--token-file")options.token_file=value;
   else if(key=="--port")options.port=integer(1024,65535);else if(key=="--left-mask")options.left_mask=integer(1,14);else if(key=="--invert-mask")options.invert_mask=integer(0,15);else if(key=="--seconds")options.seconds=integer(1,86400);
   else if(key=="--width")options.width=integer(2,4096);else if(key=="--height")options.height=integer(2,2160);else if(key=="--confidence")options.confidence=fraction();else if(key=="--nms")options.nms=fraction();
   else if(key=="--view"){if(value!="both"&&value!="left"&&value!="right"&&value!="single")throw std::runtime_error("Invalid view");options.view=value;}else throw std::runtime_error("Unknown option: "+key);
  }
  if(options.width%2||options.height%2||options.width*9!=options.height*16)throw std::runtime_error("Camera dimensions must be even and 16:9; default is 1920x1080");
  std::signal(SIGINT,interruptHandler);std::signal(SIGTERM,interruptHandler);RobotApp app(options);std::string error;if(!app.start(error)){std::cerr<<error<<'\n';return 1;}
  uint64_t start=clockMs();while(!interrupted&&(!options.seconds||clockMs()-start<uint64_t(options.seconds)*1000))std::this_thread::sleep_for(std::chrono::milliseconds(50));app.stop();return 0;
 }catch(const std::exception &e){std::cerr<<"Robot phone: "<<e.what()<<'\n';return 2;}
}
