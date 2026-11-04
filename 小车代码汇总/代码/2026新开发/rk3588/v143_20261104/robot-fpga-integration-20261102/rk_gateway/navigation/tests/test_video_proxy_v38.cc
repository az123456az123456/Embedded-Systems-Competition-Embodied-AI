// Real phone-facing8080 Gateway -> backend8082 -> TCP consumer. Only loopback,
// no NPU/camera/motor. Slow clients must not stall cached state/authorized idle.
#define main previous_gateway_straight_main
#include "test_gateway_straight.cc"
#undef main
#include "../mjpeg_proxy.h"
namespace {
std::string video_header(){return "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=robotframe\r\nConnection: close\r\n\r\n";}
std::string frame(unsigned id,size_t n){return "--robotframe\r\nContent-Type: image/jpeg\r\nContent-Length: "+std::to_string(n)+"\r\nX-Frame-Id: "+std::to_string(id)+"\r\n\r\n"+std::string(n,char('a'+id%26))+"\r\n";}
bool raw_send(int fd,const std::string& value,unsigned limit=300){
 const auto end=ms()+limit;size_t at=0;while(at<value.size()&&ms()<end){const auto n=send(fd,value.data()+at,value.size()-at,MSG_DONTWAIT|MSG_NOSIGNAL);if(n>0){at+=size_t(n);continue;}if(n<0&&(errno==EINTR||errno==EAGAIN||errno==EWOULDBLOCK)){std::this_thread::sleep_for(milliseconds(1));continue;}return false;}return at==value.size();
}
int open_video(unsigned p,bool tiny=false){int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0);if(tiny){int small=1024;check(setsockopt(fd,SOL_SOCKET,SO_RCVBUF,&small,sizeof(small))==0);}sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(p);check(connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))==0);const std::string request="GET /api/video.mjpg HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer 012345\r\n\r\n";check(raw_send(fd,request));return fd;}
std::string read_video(int fd,unsigned limit=1500){std::string out;const auto end=ms()+limit;while(ms()<end){char b[32768];auto n=recv(fd,b,sizeof(b),MSG_DONTWAIT);if(n>0){out.append(b,size_t(n));continue;}if(n==0||(n<0&&errno!=EINTR&&errno!=EAGAIN&&errno!=EWOULDBLOCK))break;std::this_thread::sleep_for(milliseconds(1));}close(fd);return out;}
}
int main(){
 Fixture f;f.prepare();std::atomic<unsigned> ended{0};
 {std::lock_guard<std::mutex> lock(f.backend.mutex);f.backend.video_handler=[&](int fd){if(raw_send(fd,video_header()))for(unsigned i=1;i<=30;++i){if(!raw_send(fd,frame(i,1024)))break;std::this_thread::sleep_for(milliseconds(4));}++ended;};}
 auto text=read_video(open_video(f.gp));check(text.find("X-Frame-Id: 1\r\n")!=std::string::npos);check(text.find("X-Frame-Id: 30\r\n")!=std::string::npos);check(f.gateway->telemetry().armed);
 // A following part may already have one byte in the read buffer. Each
 // part's age belongs to its own first byte, not the start of the stream.
 {std::lock_guard<std::mutex> lock(f.backend.mutex);f.backend.video_handler=[&](int fd){
  if(raw_send(fd,video_header()+frame(100,1024)+"-"))for(unsigned i=101;i<=118;++i){
   std::this_thread::sleep_for(milliseconds(40));
   const auto part=frame(i,1024).substr(1)+(i<118?"-":"");
   if(!raw_send(fd,part))break;
  }++ended;
 };}
 text=read_video(open_video(f.gp),1500);check(text.find("X-Frame-Id: 118\r\n")!=std::string::npos);
 // Whole JPEG arrives in70ms chunks. A per-chunk100ms limit would allow it
 // to take700ms; the production complete-frame200ms deadline must reject it.
 const auto before=ended.load();
 {std::lock_guard<std::mutex> lock(f.backend.mutex);f.backend.video_handler=[&](int fd){const auto head=video_header()+"--robotframe\r\nContent-Type: image/jpeg\r\nContent-Length: 100000\r\nX-Frame-Id: 500\r\n\r\n";if(raw_send(fd,head))for(unsigned k=0;k<10;++k){if(!raw_send(fd,std::string(10000,'x')))break;std::this_thread::sleep_for(milliseconds(70));}++ended;};}
 const auto started=ms();text=read_video(open_video(f.gp),1000);check(ms()-started<600);check(text.find("X-Frame-Id: 500")==std::string::npos);
 waitFor([&]{return ended>before;});check(f.status().status==200);check(f.control("idle").status==200);check(f.gateway->telemetry().armed);
 const auto old=ended.load();
 {std::lock_guard<std::mutex> lock(f.backend.mutex);f.backend.video_handler=[&](int fd){if(raw_send(fd,video_header()))for(unsigned i=1000;i<1100;++i){if(!raw_send(fd,frame(i,131072)))break;}++ended;};}
 const int slow=open_video(f.gp,true);unsigned state_ok=0;
 const auto slow_begin=ms();while(ms()-slow_begin<700){const auto began=ms();check(f.status().status==200);check(f.control("idle").status==200);check(ms()-began<250);++state_ok;std::this_thread::sleep_for(milliseconds(10));}
 close(slow);waitFor([&]{return ended>old;});check(state_ok>20&&f.gateway->telemetry().armed);
 // A new video connection starts from current producer content, never from a
 // previous consumer's buffered JPEG. Its failure never invokes DISARM.
 {std::lock_guard<std::mutex> lock(f.backend.mutex);f.backend.video_handler=[&](int fd){raw_send(fd,video_header()+frame(9000,1024));++ended;};}
 text=read_video(open_video(f.gp));check(text.find("X-Frame-Id: 9000\r\n")!=std::string::npos);check(text.find("X-Frame-Id: 1000\r\n")==std::string::npos);check(f.gateway->telemetry().armed);
 check(f.control("stop").status==200);
 std::printf("PASS %u v38 real gateway video checks:30 intact frames, absolute200ms complete-JPEG timeout, slow consumer bounded/RST, %u live state+idle rounds, fresh reconnect, motor authority retained\n",checks,state_ok);
}
