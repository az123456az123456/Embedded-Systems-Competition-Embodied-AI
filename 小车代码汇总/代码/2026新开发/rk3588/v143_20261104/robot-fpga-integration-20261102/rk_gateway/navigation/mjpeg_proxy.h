#pragma once
// Bounded video-only relay. It shares neither the motor/state lock nor the
// control IO lane. A partial/slow JPEG closes this video socket; no motor action.
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <deque>
#include <cstdio>
#include <string>
#include <sstream>
#include <poll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <linux/sockios.h>
namespace robotnav { namespace video_proxy {
inline uint64_t mono_ms(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline void configure(int client,int upstream){
 int bytes=131072,one=1;setsockopt(client,SOL_SOCKET,SO_SNDBUF,&bytes,sizeof(bytes));
 setsockopt(client,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
#ifdef TCP_NOTSENT_LOWAT
 int low=1024;setsockopt(client,IPPROTO_TCP,TCP_NOTSENT_LOWAT,&low,sizeof(low));
#endif
#ifdef TCP_USER_TIMEOUT
 int timeout=500;setsockopt(client,IPPROTO_TCP,TCP_USER_TIMEOUT,&timeout,sizeof(timeout));
#endif
 bytes=65536;setsockopt(upstream,SOL_SOCKET,SO_RCVBUF,&bytes,sizeof(bytes));
}
inline bool send_until(int fd,const std::string& bytes,uint64_t deadline,const std::atomic<bool>& stopped){
 size_t at=0;
 while(at<bytes.size()&&!stopped){
  const auto now=mono_ms();if(now>=deadline)return false;
  int queued=0;if(ioctl(fd,SIOCOUTQ,&queued)==0&&queued>196608)return false;
  const auto n=send(fd,bytes.data()+at,bytes.size()-at,MSG_DONTWAIT|MSG_NOSIGNAL);
  if(n>0){at+=size_t(n);continue;}if(n<0&&errno==EINTR)continue;
  if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){
   pollfd event{fd,POLLOUT,0};const int ready=poll(&event,1,int(std::min<uint64_t>(20,deadline-now)));
   if(ready<0&&errno==EINTR)continue;
   if(ready<0||(event.revents&(POLLERR|POLLHUP|POLLNVAL)))return false;
   continue;
  }return false;
 }
 return at==bytes.size();
}
class Reader {
 int upstream_,client_;const std::atomic<bool>& stopped_;std::string buffer_;uint64_t oldest_ms_=0;
 std::deque<std::pair<size_t,uint64_t>> received_;
 const char* stage_="idle";size_t header_bytes_=0,jpeg_bytes_=0;
 bool more(uint64_t deadline){
  while(!stopped_){
   const auto now=mono_ms();if(now>=deadline)return false;
   char bytes[32768];const auto n=recv(upstream_,bytes,sizeof(bytes),MSG_DONTWAIT);
   if(n>0){if(buffer_.empty())oldest_ms_=now;received_.emplace_back(size_t(n),now);buffer_.append(bytes,size_t(n));return buffer_.size()<=2*1024*1024+32768;}
   if(n==0)return false;
   if(errno==EINTR)continue;
   if(errno!=EAGAIN&&errno!=EWOULDBLOCK)return false;
   pollfd events[2]={{upstream_,POLLIN,0},{client_,POLLIN,0}};
   const int ready=poll(events,2,int(std::min<uint64_t>(20,deadline-now)));
   if(ready<0&&errno==EINTR)continue;
   if(ready<0)return false;
   if(events[0].revents&(POLLERR|POLLNVAL))return false;
   if(events[1].revents&(POLLERR|POLLHUP|POLLNVAL))return false;
   if(events[1].revents&POLLIN){char c;if(recv(client_,&c,1,MSG_PEEK|MSG_DONTWAIT)<=0)return false;}
  }return false;
 }
 void consume(size_t n){
  buffer_.erase(0,n);
  while(n&&!received_.empty()){
   if(n<received_.front().first){received_.front().first-=n;break;}
   n-=received_.front().first;received_.pop_front();
  }
  // Bytes left in the buffer belong to a newer part. Preserve their actual
  // arrival time; retaining the first part's timestamp kills healthy streams.
  oldest_ms_=received_.empty()?0:received_.front().second;
 }
 public:
 const char* stage()const{return stage_;}
 size_t buffered()const{return buffer_.size();}
 size_t header_bytes()const{return header_bytes_;}
 size_t jpeg_bytes()const{return jpeg_bytes_;}
 Reader(int upstream,int client,const std::atomic<bool>& stopped):upstream_(upstream),client_(client),stopped_(stopped){}
 bool headers(std::string& head,uint64_t deadline){
  while(true){const auto at=buffer_.find("\r\n\r\n");
   if(at!=std::string::npos){if(at>16384)return false;head=buffer_.substr(0,at+4);consume(at+4);return true;}
   if(buffer_.size()>16384||!more(deadline))return false;
  }
 }
 bool frame(std::string& bytes,uint64_t& deadline){
  // Idle waiting is distinct from the JPEG transfer budget: start the strict
  // 200ms budget when its first byte is observed, never restart it per chunk.
  stage_="idle";header_bytes_=jpeg_bytes_=0;const auto idle_deadline=mono_ms()+3000;
  while(buffer_.empty())if(!more(idle_deadline))return false;
  stage_="part_header";deadline=oldest_ms_+200;std::string head;
  if(!headers(head,deadline))return false;
  header_bytes_=head.size();stage_="boundary";
  if(head.compare(0,2,"\r\n")==0)head.erase(0,2);
  if(head.compare(0,14,"--robotframe\r\n")!=0)return false;
  stage_="length";bool seen=false;size_t length=0;std::istringstream lines(head);std::string line;
  while(std::getline(lines,line)){
   if(!line.empty()&&line.back()=='\r')line.pop_back();
   const auto colon=line.find(':');if(colon==std::string::npos)continue;
   auto key=line.substr(0,colon);for(char& c:key)c=char(std::tolower(static_cast<unsigned char>(c)));
   if(key=="content-length"){
    if(seen)return false;
    seen=true;std::istringstream number(line.substr(colon+1));unsigned long long n=0;std::string extra;
    if(!(number>>n)||(number>>extra)||!n||n>2*1024*1024)return false;
    length=size_t(n);
   }
  }
  if(!seen)return false;
  stage_="jpeg_body";jpeg_bytes_=length;
  while(buffer_.size()<length+2)if(!more(deadline))return false;
  stage_="jpeg_trailer";
  if(buffer_.compare(length,2,"\r\n")!=0||mono_ms()>=deadline)return false;
  bytes=head+buffer_.substr(0,length+2);consume(length+2);return true;
 }
};
inline void abandon(int fd){linger abort_pending{1,0};setsockopt(fd,SOL_SOCKET,SO_LINGER,&abort_pending,sizeof(abort_pending));}
inline void relay(int client,int upstream,const std::atomic<bool>& stopped){
 configure(client,upstream);Reader reader(upstream,client,stopped);std::string header;
 const auto started=mono_ms();unsigned frames=0;
 if(!reader.headers(header,mono_ms()+500)){std::fprintf(stderr,"VIDEO_RELAY close=http_header buffered=%zu\n",reader.buffered());return;}
 if(!send_until(client,header,mono_ms()+200,stopped)){abandon(client);return;}
 if(header.compare(0,13,"HTTP/1.1 200 ")!=0&&header.compare(0,13,"HTTP/1.0 200 ")!=0)return;
 while(!stopped){std::string frame;uint64_t deadline=0;
  if(!reader.frame(frame,deadline)){std::fprintf(stderr,"VIDEO_RELAY close=%s frames=%u elapsed_ms=%llu header_bytes=%zu jpeg_bytes=%zu buffered=%zu expired=%d\n",reader.stage(),frames,(unsigned long long)(mono_ms()-started),reader.header_bytes(),reader.jpeg_bytes(),reader.buffered(),int(deadline&&mono_ms()>=deadline));if(deadline)abandon(client);return;}
  if(!send_until(client,frame,deadline,stopped)){std::fprintf(stderr,"VIDEO_RELAY close=client_send frames=%u elapsed_ms=%llu bytes=%zu\n",frames,(unsigned long long)(mono_ms()-started),frame.size());abandon(client);return;}
  ++frames;
 }
}
}} // namespace robotnav::video_proxy
