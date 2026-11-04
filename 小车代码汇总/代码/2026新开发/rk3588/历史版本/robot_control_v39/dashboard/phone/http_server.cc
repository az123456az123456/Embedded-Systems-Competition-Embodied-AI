#include "http_server.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <netinet/in.h>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
namespace robotphone {
static std::string trim(const std::string &s){size_t a=s.find_first_not_of(" \t"),b=s.find_last_not_of(" \t");return a==std::string::npos?"":s.substr(a,b-a+1);}
bool parseHttpHead(const std::string &s,HttpRequest &request,size_t &length,std::string &error){
 request={};length=0;error.clear();if(s.size()>16384||s.size()<4||s.compare(s.size()-4,4,"\r\n\r\n")||s.find('\0')!=std::string::npos){error="Invalid HTTP header";return false;}
 size_t e=s.find("\r\n");if(e==std::string::npos){error="Missing request line";return false;}
 std::string line=s.substr(0,e);size_t first=line.find(' '),second=first==std::string::npos?std::string::npos:line.find(' ',first+1);
 if(first==std::string::npos||second==std::string::npos||line.find(' ',second+1)!=std::string::npos){error="Invalid request line";return false;}
 request.method=line.substr(0,first);request.path=line.substr(first+1,second-first-1);std::string version=line.substr(second+1);
 for(unsigned char c:line)if(c<32||c>=127){error="Invalid request line";return false;}
 if(request.path.empty()||request.path[0]!='/'||request.path.size()>256||request.path.find('#')!=std::string::npos||(version!="HTTP/1.1"&&version!="HTTP/1.0")){error="Invalid request line";return false;}
 if(request.method!="GET"&&request.method!="POST"){error="Unsupported method";return false;}
 size_t at=e+2;
 while(at<s.size()){
  e=s.find("\r\n",at);if(e==std::string::npos){error="Invalid header end";return false;}if(e==at){at=e+2;break;}
  std::string field=s.substr(at,e-at);size_t colon=field.find(':');
  if(colon==std::string::npos||colon==0){error="Invalid header field";return false;}
  std::string key=field.substr(0,colon),value=trim(field.substr(colon+1));
  for(char &c:key){if(!(std::isalnum(static_cast<unsigned char>(c))||c=='-')){error="Invalid header key";return false;}c=std::tolower(static_cast<unsigned char>(c));}
  for(unsigned char c:value)if((c<32&&c!='\t')||c==127){error="Invalid header value";return false;}
  if(request.headers.count(key)){error="Duplicate HTTP header";return false;}
  request.headers.emplace(key,value);at=e+2;
 }
 if(at!=s.size()||request.headers.count("transfer-encoding")){error="Unsupported request framing";return false;}
 auto cl=request.headers.find("content-length");
 if(cl!=request.headers.end()){
  if(cl->second.empty()||cl->second.size()>5){error="Invalid content length";return false;}
  for(char c:cl->second){if(c<'0'||c>'9'){error="Invalid content length";return false;}length=length*10+(c-'0');}
 }
 if(length>8192||(request.method=="GET"&&length)){error="Request body too large or unexpected";return false;}return true;
}
bool writeAll(int fd,const void *data,size_t size){
 const char *p=static_cast<const char*>(data);
 while(size){ssize_t n=send(fd,p,size,MSG_NOSIGNAL);if(n<0&&errno==EINTR)continue;if(n<=0)return false;p+=n;size-=n;}return true;
}
bool writeAllBefore(int fd,const void *data,size_t size,uint64_t deadline_ms){
 const char *p=static_cast<const char*>(data);
 while(size){
  const auto now=uint64_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
  if(now>=deadline_ms){errno=ETIMEDOUT;return false;}
  ssize_t n=send(fd,p,size,MSG_NOSIGNAL|MSG_DONTWAIT);
  if(n>0){p+=n;size-=n;continue;}
  if(n<0&&errno==EINTR)continue;
  if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){
   pollfd event{fd,POLLOUT,0};int result=poll(&event,1,int(std::min<uint64_t>(deadline_ms-now,20)));
   if(result<0&&errno==EINTR)continue;
   if(result>=0&&!(event.revents&(POLLERR|POLLHUP|POLLNVAL)))continue;
  }
  return false;
 }
 return true;
}
void httpReply(int fd,int code,const std::string &body,const std::string &type){
 const char *reason=code==200?"OK":code==400?"Bad Request":code==401?"Unauthorized":code==403?"Forbidden":code==404?"Not Found":code==409?"Conflict":code==429?"Too Many Requests":"Service Unavailable";
 std::ostringstream h;h<<"HTTP/1.1 "<<code<<' '<<reason<<"\r\nContent-Type: "<<type<<"\r\nContent-Length: "<<body.size()<<"\r\nCache-Control: no-store\r\nConnection: close\r\nX-Content-Type-Options: nosniff\r\n\r\n";
 std::string head=h.str();if(writeAll(fd,head.data(),head.size()))writeAll(fd,body.data(),body.size());
}
static bool readRequest(int fd,HttpRequest &request){
 using Clock=std::chrono::steady_clock;auto deadline=Clock::now()+std::chrono::seconds(2);std::string bytes;
 size_t end=std::string::npos,length=0;std::string error;
 while(true){
  auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
  if(remaining<=0)return false;
  pollfd event={fd,POLLIN,0};int polled=poll(&event,1,static_cast<int>(remaining));
  if(polled<0&&errno==EINTR)continue;
  if(polled<=0||!(event.revents&(POLLIN|POLLHUP)))return false;
  char buffer[4096];ssize_t n=recv(fd,buffer,sizeof(buffer),MSG_DONTWAIT);
  if(n<0&&(errno==EINTR||errno==EAGAIN||errno==EWOULDBLOCK))continue;
  if(n<=0)return false;
  bytes.append(buffer,static_cast<size_t>(n));
  if(end==std::string::npos){
   end=bytes.find("\r\n\r\n");
   if(end!=std::string::npos){end+=4;if(!parseHttpHead(bytes.substr(0,end),request,length,error))return false;}
   else if(bytes.size()>16384)return false;
  }
  if(end!=std::string::npos&&bytes.size()>=end+length){
   if(bytes.size()!=end+length)return false;
   request.body=bytes.substr(end,length);return true;
  }
  if(bytes.size()>24576)return false;
 }
}
HttpServer::~HttpServer(){stop();}
bool HttpServer::start(const std::string &address,unsigned port,Handler handler,std::string &error){
 if(!stopped_){error="Already listening";return false;}
 if(port>65535||!handler){error="Invalid port or handler";return false;}
 listen_=socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(listen_<0){error=std::strerror(errno);return false;}
 int one=1;setsockopt(listen_,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
 sockaddr_in a={};a.sin_family=AF_INET;a.sin_port=htons(port);
 if(inet_pton(AF_INET,address.c_str(),&a.sin_addr)!=1||bind(listen_,reinterpret_cast<sockaddr*>(&a),sizeof(a))||listen(listen_,16)){error=std::strerror(errno);::close(listen_);listen_=-1;return false;}
 handler_=std::move(handler);stopped_=false;
 try{for(int i=0;i<8;i++)workers_.emplace_back(&HttpServer::worker,this);
  acceptor_=std::thread(&HttpServer::acceptLoop,this);
 }catch(const std::exception &exception){error=exception.what();stop();return false;}
 return true;
}
void HttpServer::acceptLoop(){
 while(!stopped_){
  pollfd p={listen_,POLLIN,0};if(poll(&p,1,100)<=0)continue;
  int fd=accept4(listen_,nullptr,nullptr,SOCK_CLOEXEC);if(fd<0)continue;
  timeval receive={0,500000},send_timeout={1,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&receive,sizeof(receive));setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&send_timeout,sizeof(send_timeout));
  std::lock_guard<std::mutex> lock(mutex_);
  if(stopped_||queue_.size()>=16){::close(fd);continue;}queue_.push_back(fd);ready_.notify_one();
 }
}
void HttpServer::worker(){
 while(true){
  int fd;
  {std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[this]{return stopped_||!queue_.empty();});if(stopped_)return;fd=queue_.front();queue_.pop_front();active_.insert(fd);}
  HttpRequest request;
  try{if(readRequest(fd,request))handler_(fd,request);else httpReply(fd,400,"{\"ok\":false,\"error\":\"invalid_request\"}");}catch(...){httpReply(fd,503,"{\"ok\":false,\"error\":\"server_error\"}");}
  {std::lock_guard<std::mutex> lock(mutex_);active_.erase(fd);::close(fd);}
 }
}
void HttpServer::stop(){
 if(stopped_.exchange(true))return;
 {std::lock_guard<std::mutex> lock(mutex_);for(int fd:active_)shutdown(fd,SHUT_RDWR);for(int fd:queue_)::close(fd);queue_.clear();}
 ready_.notify_all();if(acceptor_.joinable())acceptor_.join();
 if(listen_>=0){::close(listen_);listen_=-1;}
 for(auto &worker:workers_)if(worker.joinable())worker.join();
 workers_.clear();
}
}
