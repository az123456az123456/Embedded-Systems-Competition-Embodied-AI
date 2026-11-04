#include "../phone/http_server.h"
#include "../phone/json_flat.h"
#include "../phone/motor_link.h"

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <climits>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

using namespace robotphone;
namespace {
int checks=0;
void check(bool value,const char *message){
 ++checks;if(!value){std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}
}
void jsonTests(){
 JsonObject object;std::string error,text;double number=0;bool boolean=false;
 check(parseFlatJson(" {\"op\":\"drive\",\"x\":-0.25e+1,\"hold\":true,\"n\":null} ",object,error),"flat scalar JSON accepted");
 check(jsonString(object,"op",text)&&text=="drive"&&jsonNumber(object,"x",number)&&number==-2.5&&jsonBool(object,"hold",boolean)&&boolean,"typed scalar lookup");
 check(!jsonNumber(object,"op",number)&&!jsonString(object,"hold",text),"typed access rejects wrong types");
 const char *invalid[]={"", "[]", "null", "{", "{\"x\":NaN}","{\"x\":Infinity}","{\"x\":1e309}","{\"x\":+1}","{\"x\":01}","{\"x\":1.}","{\"x\":1e}","{\"x\":--1}","{\"x\":falsee}","{\"x\":1,\"x\":2}","{\"x\":1,\"\\u0078\":2}","{\"x\":{\"y\":1}}","{\"x\":[1]}","{\"x\":1,}","{\"x\":1}junk","{\"x\":1}{}","{\"x\":\"\\q\"}","{\"x\":\"\\uD800\"}","{\"x\":\"\\uDC00\"}","{\"x\":\"\\uD800\\u0041\"}","{\"x\":\"unterminated}","{\"x\":\"a\nb\"}"};
 for(const char *input:invalid){check(!parseFlatJson(input,object,error)&&object.empty(),"malformed/non-flat/duplicate JSON rejected without partial values");}
 std::string bad="{\"x\":\"";bad+=char(0xc0);bad+=char(0xaf);bad+="\"}";
 check(!parseFlatJson(bad,object,error),"overlong UTF-8 rejected");
 bad="{\"x\":\"";bad+=char(0xed);bad+=char(0xa0);bad+=char(0x80);bad+="\"}";
 check(!parseFlatJson(bad,object,error),"UTF-8 surrogate encoding rejected");
 check(parseFlatJson("{\"x\":\"\\u4e2d\\u6587 \\uD83D\\uDE80\"}",object,error)&&jsonString(object,"x",text)&&text=="中文 🚀","Unicode escapes and valid surrogate pair decoded");
 const std::string original="中文 {nested-looking text} \" \\ \n\r\t\b";
 check(parseFlatJson("{\"x\":"+quoteJson(original)+"}",object,error)&&jsonString(object,"x",text)&&text==original,"JSON quote roundtrip preserves string content and control characters");
 bad="{";for(int i=0;i<33;i++){if(i)bad+=',';bad+='"'+std::to_string(i)+"\":0";}bad+='}';
 check(!parseFlatJson(bad,object,error),"JSON member count bounded");
 check(!parseFlatJson("{\"x\":\""+std::string(8192,'a')+"\"}",object,error),"JSON input size bounded");
 object["bad"]={JsonValue::Number,"1garbage"};
 check(!jsonNumber(object,"bad",number),"numeric accessor rejects invalid manually constructed values");
}

bool head(const std::string &input,size_t expected=0){
 HttpRequest request;size_t size=99;std::string error;
 return parseHttpHead(input,request,size,error)&&size==expected;
}
void httpTests(){
 check(head("GET /api/state HTTP/1.1\r\nHost: robot\r\n\r\n"),"GET HTTP head accepted");
 check(head("POST /api/control HTTP/1.1\r\ncontent-LENGTH: 17\r\nAuthorization: Bearer abc\r\n\r\n",17),"POST content length parsed case independently");
 check(head("GET / HTTP/1.0\r\n\r\n"),"HTTP/1.0 remains usable");
 const char *invalid[]={"GET / HTTP/1.1\r\n","GET / HTTP/1.1\r\nHost: x\r\n","GET / HTTP/1.1\n\n","PUT / HTTP/1.1\r\n\r\n","OPTIONS / HTTP/1.1\r\n\r\n","GET http://robot/ HTTP/1.1\r\n\r\n","GET / HTTP/2\r\n\r\n","GET  / HTTP/1.1\r\n\r\n","GET\t/ HTTP/1.1\r\n\r\n","GET /bad#fragment HTTP/1.1\r\n\r\n","GET / HTTP/1.1\r\nContent-Length: 1\r\n\r\n","POST / HTTP/1.1\r\nContent-Length: 1\r\ncontent-length: 1\r\n\r\n","POST / HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n","POST / HTTP/1.1\r\nContent-Length: 1\r\nTransfer-Encoding: identity\r\n\r\n","POST / HTTP/1.1\r\nContent-Length: -1\r\n\r\n","POST / HTTP/1.1\r\nContent-Length: 8193\r\n\r\n","POST / HTTP/1.1\r\nContent-Length: 1,1\r\n\r\n","GET / HTTP/1.1\r\n Host: evil\r\n\r\n","GET / HTTP/1.1\r\nX: x\nInjected: y\r\n\r\n","GET / HTTP/1.1\r\nHost: x\r\n\r\nTRAILING"};
 for(const char *input:invalid)check(!head(input),"malformed, injected, duplicate, unsupported HTTP rejected");
 std::string bad="GET / HTTP/1.1\r\nX: ";bad+=char(127);bad+="\r\n\r\n";
 check(!head(bad),"DEL header control rejected");
 bad="GET / HTTP/1.1\r\nX: x";bad+=char(0);bad+="\r\n\r\n";
 check(!head(bad),"NUL header injection rejected");
 check(!head("GET /"+std::string(256,'x')+" HTTP/1.1\r\n\r\n"),"request path bounded");
 check(!head("GET / HTTP/1.1\r\nX: "+std::string(16384,'a')+"\r\n\r\n"),"header total size bounded");
}

const uint8_t host[]={2,1,2,3,4,5};
void fixCrc(std::vector<uint8_t> &bytes){const auto crc=motorCrc16(bytes.data()+14,16);bytes[30]=crc>>8;bytes[31]=crc&255;}
std::vector<uint8_t> ackFrame(){
 auto bytes=motorFrame(host,0x12345678,true,240,-180,3,12);
 std::memset(bytes.data(),255,6);const uint8_t fpga[]={2,0xac,0x13,0x80,0,1};std::memcpy(bytes.data()+6,fpga,6);
 std::memcpy(bytes.data()+14,"ACK1",4);bytes[19]=5;fixCrc(bytes);return bytes;
}
void appendFcs(std::vector<uint8_t> &bytes){
 uint32_t crc=0xffffffffu;for(uint8_t byte:bytes){crc^=byte;for(int i=0;i<8;i++)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}
 crc=~crc;for(int i=0;i<4;i++)bytes.push_back((crc>>(i*8))&255);
}
void codecTests(){
 const uint8_t vector[]="123456789";
 check(motorCrc16(vector,9)==0x29b1,"CRC16-CCITT-FALSE canonical vector");
 auto command=motorFrame(host,0x12345678,true,INT_MAX,INT_MIN,3,12);
 const uint8_t golden[]={0x4d,0x4f,0x54,0x31,1,1,0x12,0x34,0x56,0x78,2,0x58,0xfd,0xa8,3,12,0xdb,0};
 check(command.size()==60&&command[12]==0x88&&command[13]==0xb6&&std::equal(std::begin(golden),std::end(golden),command.begin()+14),"MOT1 wire vector includes endian, signed clamp, CRC and version");
 check(std::all_of(command.begin()+32,command.end(),[](uint8_t v){return v==0;}),"minimum Ethernet padding zeroed");
 command=motorFrame(host,9,false,600,-600,3,12);
 check(command[19]==0&&command[24]==0&&command[25]==0&&command[26]==0&&command[27]==0,"disarmed packet always forces both outputs to zero");
 MotorAck out;auto good=ackFrame();
 check(decodeMotorAck(good.data(),good.size(),host,out)&&out.sequence==0x12345678&&out.armed&&out.configured&&!out.watchdog&&out.left==240&&out.right==-180&&out.left_mask==3&&out.invert_mask==12,"ACK1 broadcast decodes applied output and configuration");
 auto unicast=good;std::memcpy(unicast.data(),host,6);
 check(decodeMotorAck(unicast.data(),unicast.size(),host,out),"ACK1 matching unicast host accepted");
 const auto reject=[&](std::vector<uint8_t> bytes,size_t position,uint8_t change,const char *message){bytes[position]^=change;check(!decodeMotorAck(bytes.data(),bytes.size(),host,out),message);};
 reject(good,0,1,"wrong destination MAC rejected");reject(good,6,1,"wrong FPGA source MAC rejected");reject(good,12,1,"wrong EtherType rejected");reject(good,14,1,"wrong ACK magic rejected");reject(good,20,1,"corrupt sequence CRC rejected");reject(good,32,1,"nonzero Ethernet padding rejected");
 auto invalid=good;invalid[19]=8;fixCrc(invalid);check(!decodeMotorAck(invalid.data(),invalid.size(),host,out),"unknown ACK flags rejected");
 invalid=good;invalid[28]=15;fixCrc(invalid);check(!decodeMotorAck(invalid.data(),invalid.size(),host,out),"invalid motor map rejected");
 invalid=good;invalid[24]=0x7f;invalid[25]=0xff;fixCrc(invalid);check(!decodeMotorAck(invalid.data(),invalid.size(),host,out),"out of range applied PWM rejected");
 check(!decodeMotorAck(good.data(),59,host,out)&&!decodeMotorAck(nullptr,60,host,out),"bad frame sizes and null frame rejected");
 appendFcs(good);check(decodeMotorAck(good.data(),good.size(),host,out),"optional captured Ethernet FCS verified");
 reject(good,63,1,"bad optional FCS rejected");
}

MotorAck ack(uint32_t seq){MotorAck a;a.sequence=seq;a.configured=true;a.left_mask=3;a.invert_mask=12;return a;}
void ackWindowTests(){
 MotorAckWindow w;check(!w.fresh(1000)&&!w.accept(ack(1),1000),"old FPGA test program or unsolicited ACK cannot establish motor link");
 w.noteSent(1,1000,3,12);auto wrong=ack(1);wrong.invert_mask=0;
 check(!w.accept(wrong,1010),"ACK must echo this sequence's actual motor configuration");
 check(w.accept(ack(1),1010)&&w.fresh(1010),"matching recent actual transmission establishes link");
 check(!w.accept(ack(1),1200)&&!w.fresh(1301),"duplicate ACK never refreshes liveness");
 w.noteSent(2,1400,3,12);check(!w.accept(ack(2),1701),"ACK delayed more than 300ms rejected");
 check(w.accept(ack(2),1700)&&w.fresh(1700)&&!w.fresh(1701),"delayed ACK cannot extend lifetime beyond original sent command window");
 w.noteSent(3,1800,3,12);w.noteSent(4,1810,3,12);check(w.accept(ack(4),1820)&&!w.accept(ack(3),1821),"out of order older ACK cannot regress state or refresh lease");
 w.noteSent(5,1900,3,12);auto watchdog=ack(5);watchdog.watchdog=true;
 check(w.accept(watchdog,1901)&&!w.fresh(1901),"valid watchdog ACK reports unhealthy link");
 w.noteSent(6,1910,3,12);auto unconfigured=ack(6);unconfigured.configured=false;
 check(w.accept(unconfigured,1911)&&!w.fresh(1911),"unconfigured ACK cannot permit arming");
 w.clear();w.noteSent(0xffffffffu,2000,3,12);check(w.accept(ack(0xffffffffu),2001),"initial high sequence accepted");
 w.noteSent(0,2010,3,12);check(w.accept(ack(0),2011),"32-bit sequence wrap accepted as newer");
 check(!w.accept(ack(0xffffffffu),2012),"pre-wrap replay rejected");
 w.clear();for(uint32_t i=1;i<=33;i++)w.noteSent(i,3000,3,12);
 check(!w.accept(ack(1),3001)&&w.accept(ack(33),3001),"pending sent window bounded to 32 actual packets");
 auto hostile=ack(34);hostile.left=INT_MIN;w.noteSent(34,3010,3,12);
 check(!w.accept(hostile,3011),"invalid helper input rejected without signed overflow");
}

unsigned reservePort(){
 int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"loopback socket available");
 sockaddr_in a={};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
 check(bind(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))==0,"temporary loopback port available");
 socklen_t length=sizeof(a);check(getsockname(fd,reinterpret_cast<sockaddr*>(&a),&length)==0,"temporary bound port readable");
 const unsigned port=ntohs(a.sin_port);::close(fd);return port;
}
int connectLocal(unsigned port){
 int fd=socket(AF_INET,SOCK_STREAM,0);if(fd<0)return -1;
 timeval timeout={4,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
 sockaddr_in a={};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(port);
 if(connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))){::close(fd);return -1;}return fd;
}
std::string exchange(unsigned port,const std::string &request){
 int fd=connectLocal(port);if(fd<0)return {};
 if(!writeAll(fd,request.data(),request.size())){::close(fd);return {};}
 std::string response;char bytes[1024];ssize_t n;
 while((n=recv(fd,bytes,sizeof(bytes),0))>0)response.append(bytes,static_cast<size_t>(n));
 ::close(fd);return response;
}
void networkTests(){
 HttpServer server;std::string error;unsigned port=reservePort();std::atomic<int> active{0},peak{0};
 check(server.start("127.0.0.1",port,[&](int fd,const HttpRequest &request){
  int current=++active,old=peak.load();while(old<current&&!peak.compare_exchange_weak(old,current)){}
  if(request.path=="/slow")std::this_thread::sleep_for(std::chrono::milliseconds(60));
  httpReply(fd,200,"{\"ok\":true}");--active;
 },error),"loopback HTTP server starts");
 check(exchange(port,"GET /api/state HTTP/1.1\r\nHost: local\r\n\r\n").find("200 OK")!=std::string::npos,"loopback HTTP request/reply works");
 check(exchange(port,"POST /api/control HTTP/1.1\r\nHost: local\r\nContent-Length: 2\r\n\r\n{}").find("200 OK")!=std::string::npos,"body framing accepted over socket");
 check(exchange(port,"GET / HTTP/1.1\r\nHost: a\r\nHost: b\r\n\r\n").find("400 Bad Request")!=std::string::npos,"duplicate header rejected before application handler");
 std::vector<std::thread> clients;
 for(int i=0;i<20;i++)clients.emplace_back([port]{exchange(port,"GET /slow HTTP/1.1\r\nHost: local\r\n\r\n");});
 for(auto &client:clients)client.join();
 check(peak.load()<=8&&peak.load()>1,"concurrent HTTP handlers bounded to fixed eight workers");
 int slow=connectLocal(port);check(slow>=0,"slow request socket connected");
 const std::string partial="GET / HTTP/1.1\r\nX: ";writeAll(slow,partial.data(),partial.size());
 const auto begin=std::chrono::steady_clock::now();
 for(int i=0;i<12;i++){
  if(!writeAll(slow,"a",1))break;
  std::this_thread::sleep_for(std::chrono::milliseconds(220));
 }
 char reply[1024];ssize_t received=recv(slow,reply,sizeof(reply),0);
 const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-begin).count();
 check(received>0&&std::string(reply,static_cast<size_t>(received)).find("400 Bad Request")!=std::string::npos&&elapsed<3000,"slow trickle cannot extend absolute two second request deadline");
 ::close(slow);
 int idle=connectLocal(port);check(idle>=0,"idle socket connected");
 const auto stop_start=std::chrono::steady_clock::now();server.stop();
 check(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-stop_start).count()<750,"stop interrupts idle reads and joins workers promptly");
 ::close(idle);
 check(server.start("127.0.0.1",port,[](int fd,const HttpRequest &){httpReply(fd,200,"{}");},error),"server can restart after all previous threads joined");
 server.stop();
}
}  // namespace

int main(int argc,char **argv){
 jsonTests();httpTests();codecTests();ackWindowTests();
 if(argc>1&&std::string(argv[1])=="--http")networkTests();
 std::cout<<"PASS wire: "<<checks<<" checks (JSON, HTTP framing, MOT1/ACK1 codec, ACK lease"<<(argc>1?", loopback workers/deadline":"")<<")\n";
}
