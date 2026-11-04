// Independent calibration receiver; no Python, ISP, YOLO or motor commands.
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <memory>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "picojson.h"
namespace rawcal {
constexpr unsigned W=960,H=1080,Pixels=W*H,RowBytes=1922;
using Clock=std::chrono::steady_clock;
void check(bool b,const std::string& s){if(!b)throw std::runtime_error(s);}
unsigned u16(const uint8_t* p){return unsigned(p[0])<<8|p[1];}
unsigned checksum(const uint8_t* p,size_t n,unsigned sum=0){for(size_t i=0;i<n;i+=2)sum+=(unsigned(p[i])<<8)|(i+1<n?p[i+1]:0);while(sum>>16)sum=(sum&65535)+(sum>>16);return sum;}
std::string hex(const std::vector<uint8_t>& b){static const char* h="0123456789abcdef";std::string s;for(auto v:b){s+=h[v>>4];s+=h[v&15];}return s;}
const uint8_t* payload(const std::vector<uint8_t>& f){
 const uint8_t mac[]={0,10,53,1,254,192};
 if(f.size()<42||!std::equal(mac,mac+6,f.data()+6)||u16(f.data()+12)!=0x0800)return nullptr;
 const uint8_t* ip=f.data()+14;unsigned ihl=(ip[0]&15)*4,total=u16(ip+2);
 if((ip[0]>>4)!=4||ihl<20||ihl>60||f.size()<14u+total||total!=ihl+8+RowBytes||u16(ip+6)&0x3fff||ip[9]!=17||checksum(ip,ihl)!=65535)return nullptr;
 const uint8_t* udp=ip+ihl;if(u16(udp)!=5000||u16(udp+2)!=6102||u16(udp+4)!=8+RowBytes)return nullptr;
 if(u16(udp+6)!=0){unsigned sum=checksum(ip+12,8);sum+=17+8+RowBytes;if(checksum(udp,8+RowBytes,sum)!=65535)return nullptr;}
 return udp+8;
}
struct Profile {
 std::vector<uint8_t> bytes;bool terminal=false,ready=false;
 explicit Profile(const uint8_t* data){
  check(std::equal(data,data+4,reinterpret_cast<const uint8_t*>("CAL1"))&&data[4]==1,"Wrong calibration profile magic/version");
  check(u16(data+6)==W&&u16(data+8)==H&&data[10]==0&&data[11]==0,"Wrong RAW dimensions/Bayer phase");
  check(data[12]==0&&data[13]==0&&data[14]==0x41&&data[15]==1,"Wrong calibration firmware schema");
  bytes.assign(data,data+152);terminal=(data[5]&4)!=0;ready=(data[5]&7)==7;
  if(terminal&&!ready)throw std::runtime_error("FPGA camera setup failed: cam0 stage="+std::to_string(data[16])+" error="+std::to_string(data[17])+" register="+std::to_string(u16(data+20))+"; cam1 stage="+std::to_string(data[18])+" error="+std::to_string(data[19])+" register="+std::to_string(u16(data+22)));
  if(ready)for(unsigned e=0;e<2;++e){auto r=data+24+64*e;check(r[0]==0x56&&r[1]==0x40&&(r[2]&3)==3&&(r[8]&1)==1,"Profile does not confirm OV5640 AEC/AGC/AWB lock");check(u16(r+28)==W&&u16(r+30)==H,"Readback crop size differs from RAW size");}
 }
 picojson::value json()const{
  picojson::object o;o["schema"]=picojson::value(std::string("ACG138-RAW-CAL1-v1"));o["profile_hex"]=picojson::value(hex(bytes));o["ready"]=picojson::value(ready);
  o["width"]=picojson::value(double(W));o["height"]=picojson::value(double(H));o["raw_bits"]=picojson::value(8.0);o["bayer"]=picojson::value(std::string("BGGR"));
  picojson::array cameras;
  const std::array<unsigned,64> addresses={0x300a,0x300b,0x3503,0x3500,0x3501,0x3502,0x350a,0x350b,0x3406,0x3400,0x3401,0x3402,0x3403,0x3404,0x3405,0x5000,0x5001,0x4001,0x4005,0x5480,0x3800,0x3801,0x3802,0x3803,0x3804,0x3805,0x3806,0x3807,0x3808,0x3809,0x380a,0x380b,0x380c,0x380d,0x380e,0x380f,0x3810,0x3811,0x3812,0x3813,0x3814,0x3815,0x3820,0x3821,0x3034,0x3035,0x3036,0x3037,0x3108,0x460c,0x3824,0x4300,0x501f,0x4740,0x3a00,0x3008,0x3000,0x3600,0x3601,0x3022,0x3023,0x3029,0x56a1,0x4000};
  for(unsigned e=0;e<2;++e){auto r=bytes.data()+24+64*e;picojson::object c,regs;
   c["eye_id"]=picojson::value(double(e));c["connector"]=picojson::value(std::string(e?"CAM2":"CAM1"));
   c["exposure_lines"]=picojson::value(double((unsigned(r[3]&15)<<16|unsigned(r[4])<<8|r[5]))/16.0);
   c["gain_code"]=picojson::value(double(u16(r+6)));c["locked"]=picojson::value(ready);
   for(unsigned i=0;i<64;++i){std::ostringstream key;key<<"0x"<<std::hex<<addresses[i];regs[key.str()]=picojson::value(double(r[i]));}
   c["registers"]=picojson::value(regs);cameras.emplace_back(c);
  }o["cameras"]=picojson::value(cameras);return picojson::value(o);
 }
};
class Rows {
 unsigned next_=0;bool collecting_=false;
public:
 std::array<std::vector<uint8_t>,2> images={std::vector<uint8_t>(Pixels),std::vector<uint8_t>(Pixels)};
 uint64_t incomplete=0;
 bool push(const uint8_t* p){unsigned y=p[0]|unsigned(p[1])<<8;
  if(y==0){if(collecting_)++incomplete;next_=0;collecting_=true;}
  if(!collecting_)return false;
  if(y!=next_||y>=H){collecting_=false;++incomplete;return false;}
  for(unsigned e=0;e<2;++e)std::copy_n(p+2+e*W,W,images[e].data()+y*W);
  if(++next_==H){collecting_=false;return true;}return false;
 }
};
class Source {
 int fd_=-1;std::ifstream input_;bool replay_;Clock::time_point deadline_;
public:
 Source(const std::string& iface,const std::string& replay,unsigned seconds):replay_(!replay.empty()),deadline_(Clock::now()+std::chrono::seconds(seconds)){
  if(replay_){input_.open(replay,std::ios::binary);check(bool(input_),"Cannot read replay");return;}
  check(!iface.empty()&&iface.size()<IFNAMSIZ&&iface.find('/')==std::string::npos,"Invalid interface");unsigned index=if_nametoindex(iface.c_str());check(index!=0,"Internal interface missing");
  fd_=socket(AF_PACKET,SOCK_RAW|SOCK_CLOEXEC,htons(0x0003));check(fd_>=0,"Raw socket requires root: "+std::string(strerror(errno)));
  struct ifreq q{};std::copy(iface.begin(),iface.end(),q.ifr_name);
  if(ioctl(fd_,SIOCGIFMTU,&q)<0||q.ifr_mtu<1950){close(fd_);fd_=-1;throw std::runtime_error("Set internal interface MTU first: ip link set dev "+iface+" mtu 4000");}
  int bytes=16*1024*1024;setsockopt(fd_,SOL_SOCKET,SO_RCVBUF,&bytes,sizeof(bytes));
  struct sockaddr_ll a{};a.sll_family=AF_PACKET;a.sll_protocol=htons(0x0003);a.sll_ifindex=int(index);
  if(bind(fd_,reinterpret_cast<sockaddr*>(&a),sizeof(a))<0){auto err=errno;close(fd_);fd_=-1;throw std::runtime_error(strerror(err));}
 }
 ~Source(){if(fd_>=0)close(fd_);}
 bool get(std::vector<uint8_t>& frame){
  if(replay_){uint8_t h[4];input_.read(reinterpret_cast<char*>(h),4);if(input_.eof()&&input_.gcount()==0)return false;check(input_.gcount()==4,"Truncated replay record");unsigned n=h[0]|unsigned(h[1])<<8|unsigned(h[2])<<16|unsigned(h[3])<<24;check(n>=14&&n<=65536,"Invalid replay frame length");frame.resize(n);input_.read(reinterpret_cast<char*>(frame.data()),n);check(unsigned(input_.gcount())==n,"Truncated replay frame");return true;}
  while(Clock::now()<deadline_){struct pollfd p{fd_,POLLIN,0};int n=poll(&p,1,200);if(n<0&&errno==EINTR)continue;check(n>=0,"poll failed");if(!n)continue;check(!(p.revents&(POLLERR|POLLHUP|POLLNVAL)),"Interface/socket failed");
   frame.resize(65536);struct sockaddr_ll a{};socklen_t len=sizeof(a);ssize_t got=recvfrom(fd_,frame.data(),frame.size(),0,reinterpret_cast<sockaddr*>(&a),&len);if(got<0&&errno==EINTR)continue;check(got>=0,"Receive failed");if(a.sll_pkttype==PACKET_OUTGOING)continue;frame.resize(size_t(got));return true;
  }throw std::runtime_error("Timed out waiting for complete RAW frames/locked profile; check temporary bitstream, eth0 MTU, both cameras and lighting during startup.");
 }
};
class Writer {
 std::filesystem::path dir_;std::string kind_;std::array<int,2> fd_={-1,-1};bool committed_=false,owns_lock_=false;
 std::filesystem::path tmp(unsigned e)const{return dir_/("cam"+std::to_string(e)+"-"+kind_+".raw.part");}
public:
 Writer(const std::string& dir,const std::string& kind):dir_(dir),kind_(kind){
  std::filesystem::create_directories(dir_);check(::mkdir((dir_/".capture.lock").c_str(),0700)==0,"Capture directory is busy (or inspect leftover .capture.lock)");owns_lock_=true;
  try{
   check(!std::filesystem::exists(dir_/("profile-"+kind_+".json")),"Output profile already exists; choose a new directory");
   for(unsigned e=0;e<2;++e){check(!std::filesystem::exists(dir_/("cam"+std::to_string(e)+"-"+kind_+".raw")),"RAW output already exists");fd_[e]=open(tmp(e).c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0644);check(fd_[e]>=0,"Cannot create RAW output");}
  }catch(...){cleanup();throw;}
 }
 void cleanup(){for(unsigned e=0;e<2;++e)if(fd_[e]>=0){close(fd_[e]);fd_[e]=-1;if(!committed_)unlink(tmp(e).c_str());}if(owns_lock_){rmdir((dir_/".capture.lock").c_str());owns_lock_=false;}}
 ~Writer(){cleanup();}
 void write(const Rows& rows){for(unsigned e=0;e<2;++e){const auto& b=rows.images[e];size_t at=0;while(at<b.size()){ssize_t n=::write(fd_[e],b.data()+at,b.size()-at);if(n<0&&errno==EINTR)continue;check(n>0,"RAW write failed (check disk space)");at+=size_t(n);}}}
 void finish(picojson::value meta){
  for(auto fd:fd_)check(fsync(fd)==0,"RAW fsync failed");
  for(unsigned e=0;e<2;++e)std::filesystem::rename(tmp(e),dir_/("cam"+std::to_string(e)+"-"+kind_+".raw"));
  auto p=dir_/("profile-"+kind_+".json.part");std::ofstream f(p);check(bool(f),"Cannot create profile");f<<meta.serialize(true)<<'\n';f.close();check(bool(f),"Cannot save profile");std::filesystem::rename(p,dir_/("profile-"+kind_+".json"));committed_=true;
 }
};
std::string reference_hex(const std::string& path){std::ifstream f(path);check(bool(f),"Dark capture requires flat profile first: "+path);picojson::value v;std::string text((std::istreambuf_iterator<char>(f)),{});check(picojson::parse(v,text).empty()&&v.is<picojson::object>()&&v.get("profile_hex").is<std::string>(),"Invalid reference profile");return v.get("profile_hex").get<std::string>();}
unsigned arg_number(const std::string& v,unsigned lo,unsigned hi){size_t n=0;unsigned long x=std::stoul(v,&n);check(n==v.size()&&x>=lo&&x<=hi,"Argument integer out of range");return unsigned(x);}
int application(int argc,char** argv){
 std::string iface="eth0",cmd,kind,dir="/root/raw-cal",reference,replay;unsigned frames=16,discard=3,seconds=60;
 for(int i=1;i<argc;++i){std::string s=argv[i];auto value=[&](){check(i+1<argc,"Missing value for "+s);return std::string(argv[++i]);};
  if(s=="--help"||s=="-h"){std::cout<<"rawcap: temporary RAW8 dual camera calibration (no Python)\nUsage: rawcap [--interface eth0] info\n       rawcap [--interface eth0] stats [--frames 5]\n       rawcap [--interface eth0] capture --kind flat|dark --out-dir /root/raw-cal [--frames 16] [--discard 3]\nOptions: --timeout 60 --reference profile-flat.json\nOffline test input: --replay length-prefixed-ethernet.rawcap\nLoad the standalone bitstream while BOTH cameras see a uniform flat field; it locks once after 5s. Cover lenses only AFTER flat capture, without restart.\n";return 0;}
  if(s=="--interface")iface=value();else if(s=="--kind")kind=value();else if(s=="--out-dir")dir=value();else if(s=="--reference")reference=value();else if(s=="--replay")replay=value();
  else if(s=="--frames")frames=arg_number(value(),1,128);else if(s=="--discard")discard=arg_number(value(),0,120);else if(s=="--timeout")seconds=arg_number(value(),1,3600);
  else {check(cmd.empty()&&(s=="info"||s=="stats"||s=="capture"),"Unknown command/option: "+s);cmd=s;}}
 check(!cmd.empty(),"Use --help");check(cmd!="capture"||(kind=="flat"||kind=="dark"),"capture requires --kind flat|dark");
 std::string expected;if(cmd=="capture"&&kind=="dark")expected=reference_hex(reference.empty()?dir+"/profile-flat.json":reference);
 Source source(iface,replay,seconds);std::vector<uint8_t> wire;std::vector<uint8_t> current;Rows rows;unsigned seen=0,saved=0;uint64_t packets=0,rejected=0;
 std::unique_ptr<Writer> writer; // Open files only after a valid locked profile.
 while(source.get(wire)){
  const uint8_t* p=payload(wire);if(!p){++rejected;continue;}++packets;
  unsigned row=p[0]|unsigned(p[1])<<8;
  if(row==65535){Profile profile(p+2);if(!profile.ready)continue;
   if(!current.empty())check(current==profile.bytes,"Camera lock/profile changed during acquisition; capture aborted");
   if(!expected.empty())check(expected==hex(profile.bytes),"Dark and flat camera settings differ; do not generate coefficients");
   current=profile.bytes;
   if(cmd=="info"){std::cout<<profile.json().serialize(true)<<'\n';return 0;}
   if(cmd=="capture"&&!writer)writer=std::make_unique<Writer>(dir,kind);
   continue;
  }
  if(current.empty()||!rows.push(p))continue;
  if(seen++<discard)continue;
  ++saved;
  std::cerr<<"Frame "<<saved<<"/"<<frames;
  for(unsigned e=0;e<2;++e){uint64_t sum=0;unsigned sat=0;for(auto b:rows.images[e]){sum+=b;sat+=b>=254;}std::cerr<<" cam"<<e<<" mean="<<double(sum)/Pixels<<" saturated="<<100.0*sat/Pixels<<"%";}
  std::cerr<<'\n';if(writer)writer->write(rows);
  if(saved==frames){
   if(writer){Profile profile(current.data());auto meta=profile.json();auto& o=meta.get<picojson::object>();o["frames_per_eye"]=picojson::value(double(frames));o["bytes_per_eye"]=picojson::value(double(frames)*Pixels);o["discarded_complete_frames"]=picojson::value(double(discard));o["incomplete_frames"]=picojson::value(double(rows.incomplete));o["kind"]=picojson::value(kind);o["stereo_semantics"]=picojson::value(std::string("paired equal row indexes, legacy wire has no source frame IDs; static calibration only"));writer->finish(meta);}
   std::cout<<"PASS "<<saved<<" complete frames per eye, "<<uint64_t(saved)*Pixels<<" bytes/eye, incomplete="<<rows.incomplete<<", unrelated_or_invalid_packets="<<rejected<<", received="<<packets<<'\n';return 0;
  }
 }
 throw std::runtime_error("Replay ended before requested complete frames; partial files were not committed");
}
} // namespace
#ifndef RAWCAL_LIBRARY_ONLY
int main(int argc,char** argv){try{return rawcal::application(argc,argv);}catch(const std::exception& e){std::cerr<<"RAW capture error: "<<e.what()<<'\n';return 1;}}
#endif
