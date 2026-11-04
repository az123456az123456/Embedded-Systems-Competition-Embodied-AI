#include "ros_bridge_transport.h"
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <sstream>
#include <sys/file.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
namespace arb1 {
uint64_t mono_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
Id kernel_boot_id(){std::ifstream f("/proc/sys/kernel/random/boot_id");std::string s;f>>s;Id id{};unsigned n=0;int high=-1;for(char c:s){if(c=='-')continue;int v=c>='0'&&c<='9'?c-'0':c>='a'&&c<='f'?c-'a'+10:-1;if(v<0)throw std::runtime_error("invalid kernel boot ID");if(high<0)high=v;else{if(n>=16)throw std::runtime_error("boot ID length");id[n++]=uint8_t(high*16+v);high=-1;}}if(n!=16||high>=0)throw std::runtime_error("missing boot ID");return id;}
namespace {
std::vector<Capability> caps(){return {{1,1},{2,1},{3,1},{4,1},{5,1},{6,1},{7,1},{8,1},{10,1},{11,1},{12,1},{13,1}};}
struct OwnedFd{int value=-1;~OwnedFd(){if(value>=0)::close(value);}};
struct Tx{std::vector<uint8_t> bytes;size_t offset=0;uint16_t type=0,kind=0;};
sockaddr_un address(const std::string& path){sockaddr_un a{};a.sun_family=AF_UNIX;if(path.size()>=sizeof(a.sun_path))throw std::runtime_error("IPC path too long");std::copy(path.begin(),path.end(),a.sun_path);return a;}
}
struct LocalBridge::Impl {
 struct Lane {
  Impl* parent;Channel channel;std::string path;int listener=-1;std::thread thread;
  mutable std::mutex mutex;std::deque<Tx> queue;size_t bytes=0;
  std::atomic<bool> ready{false};std::atomic<uint64_t> epoch{0};uint64_t sequence=0;
  Lane(Impl* p,Channel c,std::string name):parent(p),channel(c),path(std::move(name)){}
  ~Lane(){if(thread.joinable())thread.join();if(listener>=0)::close(listener);}
  SessionConfig config(uint64_t e){SessionConfig c;c.channel=channel;c.local_endpoint=parent->server?LocalEndpoint::Bridge:LocalEndpoint::Client;c.expected_peer_role=PeerRole::Gateway;c.boot_id=parent->boot;c.bridge_epoch=e;c.capabilities=caps();c.required_capabilities={{ARB1_KIND_HELLO,1},{ARB1_KIND_MISSION,1},{ARB1_KIND_FPGA_STATE,1},{ARB1_KIND_PLAN_RESULT,1},{ARB1_KIND_CLOCK_ANCHOR,1}};c.allowed_message_types=parent->server?(channel==Channel::Control?std::vector<uint16_t>{ARB1_MSG_MISSION}:std::vector<uint16_t>{}):std::vector<uint16_t>{ARB1_MSG_MISSION,ARB1_MSG_SNAPSHOT};return c;}
  bool enqueue(Message m,bool hello=false){
   std::lock_guard<std::mutex> l(mutex);
   if(!hello&&!ready)return false;
   m.header.boot_id=parent->boot;m.header.bridge_epoch=epoch;m.header.message_sequence=++sequence;m.header.sent_mono_ns=mono_ns();
   std::vector<uint8_t>b;DecodeError error;if(!encode(m,b,&error))return false;
   const size_t limit=channel==Channel::State?ARB1_MAX_MESSAGE_BYTES:8u*16384u;
   const auto kind=m.blocks.size()==1?block_kind(m.blocks[0].payload):0;
   // Coalesce only the same data kind. A fast SCAN/STATE must never erase an
   // unsent MAP, complete PLAN or CLOCK belonging to a different stream.
   if(m.header.type==ARB1_MSG_SNAPSHOT&&kind&&kind!=ARB1_KIND_DIAGNOSTIC){
    for(auto i=queue.begin();i!=queue.end();){if(i->offset==0&&i->type==ARB1_MSG_SNAPSHOT&&i->kind==kind){bytes-=i->bytes.size();i=queue.erase(i);}else ++i;}
   }
   if(b.size()>limit||bytes>limit-b.size()||queue.size()>=8)return false;
   bytes+=b.size();queue.push_back({std::move(b),0,m.header.type,uint16_t(kind)});return true;
  }
  void flush(int fd){
   std::lock_guard<std::mutex>l(mutex);
   // Bound each visit. A large state frame cannot monopolize even its own lane.
   size_t budget=32768;
   while(!queue.empty()&&budget){auto& t=queue.front();size_t n=std::min(budget,t.bytes.size()-t.offset);auto sent=::send(fd,t.bytes.data()+t.offset,n,MSG_DONTWAIT|MSG_NOSIGNAL);
    if(sent>0){t.offset+=size_t(sent);bytes-=size_t(sent);budget-=size_t(sent);if(t.offset==t.bytes.size())queue.pop_front();}
    else if(sent<0&&(errno==EAGAIN||errno==EINTR))break;
    else throw std::runtime_error("IPC send failed");
   }
  }
  Message hello(uint64_t e){Message m;m.header.type=ARB1_MSG_HELLO;Hello h;h.boot_id=parent->boot;h.bridge_epoch=e;h.capabilities=caps();m.blocks.push_back({Metadata{},h});return m;}
  void connection(int fd){
   if(!unix_root_peer(fd)) {::close(fd);return;}
   uint64_t e=parent->bridge_epoch;std::vector<uint8_t> initial;
   if(!parent->server){
    FrameStream input;std::vector<std::vector<uint8_t>>frames;auto until=mono_ns()+100000000;
    while(!parent->stop&&mono_ns()<until&&frames.empty()){
     uint8_t b[4096];auto n=recv(fd,b,sizeof(b),MSG_DONTWAIT);
     if(n>0){if(!input.feed(b,size_t(n),frames))break;}else if(n==0)break;
     else if(errno!=EAGAIN&&errno!=EINTR)break;
     if(frames.empty())std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if(frames.size()!=1||input.buffered_bytes()){::close(fd);return;}
    Message h;if(!decode(frames[0].data(),frames[0].size(),h)||h.header.type!=ARB1_MSG_HELLO||h.header.boot_id!=parent->boot){::close(fd);return;}
    initial=std::move(frames[0]);e=h.header.bridge_epoch;
   }
   {std::lock_guard<std::mutex>l(mutex);queue.clear();bytes=0;sequence=0;epoch=e;}
   UnixConnection conn(fd,config(e),[&](const AcceptedMessage&m){ready=true;parent->receiver(channel,m);});
   if(!initial.empty()&&!const_cast<ProtocolSession&>(conn.session()).receive(initial.data(),initial.size()))return;
   enqueue(hello(e),true);const auto until=mono_ns()+100000000;
   try{while(!parent->stop){
     if(parent->server&&channel==Channel::Control)parent->heartbeat();
     flush(conn.fd());if(!conn.read_available())break;
     ready=conn.session().ready();
     if(!ready&&mono_ns()>until)break;
     std::this_thread::sleep_for(std::chrono::milliseconds(1));
   }}catch(const std::exception&){}
   ready=false;std::lock_guard<std::mutex>l(mutex);queue.clear();bytes=0;
  }
  void run(){while(!parent->stop){if(parent->server&&channel==Channel::Control)parent->heartbeat();int fd=-1;if(parent->server){pollfd p{listener,POLLIN,0};if(poll(&p,1,20)>0)fd=accept4(listener,nullptr,nullptr,SOCK_CLOEXEC|SOCK_NONBLOCK);}else{fd=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0);if(fd>=0){auto a=address(path);if(connect(fd,(sockaddr*)&a,sizeof(a))<0){::close(fd);fd=-1;}}}
    if(fd>=0)connection(fd);else std::this_thread::sleep_for(std::chrono::milliseconds(20));}}
 };
 bool server;Id boot;uint64_t bridge_epoch=0;Receiver receiver;std::string directory;OwnedFd lock,instance;std::atomic<uint64_t> heartbeat_at{0};
 std::atomic<bool>stop{false};std::array<std::unique_ptr<Lane>,2> lanes;
 Impl(std::string d,bool s,Receiver r):server(s),boot(kernel_boot_id()),receiver(std::move(r)),directory(std::move(d)){
  if(geteuid()!=0)throw std::runtime_error("ARB1 requires UID0; permissions will not be widened");
  if(server){
   if(mkdir(directory.c_str(),0700)<0&&errno!=EEXIST)throw std::runtime_error("cannot create IPC directory");
   struct stat st{};if(lstat(directory.c_str(),&st)||!S_ISDIR(st.st_mode)||st.st_uid!=0||(st.st_mode&0777)!=0700)throw std::runtime_error("IPC directory identity/mode mismatch");
   const auto path=directory+"/bridge.lock";lock.value=open(path.c_str(),O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
   if(lock.value<0||flock(lock.value,LOCK_EX|LOCK_NB)<0)throw std::runtime_error("bridge instance already locked");
   if(getrandom(&bridge_epoch,sizeof(bridge_epoch),0)!=sizeof(bridge_epoch)||!bridge_epoch)throw std::runtime_error("bridge epoch unavailable");
  }
  for(unsigned i=0;i<2;++i){auto lane=std::make_unique<Lane>(this,i?Channel::Control:Channel::State,directory+(i?"/control.sock":"/state.sock"));
   if(server){auto a=address(lane->path);struct stat st{};
    if(lstat(lane->path.c_str(),&st)==0){if(!S_ISSOCK(st.st_mode)||st.st_uid!=0)throw std::runtime_error("unexpected old IPC path");int probe=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);if(probe<0)throw std::runtime_error("IPC probe failed");int rc=connect(probe,(sockaddr*)&a,sizeof(a));int error=errno;::close(probe);if(rc==0||error!=ECONNREFUSED)throw std::runtime_error("old IPC path not proven stale");if(unlink(lane->path.c_str()))throw std::runtime_error("stale socket cleanup failed");}
    lane->listener=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0);if(lane->listener<0||bind(lane->listener,(sockaddr*)&a,sizeof(a))||chmod(lane->path.c_str(),0600)||listen(lane->listener,4))throw std::runtime_error("IPC listen failed");}
   lanes[i]=std::move(lane);
  }
  if(server){
   const auto name=directory+"/bridge.instance.json";
   instance.value=open(name.c_str(),O_WRONLY|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
   if(instance.value<0)throw std::runtime_error("instance record unavailable");
   struct stat st{};if(fstat(instance.value,&st)||st.st_uid!=0||(st.st_mode&0777)!=0600)throw std::runtime_error("instance permissions");
   heartbeat();
  }
  for(auto& lane:lanes){auto* ptr=lane.get();lane->thread=std::thread([ptr]{ptr->run();});}
 }
 void heartbeat(){
  const auto now=mono_ns();auto previous=heartbeat_at.load();if(previous&&now-previous<1000000000)return;
  if(!heartbeat_at.compare_exchange_strong(previous,now))return;
  std::ifstream file("/proc/self/stat");std::string raw;std::getline(file,raw);auto at=raw.rfind(')');
  std::string ticks="unknown";if(at!=std::string::npos){std::istringstream fields(raw.substr(at+2));for(unsigned i=0;i<=19;++i)fields>>ticks;}
  std::string id;const char*hex="0123456789abcdef";for(auto b:boot){id+=hex[b>>4];id+=hex[b&15];}
  const auto text="{\"pid\":"+std::to_string(getpid())+",\"start_ticks\":\""+ticks+"\",\"boot_id\":\""+id+"\",\"mode\":\"ros-bridge\",\"heartbeat_mono_ns\":"+std::to_string(now)+"}\n";
  if(instance.value>=0){const auto wrote=pwrite(instance.value,text.data(),text.size(),0);if(wrote!=ssize_t(text.size())||ftruncate(instance.value,text.size()))stop=true;}
 }
 ~Impl(){stop=true;for(auto& lane:lanes)if(lane&&lane->thread.joinable())lane->thread.join();if(server){for(auto& lane:lanes)if(lane)unlink(lane->path.c_str());}}
};
LocalBridge::LocalBridge(const std::string&p,bool s,Receiver r):p_(new Impl(p,s,std::move(r))){}
LocalBridge::~LocalBridge()=default;
bool LocalBridge::send(Channel c,const Message&m){return p_->lanes[c==Channel::Control]->enqueue(m);}
bool LocalBridge::ready(Channel c)const{return p_->lanes[c==Channel::Control]->ready;}
uint64_t LocalBridge::epoch()const{return p_->lanes[1]->epoch;}
size_t LocalBridge::queued_bytes(Channel c)const{auto& l=*p_->lanes[c==Channel::Control];std::lock_guard<std::mutex>guard(l.mutex);return l.bytes;}
}
