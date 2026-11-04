#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "rgb_camera.h"
#include "stage_clock.h"
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <unistd.h>

namespace fpga_rgb {
namespace {
uint16_t be16(const uint8_t *p){return static_cast<uint16_t>((p[0]<<8)|p[1]);}
uint32_t be32(const uint8_t *p){return (uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3];}
void put16(uint8_t *p,uint16_t v){p[0]=v>>8;p[1]=v;}
void put32(uint8_t *p,uint32_t v){p[0]=v>>24;p[1]=v>>16;p[2]=v>>8;p[3]=v;}
bool newer(uint32_t a,uint32_t b){const auto delta=a-b;return delta && delta<0x80000000u;}
uint64_t clock_ms(){return std::chrono::duration_cast<std::chrono::milliseconds>(StageClock::now().time_since_epoch()).count();}
bool failure(std::string &error,const std::string &message){error=message;return false;}
bool parse_mac(const char *text,std::array<uint8_t,6> &mac) {
    if(!text || std::strlen(text)!=17)return false;
    auto hex=[](char c)->int {if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;return -1;};
    for(size_t i=0;i<mac.size();++i){int a=hex(text[i*3]),b=hex(text[i*3+1]);if(a<0||b<0||(i<5&&text[i*3+2]!=':'))return false;mac[i]=(a<<4)|b;}
    return !(mac[0]&1) && std::any_of(mac.begin(),mac.end(),[](uint8_t x){return x!=0;});
}
bool random_epoch(uint16_t &epoch) {
    for(unsigned attempt=0;attempt<8;++attempt) {
        uint8_t bytes[2];ssize_t got;
        do{got=getrandom(bytes,sizeof(bytes),0);}while(got<0&&errno==EINTR);
        if(got!=2)return false;
        epoch=be16(bytes);if(epoch)return true;
    }
    return false;
}
}
uint32_t crc32_update(uint32_t running,const uint8_t *data,size_t bytes) {
    static const auto table=[] {std::array<uint32_t,256> out{};for(unsigned i=0;i<256;++i){uint32_t v=i;for(int bit=0;bit<8;++bit)v=(v>>1)^((v&1)?0xedb88320u:0);out[i]=v;}return out;}();
    for(size_t i=0;i<bytes;++i)running=table[(running^data[i])&255]^(running>>8);
    return running;
}
uint32_t packet_crc32(const uint8_t *header,const uint8_t *payload) {
    return crc32_update(crc32_update(0xffffffffu,header,28),payload,PayloadBytes)^0xffffffffu;
}
std::array<uint8_t,16> start_command(uint16_t epoch) {
    std::array<uint8_t,16> out{};std::memcpy(out.data(),"RGC1",4);out[4]=1;out[5]=1;put16(out.data()+6,epoch);
    put32(out.data()+12,crc32_update(0xffffffffu,out.data(),12)^0xffffffffu);return out;
}

struct Assembler::Impl {
    struct Slot {
        bool active=false;uint32_t seq=0,stamp32=0;uint64_t stamp64=0,first=0;unsigned count=0;
        std::array<uint64_t,ChunksPerPair/64> seen{};std::vector<uint8_t> rgb;
        void reset(){active=false;count=0;seen.fill(0);}
    };
    uint16_t epoch;uint64_t max_age;Stats stats;std::array<Slot,2> slots;
    std::vector<uint8_t> latest;FrameInfo latest_info;
    bool ready=false,have_source=false,have_completed=false,have_offset=false;
    uint32_t source_seq=0,source_stamp=0,completed_seq=0;uint64_t source_unwrapped=0,source_host=0;
    int64_t offset=0;
    explicit Impl(uint16_t e,uint64_t age):epoch(e),max_age(age){}
    void reset_stream(){for(auto &s:slots)s.reset();ready=false;have_source=false;have_completed=false;have_offset=false;}
    uint64_t mapped(uint64_t device,uint64_t fallback)const {
        if(!have_offset || device>static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))return fallback;
        const auto result=static_cast<int64_t>(device)+offset;
        return result>=0?std::min(fallback,static_cast<uint64_t>(result)):0;
    }
};
Assembler::Assembler(uint16_t epoch,uint64_t max_age_ms):p(new Impl(epoch,max_age_ms)){}
Assembler::~Assembler()=default;
const Stats &Assembler::stats()const{return p->stats;}
void Assembler::restart_epoch(uint16_t epoch){p->reset_stream();p->epoch=epoch;}
void Assembler::expire(uint64_t now) {
    for(auto &s:p->slots)if(s.active && (now<s.first || now-s.first>p->max_age || now-p->mapped(s.stamp64,now)>p->max_age)) {
        s.reset();++p->stats.partial_expired;
    }
    if(p->ready && (now<p->latest_info.completed_ms || now-p->mapped(p->latest_info.device_uptime_ms,now)>p->max_age)) {
        p->ready=false;++p->stats.complete_expired;
    }
}
bool Assembler::ingest(const uint8_t *data,size_t bytes,uint64_t now) {
    ++p->stats.packets;
    if(!data || bytes!=PacketBytes || std::memcmp(data,"RGP1",4) || data[4]!=1 || data[5]>1 || data[6]!=1 ||
       (data[7]&~1u) || be16(data+16)>=Height || data[18]>=Parts || data[19]!=Parts ||
       be16(data+20)!=Width || be16(data+22)!=Height || be16(data+24)!=PayloadBytes) {
        ++p->stats.malformed;return false;
    }
    if(!p->epoch || be16(data+26)!=p->epoch){++p->stats.wrong_epoch;return false;}
    if(be32(data+28)!=packet_crc32(data,data+HeaderBytes)){++p->stats.crc_bad;return false;}
    const uint32_t seq=be32(data+8),stamp=be32(data+12);
    // A reset is explicit and requires both source counters to move backwards.
    // First-frame RESET flags on subsequent chunks do not clear their own slot.
    if(p->have_source && (data[7]&1) && newer(p->source_seq,seq) && newer(p->source_stamp,stamp)) {
        p->reset_stream();++p->stats.resets;
    }
    expire(now);
    if(p->have_completed && !newer(seq,p->completed_seq)){++p->stats.old;return false;}
    uint64_t stamp64=stamp;
    if(!p->have_source){p->have_source=true;p->source_seq=seq;p->source_stamp=stamp;p->source_unwrapped=stamp;p->source_host=now;}
    else if(newer(seq,p->source_seq)) {
        const uint32_t dt=stamp-p->source_stamp;
        if(dt>=0x80000000u || now<p->source_host || dt>now-p->source_host+p->max_age){++p->stats.malformed;return false;}
        p->source_unwrapped+=dt;p->source_seq=seq;p->source_stamp=stamp;p->source_host=now;
    } else if(seq!=p->source_seq && p->source_seq-seq>1) {++p->stats.old;return false;}
    const uint32_t forward=stamp-p->source_stamp;
    if(forward<0x80000000u)stamp64=p->source_unwrapped+forward;
    else {
        const uint32_t backward=p->source_stamp-stamp;
        if(backward>p->source_unwrapped){++p->stats.malformed;return false;}
        stamp64=p->source_unwrapped-backward;
    }
    if(stamp64>static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) || now>static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
        ++p->stats.malformed;return false;
    }
    const auto observed=static_cast<int64_t>(now)-static_cast<int64_t>(stamp64);
    if(!p->have_offset || observed<p->offset){p->offset=observed;p->have_offset=true;}
    if(now-p->mapped(stamp64,now)>p->max_age){++p->stats.old;return false;}
    Impl::Slot *slot=nullptr;
    for(auto &s:p->slots)if(s.active&&s.seq==seq){slot=&s;break;}
    if(slot && slot->stamp32!=stamp){slot->reset();++p->stats.conflicts;return false;}
    if(!slot) {
        for(auto &s:p->slots)if(!s.active){slot=&s;break;}
        if(!slot){slot=newer(p->slots[0].seq,p->slots[1].seq)?&p->slots[1]:&p->slots[0];++p->stats.partial_evicted;}
        slot->reset();slot->active=true;slot->seq=seq;slot->stamp32=stamp;slot->stamp64=stamp64;slot->first=now;
        slot->rgb.resize(FrameBytes);
    }
    const unsigned eye=data[5],row=be16(data+16),part=data[18];
    const unsigned index=eye*Height*Parts+row*Parts+part;
    const uint64_t bit=uint64_t(1)<<(index%64);
    auto *destination=slot->rgb.data()+((static_cast<size_t>(row)*Width*2+eye*Width+part*PartPixels)*3);
    if(slot->seen[index/64]&bit) {
        ++p->stats.duplicates;
        if(std::memcmp(destination,data+HeaderBytes,PayloadBytes)){slot->reset();++p->stats.conflicts;}
        return false;
    }
    std::memcpy(destination,data+HeaderBytes,PayloadBytes);slot->seen[index/64]|=bit;++slot->count;++p->stats.accepted;
    if(slot->count==ChunksPerPair) {
        if(p->ready)++p->stats.superseded;
        p->latest.swap(slot->rgb);p->latest_info={seq,p->epoch,stamp64,p->mapped(stamp64,now),slot->first,now,3};
        p->ready=true;p->have_completed=true;p->completed_seq=seq;++p->stats.completed;slot->reset();
        for(auto &s:p->slots)if(s.active&&!newer(s.seq,seq)){s.reset();++p->stats.partial_evicted;}
    }
    return true;
}
bool Assembler::take_latest(std::vector<uint8_t> &rgb,FrameInfo &info,uint64_t now) {
    expire(now);if(!p->ready)return false;
    p->latest_info.source_ms=p->mapped(p->latest_info.device_uptime_ms,now);
    rgb.swap(p->latest);info=p->latest_info;p->ready=false;return true;
}

struct RgbCamera::Impl {
    int fd=-1,index=0;bool packet_socket=true;std::string interface;
    std::array<uint8_t,6> peer=DefaultPeer,local{{0x02,0xac,0x13,0x80,0xff,0x02}};
    uint16_t epoch=0;std::unique_ptr<Assembler> assembler;
    int requested_receive_bytes=16*1024*1024,actual_receive_bytes=0;bool forced_receive_buffer=false;
    uint64_t last_start=0,last_progress=0,last_log=0,starts=0,tx_errors=0,wrong_source=0,truncated=0,kernel_drops=0;
    ~Impl(){if(fd>=0)::close(fd);}
    bool start(uint64_t now,std::string &error) {
        if(last_start && now-last_start<250)return true;
        last_start=now;
        std::array<uint8_t,60> frame{};std::copy(peer.begin(),peer.end(),frame.begin());std::copy(local.begin(),local.end(),frame.begin()+6);put16(frame.data()+12,EtherType);
        const auto payload=start_command(epoch);std::copy(payload.begin(),payload.end(),frame.begin()+14);
        ssize_t sent;
        if(packet_socket) {
            sockaddr_ll address{};address.sll_family=AF_PACKET;address.sll_protocol=htons(EtherType);address.sll_ifindex=index;address.sll_halen=6;std::copy(peer.begin(),peer.end(),address.sll_addr);
            sent=sendto(fd,frame.data(),frame.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr *>(&address),sizeof(address));
        }else sent=send(fd,frame.data(),frame.size(),MSG_DONTWAIT);
        if(sent==static_cast<ssize_t>(frame.size())){++starts;return true;}
        ++tx_errors;
        if(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)return true;
        return failure(error,"FPGA RGB START send failed: "+std::string(std::strerror(errno)));
    }
    void log(uint64_t now) {
        if(last_log && now-last_log<2000)return;
        last_log=now;
        if(packet_socket){tpacket_stats kernel{};socklen_t size=sizeof(kernel);if(!getsockopt(fd,SOL_PACKET,PACKET_STATISTICS,&kernel,&size))kernel_drops+=kernel.tp_drops;}
        const auto &s=assembler->stats();
        std::fprintf(stderr,"FPGA_RGB interface=%s epoch=%u rx=%llu accepted=%llu pairs=%llu complete_mask=3 crc_bad=%llu malformed=%llu wrong_epoch=%llu wrong_source=%llu duplicate=%llu conflict=%llu old=%llu partial_expired=%llu partial_evicted=%llu complete_expired=%llu superseded=%llu resets=%llu kernel_drops=%llu truncated=%llu starts=%llu tx_errors=%llu rcvbuf=%d rcvbuf_forced=%d clock=source-offset-lower-bound\n",
            interface.c_str(),epoch,(unsigned long long)s.packets,(unsigned long long)s.accepted,(unsigned long long)s.completed,(unsigned long long)s.crc_bad,(unsigned long long)s.malformed,(unsigned long long)s.wrong_epoch,(unsigned long long)wrong_source,(unsigned long long)s.duplicates,(unsigned long long)s.conflicts,(unsigned long long)s.old,(unsigned long long)s.partial_expired,(unsigned long long)s.partial_evicted,(unsigned long long)s.complete_expired,(unsigned long long)s.superseded,(unsigned long long)s.resets,(unsigned long long)kernel_drops,(unsigned long long)truncated,(unsigned long long)starts,(unsigned long long)tx_errors,actual_receive_bytes,forced_receive_buffer);
    }
};
RgbCamera::RgbCamera():p(new Impl){}
RgbCamera::~RgbCamera()=default;
bool RgbCamera::open(const std::string &interface,std::string &error) {
    error.clear();p.reset(new Impl);
    if(interface.empty()||interface.size()>=IFNAMSIZ||interface.find_first_of("/ :\n\r\t")!=std::string::npos)return failure(error,"Invalid FPGA RGB interface");
    if(interface.compare(0,2,"wl")==0 || !access(("/sys/class/net/"+interface+"/wireless").c_str(),F_OK))
        return failure(error,"FPGA RGB requires the internal Ethernet interface, not WLAN");
    const char *peer=std::getenv("PERCEPTION_RGB_SOURCE_MAC");
    if(peer && !parse_mac(peer,p->peer))return failure(error,"PERCEPTION_RGB_SOURCE_MAC must be a nonzero unicast MAC");
    if(!random_epoch(p->epoch))return failure(error,"Unable to obtain random nonzero FPGA RGB session epoch");
    p->fd=socket(AF_PACKET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,htons(EtherType));
    if(p->fd<0)return failure(error,"FPGA RGB raw socket (root/CAP_NET_RAW required): "+std::string(std::strerror(errno)));
    ifreq request{};std::strncpy(request.ifr_name,interface.c_str(),IFNAMSIZ-1);
    if(ioctl(p->fd,SIOCGIFINDEX,&request)<0)return failure(error,"FPGA RGB interface lookup failed");
    p->index=request.ifr_ifindex;
    if(ioctl(p->fd,SIOCGIFHWADDR,&request)<0)return failure(error,"FPGA RGB interface MAC lookup failed");
    std::memcpy(p->local.data(),request.ifr_hwaddr.sa_data,6);
    if(ioctl(p->fd,SIOCGIFFLAGS,&request)<0)return failure(error,"FPGA RGB interface flag lookup failed");
    if(!(request.ifr_flags&IFF_UP)) {
        request.ifr_flags|=IFF_UP;
        if(ioctl(p->fd,SIOCSIFFLAGS,&request)<0)
            return failure(error,"FPGA RGB interface is down and cannot be brought up (root/CAP_NET_ADMIN required); IP/routes were not changed");
    }
    sockaddr_ll address{};address.sll_family=AF_PACKET;address.sll_protocol=htons(EtherType);address.sll_ifindex=p->index;
    if(bind(p->fd,reinterpret_cast<sockaddr *>(&address),sizeof(address))<0)return failure(error,"FPGA RGB raw socket bind failed");
#ifdef SO_RCVBUFFORCE
    p->forced_receive_buffer=!setsockopt(p->fd,SOL_SOCKET,SO_RCVBUFFORCE,&p->requested_receive_bytes,sizeof(p->requested_receive_bytes));
#endif
    if(!p->forced_receive_buffer)(void)setsockopt(p->fd,SOL_SOCKET,SO_RCVBUF,&p->requested_receive_bytes,sizeof(p->requested_receive_bytes));
    socklen_t buffer_size=sizeof(p->actual_receive_bytes);
    (void)getsockopt(p->fd,SOL_SOCKET,SO_RCVBUF,&p->actual_receive_bytes,&buffer_size);
#ifdef PACKET_IGNORE_OUTGOING
    int ignore=1;(void)setsockopt(p->fd,SOL_PACKET,PACKET_IGNORE_OUTGOING,&ignore,sizeof(ignore));
#endif
    p->interface=interface;p->assembler.reset(new Assembler(p->epoch));
    std::fprintf(stderr,"FPGA_RGB_OPEN interface=%s ether_type=0x88ba epoch=%u rcvbuf_requested=%d rcvbuf_actual=%d forced=%d interface_up=1 ip_routes_unchanged=1%s\n",
                 interface.c_str(),p->epoch,p->requested_receive_bytes,p->actual_receive_bytes,p->forced_receive_buffer,
                 p->actual_receive_bytes<1024*1024?" warning=small_socket_buffer_may_drop_stereo_chunks":"");
    return p->start(clock_ms(),error);
}
bool RgbCamera::adopt_test_socket(int fd,uint16_t epoch,std::string &error) {
    error.clear();p.reset(new Impl);if(!epoch)return failure(error,"Test epoch must be nonzero");
    p->fd=fcntl(fd,F_DUPFD_CLOEXEC,3);if(p->fd<0)return failure(error,"Test socket duplication failed");
    int flags=fcntl(p->fd,F_GETFL,0);if(flags<0||fcntl(p->fd,F_SETFL,flags|O_NONBLOCK)<0)return failure(error,"Test socket nonblock failed");
    p->packet_socket=false;p->interface="mock-datagram";p->epoch=epoch;p->assembler.reset(new Assembler(epoch));return true;
}
std::string RgbCamera::description()const{return "1280x640 RGB24 RGP1/0x88ba interface="+p->interface+" epoch="+std::to_string(p->epoch);}
int RgbCamera::read(std::vector<uint8_t> &rgb,std::string &error,ReadTiming *timing) {
    error.clear();ReadTiming local;if(!timing)timing=&local;*timing={};
    if(p->fd<0||!p->assembler){error="FPGA RGB camera is not open";return -1;}
    auto begin=StageClock::now();const auto now=clock_ms();
    if(!p->last_progress||now-p->last_progress>500)if(!p->start(now,error))return -1;
    if(p->assembler->take_latest(rgb,timing->frame,now))return 1;
    pollfd descriptor{p->fd,POLLIN,0};const int polled=poll(&descriptor,1,100);
    timing->wait_ms=stage_elapsed(begin);begin=StageClock::now();
    if(polled==0||(polled<0&&errno==EINTR)){p->assembler->expire(clock_ms());p->log(clock_ms());return 0;}
    if(polled<0||(descriptor.revents&(POLLERR|POLLHUP|POLLNVAL))){error="FPGA RGB socket poll failed";return -1;}
    std::array<std::array<uint8_t,2048>,32> buffers{};
    std::array<mmsghdr,32> messages{};std::array<iovec,32> vectors{};std::array<sockaddr_ll,32> addresses{};
    for(size_t i=0;i<messages.size();++i){vectors[i]={buffers[i].data(),buffers[i].size()};messages[i].msg_hdr.msg_iov=&vectors[i];messages[i].msg_hdr.msg_iovlen=1;messages[i].msg_hdr.msg_name=&addresses[i];}
    for(unsigned batch=0;batch<256 && stage_elapsed(begin)<10;++batch) {
        for(auto &message:messages){message.msg_len=0;message.msg_hdr.msg_namelen=sizeof(sockaddr_ll);message.msg_hdr.msg_flags=0;}
        const int count=recvmmsg(p->fd,messages.data(),messages.size(),MSG_DONTWAIT,nullptr);
        if(count<0){if(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)break;error="FPGA RGB recvmmsg failed: "+std::string(std::strerror(errno));return -1;}
        if(!count)break;
        const auto received=clock_ms();
        for(int i=0;i<count;++i) {
            const auto &message=messages[i];const auto *data=buffers[i].data();
            if((message.msg_hdr.msg_flags&MSG_TRUNC)||message.msg_len>buffers[i].size()){++p->truncated;continue;}
            if(p->packet_socket&&addresses[i].sll_pkttype==PACKET_OUTGOING)continue;
            if(message.msg_len<14 || be16(data+12)!=EtherType)continue;
            if(std::memcmp(data+6,p->peer.data(),6)){++p->wrong_source;continue;}
            const auto resets=p->assembler->stats().resets;
            if(p->assembler->ingest(data+14,message.msg_len-14,received))p->last_progress=received;
            if(p->assembler->stats().resets!=resets) {
                // Physical FPGA reboot while this process remains alive:
                // retire the old epoch so already-queued old Ethernet chunks
                // cannot be mistaken for a future frame of the restarted CPU.
                uint16_t next=0;
                for(unsigned attempt=0;attempt<8 && (!next||next==p->epoch);++attempt)
                    if(!random_epoch(next)){error="FPGA RGB reboot epoch generation failed";return -1;}
                if(!next||next==p->epoch){error="FPGA RGB reboot epoch did not change";return -1;}
                p->epoch=next;p->assembler->restart_epoch(next);p->last_progress=0;p->last_start=0;
                if(!p->start(received,error))return -1;
            }
        }
    }
    timing->receive_ms=stage_elapsed(begin);p->log(clock_ms());
    return p->assembler->take_latest(rgb,timing->frame,clock_ms())?1:0;
}
}
