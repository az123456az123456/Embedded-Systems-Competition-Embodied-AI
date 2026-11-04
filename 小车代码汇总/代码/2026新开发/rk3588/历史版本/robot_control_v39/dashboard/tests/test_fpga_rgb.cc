#include "rgb_camera.h"
#include "fpga_rgb_input.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <thread>
#include <sys/socket.h>
#include <unistd.h>
using namespace fpga_rgb;
static uint64_t checks=0;
static void ck(bool value,const char *text){++checks;if(!value)throw std::runtime_error(text);}
static void put16(uint8_t *p,uint16_t n){p[0]=n>>8;p[1]=n;}
static void put32(uint8_t *p,uint32_t n){p[0]=n>>24;p[1]=n>>16;p[2]=n>>8;p[3]=n;}
static uint16_t get16(const uint8_t *p){return (p[0]<<8)|p[1];}
using Packet=std::array<uint8_t,PacketBytes>;
static uint8_t pattern(unsigned eye,unsigned row,unsigned part,unsigned byte,unsigned seed){return (eye*73+row*7+part*31+byte*3+seed)&255;}
static void checksum(Packet &packet){put32(packet.data()+28,packet_crc32(packet.data(),packet.data()+HeaderBytes));}
static Packet make_packet(uint32_t seq,uint32_t stamp,uint16_t epoch,unsigned index,unsigned seed=0,unsigned flags=0) {
    Packet out{};std::memcpy(out.data(),"RGP1",4);out[4]=1;out[5]=index/(Height*Parts);out[6]=1;out[7]=flags;
    put32(out.data()+8,seq);put32(out.data()+12,stamp);put16(out.data()+16,(index/Parts)%Height);
    out[18]=index%Parts;out[19]=Parts;put16(out.data()+20,Width);put16(out.data()+22,Height);put16(out.data()+24,PayloadBytes);put16(out.data()+26,epoch);
    for(unsigned i=0;i<PayloadBytes;++i)out[HeaderBytes+i]=pattern(out[5],get16(out.data()+16),out[18],i,seed);
    checksum(out);return out;
}
static void pair(Assembler &assembler,uint32_t seq,uint32_t stamp,uint16_t epoch,uint64_t host,unsigned seed=0,bool shuffled=false,unsigned flags=0) {
    std::vector<unsigned> order(ChunksPerPair);std::iota(order.begin(),order.end(),0);
    if(shuffled)std::shuffle(order.begin(),order.end(),std::mt19937(991));
    for(size_t i=0;i<order.size();++i){auto packet=make_packet(seq,stamp,epoch,order[i],seed,flags);ck(assembler.ingest(packet.data(),packet.size(),host+i/256),"valid chunk rejected");}
}
static void verify_pixels(const std::vector<uint8_t> &rgb,unsigned seed) {
    ck(rgb.size()==FrameBytes,"wrong stereo frame extent");
    for(unsigned eye=0;eye<2;++eye)for(unsigned row=0;row<Height;++row)for(unsigned part=0;part<Parts;++part) {
        const auto offset=(static_cast<size_t>(row)*Width*2+eye*Width+part*PartPixels)*3;
        for(unsigned byte=0;byte<PayloadBytes;++byte)ck(rgb[offset+byte]==pattern(eye,row,part,byte,seed),"RGB order/eye/row corrupted");
    }
}
static std::vector<uint8_t> ethernet(const Packet &packet,bool good_source=true,uint16_t type=EtherType) {
    std::vector<uint8_t> out(14+packet.size(),0xff);
    std::copy(DefaultPeer.begin(),DefaultPeer.end(),out.begin()+6);if(!good_source)out[11]^=1;
    put16(out.data()+12,type);std::copy(packet.begin(),packet.end(),out.begin()+14);return out;
}
static void codec_tests() {
    const char *test="123456789";
    ck((crc32_update(0xffffffffu,reinterpret_cast<const uint8_t *>(test),9)^0xffffffffu)==0xcbf43926u,"IEEE CRC32 vector failed");
    const auto start=start_command(0x1234);
    ck(!std::memcmp(start.data(),"RGC1",4)&&start[4]==1&&start[5]==1&&get16(start.data()+6)==0x1234,"RGC1 fields failed");
    ck(start[8]==0&&start[9]==0&&start[10]==0&&start[11]==0,"RGC1 reserved nonzero");
    ck(start[12]==0x68&&start[13]==0x43&&start[14]==0xd0&&start[15]==0x5e,"Independent Python-zlib RGC1 golden differs");
    auto golden=make_packet(0x01020304,0x05060708,0xbeef,2559);
    for(unsigned i=0;i<PayloadBytes;++i)golden[HeaderBytes+i]=i&255;
    ck(packet_crc32(golden.data(),golden.data()+HeaderBytes)==0x8ff8c985u,"Independent Python-zlib RGP1 golden differs");
    Assembler malformed(0x1234);auto valid=make_packet(1,1,0x1234,0);
    for(unsigned field:{0u,4u,5u,6u,7u,16u,18u,19u,20u,22u,24u}) {
        auto packet=valid;packet[field]^=0xff;checksum(packet);
        ck(!malformed.ingest(packet.data(),packet.size(),10),"invalid header accepted");
    }
    ck(!malformed.ingest(nullptr,0,10),"null packet accepted");
    ck(!malformed.ingest(valid.data(),valid.size()-1,10),"short packet accepted");
    auto corrupt=valid;corrupt[80]^=1;
    ck(!malformed.ingest(corrupt.data(),corrupt.size(),10)&&malformed.stats().crc_bad==1,"payload CRC not enforced");
    auto foreign=make_packet(1,1,0x4321,0);
    ck(!malformed.ingest(foreign.data(),foreign.size(),10)&&malformed.stats().wrong_epoch==1,"wrong session accepted");
    ck(malformed.ingest(valid.data(),valid.size(),10),"valid packet failed");
    ck(!malformed.ingest(valid.data(),valid.size(),11)&&malformed.stats().duplicates==1,"duplicate accepted twice");
    auto conflict=make_packet(1,1,0x1234,0,3);
    ck(!malformed.ingest(conflict.data(),conflict.size(),12)&&malformed.stats().conflicts==1,"conflicting duplicate accepted");
    std::vector<uint8_t> rgb;FrameInfo info;
    ck(!malformed.take_latest(rgb,info,12),"partial frame was published");

    Assembler shuffled(0x1234);pair(shuffled,42,100,0x1234,1000,9,true);
    ck(shuffled.take_latest(rgb,info,1010),"shuffled complete pair not published");
    ck(info.sequence==42&&info.epoch==0x1234&&info.complete_mask==3&&info.source_ms==1000,"source metadata mismatch");
    verify_pixels(rgb,9);ck(!shuffled.take_latest(rgb,info,1011),"complete pair published twice");

    Assembler missing(9);
    for(unsigned i=0;i<ChunksPerPair-1;++i){auto packet=make_packet(1,1,9,i);missing.ingest(packet.data(),packet.size(),100);}
    ck(!missing.take_latest(rgb,info,100),"one missing chunk treated complete");
    auto other=make_packet(2,34,9,ChunksPerPair-1);missing.ingest(other.data(),other.size(),133);
    ck(!missing.take_latest(rgb,info,133),"different source sequences mixed");
    missing.expire(400);ck(missing.stats().partial_expired==2,"expired partials retained");

    Assembler skew(11);
    for(unsigned i=0;i<ChunksPerPair;++i){auto packet=make_packet(1,i<1280?10:11,11,i);skew.ingest(packet.data(),packet.size(),100);}
    ck(!skew.take_latest(rgb,info,110)&&skew.stats().conflicts>0,"different eye capture stamps paired");

    Assembler latest(12);pair(latest,1,10,12,100,1);pair(latest,2,43,12,133,2);
    ck(latest.stats().superseded==1&&latest.take_latest(rgb,info,145)&&info.sequence==2,"latest-slot policy failed");verify_pixels(rgb,2);
    auto old=make_packet(1,10,12,0);ck(!latest.ingest(old.data(),old.size(),145),"old completed frame revived");
    pair(latest,3,76,12,166,3);ck(!latest.take_latest(rgb,info,500)&&latest.stats().complete_expired==1,"stale complete frame published");

    Assembler wrap(13);
    for(unsigned i=0;i<4;++i){const uint32_t seq=0xfffffffeu+i,stamp=0xffffffc0u+i*32;pair(wrap,seq,stamp,13,100+i*32,i);ck(wrap.take_latest(rgb,info,110+i*32),"counter wrap lost frame");ck(info.sequence==seq,"source seq wrap corrupted");}
    ck(info.device_uptime_ms>0xffffffffu&&wrap.stats().resets==0,"normal uptime wrap mistaken for reboot");

    Assembler reboot(14);pair(reboot,100,10000,14,20000,1);ck(reboot.take_latest(rgb,info,20010),"pre-reboot frame missing");
    auto reset=make_packet(0,0,14,0,2,1);ck(reboot.ingest(reset.data(),reset.size(),20100)&&reboot.stats().resets==1,"explicit reboot not detected");
    auto obsolete=make_packet(101,10033,14,1);ck(!reboot.ingest(obsolete.data(),obsolete.size(),20101),"old timeline clock jumped forward after reboot");
    reboot.restart_epoch(15);ck(!reboot.ingest(reset.data(),reset.size(),20102),"retired reboot epoch accepted");
    pair(reboot,0,0,15,20110,4,false,1);ck(reboot.take_latest(rgb,info,20120)&&info.epoch==15,"new reboot session did not recover");verify_pixels(rgb,4);
}
static void identity_tests() {
    std::vector<uint8_t> stereo(FrameBytes),out(640u*640u*3u+640u*16u,0xaa);std::string error;detector_math::Letterbox box;
    for(size_t i=0;i<stereo.size();++i)stereo[i]=(i*7+3)&255;
    constexpr size_t row=640*3+16;
    ck(detector_math::copy_identity_rgb(stereo.data()+640*3,640,640,1280*3,out.data(),out.size(),row,640,640,box,error),"right-eye identity copy failed");
    ck(box.scale_x==1&&box.scale_y==1&&box.pad_x==0&&box.pad_y==0,"identity geometry altered");
    for(unsigned y=0;y<640;++y){ck(!std::memcmp(out.data()+y*row,stereo.data()+y*1280*3+640*3,640*3),"identity pixel changed");for(size_t x=640*3;x<row;++x)ck(out[y*row+x]==0xaa,"identity wrote stride padding");}
    ck(!detector_math::copy_identity_rgb(stereo.data(),639,640,3840,out.data(),out.size(),row,640,640,box,error),"nonmodel identity geometry accepted");
    ck(!detector_math::copy_identity_rgb(stereo.data(),640,640,10,out.data(),out.size(),row,640,640,box,error),"truncated source stride accepted");
    ck(!detector_math::copy_identity_rgb(stereo.data(),640,640,3840,out.data(),10,row,640,640,box,error),"short identity destination accepted");
}
static void socket_tests() {
    int fd[2];ck(!socketpair(AF_UNIX,SOCK_DGRAM,0,fd),"mock socketpair failed");
    timeval timeout{2,0};
    (void)setsockopt(fd[1],SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    (void)setsockopt(fd[1],SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
    int size=4*1024*1024;(void)setsockopt(fd[1],SOL_SOCKET,SO_SNDBUF,&size,sizeof(size));
    RgbCamera receiver;std::string error;ck(receiver.adopt_test_socket(fd[0],0x2468,error),"mock socket adoption failed");close(fd[0]);
    std::atomic<bool> sent{false},success{true},reboot_requested{false},abort{false};std::atomic<uint16_t> next_epoch{0};
    std::thread producer([&]{
        std::array<uint8_t,2048> control{};const auto got=recv(fd[1],control.data(),control.size(),0);
        if(got!=60||get16(control.data()+12)!=EtherType||std::memcmp(control.data()+14,"RGC1",4)||get16(control.data()+20)!=0x2468){success=false;return;}
        auto noise=ethernet(make_packet(1,1,0x2468,0),false);if(send(fd[1],noise.data(),noise.size(),0)<0){success=false;return;}
        for(unsigned i=0;i<ChunksPerPair;++i){auto bytes=ethernet(make_packet(9,100,0x2468,i,8));if(send(fd[1],bytes.data(),bytes.size(),0)!=static_cast<ssize_t>(bytes.size())){success=false;return;}}
        sent=true;
        while(!reboot_requested&&!abort)std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if(abort)return;
        auto reset=ethernet(make_packet(1,0,0x2468,0,0,1));if(send(fd[1],reset.data(),reset.size(),0)<0){success=false;return;}
        for(unsigned tries=0;tries<8;++tries){
            const auto n=recv(fd[1],control.data(),control.size(),0);
            if(n!=60||std::memcmp(control.data()+14,"RGC1",4)){success=false;return;}
            const auto epoch=get16(control.data()+20);if(epoch&&epoch!=0x2468){next_epoch=epoch;break;}
        }
        if(!next_epoch){success=false;return;}
        // Delayed, valid old-session data must not enter the new camera epoch.
        auto old=ethernet(make_packet(10,133,0x2468,0,8));if(send(fd[1],old.data(),old.size(),0)<0){success=false;return;}
        for(unsigned i=0;i<ChunksPerPair;++i){auto bytes=ethernet(make_packet(1,1,next_epoch,i,4,1));if(send(fd[1],bytes.data(),bytes.size(),0)!=static_cast<ssize_t>(bytes.size())){success=false;return;}}
    });
    std::vector<uint8_t> rgb;ReadTiming timing;int result=0;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(!result&&std::chrono::steady_clock::now()<deadline)result=receiver.read(rgb,error,&timing);
    if(result!=1){abort=true;shutdown(fd[1],SHUT_RDWR);producer.join();ck(false,"mock raw-Ethernet handshake/reassembly failed");}
    ck(success&&sent&&result==1,"mock raw-Ethernet handshake/reassembly failed");
    ck(timing.frame.sequence==9&&timing.frame.epoch==0x2468&&timing.frame.complete_mask==3,"mock read metadata corrupted");verify_pixels(rgb,8);
    reboot_requested=true;result=0;
    const auto second_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(!result&&std::chrono::steady_clock::now()<second_deadline)result=receiver.read(rgb,error,&timing);
    if(result!=1){abort=true;shutdown(fd[1],SHUT_RDWR);}
    producer.join();ck(success&&result==1&&next_epoch&&timing.frame.epoch==next_epoch&&timing.frame.sequence==1,"physical reboot/new epoch handshake did not recover");verify_pixels(rgb,4);
    const auto began=std::chrono::steady_clock::now();ck(receiver.read(rgb,error,&timing)==0,"idle socket returned phantom frame");
    ck(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<.25,"camera stop/timeout exceeded bound");close(fd[1]);
}
int main() {
    try{codec_tests();std::cerr<<"codec complete\n";identity_tests();std::cerr<<"identity complete\n";socket_tests();std::cout<<"PASS FPGA RGB codec/identity/socket "<<checks<<" checks; host mocked Ethernet only, no FPGA/NPU execution\n";return 0;}
    catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<" after "<<checks<<" checks\n";return 1;}
}
