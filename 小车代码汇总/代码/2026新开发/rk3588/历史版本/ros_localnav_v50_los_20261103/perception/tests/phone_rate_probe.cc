// Production publisher/control threads, with deterministic fixture observations.
#include "phone_service.h"
#include "jpeg_codec.h"
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
static volatile sig_atomic_t stopped;
static bool slow_jpeg;
static void stop(int) {stopped=1;}
extern "C" int __real_jpeg_from_argb(const uint32_t *,int,int,int,void (*)(void *,void *,int),void *);
extern "C" int __wrap_jpeg_from_argb(const uint32_t *pixels,int w,int h,int quality,void (*write)(void *,void *,int),void *context) {
    if(slow_jpeg)std::this_thread::sleep_for(std::chrono::milliseconds(70));
    return __real_jpeg_from_argb(pixels,w,h,quality,write,context);
}
int main(int argc,char **argv) {
    if(argc!=4)return 2;
    const std::string mode=argv[3];slow_jpeg=mode=="slow40";
    const bool detect=mode!="off30";const double fps=mode=="auto30"||mode=="off30"?30:40;
    phone_options options={};options.bind_address="127.0.0.1";
    options.port=std::strtoul(argv[1],nullptr,10);options.pair_token_file=argv[2];
    options.left_mask=10;options.demo=1;options.camera_width=1920;options.camera_height=1080;
    options.camera_view="both";options.stream_fps=mode=="cap15"?15:0;
    char error[256];auto *service=phone_start(&options,error,sizeof(error));
    if(!service){std::fprintf(stderr,"%s\n",error);return 1;}
    std::signal(SIGTERM,stop);std::signal(SIGINT,stop);
    auto frame=std::make_unique<vision_snapshot>();frame->have_frame=frame->detection_available=1;
    frame->detection_enabled=detect;frame->fps=detect?fps:0;frame->capture_fps=frame->preview_fps=fps;
    frame->npu_workers=3;
    // Incompressible image also fills a nonreading client's bounded TCP socket.
    uint32_t random=0x4567abcd;for(auto &pixel:frame->pixels){random^=random<<13;random^=random>>17;random^=random<<5;pixel=0xff000000|(random&0xffffff);}
    imu_state imu;imu_init(&imu);imu.initialized=imu.have_packet=1;imu.sample.flags=15;
    const auto start=std::chrono::steady_clock::now();auto next=start;
    while(!stopped&&std::chrono::steady_clock::now()-start<std::chrono::seconds(20)) {
        const auto now=imu_clock_ms();frame->timestamp_ms=frame->detection_timestamp_ms=now;
        frame->detection_frame_id=++frame->frame_id;imu.received_ms=now;
        phone_publish(service,frame.get(),&imu,now);
        next+=std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1./fps));
        std::this_thread::sleep_until(next);
    }
    phone_stop(service);return 0;
}
