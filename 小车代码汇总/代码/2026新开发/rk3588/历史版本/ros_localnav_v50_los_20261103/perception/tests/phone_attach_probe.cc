// Reuse sensor/NPU substitutes; vision.cc and the phone service stay production.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main unused_pipeline_test_main
#include "test_pipeline.cc"
#undef main
#pragma GCC diagnostic pop
#include "phone_service.h"
#include <csignal>
static volatile sig_atomic_t interrupted_attach;
static void attach_stop(int) {interrupted_attach=1;}
int main(int argc,char **argv) {
    if(argc!=4)return 2;
    std::signal(SIGTERM,attach_stop);std::signal(SIGINT,attach_stop);
    FILE *labels=std::fopen("build/attach-labels.txt","w");assert(labels);
    std::fputs("object\n",labels);std::fclose(labels);
    setenv("PERCEPTION_BOX_MOTION","0",1);camera_delay_ms=25;delay_ms=3;
    const bool lifecycle=std::string(argv[3])=="--lifecycle";
    for(int cycle=0;cycle<(lifecycle?12:1);cycle++) {
        vision_options vision={"unused","mock","build/attach-labels.txt","both",16,8,.35f,.45f,0,0,0,3};
        auto *worker=vision_start(&vision);assert(worker);
        phone_options options={};options.bind_address="127.0.0.1";options.port=std::strtoul(argv[1],nullptr,10);
        options.pair_token_file=argv[2];options.left_mask=10;options.demo=1;options.camera_width=1920;
        options.camera_height=1080;options.camera_view="both";options.stream_fps=0;
        char error[256];auto *service=phone_start(&options,error,sizeof(error));assert(service);
        assert(!phone_attach_vision(service,nullptr));assert(phone_attach_vision(service,worker));
        assert(!phone_attach_vision(service,worker));
        imu_state imu;imu_init(&imu);imu.initialized=imu.have_packet=1;imu.sample.flags=15;
        const auto start=std::chrono::steady_clock::now();
        while(!interrupted_attach&&std::chrono::steady_clock::now()-start<
              (lifecycle?std::chrono::milliseconds(3+cycle*3):std::chrono::milliseconds(12000))) {
            imu.received_ms=imu_clock_ms();phone_update_imu(service,&imu);
            // Deliberately no phone_publish: an HDMI-like loop runs only 10Hz.
            std::this_thread::sleep_for(std::chrono::milliseconds(lifecycle?1:100));
        }
        const auto stopping=std::chrono::steady_clock::now();
        phone_stop(service);vision_stop(worker);assert(active==0);
        assert(std::chrono::steady_clock::now()-stopping<std::chrono::seconds(2));
    }
    std::printf("PASS attached real vision/service lifecycle cycles=%d no manual phone_publish; sensor/NPU substitutes; demo motors only\n",lifecycle?12:1);
}
