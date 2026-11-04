// Real HTTP/MJPEG/control service. Only sensor/detector observations are fixtures.
// No raw motor socket is opened: demo=1 is required in this test.
#include "phone_service.h"
#include "box_stability.h"
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>
static volatile sig_atomic_t interrupted;
static void finish(int) { interrupted=1; }
static void object(vision_object &box,int eye) {
    const double center=(35+480.*569/960)*.675+eye*432;
    box.left=center-25;box.right=center+25;box.top=190;box.bottom=290;
    box.score=.9f;box.class_id=2;box.eye=eye;std::strcpy(box.name,"no_vest");
}
int main(int argc,char **argv) {
    if(argc!=5)return 2;
    setenv("PERCEPTION_CAMERA_SOURCE","fpga-rgb",1);
    setenv("FOLLOW_GEOMETRY_FILE",argv[3],1);
    phone_options options={};options.bind_address="127.0.0.1";
    options.port=std::strtoul(argv[1],nullptr,10);options.pair_token_file=argv[2];
    options.left_mask=10;options.demo=1;options.camera_width=1280;
    options.camera_height=640;options.camera_view="both";options.stream_fps=30;
    char error[256];auto *service=phone_start(&options,error,sizeof(error));
    if(!service){std::fprintf(stderr,"%s\n",error);return 1;}
    std::signal(SIGTERM,finish);std::signal(SIGINT,finish);
    auto frame=std::make_unique<vision_snapshot>();
    frame->have_frame=frame->detection_enabled=frame->detection_available=1;
    frame->capture_fps=frame->preview_fps=50;frame->fps=10;frame->npu_workers=3;
    frame->image_geometry_kind=1;frame->tensor_eye_width=frame->tensor_eye_height=640;
    frame->physical_eye_width=960;frame->physical_eye_height=1080;
    frame->tensor_content_x=35;frame->tensor_content_width=569;frame->tensor_content_height=640;
    frame->preview_content_y=27;frame->preview_content_height=432;
    frame->eye_requested_mask=3;frame->control_complete_mask=3;
    frame->pair_status=VISION_PAIR_PARTIAL;frame->control_eligible=0;
    frame->control_count=2;object(frame->control_objects[0],0);object(frame->control_objects[1],1);
    frame->control_source_epoch=frame->detection_source_epoch=7;
    frame->frame_id=100;
    for(auto &pixel:frame->pixels)pixel=0xffe0e8f0;
    imu_state imu;imu_init(&imu);imu.initialized=imu.have_packet=1;imu.calibrating=0;
    imu.sample.flags=15;imu.sample.accel[2]=1;
    const auto start=imu_clock_ms();uint64_t control_updated=0;
    box_stability::Filter display;
    const std::string miss_path=std::string(argv[4])+".miss";
    while(!interrupted&&imu_clock_ms()-start<15000) {
        const auto now=imu_clock_ms();++frame->frame_id;frame->timestamp_ms=now;
        const int eye=frame->frame_id&1;
        frame->detection_frame_id=frame->frame_id;frame->detection_timestamp_ms=now-10;
        frame->result_timestamp_ms=now;frame->detection_update_id=frame->frame_id*2;
        frame->detection_source_sequence=frame->frame_id;
        frame->eye_complete_mask=1u<<eye;frame->count=frame->source_count=1;
        frame->eye_frame_id[eye]=frame->frame_id;frame->eye_frame_id[1-eye]=0;
        frame->eye_source_timestamp_ms[eye]=now-10;frame->eye_source_timestamp_ms[1-eye]=0;
        object(frame->objects[0],eye);frame->source_objects[0]=frame->objects[0];
        // The display is always a newer partial; a complete observation can
        // only arrive through the separate production control cache contract.
        if(access(argv[4],F_OK)!=0&&now-control_updated>=100) {
            frame->control_frame_id=frame->frame_id-3;frame->control_timestamp_ms=now-80;
            frame->control_update_id=frame->control_frame_id*2+1;
            frame->control_source_sequence=frame->control_frame_id;control_updated=now;
            const bool miss=access(miss_path.c_str(),F_OK)==0;
            frame->control_count=miss?1:2;
            if(miss)object(frame->control_objects[0],1);
            else {
                const int first=frame->control_frame_id&1;
                object(frame->control_objects[0],first);object(frame->control_objects[1],1-first);
            }
        }
        frame->control_available=frame->control_frame_id&&now-frame->control_timestamp_ms<=VISION_BOX_MAX_AGE_MS;
        display.update(frame->control_frame_id,frame->control_timestamp_ms,frame->control_objects,frame->control_count,now);
        frame->display_layer_valid=1;frame->display_count=display.copy(frame->display_objects,VISION_OBJECTS,now);
        imu.initialized=access((std::string(argv[4])+".imu").c_str(),F_OK)!=0;
        imu.received_ms=now;phone_publish(service,frame.get(),&imu,now);
        sensor_state ranges={};ranges.have_sample=1;ranges.received_ms=now;ranges.sample.valid_mask=7;
        for(auto &distance:ranges.sample.distance_mm)distance=2000;
        phone_update_sensors(service,&ranges,now);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    phone_stop(service);return 0;
}
