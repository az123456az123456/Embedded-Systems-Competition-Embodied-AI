// Real bridge with a synthetic publisher: preview continues after detections stop.
#include "phone_service.h"
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <fstream>
#include <thread>
static volatile sig_atomic_t interrupted;
static void finish(int){interrupted=1;}
int main(int argc,char **argv){
 if(argc!=3)return 2;
 phone_options o={};o.bind_address="127.0.0.1";o.port=std::strtoul(argv[1],nullptr,10);o.pair_token_file=argv[2];o.left_mask=10;o.invert_mask=0;o.demo=1;o.camera_width=1920;o.camera_height=1080;o.stream_fps=30;o.camera_view="both";
 char error[256];auto *service=phone_start(&o,error,sizeof(error));if(!service){std::fprintf(stderr,"%s\n",error);return 1;}
 std::signal(SIGTERM,finish);std::signal(SIGINT,finish);
 auto v=std::make_unique<vision_snapshot>();v->have_frame=v->detection_available=v->detection_enabled=1;v->count=v->source_count=2;v->capture_fps=v->preview_fps=30;v->fps=10;v->npu_workers=3;
 for(auto &pixel:v->pixels)pixel=0xffe0e8f0;
 // Controller filters source index0. The remaining track must still receive
 // the compensated coordinates from index1, not its new vector index0.
 v->objects[0].class_id=-1;v->source_objects[0]=v->objects[0];
 auto &box=v->objects[1];box.left=180;box.right=250;box.top=200;box.bottom=300;box.score=.9;box.class_id=39;box.eye=0;std::strcpy(box.name,"bottle");
 v->source_objects[1]=box;box.left+=36;box.right+=36;
 v->motion_enabled=1;v->motion_compensated=1;v->motion_valid[1]=1;
 imu_state imu;imu_init(&imu);imu.initialized=imu.have_packet=1;imu.calibrating=0;imu.sample.flags=15;imu.sample.accel[2]=1;
 const auto start=imu_clock_ms();uint64_t last_result=0;
 while(!interrupted&&imu_clock_ms()-start<8000){
  auto now=imu_clock_ms();imu.received_ms=now;v->timestamp_ms=now;v->frame_id++;
  if(now-start<2000&&now-last_result>=90){v->detection_frame_id++;v->detection_timestamp_ms=now;last_result=now;}
  phone_publish(service,v.get(),&imu,now);sensor_state sonar={};sonar.have_sample=1;sonar.received_ms=now;sonar.sample.valid_mask=7;for(auto &d:sonar.sample.distance_mm)d=2000;
  // Test-only front echo input. This probe is always demo=1 and opens no motor socket.
  unsigned front=2000;std::ifstream input(std::string(argv[2])+".sonar");
  if(input>>front){if(front>0&&front<=4500)sonar.sample.distance_mm[0]=front;else sonar.sample.valid_mask&=~1u;}
  phone_update_sensors(service,&sonar,now);std::this_thread::sleep_for(std::chrono::milliseconds(20));
 }
 phone_stop(service);return 0;
}
