// Production threaded vision pipeline with deterministic custom-class SDK mocks.
// Exercises label semantics and three-core publishing, not board throughput.
#include "camera.h"
#include "detector.h"
#include "vision.h"
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <cstring>
#include <thread>
#include <vector>
static std::atomic<int> opened{0},frames{0},cores[3];
struct Camera::Impl { CameraLayout layout; };
Camera::Camera():p(new Impl){} Camera::~Camera(){delete p;}
bool Camera::open(const std::string &,unsigned,unsigned,std::string &){p->layout.width=16;p->layout.height=8;return true;}
const CameraLayout &Camera::layout()const{return p->layout;}
std::string Camera::description()const{return "16x8 custom-class mock";}
int Camera::read(std::vector<uint8_t> &rgb,std::string &,CameraTiming *t){
    std::this_thread::sleep_for(std::chrono::milliseconds(10));rgb.assign(16*8*3,++frames%255);
    if(t)*t={};
    return 1;
}
struct RknnDetector::Impl {int core=-1;};
RknnDetector::RknnDetector():impl_(new Impl){} RknnDetector::~RknnDetector(){}
bool RknnDetector::open(const std::string &,float,float,int nc,std::string &){assert(nc==10);++opened;return true;}
bool RknnDetector::set_core(int core,std::string &){assert(core>=0&&core<3);impl_->core=core;return true;}
bool RknnDetector::infer(const unsigned char *,int,int,int,std::vector<Detection> &out,std::string &,DetectorTiming *t){
    ++cores[impl_->core];std::this_thread::sleep_for(std::chrono::milliseconds(20));
    out={{1,1,6,6,.9f,9},{1,1,6,6,.8f,0}};if(t)*t={};return true;
}
int main(){
    {std::ofstream labels("build/custom-pipeline-labels.txt");
        labels<<"helmet\nno_helmet\nwrong_helmet\ngauge_class_0\ngauge_class_1\ngauge_class_2\ngauge_object\npipe_hole\npipe_leak\npipe_rust\n";}
    setenv("PERCEPTION_BOX_MOTION","0",1);
    vision_options options={"unused","mock","build/custom-pipeline-labels.txt","both",16,8,.35f,.45f,0,0,0,3};
    auto *worker=vision_start(&options);assert(worker);auto *snapshot=new vision_snapshot{};int published=0;
    for(int attempt=0;attempt<100;attempt++){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));vision_snapshot_copy(worker,snapshot);
        if(snapshot->count!=4)continue;
        ++published;assert(snapshot->npu_workers==3&&snapshot->npu_pinned);
        for(int eye=0;eye<2;eye++){
            const auto &last=snapshot->objects[2*eye],&zero=snapshot->objects[2*eye+1];
            assert(last.class_id==9&&!std::strcmp(last.name,"pipe_rust")&&last.eye==eye);
            assert(zero.class_id==0&&!std::strcmp(zero.name,"helmet")&&zero.eye==eye);
        }
    }
    vision_stop(worker);delete snapshot;std::remove("build/custom-pipeline-labels.txt");
    assert(opened==3&&published>10);for(const auto &n:cores)assert(n>2);
    std::printf("PASS custom 10-class pipeline: %d published previews; helmet class0 and pipe_rust class9 preserved for both eyes, three pinned contexts\n",published);
}
