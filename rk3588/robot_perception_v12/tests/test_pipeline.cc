// Real threaded pipeline with slower deterministic camera/NPU substitutes.
#include "camera.h"
#include "detector.h"
#include "vision.h"
#include <atomic>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
static std::atomic<int> captures{0},active{0},maximum{0},opens{0},calls{0};
static bool fail_second=false,fail_infer=false;
static int fail_open_at=0,fail_bind_at=-2;
static bool disorder=false;
static std::atomic<int> core_calls[3];
static int delay_ms=80,camera_delay_ms=20;
struct Camera::Impl {CameraLayout layout;};
Camera::Camera():p(new Impl){}
Camera::~Camera(){delete p;}
bool Camera::open(const std::string &,unsigned,unsigned,std::string &){p->layout.width=16;p->layout.height=8;return true;}
const CameraLayout &Camera::layout()const{return p->layout;}
std::string Camera::description()const{return "16x8 mock";}
int Camera::read(std::vector<uint8_t> &rgb,std::string &,CameraTiming *t) {
    std::this_thread::sleep_for(std::chrono::milliseconds(camera_delay_ms));
    int id=++captures;rgb.assign(16*8*3,static_cast<uint8_t>(id));
    if(t){*t={};t->wait_ms=camera_delay_ms;t->convert_ms=.25;}return 1;
}
struct RknnDetector::Impl {int core=-1;std::atomic<bool> busy{false};};
RknnDetector::RknnDetector():impl_(new Impl){}
RknnDetector::~RknnDetector(){}
bool RknnDetector::open(const std::string &,float,float,int,std::string &error) {
    const int n=++opens;
    if((n==2 && fail_second)||n==fail_open_at){error="injected context failure";return false;}return true;
}
bool RknnDetector::set_core(int core,std::string &error){
    if(core==fail_bind_at){error="injected binding failure";return false;}
    impl_->core=core;return true;
}
bool RknnDetector::infer(const unsigned char *rgb,int,int,int,std::vector<Detection> &result,std::string &error,DetectorTiming *t) {
    assert(!impl_->busy.exchange(true));
    if(impl_->core>=0)++core_calls[impl_->core];
    int n=++active,old=maximum.load();while(old<n && !maximum.compare_exchange_weak(old,n)){}
    auto value=rgb[0];
    // Force the first frame to complete after newer frames in the three-worker path.
    std::this_thread::sleep_for(std::chrono::milliseconds(disorder && value==1?250:delay_ms));
    // Input must remain unchanged while capture and preview continue using the pool.
    assert(rgb[0]==value);--active;impl_->busy=false;
    if(fail_infer && ++calls>2){error="injected inference failure";return false;}
    result={{1,1,6,6,value/255.f,0}};
    if(t){*t={};t->run_ms=delay_ms;t->total_ms=delay_ms;t->preprocess_ms=.1;}
    return true;
}
static void check(int no_detect,int serial,bool fail,int sync,int delay=80,bool inference_failure=false,const char *view="both",int npu_workers=2,int open_failure=0,int bind_failure=-2,bool reorder=false) {
    captures=active=maximum=opens=calls=0;for(auto &n:core_calls)n=0;
    fail_open_at=open_failure;fail_bind_at=bind_failure;disorder=reorder;fail_second=fail;fail_infer=inference_failure;delay_ms=delay;
    vision_options options={"unused","mock","build/pipeline-labels.txt",view,16,8,.35f,.45f,no_detect,serial,sync,npu_workers};
    auto *worker=vision_start(&options);assert(worker);
    auto snapshot=new vision_snapshot{};int changes=0,result_changes=0,delayed=0,expired=0,errors=0;
    uint64_t last_frame=0,last_result=0;
    const int steps=!no_detect&&!serial&&!fail&&!sync&&delay==80&&!inference_failure&&std::string(view)=="both"?220:110;
    for(int step=0;step<steps;step++){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));vision_snapshot_copy(worker,snapshot);
        if(!snapshot->have_frame)continue;
        if(last_frame!=snapshot->frame_id){++changes;last_frame=snapshot->frame_id;}
        if(snapshot->detection_frame_id && last_result!=snapshot->detection_frame_id){
            assert(snapshot->detection_frame_id>last_result);
            ++result_changes;last_result=snapshot->detection_frame_id;
        }
        unsigned pixel=snapshot->pixels[VISION_W*(VISION_H/2)+VISION_W/2];
        assert(((pixel>>16)&255)==(snapshot->frame_id&255));
        if(snapshot->count){
            int eyes=std::string(view)=="both"?2:1;assert(snapshot->count==eyes);
            if(eyes==2){assert(snapshot->objects[0].eye==0 && snapshot->objects[1].eye==1);assert(snapshot->objects[0].score==snapshot->objects[1].score);}
            else assert(snapshot->objects[0].eye==(std::string(view)=="right"?1:std::string(view)=="left"?0:-1));
            assert(std::fabs(snapshot->objects[0].score*255-(snapshot->detection_frame_id&255))<.01);
            assert(snapshot->detection_available && snapshot->detection_age_ms<=VISION_BOX_MAX_AGE_MS);
            if(sync)assert(snapshot->frame_id==snapshot->detection_frame_id && !snapshot->boxes_delayed);
            else if(snapshot->boxes_delayed){assert(snapshot->frame_id>snapshot->detection_frame_id);++delayed;}
        }
        if(snapshot->detection_age_ms>VISION_BOX_MAX_AGE_MS){assert(snapshot->count==0 && !snapshot->detection_available);++expired;}
        if(std::string(snapshot->status).find("AI error:")==0){++errors;assert(snapshot->count==0);}
    }
    assert(snapshot->have_frame && captures>30 && snapshot->capture_fps>25);
    if(no_detect)assert(opens==0 && maximum==0 && snapshot->count==0 && snapshot->fps==0 && snapshot->detection_age_ms<0);
    else {
        const bool both=std::string(view)=="both";
        int actual=npu_workers?npu_workers:3;
        if(serial||fail)actual=1;
        if(open_failure>0)actual=std::min(actual,open_failure-1);
        if(bind_failure!=-2)actual=std::min(actual,2);
        assert(snapshot->npu_workers==actual);
        assert(maximum==(both||actual==3?actual:1));assert(result_changes>0 || inference_failure);
        assert(snapshot->inflight_peak<=(actual==3?(both?2:3):1));
        if(actual==3 && !inference_failure)for(auto &n:core_calls)assert(n>2);
        if(reorder)assert(snapshot->discarded_pairs>0);
        if(bind_failure==-2||bind_failure==2)assert(snapshot->npu_pinned);
        if(delay>VISION_BOX_MAX_AGE_MS)assert(expired>0);
        else if(!sync)assert(delayed>0);
    }
    if(!sync){assert(changes>28 && snapshot->preview_fps>25);if(!no_detect)assert(changes>result_changes);}
    else assert(changes<=result_changes+1 && snapshot->preview_fps<20);
    if(inference_failure)assert(errors>10);
    vision_stop(worker);assert(active==0);delete snapshot;
    std::printf("PASS view=%s workers=%d sync=%d no_detect=%d serial=%d fallback=%d delay=%d error=%d preview_updates=%d result_updates=%d\n",
        view,npu_workers,sync,no_detect,serial,fail,delay,inference_failure,changes,result_changes);
}
static double benchmark(int workers) {
    captures=active=maximum=opens=calls=0;for(auto &n:core_calls)n=0;
    fail_second=fail_infer=disorder=false;fail_open_at=0;fail_bind_at=-2;
    delay_ms=120;camera_delay_ms=66;
    vision_options options={"unused","mock","build/pipeline-labels.txt","both",16,8,.35f,.45f,0,0,0,workers};
    auto *worker=vision_start(&options);assert(worker);auto snapshot=new vision_snapshot{};
    uint64_t start_count=0;auto start=std::chrono::steady_clock::now();double age_sum=0;int age_count=0;
    for(int i=0;i<400;i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));vision_snapshot_copy(worker,snapshot);
        if(i==100){start_count=snapshot->completed_pairs-snapshot->discarded_pairs;start=std::chrono::steady_clock::now();}
        if(i>100 && snapshot->count){age_sum+=snapshot->detection_age_ms;++age_count;}
    }
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    const double fps=(snapshot->completed_pairs-snapshot->discarded_pairs-start_count)/elapsed;
    assert(snapshot->preview_fps>13 && snapshot->capture_fps>13 && snapshot->inflight_peak<=2);
    assert(snapshot->npu_workers==workers && age_count);
    if(workers==3)assert(core_calls[2]>10);
    std::printf("BENCH synthetic_camera_ms=66 synthetic_eye_ms=120 workers=%d pair_fps=%.2f preview_fps=%.2f mean_box_age_ms=%.2f third_core_jobs=%d\n",
        workers,fps,snapshot->preview_fps,age_sum/age_count,core_calls[2].load());
    vision_stop(worker);delete snapshot;camera_delay_ms=20;return fps;
}
int main(int argc,char **argv){
    FILE *f=std::fopen("build/pipeline-labels.txt","w");assert(f);std::fputs("object\n",f);std::fclose(f);
    if(argc==2 && std::string(argv[1])=="--benchmark") {
        double dual=benchmark(2),triple=benchmark(3);
        assert(triple>dual*1.2);
        std::printf("PASS synthetic scheduler throughput ratio=%.3f (not measured RK3588 speed)\n",triple/dual);return 0;
    }
    check(0,0,false,0);check(0,1,false,0);check(0,0,true,0);check(1,0,false,0);
    check(0,0,false,1);check(0,0,false,0,450);check(0,0,false,0,80,true);
    check(0,0,false,0,80,false,"right");check(0,0,false,0,80,false,"single");
    check(0,0,false,0,80,false,"both",3);
    check(0,0,false,1,80,false,"both",3);
    check(0,0,false,0,450,false,"both",3);
    check(0,0,false,0,80,true,"both",3);
    check(0,0,false,0,80,false,"both",3,3);
    check(0,0,false,0,80,false,"both",3,0,2);
    check(0,0,false,0,80,false,"both",3,0,0);
    check(0,0,false,0,80,false,"both",3,0,-2,true);
    check(0,0,false,0,80,false,"single",3);
    check(0,0,false,0,80,false,"left",3);
    check(0,1,false,0,80,false,"both",3);
    check(1,0,false,0,80,false,"both",3);
    check(0,0,false,0,80,false,"both",0);
    std::puts("PASS triple workers, per-context exclusion, bounded in-flight frames, monotonic paired results, third-context/core fallback, preserved independent preview, matching stereo results, immutable pooled frames, bounded box age, sync comparison, failures and shutdown");
}
