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
#include <cstdlib>
#include <array>
#include <mutex>
#include <vector>
#include <memory>
static std::atomic<int> captures{0},active{0},maximum{0},opens{0},calls{0};
static bool fail_second=false,fail_infer=false;
static int fail_open_at=0,fail_bind_at=-2;
static bool disorder=false;
static bool moving_scene=false,many_boxes=false;
static std::atomic<int> core_calls[3];
static int delay_ms=80,camera_delay_ms=20;
struct ObservedJob {int frame,eye,core;};
static std::mutex observed_mutex;
static std::vector<ObservedJob> observed_jobs;
static bool variable_eyes=false;
static int slow_eye=-1,extra_eye_delay_ms=0,source_age_ms=0;
struct Camera::Impl {CameraLayout layout;};
Camera::Camera():p(new Impl){}
Camera::~Camera(){delete p;}
bool Camera::open(const std::string &,unsigned,unsigned,std::string &){p->layout.width=moving_scene?288:16;p->layout.height=moving_scene?162:8;return true;}
const CameraLayout &Camera::layout()const{return p->layout;}
std::string Camera::description()const{return moving_scene?"288x162 moving mock":"16x8 mock";}
int Camera::read(std::vector<uint8_t> &rgb,std::string &,CameraTiming *t) {
    std::this_thread::sleep_for(std::chrono::milliseconds(camera_delay_ms));
    int id=++captures;
    if(moving_scene){
        rgb.assign(288*162*3,40);
        unsigned random=0x87654321;std::array<unsigned char,400> texture{};
        for(auto &v:texture){random^=random<<13;random^=random>>17;random^=random<<5;v=30+(random>>24)%190;}
        for(int eye=0;eye<2;eye++)for(int y=0;y<42;y++)for(int x=0;x<32;x++){
            int gx=x/4,gy=y/4,fx=x%4,fy=y%4;
            unsigned char value=(texture[gy*20+gx]*(4-fx)*(4-fy)+texture[gy*20+gx+1]*fx*(4-fy)+texture[(gy+1)*20+gx]*(4-fx)*fy+texture[(gy+1)*20+gx+1]*fx*fy)/16;
            int at=((55+y)*288+(eye?190:40)+(id%35)+x)*3;
            rgb[at]=rgb[at+1]=rgb[at+2]=value;
        }
        for(int eye=0;eye<2;eye++){rgb[eye*144*3]=id;rgb[eye*144*3+1]=eye;}
    }else {rgb.assign(16*8*3,static_cast<uint8_t>(id));rgb[1]=0;rgb[8*3+1]=1;}
    if(t){*t={};t->wait_ms=camera_delay_ms;t->convert_ms=.25;
        if(source_age_ms)t->source_ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()-source_age_ms;
    }return 1;
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
    {std::lock_guard<std::mutex> lock(observed_mutex);observed_jobs.push_back({value,rgb[1],impl_->core});}
    // Force the first frame to complete after newer frames in the three-worker path.
    std::this_thread::sleep_for(std::chrono::milliseconds(disorder && value==1?250:
        variable_eyes?delay_ms+(rgb[1]?17:0)+(value%5)*3:delay_ms+(rgb[1]==slow_eye?extra_eye_delay_ms:0)));
    // Input must remain unchanged while capture and preview continue using the pool.
    assert(rgb[0]==value);--active;impl_->busy=false;
    if(fail_infer && ++calls>2){error="injected inference failure";return false;}
    if(moving_scene){
        int x=(rgb[1]?46:40)+value%35;result={{float(x),55,float(x+32),97,.8f,0}};
        if(many_boxes){result.resize(40,result[0]);for(int i=0;i<40;i++)result[i].score=.95f-i*.001f;}
    }
    else result={{1,1,6,6,value/255.f,0}};
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
            int eyes=std::string(view)=="both"?2:1;
            const int ready=int(bool(snapshot->eye_complete_mask&1))+int(bool(snapshot->eye_complete_mask&2));assert(snapshot->count==ready);
            assert(snapshot->source_count==snapshot->count&&snapshot->motion_compensated==0);
            for(int i=0;i<ready;i++)assert(snapshot->source_objects[i].left==snapshot->objects[i].left&&!snapshot->motion_valid[i]);
            if(ready==2){assert(snapshot->objects[0].eye==0 && snapshot->objects[1].eye==1);assert(snapshot->objects[0].score==snapshot->objects[1].score);}
            else if(eyes==1)assert(snapshot->objects[0].eye==(std::string(view)=="right"?1:std::string(view)=="left"?0:-1));
            assert(std::fabs(snapshot->objects[0].score*255-(snapshot->detection_frame_id&255))<.01);
            assert(snapshot->detection_available && snapshot->detection_age_ms<=VISION_BOX_MAX_AGE_MS);
            for(int e=0;e<2;e++)if(snapshot->eye_complete_mask&(1u<<e)){
                assert(snapshot->eye_frame_id[e]==snapshot->detection_frame_id);
                assert(snapshot->eye_source_timestamp_ms[e]==snapshot->detection_timestamp_ms);
                assert(snapshot->eye_result_timestamp_ms[e]<=snapshot->result_timestamp_ms);
            }
            assert(snapshot->eye_complete_mask && !(snapshot->eye_complete_mask&~unsigned(eyes==2?3:std::string(view)=="right"?2:1)));
            if(snapshot->pair_status==VISION_PAIR_PARTIAL)assert(!snapshot->control_eligible);
            if(sync)assert(snapshot->frame_id==snapshot->detection_frame_id && !snapshot->boxes_delayed);
            else if(snapshot->boxes_delayed){assert(snapshot->frame_id>snapshot->detection_frame_id);++delayed;}
        }
        if(snapshot->detection_age_ms>VISION_BOX_MAX_AGE_MS){
            assert(snapshot->count==0&&snapshot->source_count==0&&snapshot->motion_compensated==0&&!snapshot->detection_available);
            for(auto valid:snapshot->motion_valid)assert(!valid);
            ++expired;
        }
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
        assert(maximum==(both||actual==3?actual:1));assert(result_changes>0 || inference_failure || delay>VISION_BOX_MAX_AGE_MS);
        assert(snapshot->inflight_peak<=(actual==3?(both?2:3):1));
        if(actual==3 && !inference_failure)for(auto &n:core_calls)assert(n>2);
        if(reorder)assert(snapshot->discarded_pairs>0);
        if(bind_failure==-2||bind_failure==2)assert(snapshot->npu_pinned);
        if(delay>VISION_BOX_MAX_AGE_MS){assert(expired>0||snapshot->expired_pairs>0);assert(snapshot->count==0&&!snapshot->control_available);}
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
static void check_moving(bool enabled,bool budget=false) {
    captures=active=maximum=opens=calls=0;for(auto &n:core_calls)n=0;
    fail_second=fail_infer=disorder=false;fail_open_at=0;fail_bind_at=-2;
    moving_scene=true;many_boxes=budget;delay_ms=65;camera_delay_ms=20;
    setenv("PERCEPTION_BOX_MOTION",enabled?"1":"0",1);
    vision_options options={"unused","mock","build/pipeline-labels.txt","both",288,162,.35f,.45f,0,0,0,3};
    auto *worker=vision_start(&options);assert(worker);auto snapshot=new vision_snapshot{};
    int aligned=0,observed=0;uint64_t previous=0;
    for(int i=0;i<140;i++){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));vision_snapshot_copy(worker,snapshot);
        if(!snapshot->have_frame||!snapshot->count||snapshot->frame_id==previous)continue;
        previous=snapshot->frame_id;++observed;
        const int ready=int(bool(snapshot->eye_complete_mask&1))+int(bool(snapshot->eye_complete_mask&2));
        assert(snapshot->source_count==ready*(budget?40:1)&&snapshot->count==snapshot->source_count&&snapshot->motion_enabled==enabled);
        assert(snapshot->motion_compensated<=32);
        assert(snapshot->detection_frame_id<snapshot->frame_id&&snapshot->boxes_delayed);
        assert(snapshot->detection_timestamp_ms<snapshot->timestamp_ms);
        assert(snapshot->detection_age_ms<=VISION_BOX_MAX_AGE_MS);
        for(int index=0;index<snapshot->count;index++){
            const auto &raw=snapshot->source_objects[index];const auto &visible=snapshot->objects[index];const int eye=raw.eye;
            assert(raw.eye==eye&&visible.eye==eye&&raw.score==visible.score);
            assert(std::fabs(raw.left-((eye?190:40)+snapshot->detection_frame_id%35)*3)<.01);
            if(snapshot->motion_valid[index]){
                ++aligned;assert(enabled);
                assert(std::fabs(visible.left-((eye?190:40)+snapshot->frame_id%35)*3)<=3.1);
                assert(std::fabs(visible.top-165)<=3.1);
            }else assert(raw.left==visible.left&&raw.top==visible.top&&raw.right==visible.right&&raw.bottom==visible.bottom);
        }
    }
    assert(observed>35&&maximum==3&&snapshot->preview_fps>25);
    if(enabled)assert(aligned>30);else assert(aligned==0&&snapshot->motion_compensated==0);
    vision_stop(worker);assert(active==0);delete snapshot;moving_scene=many_boxes=false;unsetenv("PERCEPTION_BOX_MOTION");
    std::printf("PASS actual threaded image alignment enabled=%d detected_boxes=%d observed=%d aligned=%d; original source boxes and AI timestamps retained, max 32 alignments per frame\n",enabled,budget?80:2,observed,aligned);
}
static void check_v34_scheduler(bool overloaded){
    captures=active=maximum=opens=calls=0;for(auto &n:core_calls)n=0;
    fail_second=fail_infer=disorder=false;fail_open_at=0;fail_bind_at=-2;
    variable_eyes=overloaded;delay_ms=overloaded?45:5;camera_delay_ms=overloaded?20:33;
    {std::lock_guard<std::mutex> lock(observed_mutex);observed_jobs.clear();}
    setenv("PERCEPTION_BOX_MOTION","0",1);
    vision_options options={"unused","mock","build/pipeline-labels.txt","both",16,8,.35f,.45f,0,0,0,3};
    auto *worker=vision_start(&options);assert(worker);auto snapshot=new vision_snapshot{};
    uint64_t last_id=0;int updates=0;
    for(int i=0;i<240;i++){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));vision_snapshot_copy(worker,snapshot);
        if(snapshot->count){
            assert(snapshot->eye_complete_mask);
            for(int e=0;e<2;e++)if(snapshot->eye_complete_mask&(1u<<e))assert(snapshot->eye_frame_id[e]==snapshot->detection_frame_id);
            if(snapshot->eye_complete_mask==3)assert(snapshot->objects[0].score==snapshot->objects[1].score);
            if(snapshot->control_available)assert(snapshot->control_complete_mask==3&&snapshot->control_count==2);
            assert(snapshot->detection_frame_id>=last_id&&snapshot->detection_age_ms<=VISION_BOX_MAX_AGE_MS);
            if(snapshot->detection_frame_id!=last_id){last_id=snapshot->detection_frame_id;++updates;}
        }
    }
    assert(updates>20&&snapshot->published_pairs>20&&snapshot->completed_eye_jobs>=snapshot->completed_pairs*2);
    assert(snapshot->completed_pairs==snapshot->published_pairs+snapshot->discarded_pairs);
    if(overloaded)assert(snapshot->capture_superseded>0);else assert(snapshot->capture_superseded==0);
    vision_stop(worker);assert(active==0);
    std::vector<ObservedJob> jobs;{std::lock_guard<std::mutex> lock(observed_mutex);jobs=observed_jobs;}
    std::array<unsigned,256> seen{};unsigned eyes[2]={};
    for(const auto &job:jobs){assert(job.eye==0||job.eye==1);assert(!(seen[job.frame]&(1u<<job.eye)));seen[job.frame]|=1u<<job.eye;++eyes[job.eye];}
    assert(std::abs(int(eyes[0])-int(eyes[1]))<=2);
    std::printf("PASS real threaded v34 scheduler overloaded=%d pairs=%llu eye_jobs=(%u,%u) superseded=%llu updates=%d; same-source publication, bounded age, no unnecessary stable-load skip, drained shutdown\n",
        overloaded,(unsigned long long)snapshot->published_pairs,eyes[0],eyes[1],(unsigned long long)snapshot->capture_superseded,updates);
    delete snapshot;variable_eyes=false;camera_delay_ms=20;unsetenv("PERCEPTION_BOX_MOTION");
}
static void check_v35_early(int slow,int extra,bool age=false,bool reorder=false){
    captures=active=maximum=opens=calls=0;for(auto &n:core_calls)n=0;
    fail_second=fail_infer=variable_eyes=false;fail_open_at=0;fail_bind_at=-2;
    disorder=reorder;slow_eye=slow;extra_eye_delay_ms=extra;source_age_ms=age?500:0;
    delay_ms=5;camera_delay_ms=10;setenv("PERCEPTION_BOX_MOTION","0",1);
    vision_options o={"unused","mock","build/pipeline-labels.txt","both",16,8,.35f,.45f,0,0,0,3};
    auto *worker=vision_start(&o);assert(worker);auto s=std::make_unique<vision_snapshot>();
    uint64_t event=0,frame=0,control=0;unsigned partial=0,complete=0,warn=0,control_seen=0;
    for(int i=0;i<1000;i++){
        std::this_thread::sleep_for(std::chrono::milliseconds(2));vision_snapshot_copy(worker,s.get());
        if(!s->have_frame)continue;
        if(s->control_available){assert(s->control_complete_mask==3&&s->control_count==2);assert(s->control_frame_id>=control);control=s->control_frame_id;++control_seen;
            assert(s->control_objects[0].eye==0&&s->control_objects[1].eye==1&&s->control_objects[0].score==s->control_objects[1].score);}
        if(!s->count)continue;
        assert(s->detection_update_id>=event&&s->detection_frame_id>=frame);event=s->detection_update_id;frame=s->detection_frame_id;
        assert(s->eye_requested_mask==3&&s->detection_available);
        for(int n=0;n<s->count;n++){
            int e=s->objects[n].eye;assert(s->eye_complete_mask&(1u<<e));assert(s->eye_frame_id[e]==frame);
            assert(s->eye_source_timestamp_ms[e]==s->detection_timestamp_ms);
            assert(std::fabs(s->objects[n].score*255-(frame&255))<.01);
        }
        if(s->pair_status==VISION_PAIR_PARTIAL){++partial;assert(!s->control_eligible&&s->count==1&&(s->eye_complete_mask==1||s->eye_complete_mask==2));if(s->counterpart_timeout)++warn;}
        else{++complete;assert(s->pair_status==VISION_PAIR_COMPLETE&&s->eye_complete_mask==3&&s->count==2);}
    }
    if(age){assert(s->capture_age_expired>0&&s->completed_eye_jobs==0&&s->count==0&&!s->control_available);}
    else{
        assert(partial>10&&s->partial_events>=s->completed_pairs*2&&s->observation_updates>s->published_pairs);
        if(extra>=70)assert(warn>2&&s->pair_warn_timeouts>0);
        if(extra>100){assert(s->pair_control_timeouts>0&&control_seen==0&&!s->control_available);}
        else assert(control_seen>10);
        if(reorder)assert(s->out_of_order_pairs>0);
        assert(s->fps<s->eye_completion_fps*.8+1); // Complete FPS cannot count eye events.
    }
    const auto pairs=s->completed_pairs,updates=s->observation_updates,late=s->pair_control_timeouts;
    vision_stop(worker);assert(active==0);
    std::printf("PASS v35 early slow_eye=%d extra_ms=%d expired_source=%d disorder=%d partial_snapshots=%u complete_snapshots=%u warn_snapshots=%u complete_pairs=%llu events=%llu control_late=%llu; no mixed source, monotonic events, separate complete control, bounded shutdown\n",slow,extra,age,reorder,partial,complete,warn,(unsigned long long)pairs,(unsigned long long)updates,(unsigned long long)late);
    slow_eye=-1;extra_eye_delay_ms=source_age_ms=0;disorder=false;camera_delay_ms=20;unsetenv("PERCEPTION_BOX_MOTION");
}
static void check_v35_close(){
    delay_ms=15;camera_delay_ms=5;slow_eye=1;extra_eye_delay_ms=20;fail_second=fail_infer=disorder=variable_eyes=false;fail_open_at=0;fail_bind_at=-2;
    for(int i=0;i<12;i++){
        captures=active=maximum=opens=calls=0;
        vision_options o={"unused","mock","build/pipeline-labels.txt","both",16,8,.35f,.45f,0,0,0,3};
        auto *worker=vision_start(&o);assert(worker);std::this_thread::sleep_for(std::chrono::milliseconds(3+i*3));
        const auto start=std::chrono::steady_clock::now();vision_stop(worker);assert(active==0);
        assert(std::chrono::steady_clock::now()-start<std::chrono::seconds(1));
    }
    slow_eye=-1;extra_eye_delay_ms=0;camera_delay_ms=20;std::puts("PASS v35 12 stop/close races with capture/eye/partial/complete in flight; all joined");
}
int main(int argc,char **argv){
    FILE *f=std::fopen("build/pipeline-labels.txt","w");assert(f);std::fputs("object\n",f);std::fclose(f);
    if(argc==2 && std::string(argv[1])=="--benchmark") {
        double dual=benchmark(2),triple=benchmark(3);
        assert(triple>dual*1.2);
        std::printf("PASS synthetic scheduler throughput ratio=%.3f (not measured RK3588 speed)\n",triple/dual);return 0;
    }
    if(argc==2&&std::string(argv[1])=="--early-v35"){
        check_v35_early(0,70);check_v35_early(1,70);check_v35_early(0,170);check_v35_early(1,170);
        check_v35_early(1,70,false,true);check_v35_early(0,0,true);check_v35_close();return 0;
    }
    if(argc==2&&std::string(argv[1])=="--motion"){check_moving(true);check_moving(false);check_moving(true,true);return 0;}
    if(argc==2&&std::string(argv[1])=="--scheduler-v34"){check_v34_scheduler(false);check_v34_scheduler(true);check(0,0,false,0,80,true,"both",3);return 0;}
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
    check_moving(true);check_moving(false);check_moving(true,true);
    std::puts("PASS triple workers, per-context exclusion, bounded in-flight frames, monotonic paired results, third-context/core fallback, preserved independent preview, matching stereo results, immutable pooled frames, bounded box age, sync comparison, failures and shutdown");
}
