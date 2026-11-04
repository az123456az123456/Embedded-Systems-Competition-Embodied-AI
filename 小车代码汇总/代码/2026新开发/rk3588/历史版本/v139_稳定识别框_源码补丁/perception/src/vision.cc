#include "vision.h"
#include "camera.h"
#include "detector.h"
#include "stage_clock.h"
#include "perception_version.h"
#include "box_motion.h"
#include "box_stability.h"
#include "model_labels.h"
#include "eye_scheduler.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
static uint64_t now_ms(){return std::chrono::duration_cast<std::chrono::milliseconds>(StageClock::now().time_since_epoch()).count();}
struct Rate {
    uint64_t previous=0;double period=0;float fps=0;
    float update(uint64_t now){
        if(previous){double dt=std::max<uint64_t>(1,now-previous);period=period?.85*period+.15*dt:dt;fps=1000./period;}
        previous=now;return fps;
    }
};
// Count complete frames over a time window: several workers may finish together.
// Averaging reciprocal inter-completion gaps would overstate burst throughput.
struct CompletionRate {
    std::deque<uint64_t> times;
    float update(uint64_t now){
        times.push_back(now);
        while(times.size()>2 && (now-times.front()>2000 || times.size()>256))times.pop_front();
        return now>times.front()?float((times.size()-1)*1000.0/(now-times.front())):0;
    }
};
struct vision_worker {
    std::atomic<bool> stop{false};std::atomic<uint64_t> generation{0};std::thread thread;std::mutex mutex;
    vision_snapshot snapshot={};
    box_stability::Filter display_filter;
    std::string model,device,labels,view;int width,height;float confidence,nms;
    bool no_detect,serial_eyes,sync_preview,motion_enabled;int npu_workers;
};
static void status(vision_worker *w,const std::string &message) {
    std::lock_guard<std::mutex> lock(w->mutex);
    std::snprintf(w->snapshot.status,sizeof(w->snapshot.status),"%s",message.c_str());
    w->snapshot.display_count=0;
    w->display_filter.clear();
    w->snapshot.have_frame=0;w->snapshot.count=w->snapshot.source_count=w->snapshot.motion_compensated=0;
    w->snapshot.eye_complete_mask=0;w->snapshot.control_available=w->snapshot.control_count=0;
    std::fill(std::begin(w->snapshot.motion_valid),std::end(w->snapshot.motion_valid),0);
    ++w->generation;
    std::fprintf(stderr,"Vision: %s\n",message.c_str());
}
// Same nearest-neighbour mapping as before, computed once rather than per pixel/frame.
struct PreviewScaler {
    double scale;int ox,oy,ww,hh;std::vector<int> xs,ys;
    PreviewScaler(int width,int height){
        scale=std::min(VISION_W/double(width),VISION_H/double(height));
        ww=std::max(1,int(width*scale));hh=std::max(1,int(height*scale));
        ox=(VISION_W-ww)/2;oy=(VISION_H-hh)/2;xs.resize(ww);ys.resize(hh);
        for(int x=0;x<ww;x++)xs[x]=std::min(width-1,int(x/scale))*3;
        for(int y=0;y<hh;y++)ys[y]=std::min(height-1,int(y/scale))*width*3;
    }
    void render(vision_snapshot &out,const std::vector<uint8_t> &rgb)const{
        if(ww!=VISION_W || hh!=VISION_H)std::fill(std::begin(out.pixels),std::end(out.pixels),0xff14232d);
        for(int y=0;y<hh;y++){
            auto *dst=out.pixels+(y+oy)*VISION_W+ox;const auto *row=rgb.data()+ys[y];
            for(int x=0;x<ww;x++){const auto *p=row+xs[x];dst[x]=0xff000000|(p[0]<<16)|(p[1]<<8)|p[2];}
        }
    }
};
struct CapturedFrame {
    std::vector<uint8_t> rgb;uint64_t id=0,stamp=0;float fps=0;CameraTiming timing;
};
// Both consumers get immutable snapshots. A bounded pool reuses RGB allocations;
// a slot can only be written when no consumer or latest-frame pointer holds it.
struct CapturePump {
    std::atomic<bool> stop{false};std::mutex mutex;std::condition_variable changed;
    std::shared_ptr<const CapturedFrame> latest;std::string error;
    std::array<std::shared_ptr<CapturedFrame>,6> pool;std::thread thread;
    explicit CapturePump(Camera &camera){
        for(auto &slot:pool)slot=std::make_shared<CapturedFrame>();
        thread=std::thread([this,&camera]{
            try {
                uint64_t sequence=0;Rate rate;
                while(!stop){
                    std::shared_ptr<CapturedFrame> frame;
                    for(auto &slot:pool)if(slot.use_count()==1){frame=slot;break;}
                    if(!frame){std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;}
                    std::string message;int result=camera.read(frame->rgb,message,&frame->timing);
                    if(result<0){std::lock_guard<std::mutex> lock(mutex);error=message;changed.notify_all();return;}
                    if(!result)continue;
                    const auto completed=now_ms();
                    frame->stamp=frame->timing.source_ms && frame->timing.source_ms<=completed?frame->timing.source_ms:completed;
                    frame->id=++sequence;frame->fps=rate.update(completed);
                    {std::lock_guard<std::mutex> lock(mutex);latest=std::move(frame);}
                    changed.notify_all(); // preview and inference are independent consumers
                }
            }catch(const std::exception &e){std::lock_guard<std::mutex> lock(mutex);error=e.what();changed.notify_all();}
        });
    }
    ~CapturePump(){stop=true;changed.notify_all();if(thread.joinable())thread.join();}
    std::shared_ptr<const CapturedFrame> take(uint64_t after,std::string &message){
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait_for(lock,std::chrono::milliseconds(100),[&]{return stop||!error.empty()||(latest&&latest->id!=after);});
        message=error;return latest&&latest->id!=after?latest:nullptr;
    }
};
struct DetectionBatch {
    std::shared_ptr<const CapturedFrame> frame; // retained only in synchronized-preview mode
    uint64_t id=0,stamp=0,finished=0,sequence=0,skipped=0;
    float fps=0,total_ms=0;int count=0;
    std::array<vision_object,VISION_OBJECTS> objects{};
    box_motion::Gray reference;
    DetectorTiming timing[2];double queue_ms[2]={};int worker[2]={-1,-1};
    uint64_t eye_finished[2]={};unsigned complete_mask=0,requested_mask=0;
    int pair_status=VISION_PAIR_PARTIAL;bool control_eligible=false;
    double counterpart_wait_ms=0;
    uint32_t source_sequence=0;uint16_t source_epoch=0;
};
// One context is owned by one persistent thread. Each ready eye is published
// immediately; complete pairs also feed an independent control channel.
// Three workers overlap at most two frames (four eye jobs),
// retaining capture-time IDs rather than using RKNN's previous-frame async flag.
struct InferencePump {
    struct Work {
        std::shared_ptr<const CapturedFrame> frame;
        box_motion::Gray reference;
        std::vector<Detection> detections[2];DetectorTiming timing[2];
        double queue_ms[2]={};int worker[2]={-1,-1};
        int remaining,first,eyes,roi_width;uint64_t skipped;
        StageClock::time_point queued;
        eye_scheduler::PairState schedule;
        uint64_t eye_finished[2]={};
    };
    using Job=eye_scheduler::Job<Work>;
    struct Stats {
        uint64_t submitted=0,completed=0,discarded=0,jobs[3]={};
        double busy_ms[3]={};DetectorTiming timing_sum[3];int inflight_peak=0;
        uint64_t published=0,out_of_order=0,expired=0,pending_expired=0,capture_superseded=0;
        uint64_t eye_jobs[2]={},leading_eyes[2]={};double eye_queue_ms[2]={},eye_queue_max_ms[2]={};
        float eye_fps=0,complete_fps=0;
        uint64_t partial_events=0,partial_published=0,updates=0,pair_warn=0,control_timeout=0;
        uint64_t capture_age_expired=0,old_partial=0;
        double idle_ms[3]={};uint64_t queue_histogram[2][351]={};
    } stats;
    std::atomic<bool> stop{false};std::mutex mutex;std::condition_variable changed;
    std::shared_ptr<const DetectionBatch> latest,latest_control;std::string error;
    std::deque<Job> jobs;std::vector<std::thread> workers;std::thread scheduler;
    int inflight=0;uint64_t sequence=0;CompletionRate rate,eye_rate;
    InferencePump(vision_worker *w,CapturePump &capture,
                  std::vector<std::unique_ptr<RknnDetector>> &detectors,
                  int width,int height,const PreviewScaler &scaler,const std::vector<std::string> &names){
        const int first=w->view=="right"?1:0,eyes=w->view=="both"?2:1;
        // Dual mode retains one-frame scheduling for a controlled comparison.
        const int depth=detectors.size()==3?(eyes==2?2:3):1;
        try {
            for(size_t index=0;index<detectors.size();index++) {
                auto *detector=detectors[index].get();
                workers.emplace_back([this,w,detector,index,width,&scaler,&names,height]{
                    try {
                        while(!stop && !w->stop){
                            Job job;
                            {
                                std::unique_lock<std::mutex> lock(mutex);
                                auto idle_begin=StageClock::now();
                                changed.wait_for(lock,std::chrono::milliseconds(100),[&]{return stop||w->stop||!jobs.empty();});
                                stats.idle_ms[index]+=stage_elapsed(idle_begin);
                                if(stop||w->stop)return;
                                if(jobs.empty())continue;
                                // Drop only work that has never touched an NPU.
                                // A partially started pair must finish together.
                                auto dropped=eye_scheduler::drop_unstarted_expired(jobs,now_ms(),VISION_BOX_MAX_AGE_MS);
                                inflight-=static_cast<int>(dropped.size());stats.pending_expired+=dropped.size();
                                if(jobs.empty()){changed.notify_all();continue;}
                                job=eye_scheduler::take(jobs);
                            }
                            const auto &work=job.work;const int eye=job.eye;
                            const int offset=w->view=="single"?0:eye*work->roi_width;
                            std::vector<Detection> detections;DetectorTiming timing;std::string message;
                            const double queue_ms=stage_elapsed(work->queued);auto begin=StageClock::now();
                            if(!detector->infer(work->frame->rgb.data()+offset*3,work->roi_width,height,width*3,
                                                detections,message,&timing)){fail(message);return;}
                            const double busy=stage_elapsed(begin);
                            {
                                std::lock_guard<std::mutex> lock(mutex);
                                stats.jobs[index]++;stats.busy_ms[index]+=busy;
                                stats.eye_jobs[eye]++;stats.eye_queue_ms[eye]+=queue_ms;
                                stats.eye_queue_max_ms[eye]=std::max(stats.eye_queue_max_ms[eye],queue_ms);
                                ++stats.queue_histogram[eye][std::min(350u,unsigned(std::max(0.,queue_ms)))];
                                stats.eye_fps=eye_rate.update(now_ms());
                                auto &sum=stats.timing_sum[index];
                                sum.preprocess_ms+=timing.preprocess_ms;sum.input_ms+=timing.input_ms;
                                sum.run_ms+=timing.run_ms;sum.output_ms+=timing.output_ms;
                                sum.postprocess_ms+=timing.postprocess_ms;sum.release_ms+=timing.release_ms;
                                sum.prep_plan_ms+=timing.prep_plan_ms;sum.prep_fill_ms+=timing.prep_fill_ms;
                                sum.prep_resize_ms+=timing.prep_resize_ms;sum.prep_copy_ms+=timing.prep_copy_ms;
                                sum.prep_rga_ms+=timing.prep_rga_ms;sum.prep_sync_ms+=timing.prep_sync_ms;
                                sum.prep_validate_ms+=timing.prep_validate_ms;sum.input_sync_ms+=timing.input_sync_ms;
                                if(stop||w->stop)return;
                                work->detections[eye]=std::move(detections);work->timing[eye]=timing;
                                work->queue_ms[eye]=queue_ms;work->worker[eye]=static_cast<int>(index);
                                work->eye_finished[eye]=now_ms();work->schedule.completed|=1u<<eye;
                                --work->remaining;++stats.partial_events;
                                const auto finished=now_ms();
                                const bool complete=work->remaining==0;
                                const auto first_done=work->eyes==2?std::min(work->eye_finished[0],work->eye_finished[1]):finished;
                                const double pair_wait=complete&&work->eyes==2?double(finished-first_done):0;
                                if(complete){
                                    --inflight;++stats.completed;stats.complete_fps=rate.update(finished);
                                    if(pair_wait>VISION_PAIR_WARN_MS)++stats.pair_warn;
                                    if(pair_wait>VISION_PAIR_CONTROL_TIMEOUT_MS)++stats.control_timeout;
                                }
                                auto publish=[&](unsigned mask,bool is_complete){
                                    auto batch=std::make_shared<DetectionBatch>();
                                    if(w->motion_enabled)batch->reference=work->reference;
                                    for(int e=work->first;e<work->first+work->eyes;e++)if(mask&(1u<<e)) {
                                        batch->timing[e]=work->timing[e];batch->queue_ms[e]=work->queue_ms[e];
                                        batch->worker[e]=work->worker[e];batch->eye_finished[e]=work->eye_finished[e];
                                        for(const auto &d:work->detections[e]) {
                                            if(batch->count==VISION_OBJECTS)break;
                                            const int xoff=w->view=="single"?0:e*work->roi_width;
                                            auto &o=batch->objects[batch->count++];
                                            o.left=(d.left+xoff)*scaler.scale+scaler.ox;o.right=(d.right+xoff)*scaler.scale+scaler.ox;
                                            o.top=d.top*scaler.scale+scaler.oy;o.bottom=d.bottom*scaler.scale+scaler.oy;
                                            o.score=d.score;o.class_id=d.class_id;o.eye=w->view=="single"?-1:e;
                                            std::snprintf(o.name,sizeof(o.name),"%s",names.at(d.class_id).c_str());
                                        }
                                    }
                                    batch->id=work->frame->id;batch->stamp=work->frame->stamp;batch->finished=finished;
                                    batch->source_sequence=work->frame->timing.source_sequence;batch->source_epoch=work->frame->timing.source_epoch;
                                    batch->sequence=++sequence;batch->skipped=work->skipped;batch->fps=stats.complete_fps;
                                    batch->complete_mask=mask;batch->requested_mask=work->schedule.requested;
                                    batch->pair_status=is_complete?VISION_PAIR_COMPLETE:VISION_PAIR_PARTIAL;
                                    batch->counterpart_wait_ms=is_complete?pair_wait:0;
                                    const bool expired=finished>=batch->stamp&&finished-batch->stamp>VISION_BOX_MAX_AGE_MS;
                                    batch->control_eligible=is_complete&&!expired&&pair_wait<=VISION_PAIR_CONTROL_TIMEOUT_MS;
                                    batch->total_ms=stage_elapsed(work->queued);
                                    if(w->sync_preview)batch->frame=work->frame;
                                    // Control is complete-only and monotonic independently of the
                                    // display's earlier next-frame partial publication.
                                    if(batch->control_eligible&&(!latest_control||batch->id>latest_control->id))latest_control=batch;
                                    if(expired){if(is_complete){++stats.discarded;++stats.expired;}return;}
                                    if(latest&&batch->id<latest->id){
                                        if(is_complete){++stats.discarded;++stats.out_of_order;}else ++stats.old_partial;
                                        return;
                                    }
                                    // Synchronized-preview remains a complete-only comparison mode.
                                    if(w->sync_preview&&!is_complete)return;
                                    ++stats.updates;
                                    if(is_complete)++stats.published;else ++stats.partial_published;
                                    latest=std::move(batch);
                                };
                                publish(1u<<eye,false);
                                if(complete)publish(work->schedule.completed,true);
                            }
                            changed.notify_all();
                        }
                    }catch(const std::exception &e){fail(e.what());}
                });
            }
            scheduler=std::thread([this,w,&capture,first,eyes,depth,width,height,&scaler]{
                try {
                    uint64_t last_id=0,skipped=0;
                    while(!stop && !w->stop){
                        {
                            std::unique_lock<std::mutex> lock(mutex);
                            changed.wait_for(lock,std::chrono::milliseconds(100),[&]{return stop||w->stop||inflight<depth;});
                            if(stop||w->stop)return;
                            if(inflight>=depth)continue;
                        }
                        // Take the latest capture only AFTER capacity becomes available.
                        // No unbounded camera queue and no reuse of an in-flight frame.
                        std::string message;auto frame=capture.take(last_id,message);
                        if(!message.empty()){fail(message);return;}
                        if(!frame)continue;
                        if(last_id && frame->id>last_id+1)skipped+=frame->id-last_id-1;
                        last_id=frame->id;
                        // Capture is a one-slot latest-frame mailbox, not an
                        // accumulating queue. Never start already expired work.
                        if(now_ms()>=frame->stamp&&now_ms()-frame->stamp>VISION_BOX_MAX_AGE_MS){
                            std::lock_guard<std::mutex> lock(mutex);++stats.capture_age_expired;continue;
                        }
                        auto work=std::make_shared<Work>();work->frame=std::move(frame);
                        // Only a compact immutable reference follows an NPU job.
                        // No extra full-resolution frame is retained by a result.
                        if(w->motion_enabled)work->reference.from_rgb(work->frame->rgb.data(),width,height,
                            scaler.ox,scaler.oy,scaler.ww,scaler.hh,scaler.scale);
                        work->first=first;work->eyes=work->remaining=eyes;
                        work->roi_width=w->view=="single"?width:width/2;work->skipped=skipped;work->queued=StageClock::now();
                        work->schedule.frame_id=work->frame->id;work->schedule.source_ms=work->frame->stamp;
                        {
                            std::lock_guard<std::mutex> lock(mutex);
                            if(stop||w->stop)return;
                            ++inflight;++stats.submitted;stats.inflight_peak=std::max(stats.inflight_peak,inflight);
                            stats.capture_superseded=skipped;
                            const auto ordinal=stats.submitted-1;
                            ++stats.leading_eyes[eyes==2?ordinal%2:first];
                            eye_scheduler::enqueue(jobs,work,first,eyes,ordinal);
                        }
                        changed.notify_all();
                    }
                }catch(const std::exception &e){fail(e.what());}
            });
        }catch(...){shutdown();throw;}
    }
    void fail(const std::string &message){
        {std::lock_guard<std::mutex> lock(mutex);if(error.empty())error=message;latest.reset();latest_control.reset();stop=true;}
        std::fprintf(stderr,"Inference: %s\n",message.c_str());changed.notify_all();
    }
    std::shared_ptr<const DetectionBatch> get(uint64_t after,bool wait,std::string &message){
        std::unique_lock<std::mutex> lock(mutex);
        if(wait)changed.wait_for(lock,std::chrono::milliseconds(100),[&]{return stop||!error.empty()||(latest&&latest->sequence!=after);});
        message=error;
        return !wait||(latest&&latest->sequence!=after)?latest:nullptr;
    }
    std::shared_ptr<const DetectionBatch> control(){std::lock_guard<std::mutex> lock(mutex);return latest_control;}
    Stats counters(){std::lock_guard<std::mutex> lock(mutex);return stats;}
    void shutdown(){
        stop=true;changed.notify_all();
        if(scheduler.joinable())scheduler.join();
        for(auto &worker:workers)if(worker.joinable())worker.join();
    }
    ~InferencePump(){shutdown();}
};
static void expire_boxes(vision_snapshot &s,uint64_t now){
    if(s.display_layer_valid){
        int count=0;for(int i=0;i<s.display_count;i++){const auto&o=s.display_objects[i];
            if(now>=o.source_ms&&now-o.source_ms<=VISION_DISPLAY_HOLD_MS)s.display_objects[count++]=o;}s.display_count=count;
    }
    if(s.demo)return;
    if(s.have_frame)s.frame_age_ms=now>=s.timestamp_ms?now-s.timestamp_ms:0;
    s.detection_age_ms=s.detection_timestamp_ms&&now>=s.detection_timestamp_ms?float(now-s.detection_timestamp_ms):-1.f;
    s.detection_available=s.detection_timestamp_ms && s.detection_age_ms>=0 && s.detection_age_ms<=VISION_BOX_MAX_AGE_MS;
    if(!s.detection_available){s.count=s.source_count=s.motion_compensated=0;s.eye_complete_mask=0;std::fill(std::begin(s.motion_valid),std::end(s.motion_valid),0);}
    if(s.control_timestamp_ms&&now>=s.control_timestamp_ms&&now-s.control_timestamp_ms>VISION_BOX_MAX_AGE_MS){s.control_available=0;s.control_count=0;}
    if(s.pair_status==VISION_PAIR_PARTIAL&&s.eye_complete_mask){
        uint64_t first=0;for(int e=0;e<2;e++)if(s.eye_complete_mask&(1u<<e))first=first?std::min(first,s.eye_result_timestamp_ms[e]):s.eye_result_timestamp_ms[e];
        s.counterpart_wait_ms=first&&now>=first?now-first:0;s.counterpart_timeout=s.counterpart_wait_ms>VISION_PAIR_WARN_MS;
    }
    if(s.result_timestamp_ms && now-s.result_timestamp_ms>1000)s.fps=s.eye_completion_fps=0;
}
static double queue_p95(const uint64_t histogram[351],uint64_t count,double maximum){
    if(!count)return 0;
    const uint64_t target=(count*95+99)/100;uint64_t sum=0;
    for(unsigned i=0;i<351;i++){sum+=histogram[i];if(sum>=target)return i==350?maximum:double(i);}
    return 0;
}
static void print_eye(const char *eye,const DetectionBatch &b,int index,bool pinned){
    const auto &t=b.timing[index];
    std::fprintf(stderr,"STAGE eye=%s frame=%llu preprocess_ms=%.2f input_ms=%.2f run_ms=%.2f output_ms=%.2f post_ms=%.2f release_ms=%.2f total_ms=%.2f queue_ms=%.2f worker=%d core=%d preprocess_backend=%d prep_plan_ms=%.3f prep_fill_ms=%.3f prep_resize_ms=%.3f prep_copy_ms=%.3f prep_rga_ms=%.3f prep_sync_ms=%.3f prep_validate_ms=%.3f input_sync_ms=%.3f source_ms=%llu eye_finished_ms=%llu complete_mask=%u\n",
        eye,(unsigned long long)b.id,t.preprocess_ms,t.input_ms,t.run_ms,t.output_ms,t.postprocess_ms,t.release_ms,t.total_ms,b.queue_ms[index],b.worker[index],pinned?b.worker[index]:-1,
        t.preprocess_backend,t.prep_plan_ms,t.prep_fill_ms,t.prep_resize_ms,t.prep_copy_ms,t.prep_rga_ms,t.prep_sync_ms,t.prep_validate_ms,t.input_sync_ms,
        (unsigned long long)b.stamp,(unsigned long long)b.eye_finished[index],b.complete_mask);
}
static void run(vision_worker *w){
    try {
        std::vector<std::string> names;std::string error;
        std::vector<std::unique_ptr<RknnDetector>> detectors;bool pinned=true;
        const int requested=w->serial_eyes?1:(w->npu_workers?w->npu_workers:3);
        if(!w->no_detect){
            if(!load_model_labels(w->labels,names,error)){status(w,error);return;}
            std::fprintf(stderr,"LABELS path=%s classes=%zu\n",w->labels.c_str(),names.size());
            for(int index=0;index<requested;index++) {
                auto detector=std::make_unique<RknnDetector>();
                if(!detector->open(w->model,w->confidence,w->nms,names.size(),error)) {
                    if(index==0){status(w,error);return;}
                    std::fprintf(stderr,"NPU fallback: context %d unavailable (%s); using %zu workers\n",index,error.c_str(),detectors.size());
                    break;
                }
                const bool bound=detector->set_core(index,error);
                if(!bound) {
                    std::fprintf(stderr,"NPU binding: worker=%d: %s\n",index,error.c_str());
                    if(index==2){std::fprintf(stderr,"NPU fallback: core 2 binding failed; keeping two workers\n");break;}
                    pinned=false;
                }
                detectors.push_back(std::move(detector));
                if(index==1 && !pinned) {
                    std::fprintf(stderr,"NPU fallback: explicit core selection unavailable; keeping two automatic workers\n");break;
                }
            }
        }
        if(w->stop)return;
        Camera camera;
        if(!camera.open(w->device,w->width,w->height,error)){status(w,error);return;}
        const int width=camera.layout().width,height=camera.layout().height;
        const char *camera_source=std::getenv("PERCEPTION_CAMERA_SOURCE");
        const bool prepared_rgb=w->device.compare(0,9,"fpga-rgb:")==0 ||
            (camera_source&&!std::strcmp(camera_source,"fpga-rgb"));
        if(prepared_rgb&&w->view=="single"){status(w,"FPGA RGB is two prepared eyes; select both, left or right (not single)");return;}
        if(w->view!="single"&&width%2){status(w,"Stereo input width must be even; use --view single for mono");return;}
        const bool sync=w->sync_preview&&!w->no_detect;
        const int worker_count=static_cast<int>(detectors.size());
        const char *mode=worker_count==3?"pipeline3":worker_count==2?"parallel":"serial";
        const char *motion_setting=std::getenv("PERCEPTION_BOX_MOTION");
        w->motion_enabled=!sync&&!w->no_detect&&(!motion_setting||std::strcmp(motion_setting,"0"));
        std::fprintf(stderr,"PERCEPTION " PERCEPTION_VERSION ": Camera: %s, view=%s, detect=%s, stereo=%s, preview=%s, box_max_age_ms=%d npu_requested=%d npu_workers=%d core_binding=%s box_motion=%s\n",
            camera.description().c_str(),w->view.c_str(),w->no_detect?"off":"on",mode,
            sync?"synchronized":"independent",VISION_BOX_MAX_AGE_MS,requested,worker_count,pinned&&worker_count?"pinned":"auto",w->motion_enabled?"image-align":"off");
        PreviewScaler scaler(width,height);CapturePump capture(camera);
        std::unique_ptr<InferencePump> inference;
        if(!w->no_detect)inference.reset(new InferencePump(w,capture,detectors,width,height,scaler,names));
        vision_snapshot next={};Rate preview_rate;InferencePump::Stats previous_counters{};
        box_motion::Gray current_gray;
        std::snprintf(next.source,sizeof(next.source),"%s / %s",camera.description().c_str(),w->view.c_str());
        next.detection_enabled=!w->no_detect;next.npu_workers=worker_count;next.npu_pinned=pinned&&worker_count;
        uint64_t last_frame=now_ms(),last_id=0,last_result=0,report=last_frame;bool timed_out=false;
        while(!w->stop){
            std::string ai_error;std::shared_ptr<const DetectionBatch> result;
            std::shared_ptr<const CapturedFrame> frame;
            if(sync){
                result=inference->get(last_result,true,ai_error);
                if(!ai_error.empty()){status(w,"Inference: "+ai_error);return;}
                if(result){frame=result->frame;last_result=result->sequence;}
            }else {
                frame=capture.take(last_id,error);
                if(!error.empty()){status(w,error);return;}
            }
            if(!frame){
                if(!timed_out&&now_ms()-last_frame>1000){status(w,"Camera/result timeout; waiting for frames");timed_out=true;}
                continue;
            }
            last_id=frame->id;last_frame=now_ms();timed_out=false;
            next.image_geometry_kind=frame->timing.fpga_preprocessed?1:0;
            next.tensor_eye_width=frame->timing.fpga_preprocessed?640:width/2;
            next.tensor_eye_height=height;
            next.physical_eye_width=960;next.physical_eye_height=1080;
            next.tensor_content_x=frame->timing.fpga_preprocessed?35:0;next.tensor_content_y=0;
            next.tensor_content_width=frame->timing.fpga_preprocessed?569:width/2;
            next.tensor_content_height=height;next.preview_content_y=scaler.oy;next.preview_content_height=scaler.hh;
            next.camera_source_sequence=frame->timing.source_sequence;next.camera_source_epoch=frame->timing.source_epoch;
            auto tick=StageClock::now();scaler.render(next,frame->rgb);next.preview_ms=stage_elapsed(tick);
            if(!sync&&inference)result=inference->get(0,false,ai_error);
            next.count=0;next.detection_timestamp_ms=next.detection_frame_id=next.result_timestamp_ms=0;
            next.eye_complete_mask=0;std::fill(std::begin(next.eye_frame_id),std::end(next.eye_frame_id),0);
            std::fill(std::begin(next.eye_source_timestamp_ms),std::end(next.eye_source_timestamp_ms),0);
            std::fill(std::begin(next.eye_result_timestamp_ms),std::end(next.eye_result_timestamp_ms),0);
            next.fps=next.inference_ms=0;next.boxes_delayed=0;
            next.source_count=next.motion_compensated=0;next.motion_enabled=w->motion_enabled;
            next.tracking_ms=0;std::fill(std::begin(next.motion_valid),std::end(next.motion_valid),0);
            const auto counters=inference?inference->counters():InferencePump::Stats{};
            next.inflight_peak=counters.inflight_peak;next.completed_pairs=counters.completed;next.discarded_pairs=counters.discarded;
            next.completed_eye_jobs=counters.eye_jobs[0]+counters.eye_jobs[1];next.published_pairs=counters.published;
            next.out_of_order_pairs=counters.out_of_order;next.expired_pairs=counters.expired;
            next.pending_expired_pairs=counters.pending_expired;next.capture_superseded=counters.capture_superseded;
            next.eye_completion_fps=counters.eye_fps;next.fps=counters.complete_fps;
            next.partial_events=counters.partial_events;next.partial_publications=counters.partial_published;next.observation_updates=counters.updates;
            next.pair_warn_timeouts=counters.pair_warn;next.pair_control_timeouts=counters.control_timeout;
            next.capture_age_expired=counters.capture_age_expired;next.unstarted_age_expired=counters.pending_expired;
            next.control_available=next.control_count=0;next.control_complete_mask=0;
            const auto control=inference?inference->control():nullptr;
            if(control){next.control_update_id=control->sequence;next.control_frame_id=control->id;next.control_timestamp_ms=control->stamp;
                next.control_source_sequence=control->source_sequence;next.control_source_epoch=control->source_epoch;
                next.control_complete_mask=control->complete_mask;next.control_count=control->count;next.control_available=control->control_eligible;
                std::copy_n(control->objects.begin(),control->count,next.control_objects);}
            next.pair_status=next.counterpart_timeout=next.control_eligible=0;next.counterpart_wait_ms=0;
            next.eye_requested_mask=0;next.detection_update_id=0;
            if(result){
                next.detection_frame_id=result->id;next.detection_timestamp_ms=result->stamp;next.result_timestamp_ms=result->finished;
                next.inference_ms=result->total_ms;
                next.detection_update_id=result->sequence;next.pair_status=result->pair_status;
                next.control_eligible=result->control_eligible;next.counterpart_wait_ms=result->counterpart_wait_ms;
                next.counterpart_timeout=result->counterpart_wait_ms>VISION_PAIR_WARN_MS;
                next.eye_requested_mask=result->requested_mask;next.detection_source_sequence=result->source_sequence;next.detection_source_epoch=result->source_epoch;
                // Never apply a result from a newer frame to an older preview.
                if(result->id<=frame->id){
                    next.eye_complete_mask=result->complete_mask;
                    for(int eye=0;eye<2;eye++)if(result->complete_mask&(1u<<eye)){
                        next.eye_frame_id[eye]=result->id;next.eye_source_timestamp_ms[eye]=result->stamp;
                        next.eye_result_timestamp_ms[eye]=result->eye_finished[eye];
                    }
                    next.count=result->count;std::copy_n(result->objects.begin(),next.count,next.objects);
                    next.source_count=next.count;std::copy_n(result->objects.begin(),next.count,next.source_objects);
                    next.boxes_delayed=result->id!=frame->id;
                    if(w->motion_enabled&&next.boxes_delayed&&frame->stamp>=result->stamp&&
                       frame->stamp-result->stamp<=VISION_BOX_MAX_AGE_MS&&now_ms()-result->stamp<=VISION_BOX_MAX_AGE_MS&&ai_error.empty()){
                        auto tracking_begin=StageClock::now();current_gray.from_preview(next.pixels);
                        std::array<int,VISION_OBJECTS> order{};for(int i=0;i<next.count;i++)order[i]=i;
                        std::stable_sort(order.begin(),order.begin()+next.count,[&](int a,int b){return next.objects[a].score>next.objects[b].score;});
                        for(int n=0;n<std::min(next.count,box_motion::MAX_BOXES);n++){
                            int i=order[n];
                            auto motion=box_motion::estimate(result->reference,current_gray,next.source_objects[i],frame->stamp-result->stamp);
                            if(box_motion::apply(motion,next.source_objects[i],next.objects[i])){
                                next.motion_valid[i]=1;++next.motion_compensated;
                            }
                        }
                        next.tracking_ms=stage_elapsed(tracking_begin);
                    }
                }
            }
            next.have_frame=1;next.timestamp_ms=frame->stamp;next.frame_id=frame->id;next.capture_fps=frame->fps;
            const auto finished=now_ms();next.preview_fps=preview_rate.update(finished);expire_boxes(next,finished);
            w->display_filter.update(next.control_frame_id,next.control_timestamp_ms,next.control_objects,
                next.control_count,finished,next.detection_enabled&&ai_error.empty());
            next.display_layer_valid=1;
            next.display_count=w->display_filter.copy(next.display_objects,VISION_OBJECTS,finished);
            if(ai_error.empty())std::snprintf(next.status,sizeof(next.status),"%s ready=%u/%u%s",next.pair_status==VISION_PAIR_PARTIAL?"PAIR_PARTIAL":"PAIR_COMPLETE",next.eye_complete_mask,next.eye_requested_mask,next.counterpart_timeout?" counterpart_timeout":"");
            else std::snprintf(next.status,sizeof(next.status),"AI error: %s",ai_error.c_str());
            {std::lock_guard<std::mutex> lock(w->mutex);w->snapshot=next;++w->generation;}
            if(finished-report>=2000){
                const auto &c=frame->timing;
                std::fprintf(stderr,"PERF capture_fps=%.2f preview_fps=%.2f result_fps=%.2f inference_ms=%.2f frame_age_ms=%.1f box_age_ms=%.1f detect=%s stereo=%s preview=%s frame=%llu result_frame=%llu ai_skipped=%llu npu_workers=%d core_binding=%s inflight_peak=%d completed_pairs=%llu discarded_pairs=%llu eye_fps=%.2f published_pairs=%llu expired_pairs=%llu pending_expired_pairs=%llu out_of_order_pairs=%llu\n",
                    next.capture_fps,next.preview_fps,next.fps,next.inference_ms,next.frame_age_ms,next.detection_age_ms,
                    w->no_detect?"off":"on",mode,sync?"sync":"async",
                    (unsigned long long)frame->id,(unsigned long long)next.detection_frame_id,(unsigned long long)counters.capture_superseded,worker_count,pinned&&worker_count?"pinned":"auto",
                    counters.inflight_peak,(unsigned long long)counters.completed,(unsigned long long)counters.discarded,
                    next.eye_completion_fps,(unsigned long long)counters.published,(unsigned long long)counters.expired,
                    (unsigned long long)counters.pending_expired,(unsigned long long)counters.out_of_order);
                std::fprintf(stderr,"STAGE scheduler policy=oldest-started-pair-first alternating_first_eye=1 pending_capture=latest-only capture_superseded=%llu eye_completed=(%llu,%llu) mean_eye_queue_ms=(%.2f,%.2f) max_eye_queue_ms=(%.2f,%.2f) leading_eyes=(%llu,%llu) publication=early-eye-then-complete complete_mask=%u source_ids=(%llu,%llu)\n",
                    (unsigned long long)counters.capture_superseded,(unsigned long long)counters.eye_jobs[0],(unsigned long long)counters.eye_jobs[1],
                    counters.eye_queue_ms[0]/std::max<uint64_t>(1,counters.eye_jobs[0]),counters.eye_queue_ms[1]/std::max<uint64_t>(1,counters.eye_jobs[1]),
                    counters.eye_queue_max_ms[0],counters.eye_queue_max_ms[1],(unsigned long long)counters.leading_eyes[0],(unsigned long long)counters.leading_eyes[1],
                    next.eye_complete_mask,(unsigned long long)next.eye_frame_id[0],(unsigned long long)next.eye_frame_id[1]);
                std::fprintf(stderr,"STAGE pair_events partial=%llu partial_published=%llu updates=%llu complete=%llu complete_published=%llu pair_warn50=%llu control_timeout100=%llu control_frame=%llu control_ready=%d pair_status=%s event_id=%llu p95_eye_queue_ms=(%.2f,%.2f) skip_policy=capacity-latest-only skip_capacity=%llu skip_capture_age=%llu skip_unstarted_age=%llu discard_old_partial=%llu drop_pair_old=%llu drop_pair_age=%llu\n",
                    (unsigned long long)counters.partial_events,(unsigned long long)counters.partial_published,(unsigned long long)counters.updates,
                    (unsigned long long)counters.completed,(unsigned long long)counters.published,(unsigned long long)counters.pair_warn,(unsigned long long)counters.control_timeout,
                    (unsigned long long)next.control_frame_id,next.control_available,next.pair_status==VISION_PAIR_PARTIAL?"PAIR_PARTIAL":"PAIR_COMPLETE",(unsigned long long)next.detection_update_id,
                    queue_p95(counters.queue_histogram[0],counters.eye_jobs[0],counters.eye_queue_max_ms[0]),queue_p95(counters.queue_histogram[1],counters.eye_jobs[1],counters.eye_queue_max_ms[1]),
                    (unsigned long long)counters.capture_superseded,(unsigned long long)counters.capture_age_expired,(unsigned long long)counters.pending_expired,
                    (unsigned long long)counters.old_partial,(unsigned long long)counters.out_of_order,(unsigned long long)counters.expired);
                std::fprintf(stderr,"STAGE camera_frame=%llu wait_ms=%.2f dequeue_ms=%.2f color_ms=%.2f requeue_ms=%.2f preview_ms=%.2f\n",
                    (unsigned long long)frame->id,c.wait_ms,c.dequeue_ms,c.convert_ms,c.requeue_ms,next.preview_ms);
                std::fprintf(stderr,"STAGE box_motion=%s tracking_ms=%.3f aligned=%d total=%d source_frame=%llu preview_frame=%llu source_age_ms=%.1f\n",
                    w->motion_enabled?"image-align":"off",next.tracking_ms,next.motion_compensated,next.count,
                    (unsigned long long)next.detection_frame_id,(unsigned long long)next.frame_id,next.detection_age_ms);
                if(result){
                    if((result->complete_mask&1)&&w->view!="right")print_eye(w->view=="single"?"mono":"L",*result,0,pinned);
                    if((result->complete_mask&2)&&(w->view=="both"||w->view=="right"))print_eye("R",*result,1,pinned);
                }
                for(int i=0;i<worker_count;i++) {
                    std::fprintf(stderr,"STAGE worker_idle worker=%d total_idle_ms=%.2f window_idle_ms=%.2f window_ms=%llu\n",i,counters.idle_ms[i],counters.idle_ms[i]-previous_counters.idle_ms[i],(unsigned long long)(finished-report));
                    auto count=counters.jobs[i]-previous_counters.jobs[i];
                    const auto &t=counters.timing_sum[i],&old=previous_counters.timing_sum[i];
                    double n=count?count:1;
                    std::fprintf(stderr,"STAGE npu_worker=%d core=%d jobs=%llu busy_ms=%.2f window_ms=%llu window_jobs=%llu mean_eye_ms=%.2f mean_preprocess_ms=%.2f mean_input_ms=%.2f mean_run_ms=%.2f mean_output_ms=%.2f mean_prep_fill_ms=%.3f mean_prep_resize_ms=%.3f mean_prep_copy_ms=%.3f mean_prep_rga_ms=%.3f mean_prep_sync_ms=%.3f mean_prep_validate_ms=%.3f mean_input_sync_ms=%.3f\n",
                        i,pinned?i:-1,(unsigned long long)counters.jobs[i],counters.busy_ms[i],
                        (unsigned long long)(finished-report),(unsigned long long)count,
                        (counters.busy_ms[i]-previous_counters.busy_ms[i])/n,
                        (t.preprocess_ms-old.preprocess_ms)/n,(t.input_ms-old.input_ms)/n,
                        (t.run_ms-old.run_ms)/n,(t.output_ms-old.output_ms)/n,
                        (t.prep_fill_ms-old.prep_fill_ms)/n,(t.prep_resize_ms-old.prep_resize_ms)/n,
                        (t.prep_copy_ms-old.prep_copy_ms)/n,(t.prep_rga_ms-old.prep_rga_ms)/n,
                        (t.prep_sync_ms-old.prep_sync_ms)/n,(t.prep_validate_ms-old.prep_validate_ms)/n,
                        (t.input_sync_ms-old.input_sync_ms)/n);
                }
                previous_counters=counters;
                report=finished;
            }
        }
    }catch(const std::exception &e){status(w,std::string("Vision error: ")+e.what());}
}
extern "C" struct vision_worker *vision_start(const struct vision_options *o) {
    if(!o || o->npu_workers<0 || o->npu_workers>3)return nullptr;
    vision_worker *w=nullptr;
    try {
        w=new vision_worker;
        w->model=o->model;w->device=o->device;w->labels=o->labels;w->view=o->view;
        w->width=o->width;w->height=o->height;w->confidence=o->confidence;w->nms=o->nms;w->no_detect=o->no_detect;w->serial_eyes=o->serial_eyes;w->sync_preview=o->sync_preview;w->npu_workers=o->npu_workers;
        std::snprintf(w->snapshot.status,sizeof(w->snapshot.status),"Starting camera and detector...");
        w->thread=std::thread(run,w);return w;
    }catch(const std::exception &e){std::fprintf(stderr,"Vision startup: %s\n",e.what());delete w;return nullptr;}
}
extern "C" void vision_snapshot_copy(struct vision_worker *w,struct vision_snapshot *s) {
    {std::lock_guard<std::mutex> lock(w->mutex);*s=w->snapshot;}
    expire_boxes(*s,now_ms());
}
extern "C" uint64_t vision_snapshot_generation(struct vision_worker *w){return w?w->generation.load():0;}
extern "C" void vision_observation_copy(struct vision_worker *w,struct vision_snapshot *s){
    if(!w||!s)return;
    {std::lock_guard<std::mutex> lock(w->mutex);const auto offset=offsetof(vision_snapshot,timestamp_ms);
     std::memcpy(reinterpret_cast<char*>(s)+offset,reinterpret_cast<const char*>(&w->snapshot)+offset,sizeof(*s)-offset);}
    expire_boxes(*s,now_ms());
}
extern "C" void vision_stop(struct vision_worker *w){if(w){w->stop=true;if(w->thread.joinable())w->thread.join();delete w;}}
extern "C" void vision_demo(struct vision_snapshot *s,uint64_t elapsed) {
    double t=elapsed/1000.;
    for(int y=0;y<VISION_H;y++)for(int x=0;x<VISION_W;x++) {
        int local=x%(VISION_W/2);
        unsigned r=226-local/20-y/25,g=235-local/24-y/30,b=231-local/25-y/40;
        if(y>350){r=188;g=201;b=198;}
        if(local<12 || local>419){r=40;g=63;b=68;}
        s->pixels[y*VISION_W+x]=0xff000000|(r<<16)|(g<<8)|b;
    }
    s->count=2;
    for(int i=0;i<2;i++) {
        auto &o=s->objects[i];o.left=154+i*432+12*std::sin(t);o.right=o.left+74;
        o.top=196;o.bottom=355;o.score=.91f-i*.03f;o.class_id=39;o.eye=i;std::strcpy(o.name,"bottle");
        for(int y=206;y<351;y++)for(int x=int(o.left)+6;x<int(o.right)-6;x++)
            s->pixels[y*VISION_W+x]=y>260&&y<304?0xffeff7f4:0xff5a9c8d;
    }
    s->preview_fps=15;s->detection_enabled=1;s->detection_available=1;s->have_frame=1;s->demo=1;s->timestamp_ms=now_ms();s->fps=14.8f;s->capture_fps=15;s->inference_ms=57;
    std::strcpy(s->status,"DEMO");std::strcpy(s->source,"1920x1080 / both");
}
