#include "vision.h"
#include "camera.h"
#include "detector.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>
static uint64_t now_ms(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct vision_worker {
    std::atomic<bool> stop{false};std::thread thread;std::mutex mutex;
    vision_snapshot snapshot={};
    std::string model,device,labels,view;int width,height;float confidence,nms;
};
static void status(vision_worker *w,const std::string &message) {
    std::lock_guard<std::mutex> lock(w->mutex);
    std::snprintf(w->snapshot.status,sizeof(w->snapshot.status),"%s",message.c_str());
    w->snapshot.have_frame=0;w->snapshot.count=0;
    std::fprintf(stderr,"Vision: %s\n",message.c_str());
}
static void fit_frame(vision_snapshot &out,const std::vector<uint8_t> &rgb,int width,int height,
                      double &scale,int &ox,int &oy) {
    scale=std::min(VISION_W/double(width),VISION_H/double(height));
    int ww=std::max(1,int(width*scale)),hh=std::max(1,int(height*scale));
    ox=(VISION_W-ww)/2;oy=(VISION_H-hh)/2;
    std::fill(std::begin(out.pixels),std::end(out.pixels),0xff14232d);
    for(int y=0;y<hh;y++)for(int x=0;x<ww;x++) {
        int sx=std::min(width-1,int(x/scale)),sy=std::min(height-1,int(y/scale));
        const uint8_t *p=rgb.data()+(size_t(sy)*width+sx)*3;
        out.pixels[(y+oy)*VISION_W+x+ox]=0xff000000|(p[0]<<16)|(p[1]<<8)|p[2];
    }
}
static void run(vision_worker *w) {
    try {
        std::ifstream file(w->labels);std::vector<std::string> names;std::string name,error;
        while(std::getline(file,name)) {
            if(!name.empty() && name.back()=='\r')name.pop_back();
            if(name.empty() || name.size()>63 || names.size()>=256){status(w,"Invalid labels file");return;}
            names.push_back(name);
        }
        if(!file.eof() || names.empty()){status(w,"Cannot read labels: "+w->labels);return;}
        RknnDetector detector;
        if(!detector.open(w->model,w->confidence,w->nms,names.size(),error)){status(w,error);return;}
        if(w->stop)return;
        Camera camera;
        if(!camera.open(w->device,w->width,w->height,error)){status(w,error);return;}
        const int width=camera.layout().width,height=camera.layout().height;
        if(w->view!="single" && width%2){status(w,"Stereo input width must be even; use --view single for mono");return;}
        std::fprintf(stderr,"Camera: %s, view=%s, confidence=%.2f, NMS=%.2f\n",camera.description().c_str(),w->view.c_str(),w->confidence,w->nms);
        std::vector<uint8_t> rgb;vision_snapshot next={};
        std::snprintf(next.source,sizeof(next.source),"%s / %s",camera.description().c_str(),w->view.c_str());
        uint64_t previous=now_ms(),last_frame=previous;float fps=0;bool timed_out=false;
        while(!w->stop) {
            int ret=camera.read(rgb,error);
            if(ret<0){status(w,error);return;}
            if(ret==0) {
                if(!timed_out && now_ms()-last_frame>1000) {
                    status(w,"Camera timeout; waiting for frames");timed_out=true;
                }
                continue;
            }
            timed_out=false;last_frame=now_ms();uint64_t started=last_frame;
            int first=w->view=="right"?1:0;
            int eyes=w->view=="both"?2:1;
            const int roi_width=w->view=="single"?width:width/2;
            next.count=0;double scale;int ox,oy;fit_frame(next,rgb,width,height,scale,ox,oy);
            for(int eye=first;eye<first+eyes;eye++) {
                if(w->stop)return;
                std::vector<Detection> detections;
                const int offset=w->view=="single"?0:eye*roi_width;
                if(!detector.infer(rgb.data()+offset*3,roi_width,height,width*3,detections,error)){status(w,error);return;}
                for(const auto &d:detections) {
                    if(next.count==VISION_OBJECTS)break;
                    auto &o=next.objects[next.count++];
                    o.left=(d.left+offset)*scale+ox;o.right=(d.right+offset)*scale+ox;
                    o.top=d.top*scale+oy;o.bottom=d.bottom*scale+oy;
                    o.score=d.score;o.class_id=d.class_id;o.eye=w->view=="single"?-1:eye;
                    std::snprintf(o.name,sizeof(o.name),"%s",names.at(d.class_id).c_str());
                }
            }
            uint64_t finished=now_ms();next.inference_ms=finished-started;
            float instant=1000.f/std::max<uint64_t>(1,finished-previous);
            fps=fps==0?instant:.85f*fps+.15f*instant;previous=finished;next.fps=fps;
            next.have_frame=1;next.timestamp_ms=started;std::snprintf(next.status,sizeof(next.status),"Live");
            {std::lock_guard<std::mutex> lock(w->mutex);w->snapshot=next;}
        }
    }catch(const std::exception &e){status(w,std::string("Vision error: ")+e.what());}
}
extern "C" struct vision_worker *vision_start(const struct vision_options *o) {
    vision_worker *w=nullptr;
    try {
        w=new vision_worker;
        w->model=o->model;w->device=o->device;w->labels=o->labels;w->view=o->view;
        w->width=o->width;w->height=o->height;w->confidence=o->confidence;w->nms=o->nms;
        std::snprintf(w->snapshot.status,sizeof(w->snapshot.status),"Starting camera and detector...");
        w->thread=std::thread(run,w);return w;
    }catch(const std::exception &e){std::fprintf(stderr,"Vision startup: %s\n",e.what());delete w;return nullptr;}
}
extern "C" void vision_snapshot_copy(struct vision_worker *w,struct vision_snapshot *s) {
    std::lock_guard<std::mutex> lock(w->mutex);*s=w->snapshot;
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
    s->have_frame=1;s->demo=1;s->timestamp_ms=now_ms();s->fps=24.8f;s->inference_ms=37;
    std::strcpy(s->status,"DEMO");std::strcpy(s->source,"1920x1080 / both");
}
