#ifndef PERCEPTION_BOX_STABILITY_H
#define PERCEPTION_BOX_STABILITY_H
#include "vision.h"
#include <algorithm>
#include <cmath>
#include <vector>

// Display-only temporal filtering. Never changes source/control observations,
// identity tracking, source timestamps or motor eligibility.
namespace box_stability {
class Filter {
    struct Track { vision_display_object item{}; };
    std::vector<Track> tracks_;
    uint64_t last_frame_=0,last_stamp_=0,last_now_=0;
    static bool valid(const vision_object&o){
        return std::isfinite(o.left)&&std::isfinite(o.top)&&std::isfinite(o.right)&&std::isfinite(o.bottom)&&
            std::isfinite(o.score)&&o.right>o.left&&o.bottom>o.top&&o.eye>=-1&&o.eye<=1;
    }
    static double similarity(const vision_object&a,const vision_object&b){
        if(a.eye!=b.eye||a.class_id!=b.class_id)return -1;
        const double aw=a.right-a.left,ah=a.bottom-a.top,bw=b.right-b.left,bh=b.bottom-b.top;
        if(bw/aw<.5||bw/aw>2||bh/ah<.5||bh/ah>2)return -1;
        const double intersection=std::max(0.f,std::min(a.right,b.right)-std::max(a.left,b.left))*
            std::max(0.f,std::min(a.bottom,b.bottom)-std::max(a.top,b.top));
        const double iou=intersection/(aw*ah+bw*bh-intersection);
        const double dx=((a.left+a.right)-(b.left+b.right))*.5;
        const double dy=((a.top+a.bottom)-(b.top+b.bottom))*.5;
        const double distance=std::hypot(dx,dy);
        if(iou<.12&&distance>std::min(45.,std::max(aw,bw)*.55))return -1;
        return iou+.25/(1+distance/20);
    }
    static float smooth(float old,float value,double alpha){
        return std::abs(value-old)<1.5?old:float(old+(value-old)*alpha);
    }
public:
    void clear(){tracks_.clear();last_frame_=last_stamp_=last_now_=0;}
    void update(uint64_t frame,uint64_t stamp,const vision_object*objects,int count,uint64_t now,bool enabled=true){
        if(!enabled||!now||(last_now_&&now<last_now_)){clear();return;}
        last_now_=now;
        tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track&t){
            return now<t.item.source_ms||now-t.item.source_ms>VISION_DISPLAY_HOLD_MS;}),tracks_.end());
        // Re-reading a cached inference must not extend its display lifetime.
        if(!frame||!stamp||now<stamp||now-stamp>VISION_DISPLAY_HOLD_MS||frame<=last_frame_||stamp<last_stamp_)return;
        last_frame_=frame;last_stamp_=stamp;
        std::vector<bool> used(tracks_.size(),false);
        for(int i=0;i<std::min(count,VISION_OBJECTS);i++){
            const auto&o=objects[i];if(!valid(o))continue;
            int best=-1;double score=-1;
            for(size_t j=0;j<tracks_.size();j++)if(!used[j]){
                const double s=similarity(tracks_[j].item.object,o);
                if(s>score){score=s;best=int(j);}
            }
            if(best>=0&&score>=0){
                auto&t=tracks_[best].item;auto&b=t.object;
                const double movement=std::hypot((b.left+b.right-o.left-o.right)*.5,(b.top+b.bottom-o.top-o.bottom)*.5);
                const double alpha=movement>12?.8:.38;
                b.left=smooth(b.left,o.left,alpha);b.top=smooth(b.top,o.top,alpha);
                b.right=smooth(b.right,o.right,alpha);b.bottom=smooth(b.bottom,o.bottom,alpha);
                b.score=float(b.score*.85+o.score*.15);
                t.frame_id=frame;t.source_ms=stamp;t.source_index=i;used[best]=true;
            }else if(tracks_.size()<VISION_OBJECTS){
                Track t;t.item.object=o;t.item.frame_id=frame;t.item.source_ms=stamp;t.item.source_index=i;
                tracks_.push_back(t);used.push_back(true);
            }
        }
    }
    int copy(vision_display_object*out,int capacity,uint64_t now) const {
        int count=0;
        for(const auto&t:tracks_){
            if(count>=capacity)break;
            if(now<t.item.source_ms||now-t.item.source_ms>VISION_DISPLAY_HOLD_MS)continue;
            out[count]=t.item;out[count].held=t.item.frame_id!=last_frame_;++count;
        }
        return count;
    }
};
}
#endif
