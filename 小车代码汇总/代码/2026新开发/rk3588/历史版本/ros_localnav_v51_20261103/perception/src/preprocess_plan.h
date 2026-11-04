#ifndef PERCEPTION_PREPROCESS_PLAN_H
#define PERCEPTION_PREPROCESS_PLAN_H
#include "detector_math.h"
#include "preprocess_gather_v10.h"
#include "stage_clock.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
namespace detector_math {
struct PreprocessTiming {
    double plan_ms=0,fill_ms=0,resize_ms=0,copy_ms=0,rga_ms=0,sync_ms=0,validate_ms=0;
    int backend=0; // 0=cached CPU, 1=RGA staged DMA, 2=legacy/gather CPU
};
// Cached bilinear plan with byte-exact Q22 packed R/B arithmetic. Keep the
// v11 scalar pixel kernel as PERCEPTION_PREPROCESS=pixel and the earlier
// ARM gather-to-stack kernel as gather for controlled board comparisons.
class RgbPreprocessor {
    Letterbox geometry_{};
    std::vector<int> x0_,x1_,y0_,y1_;
    std::vector<unsigned> wx_,wy_;
    RgbPreprocessorV10 gather_;
    bool prepare(int w,int h,int tw,int th,std::string &error) {
        if(w<1 || h<1 || tw<1 || th<1 || w>16384 || h>16384 || tw>16384 || th>16384 ||
           static_cast<size_t>(tw)*th>64u*1024u*1024u) {error="Invalid image or model dimensions";return false;}
        if(geometry_.source_w==w && geometry_.source_h==h && geometry_.target_w==tw && geometry_.target_h==th)return true;
        Letterbox next{};if(!make_letterbox(w,h,tw,th,next,error))return false;
        x0_.resize(next.resized_w);x1_.resize(next.resized_w);wx_.resize(next.resized_w);
        y0_.resize(next.resized_h);y1_.resize(next.resized_h);wy_.resize(next.resized_h);
        for(int x=0;x<next.resized_w;++x) {
            float sx=std::clamp((x+.5f)/next.scale_x-.5f,0.f,float(w-1));
            int a=static_cast<int>(sx);x0_[x]=a*3;x1_[x]=std::min(a+1,w-1)*3;
            wx_[x]=static_cast<unsigned>(std::lround((sx-a)*2048));
        }
        for(int y=0;y<next.resized_h;++y) {
            float sy=std::clamp((y+.5f)/next.scale_y-.5f,0.f,float(h-1));
            y0_[y]=static_cast<int>(sy);y1_[y]=std::min(y0_[y]+1,h-1);
            wy_[y]=static_cast<unsigned>(std::lround((sy-y0_[y])*2048));
        }
        geometry_=next;return true;
    }
public:
    bool run(const unsigned char *rgb,int w,int h,int stride,int tw,int th,
             unsigned char *dst,size_t bytes,size_t dst_stride,Letterbox &box,std::string &error,
             PreprocessTiming *timing=nullptr) {
        PreprocessTiming local;if(!timing)timing=&local;
        const char *mode=std::getenv("PERCEPTION_PREPROCESS");
        const bool old_pixel=mode && std::strcmp(mode,"pixel")==0;
        if(mode && std::strcmp(mode,"gather")==0) {
            auto start=StageClock::now();timing->backend=2;
            const bool ok=gather_.run(rgb,w,h,stride,tw,th,dst,bytes,dst_stride,box,error);
            timing->resize_ms+=stage_elapsed(start);return ok;
        }
        auto start=StageClock::now();
        error.clear();if(!prepare(w,h,tw,th,error))return false;
        if(!rgb || stride<w*3 || !dst || dst_stride<static_cast<size_t>(tw)*3 ||
           dst_stride>64u*1024u*1024u || bytes<dst_stride*static_cast<size_t>(th-1)+tw*3) {
            error="Invalid RGB buffer, stride or destination extent";return false;
        }
        box=geometry_;
        timing->plan_ms+=stage_elapsed(start);start=StageClock::now();
        for(int y=0;y<th;++y) {
            auto *row=dst+static_cast<size_t>(y)*dst_stride;
            if(y<box.pad_y || y>=box.pad_y+box.resized_h)std::memset(row,114,tw*3);
            else {
                std::memset(row,114,box.pad_x*3);
                std::memset(row+(box.pad_x+box.resized_w)*3,114,(tw-box.pad_x-box.resized_w)*3);
            }
        }
        timing->fill_ms+=stage_elapsed(start);start=StageClock::now();
        for(int y=0;y<box.resized_h;++y) {
            const auto *a=rgb+static_cast<size_t>(y0_[y])*stride;
            const auto *b=rgb+static_cast<size_t>(y1_[y])*stride;
            auto *out=dst+static_cast<size_t>(y+box.pad_y)*dst_stride+box.pad_x*3;
            const unsigned vy=wy_[y],iy=2048-vy;
            for(int x=0;x<box.resized_w;++x) {
                int left=x0_[x],right=x1_[x];unsigned vx=wx_[x],ix=2048-vx;
                if(old_pixel) {
                    for(int c=0;c<3;++c) {
                        unsigned top=a[left+c]*ix+a[right+c]*vx;
                        unsigned bottom=b[left+c]*ix+b[right+c]*vx;
                        out[x*3+c]=static_cast<unsigned char>((top*iy+bottom*vy+(1u<<21))>>22);
                    }
                } else {
                    // Share the four Q22 bilinear coefficients across RGB.
                    // R/B occupy independent 32-bit lanes in one uint64_t;
                    // the maximum weighted sum is 255*2^22 (<2^30), so lane
                    // carries cannot leak between channels. Three-byte reads
                    // remain valid even at the last pixel of a tight ROI.
                    const uint32_t q00=ix*iy,q01=vx*iy,q10=ix*vy,q11=vx*vy;
                    const auto rb=[](const unsigned char *p)->uint64_t {
                        return static_cast<uint64_t>(p[0]) | (static_cast<uint64_t>(p[2])<<32);
                    };
                    const uint64_t pair=rb(a+left)*q00+rb(a+right)*q01+
                                        rb(b+left)*q10+rb(b+right)*q11+
                                        ((uint64_t{1}<<21)|(uint64_t{1}<<53));
                    const uint32_t green=a[left+1]*q00+a[right+1]*q01+
                                         b[left+1]*q10+b[right+1]*q11+(1u<<21);
                    out[x*3]=static_cast<unsigned char>(pair>>22);
                    out[x*3+1]=static_cast<unsigned char>(green>>22);
                    out[x*3+2]=static_cast<unsigned char>(pair>>54);
                }
            }
        }
        timing->resize_ms+=stage_elapsed(start);
        return true;
    }
};
}
#endif
