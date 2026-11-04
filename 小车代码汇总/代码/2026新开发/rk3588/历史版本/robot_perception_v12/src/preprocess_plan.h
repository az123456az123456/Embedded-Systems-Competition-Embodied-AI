#ifndef PERCEPTION_PREPROCESS_PLAN_H
#define PERCEPTION_PREPROCESS_PLAN_H
#include "detector_math.h"
#include "preprocess_gather_v10.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
namespace detector_math {
// Pixel-wise cached plan. Share each pair of offsets/weights between RGB
// channels, avoiding the v10 ARM gather-to-stack loop. Keep that older kernel
// as PERCEPTION_PREPROCESS=gather for controlled board comparisons.
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
             unsigned char *dst,size_t bytes,size_t dst_stride,Letterbox &box,std::string &error) {
        const char *mode=std::getenv("PERCEPTION_PREPROCESS");
        if(mode && std::strcmp(mode,"gather")==0)return gather_.run(rgb,w,h,stride,tw,th,dst,bytes,dst_stride,box,error);
        error.clear();if(!prepare(w,h,tw,th,error))return false;
        if(!rgb || stride<w*3 || !dst || dst_stride<static_cast<size_t>(tw)*3 ||
           dst_stride>64u*1024u*1024u || bytes<dst_stride*static_cast<size_t>(th-1)+tw*3) {
            error="Invalid RGB buffer, stride or destination extent";return false;
        }
        box=geometry_;
        for(int y=0;y<th;++y) {
            auto *row=dst+static_cast<size_t>(y)*dst_stride;
            if(y<box.pad_y || y>=box.pad_y+box.resized_h)std::memset(row,114,tw*3);
            else {
                std::memset(row,114,box.pad_x*3);
                std::memset(row+(box.pad_x+box.resized_w)*3,114,(tw-box.pad_x-box.resized_w)*3);
            }
        }
        for(int y=0;y<box.resized_h;++y) {
            const auto *a=rgb+static_cast<size_t>(y0_[y])*stride;
            const auto *b=rgb+static_cast<size_t>(y1_[y])*stride;
            auto *out=dst+static_cast<size_t>(y+box.pad_y)*dst_stride+box.pad_x*3;
            const unsigned vy=wy_[y],iy=2048-vy;
            for(int x=0;x<box.resized_w;++x) {
                int left=x0_[x],right=x1_[x];unsigned vx=wx_[x],ix=2048-vx;
                for(int c=0;c<3;++c) {
                    unsigned top=a[left+c]*ix+a[right+c]*vx;
                    unsigned bottom=b[left+c]*ix+b[right+c]*vx;
                    out[x*3+c]=static_cast<unsigned char>((top*iy+bottom*vy+(1u<<21))>>22);
                }
            }
        }
        return true;
    }
};
}
#endif
