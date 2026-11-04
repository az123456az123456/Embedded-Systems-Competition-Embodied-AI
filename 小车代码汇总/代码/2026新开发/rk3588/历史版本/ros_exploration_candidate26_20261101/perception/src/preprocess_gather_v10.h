#ifndef PERCEPTION_PREPROCESS_GATHER_V10_H
#define PERCEPTION_PREPROCESS_GATHER_V10_H
#include "detector_math.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace detector_math {
// Each detector owns its sampling plan. Geometry changes rebuild it, while
// every frame keeps the v9 half-pixel coordinates and exact 11-bit rounding.
class RgbPreprocessorV10 {
    Letterbox geometry_{};
    std::vector<int> left_,right_,upper_,lower_;
    std::vector<uint16_t> wx_,wy_;
    bool prepare(int w,int h,int tw,int th,std::string &error) {
        if(w<1 || h<1 || tw<1 || th<1 || w>16384 || h>16384 || tw>16384 || th>16384 ||
           static_cast<size_t>(tw)*th>64u*1024u*1024u) {
            error="Invalid image or model dimensions";return false;
        }
        if(geometry_.source_w==w && geometry_.source_h==h &&
           geometry_.target_w==tw && geometry_.target_h==th)return true;
        Letterbox next{};
        if(!make_letterbox(w,h,tw,th,next,error))return false;
        left_.resize(next.resized_w*3);right_.resize(left_.size());wx_.resize(left_.size());
        upper_.resize(next.resized_h);lower_.resize(next.resized_h);wy_.resize(next.resized_h);
        for(int x=0;x<next.resized_w;++x) {
            float sx=std::clamp((x+.5f)/next.scale_x-.5f,0.f,float(w-1));
            int a=static_cast<int>(sx),b=std::min(a+1,w-1);
            auto weight=static_cast<uint16_t>(std::lround((sx-a)*2048));
            for(int c=0;c<3;++c){left_[x*3+c]=a*3+c;right_[x*3+c]=b*3+c;wx_[x*3+c]=weight;}
        }
        for(int y=0;y<next.resized_h;++y) {
            float sy=std::clamp((y+.5f)/next.scale_y-.5f,0.f,float(h-1));
            upper_[y]=static_cast<int>(sy);lower_[y]=std::min(upper_[y]+1,h-1);
            wy_[y]=static_cast<uint16_t>(std::lround((sy-upper_[y])*2048));
        }
        geometry_=next;return true;
    }
public:
    bool run(const unsigned char *rgb,int w,int h,int stride,int tw,int th,
             unsigned char *dst,size_t bytes,size_t dst_stride,Letterbox &box,std::string &error) {
        error.clear();
        if(!prepare(w,h,tw,th,error))return false;
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
        const int channels=box.resized_w*3;
        for(int y=0;y<box.resized_h;++y) {
            const auto *a=rgb+static_cast<size_t>(upper_[y])*stride;
            const auto *b=rgb+static_cast<size_t>(lower_[y])*stride;
            auto *out=dst+static_cast<size_t>(y+box.pad_y)*dst_stride+box.pad_x*3;
            const unsigned vy=wy_[y];int k=0;
#if defined(__aarch64__)
            // Gather the fixed RGB sample locations, then interpolate eight
            // channels together. No floating arithmetic in the per-frame loop.
            for(;k+8<=channels;k+=8) {
                uint16_t al[8],ar[8],bl[8],br[8];
                for(int n=0;n<8;++n){int l=left_[k+n],r=right_[k+n];al[n]=a[l];ar[n]=a[r];bl[n]=b[l];br[n]=b[r];}
                uint16x8_t x=vld1q_u16(wx_.data()+k),ix=vsubq_u16(vdupq_n_u16(2048),x);
                uint16x8_t va=vld1q_u16(al),vb=vld1q_u16(ar),vc=vld1q_u16(bl),vd=vld1q_u16(br);
                uint32x4_t top0=vmlal_u16(vmull_u16(vget_low_u16(va),vget_low_u16(ix)),vget_low_u16(vb),vget_low_u16(x));
                uint32x4_t top1=vmlal_u16(vmull_u16(vget_high_u16(va),vget_high_u16(ix)),vget_high_u16(vb),vget_high_u16(x));
                uint32x4_t bot0=vmlal_u16(vmull_u16(vget_low_u16(vc),vget_low_u16(ix)),vget_low_u16(vd),vget_low_u16(x));
                uint32x4_t bot1=vmlal_u16(vmull_u16(vget_high_u16(vc),vget_high_u16(ix)),vget_high_u16(vd),vget_high_u16(x));
                auto blend=[vy](uint32x4_t top,uint32x4_t bottom){
                    return vshrq_n_u32(vaddq_u32(vmlaq_n_u32(vmulq_n_u32(top,2048-vy),bottom,vy),vdupq_n_u32(1u<<21)),22);
                };
                vst1_u8(out+k,vmovn_u16(vcombine_u16(vmovn_u32(blend(top0,bot0)),vmovn_u32(blend(top1,bot1)))));
            }
#endif
            for(;k<channels;++k) {
                const unsigned x=wx_[k],ix=2048-x;
                unsigned top=a[left_[k]]*ix+a[right_[k]]*x;
                unsigned bottom=b[left_[k]]*ix+b[right_[k]]*x;
                out[k]=static_cast<uint8_t>((top*(2048-vy)+bottom*vy+(1u<<21))>>22);
            }
        }
        return true;
    }
};
}
#endif
