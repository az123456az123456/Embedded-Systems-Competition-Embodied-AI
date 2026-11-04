#ifndef PERCEPTION_BOX_MOTION_H
#define PERCEPTION_BOX_MOTION_H
#include "vision.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

// Short-lived image alignment, not an identity tracker or a new detector.
// Each comparison starts at the actual NPU source image: errors never integrate
// across preview frames, and successful alignment never refreshes detection age.
namespace box_motion {
constexpr int W=288,H=162,HW=W/2,HH=H/2,MAX_BOXES=32;
struct Gray {
    std::array<uint8_t,W*H> full{};
    std::array<uint8_t,HW*HH> half{};
    void finish(){
        for(int y=0;y<HH;y++)for(int x=0;x<HW;x++) {
            int p=y*2*W+x*2;
            half[y*HW+x]=(unsigned(full[p])+full[p+1]+full[p+W]+full[p+W+1]+2)/4;
        }
    }
    static uint8_t gray(unsigned r,unsigned g,unsigned b){return (77*r+150*g+29*b+128)>>8;}
    void from_preview(const uint32_t *pixels){
        for(int y=0;y<H;y++)for(int x=0;x<W;x++) {
            uint32_t p=pixels[(y*3+1)*VISION_W+x*3+1];
            full[y*W+x]=gray((p>>16)&255,(p>>8)&255,p&255);
        }
        finish();
    }
    void from_rgb(const uint8_t *rgb,int width,int height,int ox,int oy,int ww,int hh,double scale){
        for(int y=0;y<H;y++)for(int x=0;x<W;x++) {
            int px=x*3+1,py=y*3+1;
            if(px<ox||py<oy||px>=ox+ww||py>=oy+hh){full[y*W+x]=gray(20,35,45);continue;}
            int sx=std::min(width-1,int((px-ox)/scale)),sy=std::min(height-1,int((py-oy)/scale));
            const auto *p=rgb+(sy*width+sx)*3;full[y*W+x]=gray(p[0],p[1],p[2]);
        }
        finish();
    }
};
struct Estimate {float dx=0,dy=0;int features=0;bool valid=false;};
namespace detail {
struct Point{int x=0,y=0;};
struct Match{int dx=0,dy=0;bool valid=false;};
template<int N> inline int patch(const uint8_t *a,int stride,int x,int y,std::array<int,N*N> &v){
    int sum=0,k=0;
    for(int yy=-N/2;yy<=N/2;yy++)for(int xx=-N/2;xx<=N/2;xx++)sum+=(v[k++]=a[(y+yy)*stride+x+xx]);
    return sum;
}
template<int N> inline int cost(const std::array<int,N*N> &reference,int refsum,
                               const uint8_t *image,int stride,int x,int y){
    std::array<int,N*N> sample;int sum=patch<N>(image,stride,x,y,sample),sad=0;
    for(int i=0;i<N*N;i++)sad+=std::abs((reference[i]-sample[i])*(N*N)-refsum+sum);
    return sad;
}
inline int texture(const Gray &image,int x,int y){
    std::array<int,25> sample;int sum=patch<5>(image.full.data(),W,x,y,sample),variance=0;
    for(int v:sample){int d=v*25-sum;variance+=d*d/25;}
    return variance/625;
}
inline Match match(const Gray &reference,const Gray &current,Point p,int radius,int low,int high){
    struct Candidate{int x=0,y=0,score=std::numeric_limits<int>::max();};
    std::array<Candidate,4> coarse{};
    std::array<int,9> small;int sum=patch<3>(reference.half.data(),HW,p.x/2,p.y/2,small);
    const int x0=std::max(low/2+1,p.x/2-(radius+1)/2),x1=std::min(high/2-2,p.x/2+(radius+1)/2);
    const int y0=std::max(1,p.y/2-(radius+1)/2),y1=std::min(HH-2,p.y/2+(radius+1)/2);
    for(int y=y0;y<=y1;y++)for(int x=x0;x<=x1;x++){
        int score=cost<3>(small,sum,current.half.data(),HW,x,y);
        if(score>=coarse.back().score)continue;
        Candidate c{x,y,score};
        for(auto &candidate:coarse)if(c.score<candidate.score)std::swap(c,candidate);
    }
    std::array<int,25> full;sum=patch<5>(reference.full.data(),W,p.x,p.y,full);
    Candidate best,second;
    // Four coarse minima retain alternatives when an odd-pixel translation is
    // imperfect at half resolution. Fine search always uses the original image.
    std::array<Candidate,100> fine{};int count=0;
    for(const auto &c:coarse){
        if(c.score==std::numeric_limits<int>::max())continue;
        for(int y=c.y*2-2;y<=c.y*2+2;y++)for(int x=c.x*2-2;x<=c.x*2+2;x++){
            if(x<low+2||x>=high-2||y<2||y>=H-2||std::abs(x-p.x)>radius||std::abs(y-p.y)>radius)continue;
            int score=cost<5>(full,sum,current.full.data(),W,x,y);
            fine[count++]={x,y,score};if(score<best.score)best={x,y,score};
        }
    }
    if(best.score==std::numeric_limits<int>::max())return {};
    for(int i=0;i<count;i++)if(std::abs(fine[i].x-best.x)>2||std::abs(fine[i].y-best.y)>2)
        if(fine[i].score<second.score)second=fine[i];
    const double error=best.score/625.;
    if(error>13)return {};
    if(second.score<std::numeric_limits<int>::max()&&best.score>second.score*.82&&second.score-best.score<1875)return {};
    std::array<int,25> sample;int current_sum=patch<5>(current.full.data(),W,best.x,best.y,sample);
    if(std::abs(current_sum-sum)>25*45)return {};
    return {best.x-p.x,best.y-p.y,true};
}
}
inline Estimate estimate(const Gray &reference,const Gray &current,const vision_object &box,uint64_t age_ms){
    if(age_ms>VISION_BOX_MAX_AGE_MS||!std::isfinite(box.left)||!std::isfinite(box.top)||
       !std::isfinite(box.right)||!std::isfinite(box.bottom)||box.right<=box.left||box.bottom<=box.top||box.eye<-1||box.eye>1)return {};
    const int low=box.eye==1?W/2:0,high=box.eye==0?W/2:W;
    float l=std::max(float(low+3),box.left/3.f),r=std::min(float(high-3),box.right/3.f);
    float t=std::max(3.f,box.top/3.f),b=std::min(float(H-3),box.bottom/3.f);
    if(r-l<8||b-t<8)return {};
    std::array<detail::Point,4> points{};int found=0;
    for(int quadrant=0;quadrant<4;quadrant++){
        int best=-1;detail::Point point;
        for(int yy=0;yy<4;yy++)for(int xx=0;xx<4;xx++){
            float fx=.15f+((quadrant&1)*4+xx)*.1f,fy=.15f+((quadrant/2)*4+yy)*.1f;
            int x=std::lround(l+(r-l)*fx),y=std::lround(t+(b-t)*fy);
            if(x<low+3||x>=high-3||y<3||y>=H-3)continue;
            int v=detail::texture(reference,x,y);
            if(v>best){best=v;point={x,y};}
        }
        if(best<45)continue;
        bool distinct=true;
        for(int i=0;i<found;i++)if(std::abs(points[i].x-point.x)<3&&std::abs(points[i].y-point.y)<3)distinct=false;
        if(distinct)points[found++]=point;
    }
    if(found<2)return {};
    std::array<detail::Match,4> matches{};int valid=0;
    const int radius=std::min(28,std::max(8,int(age_ms/7)));
    for(int i=0;i<found;i++)if((matches[valid]=detail::match(reference,current,points[i],radius,low,high)).valid)++valid;
    if(valid<2)return {};
    int best=0,center=0;
    for(int i=0;i<valid;i++){
        int agree=0;for(int j=0;j<valid;j++)if(std::abs(matches[i].dx-matches[j].dx)<=2&&std::abs(matches[i].dy-matches[j].dy)<=2)++agree;
        if(agree>best){best=agree;center=i;}
    }
    if(best<2||best*5<valid*3)return {};
    int dx=0,dy=0,count=0;
    for(int i=0;i<valid;i++)if(std::abs(matches[center].dx-matches[i].dx)<=2&&std::abs(matches[center].dy-matches[i].dy)<=2){dx+=matches[i].dx;dy+=matches[i].dy;++count;}
    return {float(dx)*3/count,float(dy)*3/count,count,true};
}
inline bool apply(const Estimate &motion,const vision_object &source,vision_object &out){
    if(!motion.valid)return false;
    const float low=source.eye==1?VISION_W/2.f:0,high=source.eye==0?VISION_W/2.f:VISION_W;
    out=source;
    out.left=std::max(low,source.left+motion.dx);out.right=std::min(high,source.right+motion.dx);
    out.top=std::max(0.f,source.top+motion.dy);out.bottom=std::min(float(VISION_H),source.bottom+motion.dy);
    if(out.right-out.left<(source.right-source.left)*.5f||out.bottom-out.top<(source.bottom-source.top)*.5f){out=source;return false;}
    return true;
}
}
#endif
