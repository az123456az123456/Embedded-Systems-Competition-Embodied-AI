#ifndef V35_CAPTURE_GOLDEN_H
#define V35_CAPTURE_GOLDEN_H
#include <cstdint>
#include <vector>
namespace capture_golden {
static int mirror(int x,int size){return x<0?-x:x>=size?2*size-2-x:x;}
static unsigned raw_pixel(int x,int y,int frame){
    x=mirror(x,960);y=mirror(y,1080);
    if(frame%3==0)return unsigned(x*3+y*5+frame*7)&255;
    if(frame%3==1)return ((x+y)&1)?220:31;
    return (uint32_t(x)*2654435761u^uint32_t(y)*2246822519u^uint32_t(frame)*3266489917u)&255;
}
static uint32_t demosaic(int x,int y,int frame){
    unsigned mc=raw_pixel(x,y,frame),ml=raw_pixel(x-1,y,frame),mr=raw_pixel(x+1,y,frame);
    unsigned tc=raw_pixel(x,y-1,frame),bc=raw_pixel(x,y+1,frame);
    unsigned cross=(tc+ml+mr+bc+2)>>2;
    unsigned diagonal=(raw_pixel(x-1,y-1,frame)+raw_pixel(x+1,y-1,frame)+
                       raw_pixel(x-1,y+1,frame)+raw_pixel(x+1,y+1,frame)+2)>>2;
    unsigned horizontal=(ml+mr+1)>>1,vertical=(tc+bc+1)>>1,r,g,b;
    switch(((y&1)<<1)|(x&1)){
        case 0:r=diagonal;g=cross;b=mc;break;
        case 3:r=mc;g=cross;b=diagonal;break;
        case 1:r=vertical;g=mc;b=horizontal;break;
        default:r=horizontal;g=mc;b=vertical;break;
    }
    return (r<<16)|(g<<8)|b;
}
struct Axis{int lo,hi,q;};
static Axis axis(int at,int src,int dst){int n=(2*at+1)*src-dst,d=2*dst;return {n/d,n/d+1,((n%d)*2048+d/2)/d};}
static std::vector<uint32_t> reference(int frame){
    std::vector<uint32_t> pixels(640*640,0x727272);
    for(int y=0;y<640;y++)for(int x=0;x<569;x++){
        Axis a=axis(x,960,569),b=axis(y,1080,640);
        uint32_t p00=demosaic(a.lo,b.lo,frame),p01=demosaic(a.hi,b.lo,frame);
        uint32_t p10=demosaic(a.lo,b.hi,frame),p11=demosaic(a.hi,b.hi,frame),result=0;
        for(int shift:{16,8,0}){
            uint64_t top=((p00>>shift)&255)*(2048-a.q)+((p01>>shift)&255)*a.q;
            uint64_t bottom=((p10>>shift)&255)*(2048-a.q)+((p11>>shift)&255)*a.q;
            result|=uint32_t((top*(2048-b.q)+bottom*b.q+(1<<21))>>22)<<shift;
        }
        pixels[y*640+x+35]=result;
    }
    return pixels;
}
}
#endif
