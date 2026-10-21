#include "camera.h"
#include "camera_color_math.h"
#include "camera_timestamp.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <linux/videodev2.h>
#include <random>

static void exhaustive_arithmetic() {
    int worst=0;uint64_t colours=0,changed=0;
    for(bool bt709:{false,true})for(bool full:{false,true}) {
        const auto &m=camera_color::matrix(bt709,full);
        // Independently confirm cached LUTs against the pre-v10 expressions.
        const double kr=bt709?.2126:.299,kb=bt709?.0722:.114,kg=1-kr-kb;
        const double ys=full?1.:255./219,cs=full?1.:255./224;
        for(int i=0;i<256;i++) {
            assert(m.luma[i]==std::lround((i-(full?0:16))*ys*65536));
            assert(m.red_v[i]==std::lround(2*(1-kr)*(i-128)*cs*65536));
            assert(m.green_u[i]==std::lround(-2*kb*(1-kb)/kg*(i-128)*cs*65536));
            assert(m.green_v[i]==std::lround(-2*kr*(1-kr)/kg*(i-128)*cs*65536));
            assert(m.blue_u[i]==std::lround(2*(1-kb)*(i-128)*cs*65536));
        }
        for(int y=0;y<256;y++)for(int u=0;u<256;u++)for(int v=0;v<256;v++) {
            uint8_t ref[3],fixed[3];camera_color::reference_pixel(m,y,u,v,ref);
            camera_color::fixed_pixel(m,y,u,v,fixed);
            for(int c=0;c<3;c++) {
                const int delta=std::abs(int(ref[c])-fixed[c]);
                worst=std::max(worst,delta);changed+=delta!=0;
                assert(delta<=1);
            }
            ++colours;
        }
    }
    std::cout<<"PASS exhaustive 601/709 full/limited: "<<colours
             <<" YUV colours, max channel error="<<worst<<", differing channels="<<changed<<"\n";
}
static void frame_cases() {
    std::mt19937 rng(0x3588);
    int worst=0;size_t cases=0;
    for(bool bt709:{false,true})for(bool full:{false,true})
    for(uint32_t format:{V4L2_PIX_FMT_NV12,V4L2_PIX_FMT_NV21,V4L2_PIX_FMT_NV12M,V4L2_PIX_FMT_NV21M})
    for(unsigned width:{2u,14u,16u,18u,30u,32u,34u,62u,64u,66u,1920u})
    for(unsigned height:{2u,6u})for(bool separate:{false,true}) {
        CameraLayout f;f.width=width;f.height=height;f.stride=width+7;
        f.chroma_stride=separate?width+11:f.stride;f.fourcc=format;f.bt709=bt709;f.full_range=full;
        const size_t ybytes=separate?f.stride*(height-1)+width:f.stride*height;
        const size_t uvbytes=f.chroma_stride*(height/2-1)+width;
        // Deliberately unaligned start, padded rows, and minimum last-row length.
        std::vector<uint8_t> y(ybytes+(separate?0:uvbytes)+1),uv(uvbytes+1);
        for(auto &b:y)b=rng();
        for(auto &b:uv)b=rng();
        const auto ycopy=y,uvcopy=uv;
        std::vector<uint8_t> reference,automatic;std::string error;
        assert(setenv("PERCEPTION_CAMERA_SCALAR","1",1)==0);
        assert(camera_to_rgb(f,y.data()+1,y.size()-1,separate?uv.data()+1:nullptr,separate?uvbytes:0,reference,error));
        assert(unsetenv("PERCEPTION_CAMERA_SCALAR")==0);
        assert(camera_to_rgb(f,y.data()+1,y.size()-1,separate?uv.data()+1:nullptr,separate?uvbytes:0,automatic,error));
        assert(reference.size()==automatic.size());
        for(size_t i=0;i<reference.size();i++) {
            const int delta=std::abs(int(reference[i])-automatic[i]);worst=std::max(worst,delta);assert(delta<=1);
        }
        assert(y==ycopy && uv==uvcopy);
        assert(!camera_to_rgb(f,y.data()+1,y.size()-2,separate?uv.data()+1:nullptr,separate?uvbytes-1:0,automatic,error));
        ++cases;
    }
    std::vector<uint8_t> out;std::string error;uint8_t bytes[32]={};
    CameraLayout f;f.width=16;f.height=2;f.stride=std::numeric_limits<size_t>::max();f.fourcc=V4L2_PIX_FMT_NV12;
    assert(!camera_to_rgb(f,bytes,sizeof(bytes),nullptr,0,out,error));
    f.stride=16;f.height=4;f.chroma_stride=std::numeric_limits<size_t>::max();
    assert(!camera_to_rgb(f,bytes,64,bytes,sizeof(bytes),out,error));
    std::cout<<"PASS "<<cases<<" NV12/NV21 packed/separate padded/unaligned/tail frames; backend="
             <<camera_color_backend()<<", max channel error="<<worst<<"; truncation/overflow rejected\n";
}
static void timestamps() {
    v4l2_buffer b={};b.flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    b.timestamp.tv_sec=10;b.timestamp.tv_usec=123456;
    assert(camera_source_timestamp(b,10199)==10123);
    b.flags|=V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
    assert(camera_source_timestamp(b,10199)==10123);
    b.flags=V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN;
    assert(camera_source_timestamp(b,10199)==10199);
    b.flags=V4L2_BUF_FLAG_TIMESTAMP_COPY;
    assert(camera_source_timestamp(b,10199)==10199);
    b.flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    assert(camera_source_timestamp(b,10100)==10100); // future
    assert(camera_source_timestamp(b,20000)==20000); // >5 seconds old
    b.timestamp.tv_usec=-1;assert(camera_source_timestamp(b,10199)==10199);
    b.timestamp.tv_usec=1000000;assert(camera_source_timestamp(b,10199)==10199);
    b.timestamp.tv_sec=-1;b.timestamp.tv_usec=0;assert(camera_source_timestamp(b,10199)==10199);
    b.timestamp.tv_sec=std::numeric_limits<decltype(b.timestamp.tv_sec)>::max();
    assert(camera_source_timestamp(b,10199)==10199);
    std::cout<<"PASS monotonic capture timestamp and malformed/realtime/future/stale fallback\n";
}
int main(){exhaustive_arithmetic();frame_cases();timestamps();}
