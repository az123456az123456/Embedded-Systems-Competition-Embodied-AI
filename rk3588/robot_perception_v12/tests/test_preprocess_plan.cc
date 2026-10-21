#include "preprocess_plan.h"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <random>
using namespace detector_math;
static void test_equivalence() {
    std::mt19937 random(20261019);std::string error;RgbPreprocessor plan;
    for(int test=0;test<45;++test) {
        int w=test==0?960:1+random()%1001,h=test==0?1080:1+random()%1101;
        int tw=test==0?640:1+random()%700,th=test==0?640:1+random()%700;
        int stride=w*3+11;size_t dst_stride=tw*3+17;
        std::vector<unsigned char> rgb(stride*h),reference,output(dst_stride*th+31,0xa5);
        for(auto &v:rgb)v=random()%256;
        Letterbox a{},b{};
        assert(preprocess_rgb(rgb.data(),w,h,stride,tw,th,reference,a,error));
        for(int repeat=0;repeat<3;++repeat) {
            assert(plan.run(rgb.data(),w,h,stride,tw,th,output.data(),output.size()-31,dst_stride,b,error));
            assert(a.pad_x==b.pad_x && a.pad_y==b.pad_y && a.scale_x==b.scale_x && a.scale_y==b.scale_y);
            for(int y=0;y<th;++y) {
                assert(!std::memcmp(output.data()+y*dst_stride,reference.data()+y*tw*3,tw*3));
                for(size_t n=tw*3;n<dst_stride;++n)assert(output[y*dst_stride+n]==0xa5);
            }
            for(size_t n=dst_stride*th;n<output.size();++n)assert(output[n]==0xa5);
        }
        assert(!plan.run(rgb.data(),w,h,stride,tw,th,output.data(),tw*3-1,dst_stride,b,error));
        assert(!plan.run(rgb.data(),w,h,stride,tw,th,output.data(),output.size(),tw*3-1,b,error));
    }
    std::puts("PASS cached RGB preprocessor: byte-exact v9 equivalence, 45 geometries, rebuild/reuse, padded output stride, guards and truncation");
    }
int main(int argc,char **) {
    unsetenv("PERCEPTION_PREPROCESS");test_equivalence();
    setenv("PERCEPTION_PREPROCESS","gather",1);test_equivalence();
    unsetenv("PERCEPTION_PREPROCESS");
    if(argc>1) {
        std::string error;RgbPreprocessor plan;
        std::vector<unsigned char> rgb(1920*1080*3,73),out(640*640*3),legacy;Letterbox b{};
        const int count=80;auto start=std::chrono::steady_clock::now();
        for(int i=0;i<count;++i)assert(preprocess_rgb(rgb.data(),960,1080,1920*3,640,640,legacy,b,error));
        auto middle=std::chrono::steady_clock::now();
        for(int i=0;i<count;++i)assert(plan.run(rgb.data(),960,1080,1920*3,640,640,out.data(),out.size(),640*3,b,error));
        auto end=std::chrono::steady_clock::now();
        setenv("PERCEPTION_PREPROCESS","gather",1);
        for(int i=0;i<count;++i)assert(plan.run(rgb.data(),960,1080,1920*3,640,640,out.data(),out.size(),640*3,b,error));
        auto gather_end=std::chrono::steady_clock::now();unsetenv("PERCEPTION_PREPROCESS");
        std::printf("BENCH preprocess 960x1080->640x640: legacy_ms=%.3f cached_pixel_ms=%.3f v10_gather_ms=%.3f (local CPU only)\n",
            std::chrono::duration<double,std::milli>(middle-start).count()/count,
            std::chrono::duration<double,std::milli>(end-middle).count()/count,
            std::chrono::duration<double,std::milli>(gather_end-end).count()/count);
    }
}
