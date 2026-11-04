#include "preprocess_rga.h"
#include <cassert>
#include <random>
using namespace detector_math;
// This fake supplies a controlled completed RGB tensor. The selector,
// letterbox geometry, guarded destinations, fallback and validation are real;
// none of these tests claims to benchmark or emulate RGA hardware speed.
struct GoldenEngine final:RgaEngine {
    int calls=0,fail_after=-1,corrupt=0;bool shift=false;
    bool process(const unsigned char *rgb,int stride,const Letterbox &box,
                 unsigned char *dst,size_t dst_stride,PreprocessTiming &t,std::string &reason)override {
        ++calls;if(fail_after>=0 && calls>fail_after){reason="injected RGA failure";return false;}
        std::vector<unsigned char> pixels;Letterbox actual;
        assert(preprocess_rgb(rgb,box.source_w,box.source_h,stride,box.target_w,box.target_h,pixels,actual,reason));
        assert(actual.pad_x==box.pad_x && actual.pad_y==box.pad_y);
        assert(actual.resized_w==box.resized_w && actual.resized_h==box.resized_h);
        for(int y=0;y<box.target_h;++y)
            std::memcpy(dst+y*dst_stride,pixels.data()+y*box.target_w*3,box.target_w*3);
        if(corrupt)dst[static_cast<size_t>(box.pad_y)*dst_stride+box.pad_x*3]^=0xff;
        if(shift)for(int y=0;y<box.resized_h;++y) {
            auto *row=dst+static_cast<size_t>(y+box.pad_y)*dst_stride+box.pad_x*3;
            std::memmove(row,row+3,(box.resized_w-1)*3);
        }
        t.backend=1;return true;
    }
};
static size_t checks=0;
#define CHECK(x) do{++checks;assert(x);}while(0)
static void equivalence() {
    std::mt19937 random(341);auto fake=new GoldenEngine;
    RgaPreprocessor rga{std::unique_ptr<RgaEngine>(fake)};
    for(int trial=0;trial<32;++trial) {
        const int w=trial==0?960:1+random()%317,h=trial==0?1080:1+random()%231;
        const int tw=trial==0?640:1+random()%181,th=trial==0?640:1+random()%223;
        const int stride=w*6+13; // stereo-like padded stride; ROI is not tight.
        const size_t row=tw*3+19;
        std::vector<unsigned char> input(static_cast<size_t>(stride)*h+7),reference;
        for(auto &x:input)x=random()%256;
        std::vector<unsigned char> output(row*th+23,0xa5);Letterbox a,b;std::string error;
        CHECK(preprocess_rgb(input.data()+7,w,h,stride,tw,th,reference,a,error));
        for(int again=0;again<2;++again) {
            PreprocessTiming t;CHECK(rga.run(input.data()+7,w,h,stride,tw,th,output.data(),row*th,row,b,t));
            CHECK(t.backend==1);CHECK(a.pad_x==b.pad_x && a.pad_y==b.pad_y);
            CHECK(a.resized_w==b.resized_w && a.resized_h==b.resized_h);
            for(int y=0;y<th;++y) {
                CHECK(!std::memcmp(output.data()+row*y,reference.data()+tw*3*y,tw*3));
                for(size_t x=tw*3;x<row;++x)CHECK(output[row*y+x]==0xa5);
            }
            for(size_t x=row*th;x<output.size();++x)CHECK(output[x]==0xa5);
        }
    }
    CHECK(fake->calls==96); // 32 deterministic probes plus 64 real frames.
}
static void failures() {
    std::vector<unsigned char> input(53*71*3,73),out(64*64*3);Letterbox b;PreprocessTiming t;
    auto fake=new GoldenEngine;fake->fail_after=2;
    RgaPreprocessor rga{std::unique_ptr<RgaEngine>(fake)};
    CHECK(rga.run(input.data(),53,71,53*3,64,64,out.data(),out.size(),64*3,b,t));
    CHECK(!rga.run(input.data(),53,71,53*3,64,64,out.data(),out.size(),64*3,b,t));
    CHECK(rga.reason()=="injected RGA failure");
    CHECK(!rga.run(input.data(),53,71,53*3,64,64,out.data(),out.size(),64*3,b,t));
    CHECK(fake->calls==3); // probe, first frame, then a latched failing frame.
    auto broken=new GoldenEngine;broken->corrupt=1;
    RgaPreprocessor mismatch{std::unique_ptr<RgaEngine>(broken)};
    CHECK(!mismatch.run(input.data(),53,71,53*3,64,64,out.data(),out.size(),64*3,b,t));
    CHECK(mismatch.reason().find("exceeds 2")!=std::string::npos);
    auto shifted=new GoldenEngine;shifted->shift=true;
    RgaPreprocessor flat_first{std::unique_ptr<RgaEngine>(shifted)};
    CHECK(!flat_first.run(input.data(),53,71,53*3,64,64,out.data(),out.size(),64*3,b,t));
    CHECK(flat_first.reason().find("exceeds 2")!=std::string::npos);
    RgbPreprocessor cpu;std::string error;std::vector<unsigned char> expected;
    CHECK(cpu.run(input.data(),53,71,53*3,64,64,out.data(),out.size(),64*3,b,error));
    CHECK(preprocess_rgb(input.data(),53,71,53*3,64,64,expected,b,error));
    CHECK(out==expected); // fallback replaces all corrupt/partially written data.
    auto invalid=new GoldenEngine;
    RgaPreprocessor guard{std::unique_ptr<RgaEngine>(invalid)};
    CHECK(!guard.run(input.data(),53,71,1,64,64,out.data(),out.size(),64*3,b,t));
    CHECK(invalid->calls==0);
    RgaPreprocessor missing{std::unique_ptr<RgaEngine>{}};
    CHECK(!missing.run(input.data(),53,71,53*3,64,64,out.data(),out.size(),64*3,b,t));
}
int main() {
    unsetenv("PERCEPTION_PREPROCESS");equivalence();failures();
    std::printf("PASS preprocess RGA selector: %zu checks, RGB golden geometry, stereo ROI stride, guards, staged-copy contract, persistent fallback, corrupt-output rejection (injected engine; no hardware speed claim)\n",checks);
}
