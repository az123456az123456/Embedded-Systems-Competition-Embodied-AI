#include "detector_math.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
// Continuous bilinear reference: checks the optimized integer path against
// the interpolation definition, including color/stride and letterbox edges.
int main() {
    std::mt19937 random(59187);std::string error;int maximum_error=0;
    for(int trial=0;trial<24;trial++) {
        int w=2+random()%1000,h=2+random()%1100,stride=w*3+7;
        std::vector<uint8_t> input(stride*h),out;
        for(auto &v:input)v=random()%256;
        detector_math::Letterbox box;
        assert(detector_math::preprocess_rgb(input.data(),w,h,stride,640,640,out,box,error));
        for(int y=0;y<640;y+=3)for(int x=0;x<640;x+=3)for(int c=0;c<3;c++) {
            int expected=114;
            if(x>=box.pad_x && x<box.pad_x+box.resized_w && y>=box.pad_y && y<box.pad_y+box.resized_h) {
                float sx=std::clamp((x-box.pad_x+.5f)/box.scale_x-.5f,0.f,float(w-1));
                float sy=std::clamp((y-box.pad_y+.5f)/box.scale_y-.5f,0.f,float(h-1));
                int x0=sx,y0=sy;double wx=sx-x0,wy=sy-y0,value=0;
                for(int yy=0;yy<2;yy++)for(int xx=0;xx<2;xx++)
                    value+=input[std::min(y0+yy,h-1)*stride+std::min(x0+xx,w-1)*3+c]*(xx?wx:1-wx)*(yy?wy:1-wy);
                expected=std::lround(value);
            }
            maximum_error=std::max(maximum_error,std::abs(int(out[(y*640+x)*3+c])-expected));
        }
    }
    assert(maximum_error<=1);
    std::printf("PASS bilinear color/letterbox equivalence: 24 random shapes, RGB max error=%d/255\n",maximum_error);
}
