#ifndef PERCEPTION_FPGA_RGB_INPUT_H
#define PERCEPTION_FPGA_RGB_INPUT_H
#include "detector_math.h"
#include <cstring>

namespace detector_math {
// A prepared 640 square is already in model pixel coordinates. Keeping every
// byte also preserves FPGA-produced letterbox114 padding, including half rows.
inline bool copy_identity_rgb(const unsigned char *source,int width,int height,int stride,
                              unsigned char *destination,size_t bytes,size_t row,
                              int target_w,int target_h,Letterbox &box,std::string &error) {
    if(width!=640||height!=640||target_w!=640||target_h!=640) {
        error="FPGA RGB identity input requires 640x640 model pixels";return false;
    }
    constexpr size_t visible=640*3;
    if(!source||!destination||stride<static_cast<int>(visible)||row<visible||row>64u*1024u*1024u ||
       bytes<row*639+visible) {
        error="Invalid FPGA RGB identity source/destination stride";return false;
    }
    box={640,640,640,640,640,640,0,0,1.f,1.f};
    if(stride==static_cast<int>(visible)&&row==visible)std::memcpy(destination,source,640*visible);
    else for(size_t y=0;y<640;++y)std::memcpy(destination+y*row,source+y*static_cast<size_t>(stride),visible);
    return true;
}
}
#endif
