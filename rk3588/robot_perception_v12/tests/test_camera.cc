#include "camera.h"
#include <cassert>
#include <iostream>
#include <linux/videodev2.h>
int main(){
    std::vector<uint8_t> rgb;std::string error;
    CameraLayout f;f.width=2;f.height=2;f.stride=4;f.fourcc=V4L2_PIX_FMT_NV12;
    uint8_t nv[]={16,235,9,9,81,145,9,9,128,128,9,9};
    assert(camera_to_rgb(f,nv,sizeof(nv),nullptr,0,rgb,error));
    for(int i=0;i<3;i++){assert(rgb[i]==0);assert(rgb[3+i]==255);}
    assert(!camera_to_rgb(f,nv,9,nullptr,0,rgb,error));
    uint8_t uv[]={90,240,0,0};nv[0]=81;f.chroma_stride=4;
    assert(camera_to_rgb(f,nv,8,uv,4,rgb,error));assert(rgb[0]>250&&rgb[1]<5&&rgb[2]<5);
    f.fourcc=V4L2_PIX_FMT_NV21;uint8_t vu[]={240,90,0,0};
    assert(camera_to_rgb(f,nv,8,vu,4,rgb,error));assert(rgb[0]>250&&rgb[1]<5&&rgb[2]<5);
    f.fourcc=V4L2_PIX_FMT_YUYV;uint8_t yuyv[]={16,128,235,128,16,128,235,128};
    assert(camera_to_rgb(f,yuyv,8,nullptr,0,rgb,error));assert(rgb[0]==0&&rgb[3]==255);
    f.fourcc=V4L2_PIX_FMT_BGR24;f.stride=8;uint8_t bgr[]={1,2,3,4,5,6,0,0,7,8,9,10,11,12,0,0};
    assert(camera_to_rgb(f,bgr,16,nullptr,0,rgb,error));assert(rgb[0]==3&&rgb[3]==6&&rgb[6]==9);
    f.fourcc=V4L2_PIX_FMT_NV12;f.stride=4;f.full_range=true;f.bt709=true;
    uint8_t full[]={0,255,0,0,0,255,0,0,128,128,0,0};
    assert(camera_to_rgb(f,full,12,nullptr,0,rgb,error));assert(rgb[0]==0&&rgb[3]==255);
    f.full_range=false;uint8_t red709[]={63,63,0,0,63,63,0,0,102,240,0,0};
    assert(camera_to_rgb(f,red709,12,nullptr,0,rgb,error));assert(rgb[0]>250&&rgb[1]<3&&rgb[2]<3);
    f.width=3;assert(!camera_to_rgb(f,full,12,nullptr,0,rgb,error));
    Camera camera;assert(!camera.open("/this-camera-does-not-exist",1920,1080,error));
    std::cout<<"PASS camera stride, separate/packed UV, NV12/NV21, YUYV, BGR, 601/709 range, truncation, open failure\n";
}
