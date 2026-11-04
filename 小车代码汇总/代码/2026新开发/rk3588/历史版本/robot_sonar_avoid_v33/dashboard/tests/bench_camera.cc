// Synthetic conversion-only A/B. Run on the RK3588 to measure ARM NEON speed.
// Does not open the camera, change clocks, or modify system settings.
#include "camera.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <linux/videodev2.h>

static double measure(const CameraLayout &f,const std::vector<uint8_t> &nv12,
                      unsigned iterations,bool scalar,uint64_t &checksum) {
    if(scalar)setenv("PERCEPTION_CAMERA_SCALAR","1",1);else unsetenv("PERCEPTION_CAMERA_SCALAR");
    std::vector<uint8_t> out;std::string error;
    for(int i=0;i<3;i++)if(!camera_to_rgb(f,nv12.data(),nv12.size(),nullptr,0,out,error)){std::cerr<<error<<"\n";std::exit(1);}
    const auto start=std::chrono::steady_clock::now();
    for(unsigned i=0;i<iterations;i++) {
        if(!camera_to_rgb(f,nv12.data(),nv12.size(),nullptr,0,out,error)){std::cerr<<error<<"\n";std::exit(1);}
        checksum+=out[(size_t(i)*131)%out.size()];
    }
    return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/iterations;
}
int main(int argc,char **argv) {
    unsigned iterations=argc>1?std::strtoul(argv[1],nullptr,10):60;
    if(!iterations||iterations>10000){std::cerr<<"Usage: bench-camera [iterations:1..10000]\n";return 2;}
    CameraLayout f;f.width=1920;f.height=1080;f.stride=1920;f.fourcc=V4L2_PIX_FMT_NV12;f.bt709=true;
    std::vector<uint8_t> nv12(size_t(f.stride)*f.height*3/2);
    uint32_t random=0x3588;
    for(auto &b:nv12){random=random*1664525+1013904223;b=random>>24;}
    uint64_t checksum=0;
    // Run reference then automatic, and reverse order to reduce warm-up bias.
    const double scalar1=measure(f,nv12,iterations,true,checksum),fast1=measure(f,nv12,iterations,false,checksum);
    const double fast2=measure(f,nv12,iterations,false,checksum),scalar2=measure(f,nv12,iterations,true,checksum);
    unsetenv("PERCEPTION_CAMERA_SCALAR");
    const double scalar=(scalar1+scalar2)/2,fast=(fast1+fast2)/2;
    std::cout<<std::fixed<<std::setprecision(3)<<"CAMERA_BENCH width=1920 height=1080 format=NV12 matrix=BT709 range=limited iterations="<<iterations
             <<" backend="<<camera_color_backend()<<" scalar_ms="<<scalar<<" auto_ms="<<fast
             <<" speedup="<<scalar/fast<<" checksum="<<checksum<<"\n";
    std::cout<<"Synthetic conversion benchmark only; use PERF/STAGE on the board for final camera/YOLO throughput.\n";
}
