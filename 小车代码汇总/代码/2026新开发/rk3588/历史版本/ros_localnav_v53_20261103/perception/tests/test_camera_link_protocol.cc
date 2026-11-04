#include "camera_link_protocol.h"
#include <array>
#include <cstdio>
#include <cstring>
#include <stdexcept>
using camera_link::Mode;
static unsigned checks;
static void ck(bool result){++checks;if(!result)throw std::runtime_error("camera wire classification");}
int main(){std::array<uint8_t,4096> p{};const uint8_t mac[]={2,0xac,0x13,0x80,0,1};const uint8_t raw_mac[]={0,0x0a,0x35,1,0xfe,0xc0};std::memcpy(p.data()+6,raw_mac,6);
 p[12]=8;p[14]=0x45;p[23]=17;p[36]=0x17;p[37]=0xd6;p[38]=7;p[39]=0x8a;
 ck(camera_link::classify(p.data(),1964)==Mode::Raw);
 p[43]=4;ck(camera_link::classify(p.data(),1964)==Mode::Raw);p[42]=0xff;p[43]=0xff;ck(camera_link::classify(p.data(),1964)==Mode::Unknown);p[42]=p[43]=0;
 ck(camera_link::classify(p.data(),1500)==Mode::Unknown);p[20]=0x20;ck(camera_link::classify(p.data(),1964)==Mode::Unknown);p[20]=0;
 std::memcpy(p.data()+6,mac,6);p[12]=0x88;p[13]=0xba;std::memcpy(p.data()+14,"RGP1",4);p[18]=1;p[19]=0;p[20]=1;p[34]=p[36]=2;p[35]=p[37]=0x80;
 ck(camera_link::classify(p.data(),1006)==Mode::PreparedRgb);p[19]=1;ck(camera_link::classify(p.data(),1006)==Mode::PreparedRgb);p[19]=2;ck(camera_link::classify(p.data(),1006)==Mode::Unknown);p[19]=0;
 p[11]=2;ck(camera_link::classify(p.data(),1006)==Mode::Unknown);p[11]=1;
 ck(camera_link::classify(nullptr,0)==Mode::Unknown);p[13]=0xb8;ck(camera_link::classify(p.data(),1006)==Mode::Unknown);
 std::printf("PASS RAW/RGB/control protocol discrimination %u checks\n",checks);
}
