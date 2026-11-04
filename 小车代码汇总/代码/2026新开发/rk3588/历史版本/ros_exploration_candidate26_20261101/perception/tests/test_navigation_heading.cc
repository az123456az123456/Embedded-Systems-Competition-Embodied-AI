#include "navigation_render.h"
#include <cassert>
#include <cstring>
#include <memory>
#include <vector>
#include <iostream>
int main(){
 auto m=std::make_unique<navigation_map>();m->loaded=m->available=m->pose_valid=1;m->width=m->height=64;m->resolution_m=.1;m->pose[0]=m->pose[1]=3.2;
 std::memset(m->cells,'.',4096);std::vector<uint32_t> pixels(640*640);
 for(double yaw:{0.,1.5707963267948966,3.141592653589793,-1.5707963267948966}){
  m->pose[2]=yaw;navigation_paint(m.get(),pixels.data(),640,640,0);
  int x=320-(int)std::lround(10*std::cos(yaw)),y=320+(int)std::lround(10*std::sin(yaw));
  assert(pixels[y*640+x]==(0xff000000u|NAV_RED));
  assert(pixels[(640-y)*640+640-x]!=(0xff000000u|NAV_RED));
  assert(m->pose[2]==yaw&&m->pose[0]==3.2&&m->pose[1]==3.2);
 }
 std::cout<<"{\"four_heading_icons_corrected\":true,\"pose_coordinates_unchanged\":true}\n";
}
