extern "C" {
#include "dashboard.h"
#include "display_transform.h"
}
#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
static void ppm(const char*file,const std::vector<uint32_t>&p,int w,int h){std::ofstream f(file,std::ios::binary);f<<"P6\n"<<w<<" "<<h<<"\n255\n";for(auto v:p){char c[]={char(v>>16),char(v>>8),char(v)};f.write(c,3);}}
int main(int argc,char**argv){
 assert(argc>=2);auto d=std::make_unique<dashboard>();dashboard_init(d.get());assert(navigation_map_read(argv[1],&d->navigation));assert(d->navigation.render_width>64);d->navigation_selected=1;
 imu_state imu{};imu_init(&imu);dashboard_render(d.get(),&imu,10000,"");
 for(int rotation:{0,90,180,270}){
  display_transform t{};int w=rotation%180?960:1536,h=rotation%180?1536:960;assert(!display_transform_init(&t,w,h,VIEW_W,VIEW_H,rotation));
  std::vector<uint32_t>pixels(size_t(w)*h+32,0x12345678);display_transform_blit(&t,pixels.data(),d->pixels,0xfff5f5f7);auto before=pixels;
  auto start=std::chrono::steady_clock::now();dashboard_navigation_native(d.get(),&t,pixels.data(),10000);double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  unsigned changed=0;for(size_t i=0;i<size_t(w)*h;i++)if(pixels[i]!=before[i]){changed++;double sx,sy;assert(display_transform_point(&t,i%w+.5,i/w+.5,&sx,&sy));assert(sx>=263&&sx<729&&sy>=148&&sy<614);}
  for(size_t i=size_t(w)*h;i<pixels.size();i++)assert(pixels[i]==0x12345678);
  assert(changed>100);std::cout<<"{\"rotation\":"<<rotation<<",\"pixels_changed\":"<<changed<<",\"native_paint_ms\":"<<elapsed<<",\"pass\":true}\n";
  if(argc>2&&rotation==0)ppm(argv[2],pixels,w,h);
 }
 {display_transform t{};assert(!display_transform_init(&t,1536,960,VIEW_W,VIEW_H,0));
  std::vector<uint32_t>a(1536*960,0),b(1536*960,0);dashboard_navigation_native(d.get(),&t,a.data(),10000);
  d->navigation.origin[0]=-3.2;d->navigation.origin[1]=-3.2;d->navigation.resolution_m=.1;
  dashboard_navigation_native(d.get(),&t,b.data(),10000);assert(a==b);
  std::cout<<"{\"full_map_after_local_FPGA_commit\":true}\n";}
 d->navigation_selected=0;display_transform t{};assert(!display_transform_init(&t,1536,960,VIEW_W,VIEW_H,0));std::vector<uint32_t>v(1536*960,0x87654321);dashboard_navigation_native(d.get(),&t,v.data(),10000);for(auto p:v)assert(p==0x87654321);
 std::cout<<"{\"camera_tab_unchanged\":true}\n";
}
