#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace robotnav {
namespace display_detail {
struct Point { double x,y; };
// Clip a transformed source-cell footprint against one display cell. Positive
// area, not boundary contact, is required. All coordinates are output-cell local.
inline double overlap(std::array<Point,12> p, double side) {
 int count=4;
 for(int edge=0;edge<4&&count;++edge){
  std::array<Point,12> out{};int n=0;
  auto distance=[&](Point a){return edge==0?a.x:edge==1?side-a.x:edge==2?a.y:side-a.y;};
  auto a=p[count-1];double da=distance(a);
  for(int i=0;i<count;++i){auto b=p[i];double db=distance(b);
   if((da>=0)!=(db>=0)){double t=da/(da-db);out[n++]={a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)};}
   if(db>=0)out[n++]=b;
   a=b;da=db;
  }
  p=out;count=n;
 }
 double area=0;
 for(int i=0;i<count;++i){auto a=p[i],b=p[(i+1)%count];area+=a.x*b.y-a.y*b.x;}
 return std::abs(area)*.5;
}
}
// DISPLAY ONLY. Pool actual intersecting cell areas so a thin measured wall
// cannot vanish between sample centres. Never invent a wall across a gap.
// Occupied wins; free requires full observed free coverage; otherwise unknown.
// This must not be used to replace the acknowledged FPGA navigable bitmap.
inline std::string display_grid(uint32_t width,uint32_t height,double resolution,
 double origin_x,double origin_y,double yaw,const std::vector<int8_t>&cells,
 double ox,double oy,double res) {
 std::string result(4096,'?');
 if(!width||!height||cells.size()!=size_t(width)*height||!std::isfinite(resolution)||resolution<=0||!std::isfinite(res)||res<=0||
    !std::isfinite(origin_x)||!std::isfinite(origin_y)||!std::isfinite(yaw)||!std::isfinite(ox)||!std::isfinite(oy))return result;
 const double c=std::cos(yaw),s=std::sin(yaw),target_area=res*res,eps=target_area*1e-10;
 for(unsigned i=0;i<4096;++i){
  const double x=ox+(i%64)*res,y=oy+(i/64)*res;
  double lo_x=1e100,lo_y=1e100,hi_x=-1e100,hi_y=-1e100;
  for(int a=0;a<=1;++a)for(int b=0;b<=1;++b){double dx=x+a*res-origin_x,dy=y+b*res-origin_y;
   double gx=(c*dx+s*dy)/resolution,gy=(-s*dx+c*dy)/resolution;
   lo_x=std::min(lo_x,gx);hi_x=std::max(hi_x,gx);lo_y=std::min(lo_y,gy);hi_y=std::max(hi_y,gy);}
  // Clamp in floating point before conversion, including entirely outside maps.
  int x0=int(std::floor(std::clamp(lo_x,0.,double(width)))),x1=int(std::ceil(std::clamp(hi_x,0.,double(width))));
  int y0=int(std::floor(std::clamp(lo_y,0.,double(height)))),y1=int(std::ceil(std::clamp(hi_y,0.,double(height))));
  double free_area=0;bool occupied=false;
  for(int gy=y0;gy<y1&&!occupied;++gy)for(int gx=x0;gx<x1;++gx){
   int v=cells[size_t(gy)*width+gx];if(v<0)continue;
   double area=0;
   if(std::abs(yaw)<1e-12){
    double sx=origin_x+gx*resolution,sy=origin_y+gy*resolution;
    area=std::max(0.,std::min(x+res,sx+resolution)-std::max(x,sx))*std::max(0.,std::min(y+res,sy+resolution)-std::max(y,sy));
   }else{
    const double half=resolution*.5,extent=(std::abs(c)+std::abs(s))*half;
    const double cx=origin_x+c*(gx+.5)*resolution-s*(gy+.5)*resolution-x;
    const double cy=origin_y+s*(gx+.5)*resolution+c*(gy+.5)*resolution-y;
    // Most source cells are entirely inside/outside a coarse overview cell.
    // Only boundary intersections need polygon clipping (same exact geometry).
    if(cx+extent<=0||cy+extent<=0||cx-extent>=res||cy-extent>=res)continue;
    if(cx-extent>=0&&cy-extent>=0&&cx+extent<=res&&cy+extent<=res)area=resolution*resolution;
    else {
     std::array<display_detail::Point,12> p{};const int xs[]={-1,1,1,-1},ys[]={-1,-1,1,1};
     for(int j=0;j<4;++j)p[j]={cx+c*xs[j]*half-s*ys[j]*half,cy+s*xs[j]*half+c*ys[j]*half};
     area=display_detail::overlap(p,res);
    }
   }
   if(area<=eps)continue;
   if(v>25){occupied=true;break;}free_area+=area;
  }
  result[i]=occupied?'#':free_area>=target_area-eps?'.':'?';
 }
 return result;
}
}
