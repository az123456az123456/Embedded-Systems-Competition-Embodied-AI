#include "../gateway/display_grid.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <chrono>
int main(int argc,char**argv){
 if(argc==3){std::ifstream f(argv[1]);unsigned w,h;double r,x,y,a,ox,oy,res;f>>w>>h>>r>>x>>y>>a>>ox>>oy>>res;std::vector<int8_t>v(size_t(w)*h);for(auto&c:v){int n;f>>n;c=n;}assert(f);auto start=std::chrono::steady_clock::now();auto out=robotnav::display_grid(w,h,r,x,y,a,v,ox,oy,res);std::ofstream(argv[2])<<out;std::cout<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<" ms\n";return 0;}
 unsigned cases=0;auto check=[&](bool b,const char*name){std::cout<<"{\"case\":\""<<name<<"\",\"pass\":"<<(b?"true":"false")<<"}\n";assert(b);++cases;};
 std::vector<int8_t> v(128*128,0);
 auto raster=[&](double ox=0,double oy=0,double yaw=0){return robotnav::display_grid(128,128,.05,0,0,yaw,v,ox,oy,.1);};
 check(raster()==std::string(4096,'.'),"all_observed_free");
 for(int y=0;y<128;++y)v[y*128+20]=100;
 auto o=raster();bool wall=true;for(int y=0;y<64;++y)wall&=o[y*64+10]=='#';check(wall,"thin_wall_between_old_sample_centres");
 check(o[9]=='.'&&o[11]=='.',"no_wall_on_touching_boundary");
 for(double shift:{.001,.025,.049,.051,.099}){o=raster(shift);bool hit=true;for(int y=0;y<64;++y)hit&=o[y*64+unsigned(std::floor((1.-shift)/.1))]=='#';check(hit,"translated_thin_wall");}
 for(int y=40;y<60;++y)v[y*128+20]=0;
 o=raster();bool door=true;for(int y=20;y<30;++y)door&=o[y*64+10]=='.';check(door,"measured_doorway_remains_open");
 v.assign(v.size(),-1);v[0]=0;check(raster()[0]=='?',"partially_unknown_not_free");
 v[0]=100;check(raster()[0]=='#',"occupied_inside_unknown_visible");
 v.assign(v.size(),0);check(raster(-.05,-.05)[0]=='?',"outside_partial_coverage_not_free");
 v.assign(v.size(),-1);check(raster()==std::string(4096,'?'),"all_unknown");
 v.assign(v.size(),0);for(int y=0;y<128;++y)v[y*128+20]=100;
 o=robotnav::display_grid(128,128,.05,6.4,0,1.5707963267948966,v,0,0,.1);wall=true;for(int x=0;x<64;++x)wall&=o[10*64+x]=='#';check(wall,"rotated_thin_wall_90deg");
 o=robotnav::display_grid(128,128,.05,0,0,0,v,0,0,.05);check(o[20]=='#'&&o[21]=='.',"identity_resolution");
 o=robotnav::display_grid(128,128,.05,0,0,.37,v,-3,0,.15);wall=true;
 for(int y=0;y<128;++y){double xw=std::cos(.37)*1.025-std::sin(.37)*(y+.5)*.05,yw=std::sin(.37)*1.025+std::cos(.37)*(y+.5)*.05;int dx=int(std::floor((xw+3)/.15)),dy=int(std::floor(yw/.15));if(dx>=0&&dx<64&&dy>=0&&dy<64)wall&=o[dy*64+dx]=='#';}check(wall,"rotated_wall_footprint_no_lost_centres");
 check(robotnav::display_grid(128,128,0,0,0,0,v,0,0,.1)==std::string(4096,'?'),"invalid_geometry_fail_unknown");
 std::cout<<"{\"total\":"<<cases<<",\"status\":\"PASS\"}\n";
}
