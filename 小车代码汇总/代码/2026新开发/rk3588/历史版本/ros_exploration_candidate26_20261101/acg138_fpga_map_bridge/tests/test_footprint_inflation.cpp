#include "acg138_fpga_map_bridge/map_core.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace acg138;
static int checks=0;
#define CHECK(x) do { ++checks; if(!(x))throw std::runtime_error("line "+std::to_string(__LINE__)+": " #x); } while(0)
static Grid room(){Grid g;g.width=g.height=200;g.resolution=.05;g.origin={-5,-5,0};g.cells.assign(40000,0);return g;}
int main(){try{
  CropConfig cfg;CHECK(cfg.footprint_radius==.15&&cfg.body_width==.30);
  auto g=room();g.cells[100*200+102]=100;
  auto p=crop_and_inflate(g,{0,0,0},cfg);
  CHECK(!p.is_blocked(p.cell(0,0)));CHECK(p.is_blocked(p.cell(.1,0)));
  CHECK(p.is_blocked(p.cell(.25,0)));CHECK(!p.is_blocked(p.cell(.55,0)));
  cfg.self_footprint_exemption=false;
  auto strict=crop_and_inflate(g,{0,0,0},cfg);CHECK(strict.is_blocked(strict.cell(0,0)));
  cfg.self_footprint_exemption=true;
  // Replanning changes the mask; old pose is no longer exempt.
  auto moved=crop_and_inflate(g,{-.5,0,0},cfg);CHECK(moved.is_blocked(moved.cell(0,0)));
  for(int v:{-1,26,100}){g=room();g.cells[100*200+100]=v;p=crop_and_inflate(g,{0,0,0},cfg);CHECK(p.is_blocked(p.cell(0,0)));}
  // A thin directly observed obstacle survives sub-cell downsampling.
  g=room();g.width=g.height=1000;g.resolution=.01;g.cells.assign(1000000,0);g.cells[500*1000+509]=100;
  p=crop_and_inflate(g,{0,0,0},cfg);CHECK(p.is_blocked(p.cell(.05,.05)));
  // Width/length orientation: a free target overlaps the rear only at yaw=0.
  g=room();g.cells[100*200+94]=100;
  auto a=crop_and_inflate(g,{.055,.05,0},cfg),b=crop_and_inflate(g,{.055,.05,1.5707963267948966},cfg);
  CHECK(!a.is_blocked(a.cell(-.15,.05)));CHECK(b.is_blocked(b.cell(-.15,.05)));
  // Area-overlap source rotation and unknown handling remain conservative.
  g=room();g.origin={0,-7.0710678118654755,.7853981633974483};p=crop_and_inflate(g,{0,0,.7},cfg);CHECK(!p.is_blocked(p.cell(0,0)));
  for(unsigned x=0;x<64;++x){CHECK(p.is_blocked(x));CHECK(p.is_blocked(63*64+x));}
  cfg.body_width=0;bool rejected=false;try{crop_and_inflate(g,{0,0,0},cfg);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);
  std::cout<<"{\"passed\":true,\"checks\":"<<checks<<",\"fpga_messages_sent\":0}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
