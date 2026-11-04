#include "acg138_fpga_map_bridge/map_core.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char**argv){if(argc!=2)return 1;std::ifstream in(argv[1]);acg138::Grid g;in>>g.width>>g.height>>g.resolution>>g.origin.x>>g.origin.y;g.cells.resize(g.width*g.height);for(auto &v:g.cells){int a;in>>a;v=a;}
 for(double radius:{0.,.05,.1,.15,.2,.2163}){acg138::CropConfig c;c.footprint_radius=radius;c.safety_margin=radius==.2163?.05:0;const auto o=acg138::crop_and_inflate(g,{},c);std::cout<<"{\"diagnostic_only_radius_m\":"<<radius<<",\"margin_m\":"<<c.safety_margin<<",\"free\":"<<o.free_cells<<",\"start_blocked\":"<<o.is_blocked(o.cell(0,0))<<"}\n";}}
