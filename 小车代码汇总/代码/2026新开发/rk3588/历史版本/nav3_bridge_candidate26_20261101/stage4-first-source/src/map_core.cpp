#include "acg138_fpga_map_bridge/map_core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace acg138 {
namespace {
constexpr double kEps = 1e-9;
using Polygon = std::array<std::array<double, 2>, 4>;
bool finite_pose(const Pose &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
// SAT: a rotated target square in source-cell coordinates versus one source
// cell. Merely touching the boundary does not create an area overlap.
bool overlaps(const Polygon &p, int x, int y) {
  const std::array<std::array<double, 2>, 4> axes{{
    {{1, 0}}, {{0, 1}},
    {{p[1][1] - p[0][1], p[0][0] - p[1][0]}},
    {{p[2][1] - p[1][1], p[1][0] - p[2][0]}}
  }};
  for (const auto &a : axes) {
    double lo = std::numeric_limits<double>::infinity(), hi = -lo;
    for (const auto &v : p) { const double d = v[0]*a[0] + v[1]*a[1]; lo = std::min(lo,d); hi=std::max(hi,d); }
    const double center = (x+.5)*a[0] + (y+.5)*a[1];
    const double radius = .5*(std::abs(a[0])+std::abs(a[1]));
    const double tolerance = kEps*std::hypot(a[0],a[1]);
    if (hi <= center-radius+tolerance || lo >= center+radius-tolerance) return false;
  }
  return true;
}
bool observed_free(const Grid &g, double wx, double wy, double side, int threshold) {
  const double c=std::cos(g.origin.yaw), s=std::sin(g.origin.yaw);
  Polygon p;
  const double offsets[4][2]={{0,0},{1,0},{1,1},{0,1}};
  double minx=std::numeric_limits<double>::infinity(),miny=minx,maxx=-minx,maxy=-minx;
  for (unsigned i=0;i<4;++i) {
    const double x=wx+offsets[i][0]*side-g.origin.x, y=wy+offsets[i][1]*side-g.origin.y;
    p[i]={{(c*x+s*y)/g.resolution,(-s*x+c*y)/g.resolution}};
    minx=std::min(minx,p[i][0]);maxx=std::max(maxx,p[i][0]);miny=std::min(miny,p[i][1]);maxy=std::max(maxy,p[i][1]);
  }
  if(minx < -kEps || miny < -kEps || maxx > g.width+kEps || maxy > g.height+kEps) return false;
  const int x0=std::max(0,int(std::floor(minx+kEps))), y0=std::max(0,int(std::floor(miny+kEps)));
  const int x1=std::min(int(g.width)-1,int(std::ceil(maxx-kEps))-1),y1=std::min(int(g.height)-1,int(std::ceil(maxy-kEps))-1);
  bool sampled=false;
  for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x)if(overlaps(p,x,y)) {
    sampled=true; const int v=g.cells[size_t(y)*g.width+unsigned(x)];
    if(v < 0 || v > threshold) return false;
  }
  return sampled;
}
}  // namespace

bool LocalGrid::is_blocked(unsigned id) const { return id>=kCells || (blocked[id>>3] & (1u<<(id&7))); }
std::vector<int8_t> LocalGrid::occupancy() const {
  std::vector<int8_t> out(kCells); for(unsigned i=0;i<kCells;++i)out[i]=is_blocked(i)?100:0;return out;
}
uint16_t LocalGrid::cell(double x,double y) const {
  if(!std::isfinite(x)||!std::isfinite(y))throw std::invalid_argument("non-finite world coordinate");
  const double a=std::floor((x-origin_x)/resolution),b=std::floor((y-origin_y)/resolution);
  if(a<0||a>=kSide||b<0||b>=kSide)throw std::out_of_range("world coordinate outside 64x64 local window");
  return static_cast<uint16_t>(unsigned(b)*kSide+unsigned(a));
}
Pose LocalGrid::center(uint16_t id) const {
  if(id>=kCells)throw std::out_of_range("cell outside 64x64 local window");
  return {origin_x+(id%kSide+.5)*resolution,origin_y+(id/kSide+.5)*resolution,0};
}
LocalGrid crop_and_inflate(const Grid &g,const Pose &pose,const CropConfig &cfg) {
  if(!g.width||!g.height||g.width>16384||g.height>16384||uint64_t(g.width)*g.height>16000000||
     g.cells.size()!=size_t(g.width)*g.height||!finite_pose(g.origin)||!finite_pose(pose)||
     !std::isfinite(g.resolution)||g.resolution<.005||g.resolution>1||
     !std::isfinite(cfg.resolution)||cfg.resolution<.02||cfg.resolution>.5||
     !std::isfinite(cfg.footprint_radius)||cfg.footprint_radius<0||cfg.footprint_radius>1||
     !std::isfinite(cfg.safety_margin)||cfg.safety_margin<0||cfg.safety_margin>1||
     cfg.free_threshold<0||cfg.free_threshold>99)
    throw std::invalid_argument("invalid grid dimensions, geometry or crop configuration");
  for(int v:g.cells)if(v < -1 || v>100)throw std::invalid_argument("OccupancyGrid value outside -1..100");
  LocalGrid out;out.resolution=cfg.resolution;
  out.origin_x=(std::floor(pose.x/cfg.resolution)-32)*cfg.resolution;
  out.origin_y=(std::floor(pose.y/cfg.resolution)-32)*cfg.resolution;
  if(!std::isfinite(out.origin_x)||!std::isfinite(out.origin_y))throw std::invalid_argument("local window origin overflows");
  std::array<bool,kCells> free{};
  for(unsigned y=1;y<kSide-1;++y)for(unsigned x=1;x<kSide-1;++x)
    free[y*kSide+x]=observed_free(g,out.origin_x+x*out.resolution,out.origin_y+y*out.resolution,out.resolution,cfg.free_threshold);
  out.blocked.fill(0xff);
  const double radius=cfg.footprint_radius+cfg.safety_margin;
  // Minkowski padding of the WHOLE blocked square, not just its center.
  // Target center-to-source-square distance makes the grid discretization
  // conservative without inventing a free bubble under the vehicle.
  const int n=int(std::ceil(radius/out.resolution+.5));
  std::vector<std::array<int,2>> offsets;
  for(int dy=-n;dy<=n;++dy)for(int dx=-n;dx<=n;++dx) {
    const double nearx=std::max(0.,(std::abs(dx)-.5)*out.resolution),neary=std::max(0.,(std::abs(dy)-.5)*out.resolution);
    if(std::hypot(nearx,neary)<=radius+kEps)offsets.push_back({{dx,dy}});
  }
  for(int y=1;y<int(kSide)-1;++y)for(int x=1;x<int(kSide)-1;++x)if(free[y*kSide+x]) {
    bool clear=true;
    for(const auto &d:offsets){const int xx=x+d[0],yy=y+d[1];if(xx<0||yy<0||xx>=int(kSide)||yy>=int(kSide)||!free[yy*kSide+xx]){clear=false;break;}}
    if(clear){const unsigned id=unsigned(y)*kSide+unsigned(x);out.blocked[id>>3]&=uint8_t(~(1u<<(id&7)));++out.free_cells;}
  }
  return out;
}
std::string json_escape(const std::string &in) {
  static const char hex[]="0123456789abcdef";std::string out;
  for(unsigned char c:in){if(c=='"'||c=='\\'){out+='\\';out+=char(c);}else if(c<32){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}else out+=char(c);}return out;
}
}  // namespace acg138
