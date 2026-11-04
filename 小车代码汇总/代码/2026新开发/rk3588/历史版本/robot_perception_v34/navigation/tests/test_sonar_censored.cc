#include "../navcore.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace robot_nav;
int main(){
    unsigned checks=0;auto check=[&](bool v){assert(v);++checks;};
    Config c;c.configured=true;c.drive_mps_per_full_pwm=.6;c.map_ttl_s=60;
    const Point origin{0,-3.2};const Pose p{};
    const double sx=c.sonar_mounts[0][0],sy=c.sonar_mounts[0][1];
    for(double over:{4.001,4.2,4.5}){
        OccupancyGrid g(c);check(g.import_observations(std::vector<int8_t>(4096,-1),origin,1.));
        g.update_sonar(p,0,over,2.);
        check(g.state(g.cell(sx+2.,sy),2.)==0);
        check(g.state(g.cell(sx+4.,sy),2.)==-1);
        check(g.state(g.cell(sx+4.4,sy),2.)==-1);
        for(unsigned i=0;i<4096;++i){
            check(g.values()[i]<=0); // no invented occupied ring anywhere
            const auto center=g.center({int(i%64),int(i/64)});
            if(std::hypot(center.first-sx,center.second-sy)>=4.)check(!std::isfinite(g.observed_s()[i]));
        }
    }
    {
        OccupancyGrid g(c);check(g.import_observations(std::vector<int8_t>(4096,-1),origin,1.));
        g.update_sonar(p,0,4.,2.);check(g.state(g.cell(sx+4.,sy),2.)==1);
        const auto rev=g.revision();
        for(double bad:{0.,.029,4.501,std::numeric_limits<double>::quiet_NaN()}){
            g.update_sonar(p,0,bad,3.);check(g.revision()==rev);
        }
    }
    {
        OccupancyGrid g(c);std::vector<int8_t> cells(4096,-1);
        // Keep the >4m region inside the map so accidental clearing is visible.
        g.import_observations(cells,origin,1.);
        const Cell a=g.cell(sx+4.05,sy),b=g.cell(sx+4.45,sy);
        cells[g.index(a)]=cells[g.index(b)]=1;check(g.import_observations(cells,origin,1.));
        for(unsigned i=0;i<20;++i)g.update_sonar(p,0,4.5,2.+i*.1);
        check(g.state(a,4.)==1&&g.state(b,4.)==1);
        check(g.state(a,61.1)==-1&&g.state(b,61.1)==-1); // not refreshed
    }
    std::cout<<"PASS "<<checks<<" censored sonar map checks: >4m clears only near cone, no fake endpoint, exact4m echo retained, invalids unknown\n";
}
