#ifndef ROBOT_PHONE_STREAM_RATE_H
#define ROBOT_PHONE_STREAM_RATE_H
#include <algorithm>
#include <cmath>
namespace robotphone {
// Explicit FPS is a cap, not permission to duplicate frames or overrun the
// detector. Auto follows complete-pair throughput, never partial event rate.
inline double streamTargetFps(bool detect,double result,double preview,double capture,unsigned cap){
    auto usable=[](double value){return std::isfinite(value)&&value>0;};
    double rate=detect?(usable(result)?result:1.):usable(preview)?preview:usable(capture)?capture:1.;
    if(usable(preview))rate=std::min(rate,preview);
    if(usable(capture))rate=std::min(rate,capture);
    return std::max(1.,std::min(rate,double(cap?std::min(cap,60u):60u)));
}
}
#endif
