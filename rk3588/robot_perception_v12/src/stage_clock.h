#ifndef PERCEPTION_STAGE_CLOCK_H
#define PERCEPTION_STAGE_CLOCK_H
#include <chrono>
using StageClock = std::chrono::steady_clock;
inline double stage_elapsed(StageClock::time_point start) {
    return std::chrono::duration<double,std::milli>(StageClock::now()-start).count();
}
// Records elapsed time on success and every early-return path.
struct StageTotal {
    double *value;
    StageClock::time_point start=StageClock::now();
    explicit StageTotal(double *out):value(out){}
    ~StageTotal(){if(value)*value=stage_elapsed(start);}
};
#endif
