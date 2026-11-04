#pragma once
#include "controller.h"
extern "C" {
#include "sensor_data.h"
}
#include <algorithm>
namespace robotphone {
// Preserve the oldest actual measurement time, not the time the packet/UI was
// received. A stalled UI must never grant an old sonar measurement a new lease.
inline FollowRanges followRanges(const sensor_state& sensors,uint64_t now) {
    FollowRanges result;result.time_ms=sensors.received_ms;
    for(unsigned i=0;i<3;i++) {
        result.valid[i]=sensor_distance(&sensors,i,now,&result.metres[i])==SENSOR_VALID &&
            now>=sensors.received_ms && now-sensors.received_ms+sensors.sample.age_ms[i]<=500;
        if(sensors.received_ms<sensors.sample.age_ms[i])result.valid[i]=false;
        else result.time_ms=std::min(result.time_ms,sensors.received_ms-sensors.sample.age_ms[i]);
    }
    return result;
}
}
