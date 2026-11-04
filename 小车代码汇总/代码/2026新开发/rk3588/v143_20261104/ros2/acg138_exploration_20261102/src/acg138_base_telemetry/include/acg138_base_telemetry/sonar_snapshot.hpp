#pragma once
#include <cstdint>
#include <sstream>
#include <string>
extern "C" {
#include "sensor_data.h"
}
namespace acg138 {
// Capture source ages at the kernel-receive monotonic timestamp. Repeated
// packets from one device millisecond retain sensor_state.received_ms.
inline std::string sonar_snapshot(const sensor_state& state) {
 const auto& sample=state.sample;
 std::ostringstream out;
 out<<"{\"received_mono_ns\":\""<<state.received_ms*UINT64_C(1000000)
    <<"\",\"sequence\":"<<sample.sequence<<",\"uptime_ms\":"<<sample.uptime_ms
    <<",\"valid_mask\":"<<unsigned(sample.valid_mask)<<",\"fault_mask\":"<<unsigned(sample.fault_mask)
    <<",\"distance_mm\":["<<sample.distance_mm[0]<<','<<sample.distance_mm[1]<<','<<sample.distance_mm[2]
    <<"],\"age_ms\":["<<sample.age_ms[0]<<','<<sample.age_ms[1]<<','<<sample.age_ms[2]<<"]}";
 return out.str();
}
}
