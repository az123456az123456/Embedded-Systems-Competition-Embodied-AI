#pragma once
#include <chrono>
namespace acg138::nav3 {
// Advance against service entry, not slow send completion. Skip missed slots;
// never queue/retransmit an old TF sample to make up a nominal 50 Hz count.
inline std::chrono::steady_clock::time_point next_pose_slot(
    std::chrono::steady_clock::time_point due,
    std::chrono::steady_clock::time_point service_started) {
  const auto period=std::chrono::milliseconds(20);
  if(service_started<due)return due;
  return due+period*(1+(service_started-due)/period);
}
}
