#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace robot_nav {
// Both timestamps use the FPGA clk50m millisecond epoch. Host arrival time is
// intentionally absent: callers retain it separately for telemetry freshness.
struct ImuYawSample {
    uint32_t timestamp_ms=0, sequence=0;
    double yaw_rad=0;
};
enum class ImuPushStatus { Accepted, Duplicate, IgnoredOld, Fault };
enum class ImuLookupStatus { Ready, NeedFuture, BeforeHistory, Fault };
struct ImuAlignmentDiagnostics {
    uint32_t before_sequence=0, after_sequence=0;
    uint32_t before_timestamp_ms=0, after_timestamp_ms=0;
    double dt_s=0, dyaw_rad=0;
    std::string event="empty";
};
struct AlignedYaw {
    ImuLookupStatus status=ImuLookupStatus::NeedFuture;
    double yaw_rad=0;
    int64_t source_unwrapped_ms=0;
    ImuAlignmentDiagnostics bracket;
};
class ImuAlignment {
public:
    explicit ImuAlignment(double max_turn_rps=5., double jump_allowance_rad=.06,
                          uint32_t max_gap_ms=250);
    ImuPushStatus push(const ImuYawSample& sample);
    AlignedYaw lookup(uint32_t sensor_uptime_ms) const;
    void reset(); // only explicit map/localization reset, never HTTP recovery
    bool faulted() const { return !fault_.empty(); }
    const std::string& fault_reason() const { return fault_; }
    const ImuAlignmentDiagnostics& diagnostics() const { return diagnostics_; }
    std::optional<int64_t> latest_unwrapped_ms() const;
    size_t size() const { return history_.size(); }
private:
    struct Entry {ImuYawSample sample; int64_t time_ms=0;};
    std::deque<Entry> history_;
    double max_turn_rps_, jump_allowance_rad_;
    uint32_t max_gap_ms_;
    std::string fault_;
    ImuAlignmentDiagnostics diagnostics_;
    ImuPushStatus fail(const std::string& reason);
};
} // namespace robot_nav
