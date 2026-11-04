#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// The postprocessing core is deliberately independent of ROS.  The wire
// adapter can convert an incoming message to these fixed-width records and
// publish the JSON produced here without changing the FPGA contract.
namespace acg138::rk_postprocess {

struct FpgaState {
  std::uint8_t state = 0;
  std::uint8_t avoid = 0;
  std::uint16_t reason = 0;
  std::uint32_t state_flags = 0;
  std::uint64_t nav_epoch = 0;
  std::uint64_t state_seq = 0;
  std::uint64_t pose_seq = 0;
  std::int32_t x_mm = 0;
  std::int32_t y_mm = 0;
  std::int32_t yaw_urad = 0;
  std::uint32_t pose_age_ms = 0;
  std::uint16_t cursor = 0;
  std::uint16_t path_count = 0;
  std::uint32_t remaining_mm = 0;
  std::uint32_t permit_remaining_ms = 0;
  std::uint32_t auth_remaining_ms = 0;
  std::uint32_t active_crc32 = 0;
  std::uint32_t pending_crc32 = 0;
  std::uint32_t upload_crc32 = 0;
  std::uint32_t gate_reasons = 0;
  std::uint64_t pending_map_epoch = 0;
  std::uint64_t pending_map_revision = 0;
  std::uint64_t upload_map_epoch = 0;
  std::uint64_t upload_map_revision = 0;
  std::uint64_t last_ack_seq = 0;
  std::uint32_t device_ms = 0;
  std::uint32_t owner = 0;
  std::uint16_t upload_chunks = 0;
  std::uint16_t reserved = 0;
  std::uint32_t diagnostic = 0;
  std::uint64_t last_request_id = 0;
  std::uint32_t active_frame_token = 0;
  std::uint32_t pending_frame_token = 0;
  // Bytes 152..159 in the fixed STATE payload, represented as a BE word.
  std::uint64_t reserved_tail = 0;
};

struct FpgaPlanResult {
  std::uint16_t status = 0;
  std::uint16_t reason = 0;
  std::uint32_t device_ms = 0;
  std::uint64_t nav_epoch = 0;
  std::uint64_t state_seq = 0;
  std::uint64_t start_pose_seq = 0;
  std::uint32_t expanded = 0;
  std::uint32_t cycles = 0;
  std::uint16_t path_length = 0;
  std::uint16_t start_cell = 0;
  std::uint16_t goal_cell = 0;
  std::uint16_t result_flags = 0;
  std::uint32_t path_revision = 0;
  std::uint32_t frame_token = 0;
  std::uint32_t cpu_hz = 0;
  // Bytes 60..63 in the fixed PLAN_RESULT payload.
  std::uint32_t reserved_tail = 0;
};

// State and plan serializers use a fixed key order.  All source fields are
// emitted as decimal integers; the extra values are derived read-only hints.
std::string state_to_json(const FpgaState &state);
std::string plan_result_to_json(const FpgaPlanResult &result);
std::string gui_state_json(const FpgaState &state);
std::string phone_state_json(const FpgaState &state);
std::string gui_plan_result_json(const FpgaPlanResult &result);

// Parse the flat JSON emitted by the serializers.  Unknown fields are ignored
// so a future producer can add fields without breaking this consumer.  The
// function returns false and optionally writes a short error for malformed
// JSON, duplicate known keys, or out-of-range integers.
bool parse_state_json(const std::string &json, FpgaState &out,
                      std::string *error = nullptr);
bool parse_plan_result_json(const std::string &json, FpgaPlanResult &out,
                            std::string *error = nullptr);

struct PathPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PathSmoothingConfig {
  std::size_t radius = 1;
  double alpha = 0.25;
  double max_deviation = 0.5;
  double max_coordinate = 1.0e6;
};

// Smooths only for display.  The output has the same number of samples and
// exactly preserves both endpoints.  Interior points are blended with a
// bounded neighborhood; no control or execution data is produced.
std::vector<PathPoint> smooth_display_path(
    const std::vector<PathPoint> &path,
    const PathSmoothingConfig &config = {});
inline std::vector<PathPoint> smooth_path(
    const std::vector<PathPoint> &path,
    const PathSmoothingConfig &config = {}) {
  return smooth_display_path(path, config);
}

struct DiagnosticSample {
  std::string source;
  std::uint32_t reason = 0;
  std::uint32_t gate = 0;
  std::uint8_t severity = 0;
  std::uint64_t stamp_ms = 0;
  std::string message;
};

struct DiagnosticBucket {
  std::string source;
  std::uint32_t reason = 0;
  std::uint32_t gate = 0;
  std::uint64_t count = 0;
  std::uint64_t last_seen_ms = 0;
  std::uint8_t last_severity = 0;
  std::string last_message;
};

struct DiagnosticSummary {
  std::uint64_t sample_count = 0;
  std::uint64_t last_seen_ms = 0;
  std::vector<DiagnosticBucket> buckets;
};

class DiagnosticAggregator {
 public:
  DiagnosticAggregator() = default;
  void observe(const DiagnosticSample &sample);
  void clear();
  DiagnosticSummary summary() const;
  std::string to_json() const;

 private:
  std::vector<DiagnosticBucket> buckets_;
  std::uint64_t sample_count_ = 0;
  std::uint64_t last_seen_ms_ = 0;
};

DiagnosticSummary aggregate_diagnostics(
    const std::vector<DiagnosticSample> &samples);
std::string diagnostics_to_json(const DiagnosticSummary &summary);

}  // namespace acg138::rk_postprocess
