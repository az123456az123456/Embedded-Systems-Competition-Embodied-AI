#include "acg138_rk_postprocess/postprocess.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace acg138::rk_postprocess;

namespace {
unsigned checks = 0;

#define CHECK(condition)                                                        \
  do {                                                                          \
    ++checks;                                                                   \
    if (!(condition))                                                           \
      throw std::runtime_error(std::string("check failed line ") +             \
                               std::to_string(__LINE__) + ": " #condition);    \
  } while (false)

template <typename Function>
void rejects(Function function) {
  bool rejected = false;
  try {
    function();
  } catch (const std::exception &) {
    rejected = true;
  }
  CHECK(rejected);
}

bool same_state(const FpgaState &a, const FpgaState &b) {
  return a.state == b.state && a.avoid == b.avoid && a.reason == b.reason &&
         a.state_flags == b.state_flags && a.nav_epoch == b.nav_epoch &&
         a.state_seq == b.state_seq && a.pose_seq == b.pose_seq && a.x_mm == b.x_mm &&
         a.y_mm == b.y_mm && a.yaw_urad == b.yaw_urad && a.pose_age_ms == b.pose_age_ms &&
         a.cursor == b.cursor && a.path_count == b.path_count &&
         a.remaining_mm == b.remaining_mm && a.permit_remaining_ms == b.permit_remaining_ms &&
         a.auth_remaining_ms == b.auth_remaining_ms && a.active_crc32 == b.active_crc32 &&
         a.pending_crc32 == b.pending_crc32 && a.upload_crc32 == b.upload_crc32 &&
         a.gate_reasons == b.gate_reasons && a.pending_map_epoch == b.pending_map_epoch &&
         a.pending_map_revision == b.pending_map_revision &&
         a.upload_map_epoch == b.upload_map_epoch &&
         a.upload_map_revision == b.upload_map_revision && a.last_ack_seq == b.last_ack_seq &&
         a.device_ms == b.device_ms && a.owner == b.owner && a.upload_chunks == b.upload_chunks &&
         a.reserved == b.reserved && a.diagnostic == b.diagnostic &&
         a.last_request_id == b.last_request_id && a.active_frame_token == b.active_frame_token &&
         a.pending_frame_token == b.pending_frame_token && a.reserved_tail == b.reserved_tail;
}

bool same_result(const FpgaPlanResult &a, const FpgaPlanResult &b) {
  return a.status == b.status && a.reason == b.reason && a.device_ms == b.device_ms &&
         a.nav_epoch == b.nav_epoch && a.state_seq == b.state_seq &&
         a.start_pose_seq == b.start_pose_seq && a.expanded == b.expanded &&
         a.cycles == b.cycles && a.path_length == b.path_length &&
         a.start_cell == b.start_cell && a.goal_cell == b.goal_cell &&
         a.result_flags == b.result_flags && a.path_revision == b.path_revision &&
         a.frame_token == b.frame_token && a.cpu_hz == b.cpu_hz &&
         a.reserved_tail == b.reserved_tail;
}

void test_json_round_trip() {
  FpgaState state;
  state.state = 3;
  state.avoid = 2;
  state.reason = 0x1234;
  state.state_flags = 0xa5;
  state.nav_epoch = 1;
  state.state_seq = 0x1020304050607080ULL;
  state.pose_seq = 99;
  state.x_mm = -1234;
  state.y_mm = 5678;
  state.yaw_urad = -3141592;
  state.pose_age_ms = 11;
  state.cursor = 12;
  state.path_count = 64;
  state.remaining_mm = 1000;
  state.permit_remaining_ms = 499;
  state.auth_remaining_ms = 29999;
  state.active_crc32 = 0xdeadbeef;
  state.pending_crc32 = 0x12345678;
  state.upload_crc32 = 0x87654321;
  state.gate_reasons = 0x80000001;
  state.pending_map_epoch = 2;
  state.pending_map_revision = 3;
  state.upload_map_epoch = 4;
  state.upload_map_revision = 5;
  state.last_ack_seq = 0xabcdef0123456789ULL;
  state.device_ms = 100;
  state.owner = 1;
  state.upload_chunks = 16;
  state.reserved = 0;
  state.diagnostic = 0x42;
  state.last_request_id = 77;
  state.active_frame_token = 88;
  state.pending_frame_token = 89;
  state.reserved_tail = 0;
  const std::string json = state_to_json(state);
  CHECK(json == gui_state_json(state));
  CHECK(json == phone_state_json(state));
  CHECK(json.find("\"state_name\":\"AVOIDING\"") != std::string::npos);
  CHECK(json.find("\"motion_allowed\":true") != std::string::npos);
  FpgaState decoded;
  std::string error;
  CHECK(parse_state_json(json + "\n", decoded, &error));
  CHECK(error.empty());
  CHECK(same_state(state, decoded));
  CHECK(parse_state_json("{\"state\":1,\"state_flags\":0,\"extra\":{\"ok\":[1,true]}}", decoded));
  CHECK(decoded.state == 1 && decoded.state_flags == 0);
  CHECK(!parse_state_json("{\"state\":256,\"state_flags\":0}", decoded, &error));
  CHECK(!parse_state_json("{\"state\":1,\"state_flags\":256}", decoded, &error));
  CHECK(!parse_state_json("{\"state\":1,\"state\":2}", decoded, &error));
  CHECK(!parse_state_json("{\"state\":1}", decoded, &error));
  CHECK(!parse_state_json("[]", decoded, &error));

  FpgaPlanResult result;
  result.status = 0;
  result.reason = 9;
  result.device_ms = 444;
  result.nav_epoch = 1;
  result.state_seq = 10;
  result.start_pose_seq = 11;
  result.expanded = 123;
  result.cycles = 456;
  result.path_length = 17;
  result.start_cell = 65;
  result.goal_cell = 4030;
  result.result_flags = 1;
  result.path_revision = 8;
  result.frame_token = 9;
  result.cpu_hz = 100000000;
  result.reserved_tail = 0;
  const std::string plan_json = plan_result_to_json(result);
  CHECK(plan_json == gui_plan_result_json(result));
  FpgaPlanResult result_decoded;
  CHECK(parse_plan_result_json(plan_json, result_decoded));
  CHECK(same_result(result, result_decoded));
  CHECK(!parse_plan_result_json("{\"status\":1.5}", result_decoded, &error));
  CHECK(!parse_plan_result_json("{\"status\":0,\"result_flags\":65536}", result_decoded, &error));
}

void test_smoothing() {
  const std::vector<PathPoint> source{{0, 0, 0}, {1, 8, 0}, {2, 0, 0}, {3, 8, 0}, {4, 0, 0}};
  const auto smoothed = smooth_path(source);
  CHECK(smoothed.size() == source.size());
  CHECK(smoothed.front().x == source.front().x && smoothed.front().y == source.front().y);
  CHECK(smoothed.back().x == source.back().x && smoothed.back().y == source.back().y);
  CHECK(smoothed[1].y < source[1].y && smoothed[1].y > 0);
  for (std::size_t i = 0; i < smoothed.size(); ++i) {
    CHECK(std::isfinite(smoothed[i].x) && std::isfinite(smoothed[i].y));
    CHECK(std::hypot(smoothed[i].x - source[i].x, smoothed[i].y - source[i].y) <= .5 + 1e-12);
  }
  PathSmoothingConfig no_change;
  no_change.alpha = 0;
  const auto unchanged = smooth_path(source, no_change);
  CHECK(unchanged.size() == source.size());
  for (std::size_t i = 0; i < source.size(); ++i) {
    CHECK(unchanged[i].x == source[i].x && unchanged[i].y == source[i].y &&
          unchanged[i].z == source[i].z);
  }
  rejects([&] { smooth_path({{0, 0, 0}, {std::numeric_limits<double>::quiet_NaN(), 0, 0}}); });
  rejects([&] {
    PathSmoothingConfig bad;
    bad.alpha = 2;
    smooth_path(source, bad);
  });
}

void test_diagnostics() {
  DiagnosticAggregator aggregator;
  aggregator.observe({"fpga", 2, 4, 1, 200, "late"});
  aggregator.observe({"base", 1, 0, 2, 300, "encoder"});
  aggregator.observe({"fpga", 2, 4, 3, 250, "newer"});
  aggregator.observe({"fpga", 2, 4, 0, 150, "older"});
  const DiagnosticSummary summary = aggregator.summary();
  CHECK(summary.sample_count == 4 && summary.last_seen_ms == 300);
  CHECK(summary.buckets.size() == 2);
  CHECK(summary.buckets[0].source == "base");
  CHECK(summary.buckets[1].count == 3 && summary.buckets[1].last_seen_ms == 250);
  CHECK(summary.buckets[1].last_severity == 3 && summary.buckets[1].last_message == "newer");
  const std::string json = aggregator.to_json();
  CHECK(json.find("diagnostics_summary") != std::string::npos);
  CHECK(json.find("\"last_message\":\"newer\"") != std::string::npos);
  CHECK(aggregate_diagnostics({}).buckets.empty());
  rejects([&] { aggregator.observe({std::string(257, 'x'), 0, 0, 0, 0, ""}); });
  aggregator.clear();
  CHECK(aggregator.summary().sample_count == 0 && aggregator.summary().buckets.empty());
}

}  // namespace

int main() {
  try {
    test_json_round_trip();
    test_smoothing();
    test_diagnostics();
    std::cout << "PASS rk_postprocess checks=" << checks
              << " movement_messages=0 control_publishers=0\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
