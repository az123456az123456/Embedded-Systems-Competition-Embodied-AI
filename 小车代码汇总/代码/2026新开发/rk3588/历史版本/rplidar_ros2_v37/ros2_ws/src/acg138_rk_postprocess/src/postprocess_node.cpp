#include "acg138_rk_postprocess/postprocess.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace acg138::rk_postprocess {
namespace {

// The raw status topics are std_msgs/String in this workspace. Keep malformed
// input observable without pretending that it is a valid FPGA event.
std::string json_escape(const std::string &value) {
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (const unsigned char c : value) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) out << "\\u" << std::setw(4) << static_cast<unsigned>(c);
        else out << static_cast<char>(c);
    }
  }
  return out.str();
}

std::string invalid_json(const std::string &kind, const std::string &raw,
                         const std::string &error) {
  return "{\"valid\":false,\"kind\":\"" + json_escape(kind) +
         "\",\"error\":\"" + json_escape(error) +
         "\",\"raw\":\"" + json_escape(raw) + "\"}";
}

bool parse_uint_field(const std::string &json, const std::string &key,
                      std::uint64_t &value) {
  const std::string marker = "\"" + key + "\"";
  const auto at = json.find(marker);
  if (at == std::string::npos) return false;
  auto pos = json.find(':', at + marker.size());
  if (pos == std::string::npos) return false;
  ++pos;
  while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) ++pos;
  if (pos == json.size() || json[pos] == '-') return false;
  std::uint64_t parsed = 0;
  bool any = false;
  while (pos < json.size() && json[pos] >= '0' && json[pos] <= '9') {
    any = true;
    const auto digit = static_cast<std::uint64_t>(json[pos] - '0');
    if (parsed > (std::numeric_limits<std::uint64_t>::max() - digit) / 10u)
      return false;
    parsed = parsed * 10u + digit;
    ++pos;
  }
  if (!any) return false;
  value = parsed;
  return true;
}

std::string diagnostics_sample(const std::string &raw, std::uint64_t now_ms,
                               DiagnosticSample &sample) {
  sample.source = "base";
  sample.message = raw;
  sample.stamp_ms = now_ms;
  std::uint64_t value = 0;
  if (parse_uint_field(raw, "reason", value) &&
      value <= std::numeric_limits<std::uint32_t>::max()) {
    sample.reason = static_cast<std::uint32_t>(value);
  }
  if (parse_uint_field(raw, "gate", value) &&
      value <= std::numeric_limits<std::uint32_t>::max()) {
    sample.gate = static_cast<std::uint32_t>(value);
  }
  if (parse_uint_field(raw, "severity", value) &&
      value <= std::numeric_limits<std::uint8_t>::max()) {
    sample.severity = static_cast<std::uint8_t>(value);
  }
  if (parse_uint_field(raw, "stamp_ms", value)) sample.stamp_ms = value;
  if (raw.find("\"source\":\"") != std::string::npos) {
    const auto begin = raw.find("\"source\":\"") + 10;
    const auto end = raw.find('"', begin);
    if (end != std::string::npos) sample.source = raw.substr(begin, end - begin);
  }
  if (raw.find("\"message\":\"") != std::string::npos) {
    const auto begin = raw.find("\"message\":\"") + 11;
    const auto end = raw.find('"', begin);
    if (end != std::string::npos) sample.message = raw.substr(begin, end - begin);
  }
  return raw;
}

}  // namespace

class PostprocessNode final : public rclcpp::Node {
 public:
  PostprocessNode() : Node("acg138_rk_postprocess") {
    const auto state_topic = declare_parameter<std::string>("state_topic", "/fpga/state");
    const auto plan_topic = declare_parameter<std::string>("plan_result_topic", "/fpga/plan_result");
    const auto raw_path_topic = declare_parameter<std::string>("raw_path_topic", "/fpga/planned_path");
    const auto diagnostics_topic = declare_parameter<std::string>("diagnostics_topic", "/base/diagnostics");
    const auto qos_depth = declare_parameter<int>("qos_depth", 10);
    if (qos_depth < 1 || qos_depth > 1000) throw std::runtime_error("qos_depth must be 1..1000");
    const auto qos = rclcpp::QoS(static_cast<std::size_t>(qos_depth)).reliable();
    const auto latched = rclcpp::QoS(1).reliable().transient_local();

    gui_state_pub_ = create_publisher<std_msgs::msg::String>("/fpga/gui_state", latched);
    phone_state_pub_ = create_publisher<std_msgs::msg::String>("/fpga/phone_state", latched);
    gui_plan_pub_ = create_publisher<std_msgs::msg::String>("/fpga/gui_plan_result", latched);
    phone_plan_pub_ = create_publisher<std_msgs::msg::String>("/fpga/phone_plan_result", latched);
    display_path_pub_ = create_publisher<nav_msgs::msg::Path>("/fpga/display_path", latched);
    diagnostics_pub_ = create_publisher<std_msgs::msg::String>("/fpga/diagnostics_summary", latched);

    state_sub_ = create_subscription<std_msgs::msg::String>(
        state_topic, qos, [this](std_msgs::msg::String::ConstSharedPtr message) { handle_state(*message); });
    plan_sub_ = create_subscription<std_msgs::msg::String>(
        plan_topic, qos, [this](std_msgs::msg::String::ConstSharedPtr message) { handle_plan(*message); });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        raw_path_topic, qos, [this](nav_msgs::msg::Path::ConstSharedPtr message) { handle_path(*message); });
    diagnostics_sub_ = create_subscription<std_msgs::msg::String>(
        diagnostics_topic, qos, [this](std_msgs::msg::String::ConstSharedPtr message) {
          handle_diagnostics(*message);
        });
    RCLCPP_INFO(get_logger(),
                "Read-only FPGA postprocessing: state/plan JSON, display path, diagnostics summary");
  }

 private:
  using StringPublisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;

  void handle_state(const std_msgs::msg::String &message) {
    FpgaState state;
    std::string error;
    if (!parse_state_json(message.data, state, &error)) {
      publish_json(gui_state_pub_, invalid_json("state", message.data, error));
      publish_json(phone_state_pub_, invalid_json("state", message.data, error));
      return;
    }
    publish_json(gui_state_pub_, gui_state_json(state));
    publish_json(phone_state_pub_, phone_state_json(state));
  }

  void handle_plan(const std_msgs::msg::String &message) {
    FpgaPlanResult result;
    std::string error;
    if (!parse_plan_result_json(message.data, result, &error)) {
      publish_json(gui_plan_pub_, invalid_json("plan_result", message.data, error));
      publish_json(phone_plan_pub_, invalid_json("plan_result", message.data, error));
      clear_display_path();
      return;
    }
    publish_json(gui_plan_pub_, gui_plan_result_json(result));
    publish_json(phone_plan_pub_, plan_result_to_json(result));
    if ((result.result_flags & 1u) == 0u) clear_display_path();
  }

  void handle_path(const nav_msgs::msg::Path &message) {
    std::vector<PathPoint> input;
    input.reserve(message.poses.size());
    for (const auto &pose : message.poses) {
      input.push_back({pose.pose.position.x, pose.pose.position.y, pose.pose.position.z});
    }
    std::vector<PathPoint> smoothed;
    try {
      smoothed = smooth_display_path(input, path_config_);
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "discarding invalid display path: %s", error.what());
      // Publish an empty display path so a previously latched route cannot
      // remain visible after a malformed raw sample.
      nav_msgs::msg::Path empty;
      empty.header = message.header;
      display_path_pub_->publish(std::move(empty));
      return;
    }
    nav_msgs::msg::Path output;
    output.header = message.header;
    output.poses.reserve(smoothed.size());
    for (std::size_t i = 0; i < smoothed.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.poses[i].header;
      if (pose.header.frame_id.empty()) pose.header = output.header;
      pose.pose.position.x = smoothed[i].x;
      pose.pose.position.y = smoothed[i].y;
      pose.pose.position.z = smoothed[i].z;
      pose.pose.orientation.w = 1.0;
      output.poses.push_back(std::move(pose));
    }
    display_path_pub_->publish(std::move(output));
  }

  void clear_display_path() {
    nav_msgs::msg::Path empty;
    display_path_pub_->publish(std::move(empty));
  }

  void handle_diagnostics(const std_msgs::msg::String &message) {
    const auto now_ms = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    DiagnosticSample sample;
    diagnostics_sample(message.data, now_ms, sample);
    try {
      diagnostics_.observe(sample);
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "discarding invalid diagnostic sample: %s", error.what());
      return;
    }
    publish_json(diagnostics_pub_, diagnostics_.to_json());
  }

  static void publish_json(const StringPublisher &publisher, const std::string &value) {
    std_msgs::msg::String message;
    message.data = value;
    publisher->publish(std::move(message));
  }

  PathSmoothingConfig path_config_;
  DiagnosticAggregator diagnostics_;
  StringPublisher gui_state_pub_, phone_state_pub_, gui_plan_pub_, phone_plan_pub_, diagnostics_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr display_path_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_, plan_sub_, diagnostics_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

}  // namespace acg138::rk_postprocess

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<acg138::rk_postprocess::PostprocessNode>());
  } catch (const std::exception &error) {
    std::fprintf(stderr, "acg138_rk_postprocess: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
