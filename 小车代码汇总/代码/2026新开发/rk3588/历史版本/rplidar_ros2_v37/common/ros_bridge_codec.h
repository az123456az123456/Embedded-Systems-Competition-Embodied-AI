#ifndef ROS_BRIDGE_CODEC_H
#define ROS_BRIDGE_CODEC_H
#include "ros_bridge_protocol.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace arb1 {
using Id = std::array<uint8_t, 16>;
enum class ErrorCode {
  None, Truncated, Magic, Version, Length, MessageType, BlockCount,
  Reserved, Alignment, Overlap, Duplicate, RequiredUnknown, Payload,
  Bound, NonFinite, Utf8, Identity
};
struct DecodeError {
  ErrorCode code = ErrorCode::None;
  size_t offset = 0;
  std::string detail;
};
struct Header {
  uint16_t major = ARB1_PROTOCOL_MAJOR, minor = ARB1_PROTOCOL_MINOR;
  uint16_t type = ARB1_MSG_SNAPSHOT;
  uint32_t flags = 0;
  uint64_t bridge_epoch = 0, message_sequence = 0, sent_mono_ns = 0;
  Id boot_id{};
};
struct Metadata {
  uint32_t flags = ARB1_BLOCK_REQUIRED;
  uint64_t sample_sequence = 0, received_mono_ns = 0;
  int64_t source_stamp_ns = 0;
  uint64_t source_age_ns = ARB1_UNKNOWN_SOURCE_AGE;
};
struct Map {
  uint32_t width = 0, height = 0;
  double resolution = 0, origin_x = 0, origin_y = 0, origin_yaw = 0;
  uint64_t map_epoch = 0, map_revision = 0;
  std::string frame_id;
  std::vector<int8_t> cells;
  bool prefer_rle = true;
  uint32_t wire_encoding = ARB1_ENCODING_RAW;
};
struct Odom {
  std::string frame_id, child_frame_id;
  double x = 0, y = 0, theta = 0, vx = 0, wz = 0;
  uint64_t localization_epoch = 0;
};
struct Tf {
  std::string frame_id, child_frame_id;
  double x = 0, y = 0, yaw = 0;
  uint64_t localization_epoch = 0;
};
struct Scan {
  std::string frame_id;
  double angle_min = 0, angle_increment = 0, range_min = 0, range_max = 0;
  std::vector<float> ranges;
};
struct Diagnostic { std::string json; };
struct Point { double x = 0, y = 0; };
struct Path {
  std::string frame_id;
  Id goal_id{};
  uint64_t map_epoch = 0, map_revision = 0, route_revision = 0;
  std::vector<Point> points;
};
struct DriveIntent {
  uint64_t generation = 0, route_revision = 0, command_sequence = 0;
  Id goal_id{};
  double linear_mps = 0, angular_rps = 0;
  uint64_t produced_mono_ns = 0;
  uint32_t valid_for_ms = 0, flags = 0;
  uint64_t localization_epoch = 0, map_epoch = 0, map_revision = 0;
};
struct Capability { uint16_t kind = 0, schema = ARB1_SCHEMA_V1; };
struct Hello {
  uint16_t major = ARB1_PROTOCOL_MAJOR, minor = ARB1_PROTOCOL_MINOR;
  uint64_t bridge_epoch = 0;
  Id boot_id{};
  std::vector<Capability> capabilities;
};
struct Error {
  uint32_t code = 0;
  uint64_t offending_sequence = 0;
  std::string detail;
};
/* Optional unknown/unsupported blocks are retained for transparent round trips.
 * A known kind with unsupported schema is treated as unknown, but still counts
 * for the duplicate-known-kind check. Required unknown blocks always fail.
 */
struct Unknown {
  uint16_t kind = 0, schema = 0;
  std::vector<uint8_t> bytes;
};
using Payload = std::variant<Hello, Error, Map, Odom, Tf, Scan, Diagnostic, Path,
                             DriveIntent, Unknown>;
struct Block { Metadata metadata; Payload payload; };
struct Message { Header header; std::vector<Block> blocks; };

uint16_t block_kind(const Payload& payload);
/* inspect needs at least 64 bytes, checks the complete fixed header and returns
 * its bounded total length. It does not inspect descriptors or payloads.
 * Every public operation leaves its output unchanged on failure.
 */
bool inspect_frame_size(const uint8_t* data, size_t size, size_t& total,
                        DecodeError* error = nullptr);
bool decode(const uint8_t* data, size_t size, Message& out,
            DecodeError* error = nullptr);
bool encode(const Message& message, std::vector<uint8_t>& out,
            DecodeError* error = nullptr);
/* A decoded valid_for_ms=0 remains representable but cannot execute. Timing,
 * generation and source freshness policy belong to the later consumer layer.
 */
bool executable(const DriveIntent& intent);
}  // namespace arb1
#endif
