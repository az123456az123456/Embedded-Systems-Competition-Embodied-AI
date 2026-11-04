#ifndef ROS_BRIDGE_PROTOCOL_H
#define ROS_BRIDGE_PROTOCOL_H

/* ARB1 v1.0. All scalar fields use little endian IEEE-754 wire encoding.
 * Packed records document offsets only; a receiver MUST NOT cast input to them.
 * Payload starts are aligned to 8 bytes. Alignment gaps are zero. Frames and
 * descriptor payload lengths include no implicit trailing padding.
 * frame_id fields are UTF-8, NUL terminated (at most 63 bytes), with zero tails.
 */
#include <stdint.h>
#include <stddef.h>

#define ARB1_PROTOCOL_MAJOR 1u
#define ARB1_PROTOCOL_MINOR 0u
#define ARB1_MAX_MESSAGE_BYTES 2097152u
#define ARB1_MAX_BLOCKS 16u
#define ARB1_MAX_MAP_CELLS 1048576u
#define ARB1_MAX_SCAN_POINTS 360u
#define ARB1_MAX_PATH_POINTS 4096u
#define ARB1_MAX_DIAGNOSTIC_BYTES 16384u
#define ARB1_MAX_ERROR_DETAIL_BYTES 256u
#define ARB1_MAX_FRAME_ID_BYTES 63u
#define ARB1_UNKNOWN_SOURCE_AGE UINT64_MAX
#define ARB1_BLOCK_REQUIRED 1u
#define ARB1_BLOCK_VALID 2u
#define ARB1_BLOCK_STATIC_TF 4u
#define ARB1_BLOCK_SOURCE_ROS_TIME 8u
#define ARB1_BLOCK_SOURCE_SYSTEM_TIME 16u
#define ARB1_BLOCK_KNOWN_FLAGS 31u
/* VALID and source clock flags apply to MAP..PATH; producers set VALID
 * explicitly. No VALID means unavailable. ROS_TIME and SYSTEM_TIME are
 * mutually exclusive; neither means an unspecified source clock. STATIC_TF
 * applies only to TF. Optional unknown blocks retain flags as opaque metadata.
 * TF identity across messages is (kind,schema,frame_id,child_frame_id); the
 * duplicate-known-kind rule still permits only one TF per snapshot.
 */
#define ARB1_E_HANDSHAKE_REQUIRED 0x00010001u
#define ARB1_E_PROTOCOL_VERSION 0x00010002u
#define ARB1_E_QUEUE_FULL 0x00040001u

enum arb1_message_type {
  ARB1_MSG_HELLO = 1, ARB1_MSG_ERROR = 2,
  ARB1_MSG_SNAPSHOT = 3, ARB1_MSG_DRIVE_INTENT = 4
};
enum arb1_block_kind {
  ARB1_KIND_HELLO = 1, ARB1_KIND_ERROR = 2, ARB1_KIND_MAP = 3,
  ARB1_KIND_ODOM = 4, ARB1_KIND_TF = 5, ARB1_KIND_SCAN = 6,
  ARB1_KIND_DIAGNOSTIC = 7, ARB1_KIND_PATH = 8, ARB1_KIND_DRIVE_INTENT = 9
};
enum arb1_encoding { ARB1_ENCODING_RAW = 0, ARB1_ENCODING_RLE = 1 };
enum arb1_data_format { ARB1_SCAN_FLOAT32 = 1, ARB1_DIAGNOSTIC_UTF8_JSON = 1 };
enum { ARB1_SCHEMA_V1 = 1 };

#pragma pack(push, 1)
typedef struct arb1_header64 {
  uint8_t magic[4];
  uint16_t major, minor;
  uint32_t total_bytes;
  uint16_t message_type, block_count;
  uint32_t flags, reserved;
  uint64_t bridge_epoch, message_sequence, sent_mono_ns;
  uint8_t boot_id[16];
} arb1_header64;
typedef struct arb1_descriptor48 {
  uint16_t kind, schema;
  uint32_t flags;
  uint64_t sample_sequence, received_mono_ns;
  int64_t source_stamp_ns;
  uint64_t source_age_ns;
  uint32_t offset, length;
} arb1_descriptor48;
typedef struct arb1_map128 {
  uint32_t width, height;
  double resolution, origin_x, origin_y, origin_yaw;
  uint64_t map_epoch, map_revision;
  uint32_t encoding, data_bytes;
  char frame_id[64];
} arb1_map128;
typedef struct arb1_odom176 {
  char frame_id[64], child_frame_id[64];
  double x, y, theta, vx, wz;
  uint64_t localization_epoch;
} arb1_odom176;
typedef struct arb1_tf160 {
  char frame_id[64], child_frame_id[64];
  double x, y, yaw;
  uint64_t localization_epoch;
} arb1_tf160;
typedef struct arb1_scan112 {
  char frame_id[64];
  uint32_t npoints, ranges_bytes;
  double angle_min, angle_increment, range_min, range_max;
  uint32_t format, reserved;
} arb1_scan112;
typedef struct arb1_diagnostic8 { uint32_t encoding, data_bytes; } arb1_diagnostic8;
typedef struct arb1_path112 {
  char frame_id[64];
  uint8_t goal_id[16];
  uint64_t map_epoch, map_revision, route_revision;
  uint32_t npoints, points_bytes;
} arb1_path112;
typedef struct arb1_drive_intent96 {
  uint64_t generation, route_revision, command_sequence;
  uint8_t goal_id[16];
  double linear_mps, angular_rps;
  uint64_t produced_mono_ns;
  uint32_t valid_for_ms, flags;
  uint64_t localization_epoch, map_epoch, map_revision;
} arb1_drive_intent96;
/* HELLO is exactly one required kind=HELLO/schema=1 block. Its fixed record
 * repeats the message identity; capability_count entries of capability4 follow.
 * Capabilities are unique (kind,schema), nonzero and at most MAX_BLOCKS.
 */
typedef struct arb1_hello32 {
  uint16_t major, minor, capability_count, reserved;
  uint64_t bridge_epoch;
  uint8_t boot_id[16];
} arb1_hello32;
typedef struct arb1_capability4 { uint16_t kind, schema; } arb1_capability4;
typedef struct arb1_error16 {
  uint32_t code, detail_bytes;
  uint64_t offending_sequence;
} arb1_error16;
#pragma pack(pop)

#ifdef __cplusplus
#define ARB1_STATIC_ASSERT(c, m) static_assert(c, m)
#else
#define ARB1_STATIC_ASSERT(c, m) _Static_assert(c, m)
#endif
ARB1_STATIC_ASSERT(sizeof(arb1_header64) == 64, "Header64");
ARB1_STATIC_ASSERT(sizeof(arb1_descriptor48) == 48, "Descriptor48");
ARB1_STATIC_ASSERT(sizeof(arb1_map128) == 128, "Map128");
ARB1_STATIC_ASSERT(sizeof(arb1_odom176) == 176, "Odom176");
ARB1_STATIC_ASSERT(sizeof(arb1_tf160) == 160, "Tf160");
ARB1_STATIC_ASSERT(sizeof(arb1_scan112) == 112, "Scan112");
ARB1_STATIC_ASSERT(sizeof(arb1_diagnostic8) == 8, "Diagnostic8");
ARB1_STATIC_ASSERT(sizeof(arb1_path112) == 112, "Path112");
ARB1_STATIC_ASSERT(sizeof(arb1_drive_intent96) == 96, "DriveIntent96");
ARB1_STATIC_ASSERT(sizeof(arb1_hello32) == 32, "Hello32");
ARB1_STATIC_ASSERT(sizeof(arb1_capability4) == 4, "Capability4");
ARB1_STATIC_ASSERT(sizeof(arb1_error16) == 16, "Error16");
ARB1_STATIC_ASSERT(offsetof(arb1_header64, bridge_epoch) == 24, "header offsets");
ARB1_STATIC_ASSERT(offsetof(arb1_descriptor48, offset) == 40, "descriptor offsets");
#undef ARB1_STATIC_ASSERT
#endif
