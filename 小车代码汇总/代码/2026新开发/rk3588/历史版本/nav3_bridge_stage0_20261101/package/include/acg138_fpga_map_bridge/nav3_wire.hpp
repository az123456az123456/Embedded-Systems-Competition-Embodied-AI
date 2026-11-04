#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

// NAV3/v1 only. No socket, motor enable, NVA1 PERMIT, or client retry policy.
// Canonical oracle: candidate26 common/nav_exec_wire.{h,c} (tests only).
namespace acg138::nav3 {
inline constexpr std::size_t kBodyBytes=256, kWireBytes=258;
inline constexpr std::size_t kHeaderBytes=96, kPayloadBytes=160, kAckBytes=32;
inline constexpr std::size_t kMapBytes=512, kMapChunks=16, kMapChunkBytes=32;
inline constexpr std::size_t kPageCells=64, kEthernetFrameBytes=272;
inline constexpr std::uint8_t kVersion=1;
inline constexpr std::uint16_t kEtherType=0x88b9;
inline constexpr std::array<std::uint8_t,6> kFpgaMac{{0x02,0xac,0x13,0x80,0x00,0x01}};
enum class Opcode : std::uint8_t {
  Info=0x01, FrameBind=0x02, SessionBind=0x03, ClockAnchor=0x04,
  MapBegin=0x10, MapChunk=0x11, MapCommit=0x12, GoalPlan=0x20,
  Start=0x21, Cancel=0x22, Pose=0x30, State=0x40, PathPage=0x41,
  Event=0x42, PlanResult=0x43
};
enum class Status : std::uint16_t {
  Ok=0, BadRequest=1, Unbound=2, Session=3, Stale=4, Identity=5,
  NoMap=6, Incomplete=7, MapCrc=8, NoRoute=9, Busy=10,
  PoseStale=11, NoPermit=12, GoalOutsideWindow=13, Rebind=14,
  Cancelled=15, Internal=16
};
enum class Offset : std::size_t {
  Magic=0, Version=4, Opcode=5, Flags=6, Transport=8, Session=16, Run=24,
  Goal=32, MapEpoch=48, MapRevision=56, Localization=64, Plan=72,
  Request=80, PayloadBytes=88, Reserved=90, Payload=96, Crc16=256
};
enum class Error {
  Ok=0, Length=1, Magic=2, Version=3, Crc=4, Flags=5, Reserved=6,
  Opcode=7, Shape=8, Range=9, Pointer=10
};
using Body=std::array<std::uint8_t,kBodyBytes>;
using Wire=std::array<std::uint8_t,kWireBytes>;
using Payload=std::array<std::uint8_t,kPayloadBytes>;
struct Header {
  std::uint8_t opcode=static_cast<std::uint8_t>(Opcode::Info); // ACK bit retained.
  std::uint16_t flags=0;
  std::uint64_t transport_seq=0, session_id=0, run_generation=0;
  std::array<std::uint8_t,16> goal_id{};
  std::uint64_t map_epoch=0, map_revision=0, localization_epoch=0;
  std::uint64_t plan_id=0, request_id=0;
  std::uint16_t payload_bytes=0;
};
struct Message { Header header{}; Payload payload{}; }; // Owns bytes; no dangling view.
struct AckPrefix {
  Status status=Status::Ok;
  std::uint32_t device_ms=0;
  std::uint64_t nav_epoch=0, state_seq=0;
  std::uint32_t value=0; // PATH_PAGE: high 16 bits offset, low 16 bits count.
};
struct InfoExtra {
  std::uint32_t capabilities=0, cpu_hz=0;
  std::uint64_t info_cookie=0, current_session=0;
  std::uint32_t active_map_crc32=0;
  bool ready=false;
};
constexpr std::uint8_t request_opcode(Opcode op) noexcept {
  return static_cast<std::uint8_t>(op);
}
constexpr std::uint8_t reply_opcode(Opcode op) noexcept {
  return static_cast<std::uint8_t>(request_opcode(op)|0x80u);
}
// Pointer must address size bytes; nullptr is permitted only when size==0.
std::uint16_t crc16_ccitt_false(const std::uint8_t* bytes, std::size_t size) noexcept;
// On any error, output is unchanged. Reserved payload tail is zeroed on encode.
Error encode(const Header&, const Payload&, Wire& output) noexcept;
Error decode(const std::uint8_t* wire, std::size_t size, Message& output) noexcept;
inline Error decode(const Wire& wire, Message& output) noexcept {
  return decode(wire.data(),wire.size(),output);
}
// Validate opcode/length before extracting: async STATE/PLAN_RESULT are not ACKs.
Error parse_ack_prefix(const Message&, AckPrefix& output) noexcept;
Error parse_info_extra(const Message&, InfoExtra& output) noexcept;
const char* error_name(Error) noexcept;
} // namespace acg138::nav3
