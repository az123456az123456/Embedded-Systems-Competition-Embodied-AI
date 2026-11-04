#ifndef ROS_BRIDGE_SESSION_H
#define ROS_BRIDGE_SESSION_H

#include "ros_bridge_codec.h"
#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <sys/types.h>
#include <tuple>

namespace arb1 {
constexpr uint64_t kErrorSendDeadlineNs = 100000000;

// Stream framing never exposes an incomplete or structurally invalid message.
// Completed frames preceding a malformed frame remain available to the caller.
class FrameStream {
 public:
  bool feed(const uint8_t* bytes, size_t size,
            std::vector<std::vector<uint8_t>>& complete,
            DecodeError* error = nullptr);
  bool finish(DecodeError* error = nullptr);
  void reset();
  size_t buffered_bytes() const { return pending_.size(); }
 private:
  std::vector<uint8_t> pending_;
  size_t expected_ = 0;
  bool failed_ = false;
};

enum class Channel { State, Control };
enum class PeerRole { Viewer, Gateway, Follower, Monitor };
enum class LocalEndpoint { Bridge, Client };
enum class SessionState { AwaitHello, Ready, Closed };
enum class CloseReason {
  None, PeerCredentials, HandshakeRequired, InvalidFrame, TruncatedFrame,
  BootMismatch, EpochMismatch, VersionMismatch, CapabilityMismatch,
  SequenceReplay, RepeatedHello, DirectionViolation, PeerError, PeerEof,
  ErrorSendDeadline, ErrorSendFailure
};

struct SessionConfig {
  Channel channel = Channel::State;
  PeerRole expected_peer_role = PeerRole::Viewer;
  LocalEndpoint local_endpoint = LocalEndpoint::Bridge;
  Id boot_id{};
  uint64_t bridge_epoch = 0;
  std::vector<Capability> capabilities{{ARB1_KIND_HELLO, ARB1_SCHEMA_V1},
                                      {ARB1_KIND_ERROR, ARB1_SCHEMA_V1}};
  // Inbound direction and mandatory schemas are trusted endpoint policy.
  // Neither the claimed role nor the HELLO capability list grants a type.
  std::vector<uint16_t> allowed_message_types;
  std::vector<Capability> required_capabilities{{ARB1_KIND_HELLO, ARB1_SCHEMA_V1}};
};

struct SessionCounters {
  uint64_t accepted_messages = 0;
  uint64_t queue_admissions = 0;
  // This protocol layer has no claim operation. A later Gateway must count
  // its actual authority claims independently at the admission point.
  uint64_t claim_admissions = 0;
  uint64_t logical_errors = 0;
  uint64_t duplicate_blocks = 0;
};

struct AcceptedMessage {
  Message message;
  // A repeated/older sample sequence is delivered as not fresh, never renewed.
  std::vector<bool> fresh_blocks;
};

// Pure protocol state. Socket consumers must use UnixConnection, whose input
// path verifies actual SO_PEERCRED before handing bytes to this engine.
class ProtocolSession {
 public:
  using Receiver = std::function<void(const AcceptedMessage&)>;
  explicit ProtocolSession(SessionConfig config, Receiver receiver = {});
  bool receive(const uint8_t* bytes, size_t size);
  bool eof();
  void close(CloseReason reason, std::string detail = {});
  SessionState state() const { return state_; }
  bool ready() const { return state_ == SessionState::Ready; }
  bool has_motion_authority() const { return false; }
  CloseReason close_reason() const { return close_reason_; }
  const std::string& close_detail() const { return close_detail_; }
  const SessionCounters& counters() const { return counters_; }
  const std::vector<Capability>& negotiated_capabilities() const { return negotiated_; }
  const std::vector<uint8_t>& pending_error() const { return pending_error_; }
 private:
  bool accept(const Message& message);
  bool accept_hello(const Message& message);
  bool capability(uint16_t kind, uint16_t schema) const;
  bool direction_allows(uint16_t type) const;
  void handshake_required(uint64_t offending_sequence);
  void queue_error(uint32_t code, uint64_t offending_sequence, const char* detail);
  SessionConfig config_;
  Receiver receiver_;
  FrameStream stream_;
  SessionState state_ = SessionState::AwaitHello;
  CloseReason close_reason_ = CloseReason::None;
  std::string close_detail_;
  SessionCounters counters_;
  std::vector<Capability> negotiated_;
  uint64_t last_frame_sequence_ = 0;
  using RevisionKey = std::tuple<uint16_t, uint16_t, std::string, std::string>;
  std::map<RevisionKey, uint64_t> sample_sequences_;
  std::vector<uint8_t> pending_error_;
};

struct PeerCredentials { pid_t pid = -1; uid_t uid = static_cast<uid_t>(-1);
                         gid_t gid = static_cast<gid_t>(-1); };
// Reads the connected AF_UNIX socket itself. No identity supplied in HELLO or
// by the caller can satisfy this check; the only accepted effective UID is 0.
bool unix_root_peer(int fd, PeerCredentials* peer = nullptr,
                    std::string* detail = nullptr);

struct ErrorSendResult {
  size_t bytes_sent = 0;
  bool complete = false;
  bool deadline_expired = false;
  int system_error = 0;
  uint64_t elapsed_ns = 0;
};
// Uses MSG_DONTWAIT/MSG_NOSIGNAL and an absolute CLOCK_MONOTONIC deadline.
// At most one logical frame is supplied. Partial writes resume at their exact
// offset. Delivery cannot be promised when the peer does not read.
ErrorSendResult send_error_before_close(int fd, const std::vector<uint8_t>& frame);

// Owns one already connected AF_UNIX socket and its independent handshake.
// Construction verifies root SO_PEERCRED. Destruction closes the socket.
class UnixConnection {
 public:
  UnixConnection(int fd, SessionConfig config, ProtocolSession::Receiver receiver = {});
  ~UnixConnection();
  UnixConnection(const UnixConnection&) = delete;
  UnixConnection& operator=(const UnixConnection&) = delete;
  bool read_available();
  const ProtocolSession& session() const { return session_; }
  int fd() const { return fd_; }
  const ErrorSendResult& error_send_result() const { return error_send_; }
  CloseReason transport_close_reason() const { return transport_close_reason_; }
 private:
  void settle_close();
  int fd_ = -1;
  ProtocolSession session_;
  ErrorSendResult error_send_;
  CloseReason transport_close_reason_ = CloseReason::None;
  bool error_attempted_ = false;
};

const char* close_reason_name(CloseReason reason);
}  // namespace arb1
#endif
