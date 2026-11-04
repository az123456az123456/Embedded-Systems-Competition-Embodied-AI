#include "ros_bridge_session.h"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <limits>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace arb1 {
namespace {
uint64_t mono_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000u + ts.tv_nsec;
}
bool fail(DecodeError* error, ErrorCode code, const char* detail) {
  if (error) *error = {code, 0, detail};
  return false;
}
bool has(const std::vector<Capability>& entries, uint16_t kind, uint16_t schema) {
  return std::any_of(entries.begin(), entries.end(), [=](const Capability& c) {
    return c.kind == kind && c.schema == schema;
  });
}
uint16_t schema_of(const Payload& payload) {
  if (const auto* u = std::get_if<Unknown>(&payload)) return u->schema;
  return ARB1_SCHEMA_V1;
}
}

bool FrameStream::feed(const uint8_t* bytes, size_t size,
                       std::vector<std::vector<uint8_t>>& complete,
                       DecodeError* error) {
  if (failed_) return fail(error, ErrorCode::Length, "stream already failed");
  if (size && !bytes) {
    failed_ = true;
    return fail(error, ErrorCode::Truncated, "null input");
  }
  size_t used = 0;
  while (used < size) {
    const size_t target = expected_ ? expected_ : sizeof(arb1_header64);
    const size_t count = std::min(size - used, target - pending_.size());
    pending_.insert(pending_.end(), bytes + used, bytes + used + count);
    used += count;
    if (!expected_ && pending_.size() == sizeof(arb1_header64)) {
      size_t total = 0;
      if (!inspect_frame_size(pending_.data(), pending_.size(), total, error)) {
        failed_ = true;
        return false;
      }
      expected_ = total;
    }
    if (expected_ && pending_.size() == expected_) {
      Message decoded;
      if (!decode(pending_.data(), pending_.size(), decoded, error)) {
        failed_ = true;
        return false;
      }
      complete.push_back(std::move(pending_));
      pending_.clear();
      expected_ = 0;
    }
  }
  if (error) *error = {};
  return true;
}
bool FrameStream::finish(DecodeError* error) {
  if (failed_) return fail(error, ErrorCode::Length, "stream already failed");
  if (!pending_.empty()) {
    failed_ = true;
    return fail(error, ErrorCode::Truncated, "EOF inside frame");
  }
  if (error) *error = {};
  return true;
}
void FrameStream::reset() { pending_.clear(); expected_ = 0; failed_ = false; }

ProtocolSession::ProtocolSession(SessionConfig config, Receiver receiver)
    : config_(std::move(config)), receiver_(std::move(receiver)) {}
void ProtocolSession::close(CloseReason reason, std::string detail) {
  if (state_ == SessionState::Closed) return;
  state_ = SessionState::Closed;
  close_reason_ = reason;
  close_detail_ = std::move(detail);
}
bool ProtocolSession::receive(const uint8_t* bytes, size_t size) {
  if (state_ == SessionState::Closed) return false;
  std::vector<std::vector<uint8_t>> frames;
  DecodeError framing_error;
  const bool framed = stream_.feed(bytes, size, frames, &framing_error);
  for (const auto& frame : frames) {
    Message message;
    DecodeError error;
    if (!decode(frame.data(), frame.size(), message, &error)) {
      close(CloseReason::InvalidFrame, error.detail);
      return false;
    }
    if (!accept(message)) return false;
  }
  if (!framed) {
    if (framing_error.code == ErrorCode::Version) {
      queue_error(ARB1_E_PROTOCOL_VERSION, 0, "Only ARB1 protocol 1.0 is supported");
      close(CloseReason::VersionMismatch, framing_error.detail);
    } else close(CloseReason::InvalidFrame, framing_error.detail);
  }
  return state_ != SessionState::Closed;
}
bool ProtocolSession::eof() {
  if (state_ == SessionState::Closed) return false;
  DecodeError error;
  if (!stream_.finish(&error)) close(CloseReason::TruncatedFrame, error.detail);
  else close(CloseReason::PeerEof, "peer closed");
  return false;
}
bool ProtocolSession::capability(uint16_t kind, uint16_t schema) const {
  return has(negotiated_, kind, schema);
}
bool ProtocolSession::direction_allows(uint16_t type) const {
  return std::find(config_.allowed_message_types.begin(),
                   config_.allowed_message_types.end(), type) !=
         config_.allowed_message_types.end();
}
void ProtocolSession::queue_error(uint32_t code, uint64_t offending_sequence, const char* detail) {
  if (counters_.logical_errors == 0) {
    Message response;
    response.header.type = ARB1_MSG_ERROR;
    response.header.bridge_epoch = config_.bridge_epoch;
    response.header.boot_id = config_.boot_id;
    response.header.message_sequence = 1;
    response.header.sent_mono_ns = mono_ns();
    Error error;
    error.code = code;
    error.offending_sequence = offending_sequence;
    error.detail = detail;
    response.blocks.push_back({Metadata{}, std::move(error)});
    DecodeError encoding_error;
    if (encode(response, pending_error_, &encoding_error)) ++counters_.logical_errors;
  }
}
void ProtocolSession::handshake_required(uint64_t offending_sequence) {
  queue_error(ARB1_E_HANDSHAKE_REQUIRED, offending_sequence,
               "HELLO required before any other message");
  close(CloseReason::HandshakeRequired, "HELLO required");
}
bool ProtocolSession::accept_hello(const Message& message) {
  const auto* hello = message.blocks.size() == 1
                          ? std::get_if<Hello>(&message.blocks[0].payload) : nullptr;
  if (!hello) { close(CloseReason::InvalidFrame, "HELLO payload missing"); return false; }
  if (message.header.boot_id != config_.boot_id || hello->boot_id != config_.boot_id) {
    close(CloseReason::BootMismatch, "boot ID mismatch"); return false;
  }
  if (message.header.bridge_epoch != config_.bridge_epoch ||
      hello->bridge_epoch != config_.bridge_epoch) {
    close(CloseReason::EpochMismatch, "bridge epoch mismatch"); return false;
  }
  if (hello->major != ARB1_PROTOCOL_MAJOR || hello->minor != ARB1_PROTOCOL_MINOR ||
      hello->major != message.header.major || hello->minor != message.header.minor) {
    close(CloseReason::VersionMismatch, "HELLO version mismatch"); return false;
  }
  if (!has(hello->capabilities, ARB1_KIND_HELLO, ARB1_SCHEMA_V1)) {
    close(CloseReason::CapabilityMismatch, "HELLO capability missing"); return false;
  }
  for (const Capability& required : config_.required_capabilities) {
    if (!has(hello->capabilities, required.kind, required.schema) ||
        !has(config_.capabilities, required.kind, required.schema)) {
      close(CloseReason::CapabilityMismatch, "required capability missing"); return false;
    }
  }
  for (const Capability& cap : hello->capabilities) {
    if (has(config_.capabilities, cap.kind, cap.schema)) negotiated_.push_back(cap);
  }
  last_frame_sequence_ = message.header.message_sequence;
  state_ = SessionState::Ready;
  ++counters_.accepted_messages;
  return true;
}
bool ProtocolSession::accept(const Message& message) {
  if (state_ == SessionState::AwaitHello && message.header.type != ARB1_MSG_HELLO) {
    handshake_required(message.header.message_sequence);
    return false;
  }
  if (message.header.message_sequence == 0 ||
      message.header.message_sequence <= last_frame_sequence_) {
    close(CloseReason::SequenceReplay, "frame sequence is not increasing"); return false;
  }
  if (state_ == SessionState::AwaitHello) return accept_hello(message);
  if (message.header.boot_id != config_.boot_id) {
    close(CloseReason::BootMismatch, "boot ID changed"); return false;
  }
  if (message.header.bridge_epoch != config_.bridge_epoch) {
    close(CloseReason::EpochMismatch, "bridge epoch changed"); return false;
  }
  if (message.header.major != ARB1_PROTOCOL_MAJOR || message.header.minor != ARB1_PROTOCOL_MINOR) {
    close(CloseReason::VersionMismatch, "protocol version changed"); return false;
  }
  if (message.header.type == ARB1_MSG_HELLO) {
    close(CloseReason::RepeatedHello, "HELLO cannot renew a connection"); return false;
  }
  if (message.header.type == ARB1_MSG_ERROR) {
    close(CloseReason::PeerError, "peer returned ERROR"); return false;
  }
  if (!direction_allows(message.header.type)) {
    close(CloseReason::DirectionViolation, "type forbidden by endpoint direction"); return false;
  }
  // Validate all required schemas before mutating revisions or admitting work.
  for (const Block& block : message.blocks) {
    if (!capability(block_kind(block.payload), schema_of(block.payload)) &&
        (block.metadata.flags & ARB1_BLOCK_REQUIRED)) {
      close(CloseReason::CapabilityMismatch, "unnegotiated required block"); return false;
    }
  }
  AcceptedMessage accepted;
  accepted.message.header = message.header;
  for (const Block& block : message.blocks) {
    const uint16_t kind = block_kind(block.payload), schema = schema_of(block.payload);
    if (!capability(kind, schema)) continue;
    std::string parent, child;
    if (const auto* tf = std::get_if<Tf>(&block.payload)) {
      parent = tf->frame_id; child = tf->child_frame_id;
    } else if (const auto* odom = std::get_if<Odom>(&block.payload)) {
      parent = odom->frame_id; child = odom->child_frame_id;
    }
    const auto key = std::make_tuple(kind, schema, std::move(parent), std::move(child));
    const auto it = sample_sequences_.find(key);
    const bool fresh = it == sample_sequences_.end() || block.metadata.sample_sequence > it->second;
    if (fresh) sample_sequences_[key] = block.metadata.sample_sequence;
    else ++counters_.duplicate_blocks;
    accepted.message.blocks.push_back(block);
    accepted.fresh_blocks.push_back(fresh);
  }
  last_frame_sequence_ = message.header.message_sequence;
  ++counters_.accepted_messages;
  if (receiver_ && !accepted.message.blocks.empty()) {
    ++counters_.queue_admissions;
    receiver_(accepted);
  }
  return true;
}

bool unix_root_peer(int fd, PeerCredentials* peer, std::string* detail) {
  sockaddr_un address{};
  socklen_t address_size = sizeof(address);
  if (getsockname(fd, reinterpret_cast<sockaddr*>(&address), &address_size) != 0 ||
      address.sun_family != AF_UNIX) {
    if (detail) *detail = "connected AF_UNIX socket required";
    return false;
  }
  int type = 0;
  socklen_t type_size = sizeof(type);
  if (getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &type_size) != 0 || type != SOCK_STREAM) {
    if (detail) *detail = "AF_UNIX SOCK_STREAM required";
    return false;
  }
  ucred credentials{};
  socklen_t credentials_size = sizeof(credentials);
  if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &credentials, &credentials_size) != 0 ||
      credentials_size != sizeof(credentials) || credentials.pid <= 0) {
    if (detail) *detail = "SO_PEERCRED unavailable";
    return false;
  }
  if (peer) *peer = {credentials.pid, credentials.uid, credentials.gid};
  if (credentials.uid != 0) {
    if (detail) *detail = "SO_PEERCRED UID must be 0";
    return false;
  }
  if (detail) detail->clear();
  return true;
}

ErrorSendResult send_error_before_close(int fd, const std::vector<uint8_t>& frame) {
  ErrorSendResult result;
  const uint64_t start = mono_ns();
  const uint64_t deadline = start + kErrorSendDeadlineNs;
  while (result.bytes_sent < frame.size()) {
    const uint64_t now = mono_ns();
    if (now >= deadline) { result.deadline_expired = true; break; }
    const ssize_t n = send(fd, frame.data() + result.bytes_sent,
                           frame.size() - result.bytes_sent, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (n > 0) { result.bytes_sent += static_cast<size_t>(n); continue; }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      const uint64_t after_send = mono_ns();
      const uint64_t remaining = deadline > after_send ? deadline - after_send : 0;
      if (remaining == 0) { result.deadline_expired = true; break; }
      timespec timeout{static_cast<time_t>(remaining / 1000000000u),
                       static_cast<long>(remaining % 1000000000u)};
      pollfd pfd{fd, POLLOUT, 0};
      const int rc = ppoll(&pfd, 1, &timeout, nullptr);
      if (rc < 0 && errno != EINTR) { result.system_error = errno; break; }
      continue;
    }
    result.system_error = n == 0 ? EPIPE : errno;
    break;
  }
  result.complete = result.bytes_sent == frame.size();
  result.elapsed_ns = mono_ns() - start;
  return result;
}

UnixConnection::UnixConnection(int fd, SessionConfig config, ProtocolSession::Receiver receiver)
    : fd_(fd), session_(std::move(config), std::move(receiver)) {
  std::string detail;
  if (!unix_root_peer(fd_, nullptr, &detail)) {
    session_.close(CloseReason::PeerCredentials, detail);
    settle_close();
  }
}
UnixConnection::~UnixConnection() { if (fd_ >= 0) ::close(fd_); }
void UnixConnection::settle_close() {
  if (session_.state() != SessionState::Closed || fd_ < 0) return;
  if (!error_attempted_ && !session_.pending_error().empty()) {
    error_attempted_ = true;
    error_send_ = send_error_before_close(fd_, session_.pending_error());
  }
  transport_close_reason_ = error_send_.deadline_expired ? CloseReason::ErrorSendDeadline :
      error_send_.system_error ? CloseReason::ErrorSendFailure : session_.close_reason();
  ::close(fd_);
  fd_ = -1;
}
bool UnixConnection::read_available() {
  if (fd_ < 0) return false;
  uint8_t bytes[16384];
  for (;;) {
    const ssize_t n = recv(fd_, bytes, sizeof(bytes), MSG_DONTWAIT);
    if (n > 0) {
      if (!session_.receive(bytes, static_cast<size_t>(n))) { settle_close(); return false; }
      continue;
    }
    if (n == 0) { session_.eof(); settle_close(); return false; }
    if (errno == EINTR) continue;
    if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
    session_.close(CloseReason::PeerEof, std::strerror(errno));
    settle_close();
    return false;
  }
}
const char* close_reason_name(CloseReason reason) {
  switch (reason) {
#define CASE(name) case CloseReason::name: return #name
    CASE(None); CASE(PeerCredentials); CASE(HandshakeRequired); CASE(InvalidFrame);
    CASE(TruncatedFrame); CASE(BootMismatch); CASE(EpochMismatch); CASE(VersionMismatch);
    CASE(CapabilityMismatch); CASE(SequenceReplay); CASE(RepeatedHello);
    CASE(DirectionViolation); CASE(PeerError); CASE(PeerEof); CASE(ErrorSendDeadline);
    CASE(ErrorSendFailure);
#undef CASE
  }
  return "Unknown";
}
}  // namespace arb1
