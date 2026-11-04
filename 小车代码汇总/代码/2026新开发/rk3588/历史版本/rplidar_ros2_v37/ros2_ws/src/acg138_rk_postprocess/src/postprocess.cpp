#include "acg138_rk_postprocess/postprocess.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <map>
#include <stdexcept>

namespace acg138::rk_postprocess {
namespace {

constexpr std::uint32_t kKnownStateFlags = 0xffu;

std::string escape_json(const std::string &value) {
  static constexpr char hex[] = "0123456789abcdef";
  std::string result;
  result.reserve(value.size() + 8);
  for (const unsigned char c : value) {
    switch (c) {
      case '"': result += "\\\""; break;
      case '\\': result += "\\\\"; break;
      case '\b': result += "\\b"; break;
      case '\f': result += "\\f"; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      default:
        if (c < 0x20u) {
          result += "\\u00";
          result += hex[c >> 4];
          result += hex[c & 0x0fu];
        } else {
          result += static_cast<char>(c);
        }
    }
  }
  return result;
}

template <typename T>
void add_uint(std::string &json, const char *name, T value, bool &first) {
  if (!first) json += ',';
  first = false;
  json += '"';
  json += name;
  json += "\":";
  json += std::to_string(static_cast<std::uint64_t>(value));
}

template <typename T>
void add_int(std::string &json, const char *name, T value, bool &first) {
  if (!first) json += ',';
  first = false;
  json += '"';
  json += name;
  json += "\":";
  json += std::to_string(static_cast<std::int64_t>(value));
}

void add_bool(std::string &json, const char *name, bool value, bool &first) {
  if (!first) json += ',';
  first = false;
  json += '"';
  json += name;
  json += value ? "\":true" : "\":false";
}

void add_string(std::string &json, const char *name, const std::string &value,
                bool &first) {
  if (!first) json += ',';
  first = false;
  json += '"';
  json += name;
  json += "\":\"";
  json += escape_json(value);
  json += '"';
}

const char *state_name(std::uint8_t state) {
  switch (state) {
    case 0: return "IDLE";
    case 1: return "PLANNING";
    case 2: return "FOLLOWING";
    case 3: return "AVOIDING";
    case 4: return "ARRIVED";
    case 5: return "CANCELLED";
    default: return "UNKNOWN";
  }
}

std::string make_state_json(const FpgaState &state) {
  std::string json = "{";
  bool first = true;
  add_string(json, "kind", "fpga_state", first);
  add_uint(json, "schema_version", 1u, first);
  add_uint(json, "state", state.state, first);
  add_uint(json, "avoid", state.avoid, first);
  add_uint(json, "reason", state.reason, first);
  add_uint(json, "state_flags", state.state_flags, first);
  add_uint(json, "nav_epoch", state.nav_epoch, first);
  add_uint(json, "state_seq", state.state_seq, first);
  add_uint(json, "pose_seq", state.pose_seq, first);
  add_int(json, "x_mm", state.x_mm, first);
  add_int(json, "y_mm", state.y_mm, first);
  add_int(json, "yaw_urad", state.yaw_urad, first);
  add_uint(json, "pose_age_ms", state.pose_age_ms, first);
  add_uint(json, "cursor", state.cursor, first);
  add_uint(json, "path_count", state.path_count, first);
  add_uint(json, "remaining_mm", state.remaining_mm, first);
  add_uint(json, "permit_remaining_ms", state.permit_remaining_ms, first);
  add_uint(json, "auth_remaining_ms", state.auth_remaining_ms, first);
  add_uint(json, "active_crc32", state.active_crc32, first);
  add_uint(json, "pending_crc32", state.pending_crc32, first);
  add_uint(json, "upload_crc32", state.upload_crc32, first);
  add_uint(json, "gate_reasons", state.gate_reasons, first);
  add_uint(json, "pending_map_epoch", state.pending_map_epoch, first);
  add_uint(json, "pending_map_revision", state.pending_map_revision, first);
  add_uint(json, "upload_map_epoch", state.upload_map_epoch, first);
  add_uint(json, "upload_map_revision", state.upload_map_revision, first);
  add_uint(json, "last_ack_seq", state.last_ack_seq, first);
  add_uint(json, "device_ms", state.device_ms, first);
  add_uint(json, "owner", state.owner, first);
  add_uint(json, "upload_chunks", state.upload_chunks, first);
  add_uint(json, "reserved", state.reserved, first);
  add_uint(json, "diagnostic", state.diagnostic, first);
  add_uint(json, "last_request_id", state.last_request_id, first);
  add_uint(json, "active_frame_token", state.active_frame_token, first);
  add_uint(json, "pending_frame_token", state.pending_frame_token, first);
  add_uint(json, "reserved_tail", state.reserved_tail, first);
  add_string(json, "state_name", state_name(state.state), first);
  add_bool(json, "motion_allowed", (state.state_flags & (1u << 0)) != 0, first);
  add_bool(json, "plan_ready", (state.state_flags & (1u << 1)) != 0, first);
  add_bool(json, "arrived", (state.state_flags & (1u << 2)) != 0, first);
  add_bool(json, "no_path", (state.state_flags & (1u << 3)) != 0, first);
  add_bool(json, "rebind_required", (state.state_flags & (1u << 4)) != 0, first);
  add_bool(json, "pose_valid", (state.state_flags & (1u << 5)) != 0, first);
  add_bool(json, "permit_valid", (state.state_flags & (1u << 6)) != 0, first);
  add_bool(json, "auth_valid", (state.state_flags & (1u << 7)) != 0, first);
  json += '}';
  return json;
}

std::string make_plan_json(const FpgaPlanResult &result) {
  std::string json = "{";
  bool first = true;
  add_string(json, "kind", "fpga_plan_result", first);
  add_uint(json, "schema_version", 1u, first);
  add_uint(json, "status", result.status, first);
  add_uint(json, "reason", result.reason, first);
  add_uint(json, "device_ms", result.device_ms, first);
  add_uint(json, "nav_epoch", result.nav_epoch, first);
  add_uint(json, "state_seq", result.state_seq, first);
  add_uint(json, "start_pose_seq", result.start_pose_seq, first);
  add_uint(json, "expanded", result.expanded, first);
  add_uint(json, "cycles", result.cycles, first);
  add_uint(json, "path_length", result.path_length, first);
  add_uint(json, "start_cell", result.start_cell, first);
  add_uint(json, "goal_cell", result.goal_cell, first);
  add_uint(json, "result_flags", result.result_flags, first);
  add_uint(json, "path_revision", result.path_revision, first);
  add_uint(json, "frame_token", result.frame_token, first);
  add_uint(json, "cpu_hz", result.cpu_hz, first);
  add_uint(json, "reserved_tail", result.reserved_tail, first);
  add_bool(json, "valid", (result.result_flags & 1u) != 0, first);
  json += '}';
  return json;
}

// A small JSON reader is sufficient here because the wire adapter only needs
// flat integer records.  It still consumes nested/unknown JSON values so
// forward-compatible producers do not desynchronize parsing.
struct Value {
  enum class Kind { Number, String, Bool, Null, Compound } kind = Kind::Null;
  std::string text;
  bool boolean = false;
};

class JsonParser {
 public:
  explicit JsonParser(const std::string &text) : text_(text) {}

  bool object(std::map<std::string, Value> &values) {
    skip_space();
    if (!take('{')) return fail("expected object");
    skip_space();
    if (take('}')) return true;
    while (true) {
      std::string key;
      if (!string(key)) return false;
      skip_space();
      if (!take(':')) return fail("expected ':'");
      skip_space();
      Value value;
      if (!value_token(value)) return false;
      if (values.find(key) != values.end()) return fail("duplicate key");
      values.emplace(std::move(key), std::move(value));
      skip_space();
      if (take('}')) return true;
      if (!take(',')) return fail("expected ','");
      skip_space();
    }
  }

  bool finished() {
    skip_space();
    return pos_ == text_.size();
  }

  const std::string &error() const { return error_; }

 private:
  bool fail(const char *message) {
    if (error_.empty()) error_ = message;
    return false;
  }

  void skip_space() {
    while (pos_ < text_.size() &&
           (text_[pos_] == ' ' || text_[pos_] == '\n' || text_[pos_] == '\r' ||
            text_[pos_] == '\t')) {
      ++pos_;
    }
  }

  bool take(char expected) {
    if (pos_ < text_.size() && text_[pos_] == expected) {
      ++pos_;
      return true;
    }
    return false;
  }

  bool string(std::string &result) {
    if (!take('"')) return fail("expected string");
    result.clear();
    while (pos_ < text_.size()) {
      const unsigned char c = static_cast<unsigned char>(text_[pos_++]);
      if (c == '"') return true;
      if (c < 0x20u) return fail("control character in string");
      if (c != '\\') {
        result += static_cast<char>(c);
        continue;
      }
      if (pos_ >= text_.size()) return fail("truncated escape");
      const char escaped = text_[pos_++];
      switch (escaped) {
        case '"': result += '"'; break;
        case '\\': result += '\\'; break;
        case '/': result += '/'; break;
        case 'b': result += '\b'; break;
        case 'f': result += '\f'; break;
        case 'n': result += '\n'; break;
        case 'r': result += '\r'; break;
        case 't': result += '\t'; break;
        case 'u': {
          if (pos_ + 4 > text_.size()) return fail("truncated unicode escape");
          unsigned value = 0;
          for (unsigned i = 0; i < 4; ++i) {
            const char h = text_[pos_++];
            unsigned digit = 0;
            if (h >= '0' && h <= '9') digit = static_cast<unsigned>(h - '0');
            else if (h >= 'a' && h <= 'f') digit = static_cast<unsigned>(h - 'a' + 10);
            else if (h >= 'A' && h <= 'F') digit = static_cast<unsigned>(h - 'A' + 10);
            else return fail("invalid unicode escape");
            value = (value << 4) | digit;
          }
          // Diagnostic strings are bounded ASCII in the wire adapter.  Keep
          // unicode BMP values representable without locale dependencies.
          if (value <= 0x7fu) result += static_cast<char>(value);
          else if (value <= 0x7ffu) {
            result += static_cast<char>(0xc0u | (value >> 6));
            result += static_cast<char>(0x80u | (value & 0x3fu));
          } else {
            result += static_cast<char>(0xe0u | (value >> 12));
            result += static_cast<char>(0x80u | ((value >> 6) & 0x3fu));
            result += static_cast<char>(0x80u | (value & 0x3fu));
          }
          break;
        }
        default: return fail("invalid escape");
      }
    }
    return fail("unterminated string");
  }

  bool value_token(Value &value) {
    if (pos_ >= text_.size()) return fail("missing value");
    if (text_[pos_] == '"') {
      value.kind = Value::Kind::String;
      return string(value.text);
    }
    if (text_[pos_] == '{' || text_[pos_] == '[') {
      std::vector<char> stack;
      stack.push_back(text_[pos_++]);
      bool in_string = false;
      bool escaped = false;
      while (pos_ < text_.size() && !stack.empty()) {
        const char c = text_[pos_++];
        if (in_string) {
          if (escaped) escaped = false;
          else if (c == '\\') escaped = true;
          else if (c == '"') in_string = false;
        } else if (c == '"') {
          in_string = true;
        } else if (c == '{' || c == '[') {
          stack.push_back(c);
        } else if (c == '}' || c == ']') {
          if (stack.empty() || (c == '}' && stack.back() != '{') ||
              (c == ']' && stack.back() != '[')) {
            return fail("mismatched compound value");
          }
          stack.pop_back();
        }
      }
      if (in_string || !stack.empty()) return fail("unterminated compound value");
      value.kind = Value::Kind::Compound;
      return true;
    }
    if (text_.compare(pos_, 4, "true") == 0) {
      pos_ += 4;
      value.kind = Value::Kind::Bool;
      value.boolean = true;
      return true;
    }
    if (text_.compare(pos_, 5, "false") == 0) {
      pos_ += 5;
      value.kind = Value::Kind::Bool;
      value.boolean = false;
      return true;
    }
    if (text_.compare(pos_, 4, "null") == 0) {
      pos_ += 4;
      value.kind = Value::Kind::Null;
      return true;
    }
    const std::size_t begin = pos_;
    if (text_[pos_] == '-') ++pos_;
    const std::size_t integer_begin = pos_;
    while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
    if (pos_ == integer_begin) return fail("invalid value");
    if (pos_ < text_.size() && text_[pos_] == '.') {
      ++pos_;
      while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
    }
    if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
      ++pos_;
      if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-')) ++pos_;
      const std::size_t exp_begin = pos_;
      while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
      if (pos_ == exp_begin) return fail("invalid exponent");
    }
    value.kind = Value::Kind::Number;
    value.text = text_.substr(begin, pos_ - begin);
    return true;
  }

  const std::string &text_;
  std::size_t pos_ = 0;
  std::string error_;
};

template <typename T>
bool parse_integer(const Value &value, T &result) {
  if (value.kind != Value::Kind::Number || value.text.empty()) return false;
  const auto *begin = value.text.data();
  const auto *end = begin + value.text.size();
  const auto conversion = std::from_chars(begin, end, result);
  return conversion.ec == std::errc() && conversion.ptr == end;
}

template <typename Setter>
bool assign_unsigned(const std::map<std::string, Value> &values, const char *key,
                     std::uint64_t max, Setter setter, bool &seen) {
  const auto it = values.find(key);
  if (it == values.end()) return true;
  std::uint64_t value = 0;
  if (!parse_integer(it->second, value) || value > max) return false;
  setter(value);
  seen = true;
  return true;
}

template <typename Setter>
bool assign_signed(const std::map<std::string, Value> &values, const char *key,
                   std::int64_t min, std::int64_t max, Setter setter,
                   bool &seen) {
  const auto it = values.find(key);
  if (it == values.end()) return true;
  std::int64_t value = 0;
  if (!parse_integer(it->second, value) || value < min || value > max) return false;
  setter(value);
  seen = true;
  return true;
}

template <typename Record, typename Fill>
bool parse_record(const std::string &json, Record &out, Fill fill,
                  std::initializer_list<const char *> required,
                  std::string *error) {
  JsonParser parser(json);
  std::map<std::string, Value> values;
  if (!parser.object(values) || !parser.finished()) {
    if (error) *error = parser.error().empty() ? "invalid JSON" : parser.error();
    return false;
  }
  out = Record{};
  bool seen = false;
  if (!fill(values, out, seen)) {
    if (error) *error = "invalid or out-of-range field";
    return false;
  }
  for (const auto *key : required) {
    if (values.find(key) == values.end()) {
      if (error) *error = "missing required field";
      return false;
    }
  }
  if (error) error->clear();
  return seen;
}

}  // namespace

std::string state_to_json(const FpgaState &state) { return make_state_json(state); }
std::string plan_result_to_json(const FpgaPlanResult &result) { return make_plan_json(result); }
std::string gui_state_json(const FpgaState &state) { return make_state_json(state); }
std::string phone_state_json(const FpgaState &state) { return make_state_json(state); }
std::string gui_plan_result_json(const FpgaPlanResult &result) { return make_plan_json(result); }

bool parse_state_json(const std::string &json, FpgaState &out, std::string *error) {
  return parse_record<FpgaState>(
      json, out,
      [](const std::map<std::string, Value> &values, FpgaState &state, bool &seen) {
        if (values.find("state_flags") == values.end()) return false;
        if (!assign_unsigned(values, "state", std::numeric_limits<std::uint8_t>::max(),
                             [&](std::uint64_t v) { state.state = static_cast<std::uint8_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "avoid", std::numeric_limits<std::uint8_t>::max(),
                             [&](std::uint64_t v) { state.avoid = static_cast<std::uint8_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "reason", std::numeric_limits<std::uint16_t>::max(),
                             [&](std::uint64_t v) { state.reason = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "state_flags", std::numeric_limits<std::uint32_t>::max(),
                             [&](std::uint64_t v) { state.state_flags = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "nav_epoch", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.nav_epoch = v; }, seen)) return false;
        if (!assign_unsigned(values, "state_seq", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.state_seq = v; }, seen)) return false;
        if (!assign_unsigned(values, "pose_seq", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.pose_seq = v; }, seen)) return false;
        if (!assign_signed(values, "x_mm", std::numeric_limits<std::int32_t>::min(), std::numeric_limits<std::int32_t>::max(), [&](std::int64_t v) { state.x_mm = static_cast<std::int32_t>(v); }, seen)) return false;
        if (!assign_signed(values, "y_mm", std::numeric_limits<std::int32_t>::min(), std::numeric_limits<std::int32_t>::max(), [&](std::int64_t v) { state.y_mm = static_cast<std::int32_t>(v); }, seen)) return false;
        if (!assign_signed(values, "yaw_urad", std::numeric_limits<std::int32_t>::min(), std::numeric_limits<std::int32_t>::max(), [&](std::int64_t v) { state.yaw_urad = static_cast<std::int32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "pose_age_ms", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.pose_age_ms = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "cursor", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { state.cursor = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "path_count", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { state.path_count = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "remaining_mm", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.remaining_mm = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "permit_remaining_ms", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.permit_remaining_ms = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "auth_remaining_ms", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.auth_remaining_ms = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "active_crc32", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.active_crc32 = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "pending_crc32", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.pending_crc32 = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "upload_crc32", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.upload_crc32 = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "gate_reasons", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.gate_reasons = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "pending_map_epoch", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.pending_map_epoch = v; }, seen)) return false;
        if (!assign_unsigned(values, "pending_map_revision", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.pending_map_revision = v; }, seen)) return false;
        if (!assign_unsigned(values, "upload_map_epoch", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.upload_map_epoch = v; }, seen)) return false;
        if (!assign_unsigned(values, "upload_map_revision", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.upload_map_revision = v; }, seen)) return false;
        if (!assign_unsigned(values, "last_ack_seq", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.last_ack_seq = v; }, seen)) return false;
        if (!assign_unsigned(values, "device_ms", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.device_ms = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "owner", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.owner = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "upload_chunks", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { state.upload_chunks = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "reserved", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { state.reserved = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "diagnostic", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.diagnostic = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "last_request_id", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.last_request_id = v; }, seen)) return false;
        if (!assign_unsigned(values, "active_frame_token", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.active_frame_token = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "pending_frame_token", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { state.pending_frame_token = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "reserved_tail", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { state.reserved_tail = v; }, seen)) return false;
        if (state.state > 5u || state.avoid > 3u ||
            (state.state_flags & ~kKnownStateFlags) != 0 || state.reserved != 0 ||
            state.reserved_tail != 0) return false;
        return true;
      },
      {"state", "state_flags"},
      error);
}

bool parse_plan_result_json(const std::string &json, FpgaPlanResult &out,
                            std::string *error) {
  return parse_record<FpgaPlanResult>(
      json, out,
      [](const std::map<std::string, Value> &values, FpgaPlanResult &result, bool &seen) {
        if (!assign_unsigned(values, "status", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { result.status = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "reason", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { result.reason = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "device_ms", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { result.device_ms = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "nav_epoch", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { result.nav_epoch = v; }, seen)) return false;
        if (!assign_unsigned(values, "state_seq", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { result.state_seq = v; }, seen)) return false;
        if (!assign_unsigned(values, "start_pose_seq", std::numeric_limits<std::uint64_t>::max(), [&](std::uint64_t v) { result.start_pose_seq = v; }, seen)) return false;
        if (!assign_unsigned(values, "expanded", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { result.expanded = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "cycles", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { result.cycles = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "path_length", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { result.path_length = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "start_cell", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { result.start_cell = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "goal_cell", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { result.goal_cell = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "result_flags", std::numeric_limits<std::uint16_t>::max(), [&](std::uint64_t v) { result.result_flags = static_cast<std::uint16_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "path_revision", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { result.path_revision = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "frame_token", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { result.frame_token = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "cpu_hz", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { result.cpu_hz = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (!assign_unsigned(values, "reserved_tail", std::numeric_limits<std::uint32_t>::max(), [&](std::uint64_t v) { result.reserved_tail = static_cast<std::uint32_t>(v); }, seen)) return false;
        if (result.reserved_tail != 0) return false;
        return true;
      },
      {"status"},
      error);
}

std::vector<PathPoint> smooth_display_path(const std::vector<PathPoint> &path,
                                           const PathSmoothingConfig &config) {
  if (!std::isfinite(config.alpha) || config.alpha < 0.0 || config.alpha > 1.0 ||
      !std::isfinite(config.max_deviation) || config.max_deviation < 0.0 ||
      !std::isfinite(config.max_coordinate) || config.max_coordinate <= 0.0 ||
      config.max_coordinate > 1.0e12 || config.radius > 1024u) {
    throw std::invalid_argument("invalid path smoothing configuration");
  }
  for (const auto &point : path) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        std::abs(point.x) > config.max_coordinate || std::abs(point.y) > config.max_coordinate ||
        std::abs(point.z) > config.max_coordinate) {
      throw std::invalid_argument("path contains non-finite or out-of-range point");
    }
  }
  if (path.size() < 3u || config.radius == 0u || config.alpha == 0.0 ||
      config.max_deviation == 0.0) {
    return path;
  }
  std::vector<PathPoint> result = path;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const std::size_t begin = i > config.radius ? i - config.radius : 0u;
    const std::size_t end = std::min(path.size() - 1u, i + config.radius);
    long double sx = 0.0L, sy = 0.0L, sz = 0.0L;
    std::size_t count = 0;
    for (std::size_t j = begin; j <= end; ++j) {
      sx += path[j].x;
      sy += path[j].y;
      sz += path[j].z;
      ++count;
    }
    const PathPoint average{static_cast<double>(sx / count), static_cast<double>(sy / count),
                            static_cast<double>(sz / count)};
    double dx = average.x - path[i].x;
    double dy = average.y - path[i].y;
    double dz = average.z - path[i].z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(distance)) throw std::invalid_argument("path smoothing overflow");
    if (distance > config.max_deviation && distance > 0.0) {
      const double scale = config.max_deviation / distance;
      dx *= scale;
      dy *= scale;
      dz *= scale;
    }
    result[i].x = std::clamp(path[i].x + config.alpha * dx, -config.max_coordinate, config.max_coordinate);
    result[i].y = std::clamp(path[i].y + config.alpha * dy, -config.max_coordinate, config.max_coordinate);
    result[i].z = std::clamp(path[i].z + config.alpha * dz, -config.max_coordinate, config.max_coordinate);
  }
  return result;
}

void DiagnosticAggregator::observe(const DiagnosticSample &sample) {
  if (sample.source.size() > 256u || sample.message.size() > 2048u) {
    throw std::invalid_argument("diagnostic source or message is too long");
  }
  if (sample_count_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("diagnostic sample count overflow");
  }
  ++sample_count_;
  last_seen_ms_ = std::max(last_seen_ms_, sample.stamp_ms);
  auto it = std::find_if(buckets_.begin(), buckets_.end(), [&](const DiagnosticBucket &bucket) {
    return bucket.source == sample.source && bucket.reason == sample.reason && bucket.gate == sample.gate;
  });
  if (it == buckets_.end()) {
    buckets_.push_back({sample.source, sample.reason, sample.gate, 1u, sample.stamp_ms,
                        sample.severity, sample.message});
    return;
  }
  if (it->count == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("diagnostic bucket count overflow");
  }
  ++it->count;
  if (sample.stamp_ms >= it->last_seen_ms) {
    it->last_seen_ms = sample.stamp_ms;
    it->last_severity = sample.severity;
    it->last_message = sample.message;
  }
}

void DiagnosticAggregator::clear() {
  buckets_.clear();
  sample_count_ = 0;
  last_seen_ms_ = 0;
}

DiagnosticSummary DiagnosticAggregator::summary() const {
  DiagnosticSummary result{sample_count_, last_seen_ms_, buckets_};
  std::sort(result.buckets.begin(), result.buckets.end(), [](const DiagnosticBucket &a, const DiagnosticBucket &b) {
    if (a.source != b.source) return a.source < b.source;
    if (a.reason != b.reason) return a.reason < b.reason;
    return a.gate < b.gate;
  });
  return result;
}

std::string diagnostics_to_json(const DiagnosticSummary &summary) {
  std::string json = "{\"kind\":\"diagnostics_summary\",\"schema_version\":1,\"sample_count\":";
  json += std::to_string(summary.sample_count);
  json += ",\"last_seen_ms\":";
  json += std::to_string(summary.last_seen_ms);
  json += ",\"buckets\":[";
  auto ordered = summary.buckets;
  std::sort(ordered.begin(), ordered.end(), [](const DiagnosticBucket &a,
                                               const DiagnosticBucket &b) {
    if (a.source != b.source) return a.source < b.source;
    if (a.reason != b.reason) return a.reason < b.reason;
    return a.gate < b.gate;
  });
  for (std::size_t i = 0; i < ordered.size(); ++i) {
    if (i) json += ',';
    const auto &bucket = ordered[i];
    json += "{\"source\":\"";
    json += escape_json(bucket.source);
    json += "\",\"reason\":";
    json += std::to_string(bucket.reason);
    json += ",\"gate\":";
    json += std::to_string(bucket.gate);
    json += ",\"count\":";
    json += std::to_string(bucket.count);
    json += ",\"last_seen_ms\":";
    json += std::to_string(bucket.last_seen_ms);
    json += ",\"last_severity\":";
    json += std::to_string(bucket.last_severity);
    json += ",\"last_message\":\"";
    json += escape_json(bucket.last_message);
    json += "\"}";
  }
  json += "]}";
  return json;
}

std::string DiagnosticAggregator::to_json() const { return diagnostics_to_json(summary()); }

DiagnosticSummary aggregate_diagnostics(const std::vector<DiagnosticSample> &samples) {
  DiagnosticAggregator aggregator;
  for (const auto &sample : samples) aggregator.observe(sample);
  return aggregator.summary();
}

}  // namespace acg138::rk_postprocess
