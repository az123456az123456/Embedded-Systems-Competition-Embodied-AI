#include "ros_bridge_codec.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <utility>

namespace arb1 {
namespace {
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "IEEE binary32 required");
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
              "IEEE binary64 required");
using Bytes = std::vector<uint8_t>;
bool fail(DecodeError* e, ErrorCode code, size_t offset, const char* detail) {
  if (e) *e = {code, offset, detail};
  return false;
}
void success(DecodeError* e) { if (e) *e = {}; }
uint16_t u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
uint32_t u32(const uint8_t* p) {
  uint32_t v = 0;
  for (unsigned i = 0; i != 4; ++i) v |= uint32_t(p[i]) << (8 * i);
  return v;
}
uint64_t u64(const uint8_t* p) {
  uint64_t v = 0;
  for (unsigned i = 0; i != 8; ++i) v |= uint64_t(p[i]) << (8 * i);
  return v;
}
int64_t i64(const uint8_t* p) {
  uint64_t v = u64(p); int64_t out; std::memcpy(&out, &v, 8); return out;
}
double f64(const uint8_t* p) {
  uint64_t v = u64(p); double out; std::memcpy(&out, &v, 8); return out;
}
float f32(const uint8_t* p) {
  uint32_t v = u32(p); float out; std::memcpy(&out, &v, 4); return out;
}
void put(Bytes& b, size_t at, uint64_t v, unsigned n) {
  for (unsigned i = 0; i != n; ++i) b[at + i] = uint8_t(v >> (8 * i));
}
void put_double(Bytes& b, size_t at, double v) {
  uint64_t bits; std::memcpy(&bits, &v, 8); put(b, at, bits, 8);
}
void put_float(Bytes& b, size_t at, float v) {
  uint32_t bits; std::memcpy(&bits, &v, 4); put(b, at, bits, 4);
}
void copy_id(const uint8_t* p, Id& id) { std::copy(p, p + 16, id.begin()); }
void put_id(Bytes& b, size_t at, const Id& id) {
  std::copy(id.begin(), id.end(), b.begin() + at);
}
bool known_kind(uint16_t kind) {
  return kind >= ARB1_KIND_HELLO && kind <= ARB1_KIND_DRIVE_INTENT;
}
bool utf8(const uint8_t* p, size_t n) {
  size_t i = 0;
  while (i < n) {
    uint32_t c = p[i++]; unsigned need = 0; uint32_t min = 0;
    if (c <= 0x7f) continue;
    if (c >= 0xc2 && c <= 0xdf) { c &= 31; need = 1; min = 0x80; }
    else if (c >= 0xe0 && c <= 0xef) { c &= 15; need = 2; min = 0x800; }
    else if (c >= 0xf0 && c <= 0xf4) { c &= 7; need = 3; min = 0x10000; }
    else return false;
    if (need > n - i) return false;
    while (need--) {
      uint8_t d = p[i++]; if ((d & 0xc0) != 0x80) return false;
      c = (c << 6) | (d & 63);
    }
    if (c < min || c > 0x10ffff || (c >= 0xd800 && c <= 0xdfff)) return false;
  }
  return true;
}
bool utf8(const std::string& s) {
  return utf8(reinterpret_cast<const uint8_t*>(s.data()), s.size());
}
/* Iterative JSON syntax validation: even adversarial nesting uses heap storage
 * bounded by the diagnostic byte limit, never the C++ call stack. */
bool json(const std::string& s) {
  size_t i = 0;
  auto ws = [&]() { while (i < s.size() && (s[i]==' ' || s[i]=='\n' ||
                              s[i]=='\r' || s[i]=='\t')) ++i; };
  auto hex = [](char c) { return (c >= '0' && c <= '9') ||
      (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); };
  auto string_token = [&]() {
    if (i >= s.size() || s[i++] != '"') return false;
    while (i < s.size()) {
      unsigned char c = static_cast<unsigned char>(s[i++]);
      if (c == '"') return true;
      if (c < 0x20) return false;
      if (c == '\\') {
        if (i == s.size()) return false;
        char esc = s[i++];
        if (esc == 'u') {
          if (s.size() - i < 4) return false;
          for (unsigned j = 0; j < 4; ++j) if (!hex(s[i++])) return false;
        } else if (esc!='"' && esc!='\\' && esc!='/' && esc!='b' &&
                   esc!='f' && esc!='n' && esc!='r' && esc!='t') return false;
      }
    }
    return false;
  };
  // States: root-value=0, root-end=1, object-key-or-end=2, key=3,
  // colon=4, object-value=5, object-comma-or-end=6, array-value-or-end=7,
  // array-value=8, array-comma-or-end=9.
  std::vector<unsigned> stack{0};
  while (!stack.empty()) {
    ws(); unsigned state = stack.back();
    if (state == 1) return i == s.size();
    if (i == s.size()) return false;
    if (state == 2 || state == 3) {
      if (state == 2 && s[i] == '}') { ++i; stack.pop_back(); continue; }
      if (!string_token()) return false;
      stack.back() = 4; continue;
    }
    if (state == 4) {
      if (s[i++] != ':') return false;
      stack.back() = 5; continue;
    }
    if (state == 6 || state == 9) {
      if (s[i] == (state == 6 ? '}' : ']')) {
        ++i; stack.pop_back(); continue;
      }
      if (s[i++] != ',') return false;
      stack.back() = state == 6 ? 3 : 8; continue;
    }
    if (state == 7 && s[i] == ']') { ++i; stack.pop_back(); continue; }
    stack.back() = state == 0 ? 1 : (state == 5 ? 6 : 9);
    char c = s[i];
    if (c == '{' || c == '[') { ++i; stack.push_back(c == '{' ? 2 : 7); }
    else if (c == '"') { if (!string_token()) return false; }
    else if (c == 't' || c == 'f' || c == 'n') {
      const char* token = c == 't' ? "true" : (c == 'f' ? "false" : "null");
      size_t n = std::strlen(token);
      if (s.compare(i, n, token) != 0) return false;
      i += n;
    } else {
      if (s[i] == '-' && ++i == s.size()) return false;
      if (s[i] == '0') ++i;
      else {
        if (s[i] < '1' || s[i] > '9') return false;
        while (i < s.size() && s[i] >= '0' && s[i] <= '9') ++i;
      }
      if (i < s.size() && s[i] == '.') {
        ++i; size_t begin = i;
        while (i < s.size() && s[i] >= '0' && s[i] <= '9') ++i;
        if (i == begin) return false;
      }
      if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
        ++i; if (i < s.size() && (s[i]=='+' || s[i]=='-')) ++i;
        size_t begin = i;
        while (i < s.size() && s[i] >= '0' && s[i] <= '9') ++i;
        if (i == begin) return false;
      }
    }
  }
  return false;
}
bool read_frame(const uint8_t* p, std::string& out) {
  size_t n = 0; while (n < 64 && p[n]) ++n;
  if (n == 64 || !utf8(p, n)) return false;
  for (size_t i = n; i < 64; ++i) if (p[i]) return false;
  out.assign(reinterpret_cast<const char*>(p), n); return true;
}
bool write_frame(Bytes& b, size_t at, const std::string& s) {
  if (s.size() > ARB1_MAX_FRAME_ID_BYTES || s.find('\0') != std::string::npos ||
      !utf8(s)) return false;
  std::copy(s.begin(), s.end(), b.begin() + at); return true;
}
bool finite(std::initializer_list<double> v) {
  return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
}
bool occupancy(uint8_t c) { return c == 255 || c <= 100; }
int8_t cell(uint8_t c) { return c == 255 ? int8_t(-1) : static_cast<int8_t>(c); }

bool decode_payload(uint16_t kind, const uint8_t* p, size_t n,
                    const Header& h, Payload& out, DecodeError* e, size_t base) {
  auto bad = [&](const char* detail) { return fail(e, ErrorCode::Payload, base, detail); };
  auto nonfinite = [&]() { return fail(e, ErrorCode::NonFinite, base, "nonfinite geometry"); };
  auto frame_error = [&]() { return fail(e, ErrorCode::Utf8, base, "frame_id must be UTF-8 NUL terminated with zero tail"); };
  switch (kind) {
    case ARB1_KIND_HELLO: {
      if (n < 32) return bad("short HELLO");
      size_t count = u16(p + 4);
      if (count > ARB1_MAX_BLOCKS) return fail(e, ErrorCode::Bound, base + 4, "capabilities limit");
      if (n != 32 + 4 * count || u16(p + 6)) return bad("HELLO length/reserved");
      Hello v; v.major = u16(p); v.minor = u16(p + 2);
      v.bridge_epoch = u64(p + 8); copy_id(p + 16, v.boot_id);
      if (v.major != h.major || v.minor != h.minor || v.bridge_epoch != h.bridge_epoch || v.boot_id != h.boot_id)
        return fail(e, ErrorCode::Identity, base, "HELLO/header identity mismatch");
      std::set<std::pair<uint16_t, uint16_t>> seen;
      for (size_t i = 0; i < count; ++i) {
        Capability c{u16(p + 32 + 4*i), u16(p + 34 + 4*i)};
        if (!c.kind || !c.schema || !seen.emplace(c.kind, c.schema).second) return bad("invalid/duplicate capability");
        v.capabilities.push_back(c);
      }
      out = std::move(v); return true;
    }
    case ARB1_KIND_ERROR: {
      if (n < 16) return bad("short ERROR");
      size_t bytes = u32(p + 4);
      if (bytes > ARB1_MAX_ERROR_DETAIL_BYTES) return fail(e, ErrorCode::Bound, base + 4, "error detail limit");
      if (n != 16 + bytes) return bad("ERROR length");
      if (!utf8(p + 16, bytes)) return fail(e, ErrorCode::Utf8, base + 16, "error detail UTF-8");
      out = Error{u32(p), u64(p + 8), std::string(reinterpret_cast<const char*>(p + 16), bytes)};
      return true;
    }
    case ARB1_KIND_MAP: {
      if (n < 128) return bad("short MAP");
      Map v; v.width = u32(p); v.height = u32(p + 4);
      uint64_t count = uint64_t(v.width) * uint64_t(v.height);
      if (!count || count > ARB1_MAX_MAP_CELLS) return fail(e, ErrorCode::Bound, base, "map cells limit/empty");
      v.resolution = f64(p + 8); v.origin_x = f64(p + 16);
      v.origin_y = f64(p + 24); v.origin_yaw = f64(p + 32);
      if (!finite({v.resolution,v.origin_x,v.origin_y,v.origin_yaw})) return nonfinite();
      if (v.resolution <= 0) return bad("map resolution must be positive");
      v.map_epoch = u64(p + 40); v.map_revision = u64(p + 48);
      v.wire_encoding = u32(p + 56); size_t bytes = u32(p + 60);
      if (bytes != n - 128) return bad("MAP data length");
      if (!read_frame(p + 64, v.frame_id)) return frame_error();
      v.cells.reserve(static_cast<size_t>(count));
      if (v.wire_encoding == ARB1_ENCODING_RAW) {
        if (bytes != count) return bad("raw MAP count");
        for (size_t i = 0; i < bytes; ++i) {
          if (!occupancy(p[128 + i])) return bad("occupancy value");
          v.cells.push_back(cell(p[128 + i]));
        }
      } else if (v.wire_encoding == ARB1_ENCODING_RLE) {
        if (bytes % 5) return bad("partial RLE record");
        uint64_t sum = 0;
        for (size_t i = 0; i < bytes; i += 5) {
          uint32_t run = u32(p + 128 + i); uint8_t value = p[132 + i];
          if (!run || !occupancy(value) || run > count - sum) return bad("RLE run/value/sum");
          sum += run; v.cells.insert(v.cells.end(), run, cell(value));
        }
        if (sum != count) return bad("RLE sum mismatch");
      } else return bad("unknown map encoding");
      out = std::move(v); return true;
    }
    case ARB1_KIND_ODOM: {
      if (n != 176) return bad("ODOM length");
      Odom v;
      if (!read_frame(p, v.frame_id) || !read_frame(p + 64, v.child_frame_id)) return frame_error();
      v.x=f64(p+128); v.y=f64(p+136); v.theta=f64(p+144);
      v.vx=f64(p+152); v.wz=f64(p+160); v.localization_epoch=u64(p+168);
      if (!finite({v.x,v.y,v.theta,v.vx,v.wz})) return nonfinite();
      out = std::move(v); return true;
    }
    case ARB1_KIND_TF: {
      if (n != 160) return bad("TF length");
      Tf v;
      if (!read_frame(p, v.frame_id) || !read_frame(p + 64, v.child_frame_id)) return frame_error();
      v.x=f64(p+128); v.y=f64(p+136); v.yaw=f64(p+144); v.localization_epoch=u64(p+152);
      if (!finite({v.x,v.y,v.yaw})) return nonfinite();
      out = std::move(v); return true;
    }
    case ARB1_KIND_SCAN: {
      if (n < 112) return bad("short SCAN");
      Scan v; size_t count=u32(p+64), bytes=u32(p+68);
      if (count > ARB1_MAX_SCAN_POINTS) return fail(e, ErrorCode::Bound, base+64, "scan points limit");
      if (bytes != count * 4 || n != 112 + bytes || u32(p+104) != ARB1_SCAN_FLOAT32 || u32(p+108)) return bad("SCAN bytes/format/reserved");
      if (!read_frame(p,v.frame_id)) return frame_error();
      v.angle_min=f64(p+72); v.angle_increment=f64(p+80); v.range_min=f64(p+88); v.range_max=f64(p+96);
      if (!finite({v.angle_min,v.angle_increment,v.range_min,v.range_max})) return nonfinite();
      v.ranges.reserve(count);
      for (size_t i=0; i<count; ++i) v.ranges.push_back(f32(p+112+i*4));
      out=std::move(v); return true;
    }
    case ARB1_KIND_DIAGNOSTIC: {
      if (n < 8) return bad("short DIAGNOSTIC");
      size_t bytes=u32(p+4);
      if (bytes > ARB1_MAX_DIAGNOSTIC_BYTES) return fail(e,ErrorCode::Bound,base+4,"diagnostic limit");
      if (u32(p)!=ARB1_DIAGNOSTIC_UTF8_JSON || n!=8+bytes) return bad("DIAGNOSTIC encoding/length");
      Diagnostic v{std::string(reinterpret_cast<const char*>(p+8),bytes)};
      if (!utf8(v.json)) return fail(e,ErrorCode::Utf8,base+8,"diagnostic UTF-8");
      if (!json(v.json)) return bad("diagnostic JSON syntax");
      out=std::move(v); return true;
    }
    case ARB1_KIND_PATH: {
      if (n < 112) return bad("short PATH");
      Path v; size_t count=u32(p+104),bytes=u32(p+108);
      if (count>ARB1_MAX_PATH_POINTS) return fail(e,ErrorCode::Bound,base+104,"path points limit");
      if (bytes!=count*16 || n!=112+bytes) return bad("PATH bytes");
      if (!read_frame(p,v.frame_id)) return frame_error();
      copy_id(p+64,v.goal_id); v.map_epoch=u64(p+80); v.map_revision=u64(p+88); v.route_revision=u64(p+96);
      v.points.reserve(count);
      for (size_t i=0;i<count;++i) {
        Point q{f64(p+112+16*i),f64(p+120+16*i)};
        if (!finite({q.x,q.y})) return nonfinite();
        v.points.push_back(q);
      }
      out=std::move(v); return true;
    }
    case ARB1_KIND_DRIVE_INTENT: {
      if (n!=96) return bad("DRIVE_INTENT length");
      DriveIntent v; v.generation=u64(p); v.route_revision=u64(p+8); v.command_sequence=u64(p+16);
      copy_id(p+24,v.goal_id); v.linear_mps=f64(p+40); v.angular_rps=f64(p+48);
      v.produced_mono_ns=u64(p+56); v.valid_for_ms=u32(p+64); v.flags=u32(p+68);
      v.localization_epoch=u64(p+72); v.map_epoch=u64(p+80); v.map_revision=u64(p+88);
      if (v.flags) return fail(e,ErrorCode::Reserved,base+68,"drive flags reserved");
      if (!finite({v.linear_mps,v.angular_rps})) return nonfinite();
      out=v; return true;
    }
    default: return bad("internal unknown kind");
  }
}

bool encode_payload(const Payload& value, Bytes& b, uint16_t& schema,
                    DecodeError* e) {
  schema=ARB1_SCHEMA_V1;
  auto frame_error=[&]() { return fail(e,ErrorCode::Utf8,0,"invalid frame_id"); };
  return std::visit([&](const auto& v)->bool {
    using T=std::decay_t<decltype(v)>;
    if constexpr (std::is_same_v<T,Hello>) {
      if (v.capabilities.size()>ARB1_MAX_BLOCKS) return fail(e,ErrorCode::Bound,0,"capabilities limit");
      b.resize(32+4*v.capabilities.size()); put(b,0,v.major,2); put(b,2,v.minor,2);
      put(b,4,v.capabilities.size(),2); put(b,8,v.bridge_epoch,8); put_id(b,16,v.boot_id);
      for(size_t i=0;i<v.capabilities.size();++i) { put(b,32+4*i,v.capabilities[i].kind,2); put(b,34+4*i,v.capabilities[i].schema,2); }
    } else if constexpr (std::is_same_v<T,Error>) {
      if(v.detail.size()>ARB1_MAX_ERROR_DETAIL_BYTES) return fail(e,ErrorCode::Bound,0,"error detail limit");
      b.resize(16+v.detail.size()); put(b,0,v.code,4); put(b,4,v.detail.size(),4); put(b,8,v.offending_sequence,8);
      std::copy(v.detail.begin(),v.detail.end(),b.begin()+16);
    } else if constexpr (std::is_same_v<T,Map>) {
      uint64_t count=uint64_t(v.width)*v.height;
      if(!count || count>ARB1_MAX_MAP_CELLS || count!=v.cells.size()) return fail(e,ErrorCode::Bound,0,"map cells/count");
      Bytes data; data.reserve(v.cells.size());
      for(int8_t c:v.cells) { uint8_t q=static_cast<uint8_t>(c); if(!occupancy(q)) return fail(e,ErrorCode::Payload,0,"occupancy value"); data.push_back(q); }
      uint32_t encoding=ARB1_ENCODING_RAW;
      if(v.prefer_rle) {
        Bytes rle;
        for(size_t i=0;i<data.size();) {
          size_t end=i+1; while(end<data.size()&&data[end]==data[i]) ++end;
          // A candidate that reaches raw size cannot later become smaller.
          if(rle.size()+5>=data.size()) { rle.clear(); break; }
          size_t at=rle.size(); rle.resize(at+5); put(rle,at,end-i,4); rle[at+4]=data[i]; i=end;
        }
        if(!rle.empty()&&rle.size()<data.size()) { data=std::move(rle); encoding=ARB1_ENCODING_RLE; }
      }
      b.resize(128+data.size()); put(b,0,v.width,4); put(b,4,v.height,4);
      put_double(b,8,v.resolution); put_double(b,16,v.origin_x); put_double(b,24,v.origin_y); put_double(b,32,v.origin_yaw);
      put(b,40,v.map_epoch,8); put(b,48,v.map_revision,8); put(b,56,encoding,4); put(b,60,data.size(),4);
      if(!write_frame(b,64,v.frame_id)) return frame_error();
      std::copy(data.begin(),data.end(),b.begin()+128);
    } else if constexpr (std::is_same_v<T,Odom>) {
      b.resize(176); if(!write_frame(b,0,v.frame_id)||!write_frame(b,64,v.child_frame_id)) return frame_error();
      put_double(b,128,v.x); put_double(b,136,v.y); put_double(b,144,v.theta); put_double(b,152,v.vx); put_double(b,160,v.wz); put(b,168,v.localization_epoch,8);
    } else if constexpr (std::is_same_v<T,Tf>) {
      b.resize(160); if(!write_frame(b,0,v.frame_id)||!write_frame(b,64,v.child_frame_id)) return frame_error();
      put_double(b,128,v.x); put_double(b,136,v.y); put_double(b,144,v.yaw); put(b,152,v.localization_epoch,8);
    } else if constexpr (std::is_same_v<T,Scan>) {
      if(v.ranges.size()>ARB1_MAX_SCAN_POINTS) return fail(e,ErrorCode::Bound,0,"scan points limit");
      b.resize(112+4*v.ranges.size()); if(!write_frame(b,0,v.frame_id)) return frame_error();
      put(b,64,v.ranges.size(),4); put(b,68,4*v.ranges.size(),4); put_double(b,72,v.angle_min); put_double(b,80,v.angle_increment);
      put_double(b,88,v.range_min); put_double(b,96,v.range_max); put(b,104,ARB1_SCAN_FLOAT32,4);
      for(size_t i=0;i<v.ranges.size();++i) put_float(b,112+4*i,v.ranges[i]);
    } else if constexpr (std::is_same_v<T,Diagnostic>) {
      if(v.json.size()>ARB1_MAX_DIAGNOSTIC_BYTES) return fail(e,ErrorCode::Bound,0,"diagnostic limit");
      b.resize(8+v.json.size()); put(b,0,ARB1_DIAGNOSTIC_UTF8_JSON,4); put(b,4,v.json.size(),4); std::copy(v.json.begin(),v.json.end(),b.begin()+8);
    } else if constexpr (std::is_same_v<T,Path>) {
      if(v.points.size()>ARB1_MAX_PATH_POINTS) return fail(e,ErrorCode::Bound,0,"path points limit");
      b.resize(112+16*v.points.size()); if(!write_frame(b,0,v.frame_id)) return frame_error();
      put_id(b,64,v.goal_id); put(b,80,v.map_epoch,8); put(b,88,v.map_revision,8); put(b,96,v.route_revision,8);
      put(b,104,v.points.size(),4); put(b,108,v.points.size()*16,4);
      for(size_t i=0;i<v.points.size();++i) { put_double(b,112+16*i,v.points[i].x); put_double(b,120+16*i,v.points[i].y); }
    } else if constexpr (std::is_same_v<T,DriveIntent>) {
      b.resize(96); put(b,0,v.generation,8); put(b,8,v.route_revision,8); put(b,16,v.command_sequence,8); put_id(b,24,v.goal_id);
      put_double(b,40,v.linear_mps); put_double(b,48,v.angular_rps); put(b,56,v.produced_mono_ns,8); put(b,64,v.valid_for_ms,4); put(b,68,v.flags,4);
      put(b,72,v.localization_epoch,8); put(b,80,v.map_epoch,8); put(b,88,v.map_revision,8);
    } else {
      if(v.bytes.size()>ARB1_MAX_MESSAGE_BYTES) return fail(e,ErrorCode::Bound,0,"unknown payload limit");
      schema=v.schema; b=v.bytes;
    }
    return true;
  },value);
}
}  // namespace

uint16_t block_kind(const Payload& p) {
  return std::visit([](const auto& v)->uint16_t {
    using T=std::decay_t<decltype(v)>;
    if constexpr(std::is_same_v<T,Hello>) return ARB1_KIND_HELLO;
    else if constexpr(std::is_same_v<T,Error>) return ARB1_KIND_ERROR;
    else if constexpr(std::is_same_v<T,Map>) return ARB1_KIND_MAP;
    else if constexpr(std::is_same_v<T,Odom>) return ARB1_KIND_ODOM;
    else if constexpr(std::is_same_v<T,Tf>) return ARB1_KIND_TF;
    else if constexpr(std::is_same_v<T,Scan>) return ARB1_KIND_SCAN;
    else if constexpr(std::is_same_v<T,Diagnostic>) return ARB1_KIND_DIAGNOSTIC;
    else if constexpr(std::is_same_v<T,Path>) return ARB1_KIND_PATH;
    else if constexpr(std::is_same_v<T,DriveIntent>) return ARB1_KIND_DRIVE_INTENT;
    else return v.kind;
  },p);
}

bool inspect_frame_size(const uint8_t* p,size_t size,size_t& total,DecodeError* e) {
  if(!p || size<64) return fail(e,ErrorCode::Truncated,0,"need Header64");
  if(p[0]!='A'||p[1]!='R'||p[2]!='B'||p[3]!='1') return fail(e,ErrorCode::Magic,0,"ARB1 magic");
  if(u16(p+4)!=ARB1_PROTOCOL_MAJOR||u16(p+6)!=ARB1_PROTOCOL_MINOR) return fail(e,ErrorCode::Version,4,"unsupported version");
  size_t n=u32(p+8), count=u16(p+14); uint16_t type=u16(p+12);
  if(type<ARB1_MSG_HELLO||type>ARB1_MSG_DRIVE_INTENT) return fail(e,ErrorCode::MessageType,12,"unknown message type");
  if(count>ARB1_MAX_BLOCKS) return fail(e,ErrorCode::BlockCount,14,"block count limit");
  if((type==ARB1_MSG_SNAPSHOT&&count==0)||(type!=ARB1_MSG_SNAPSHOT&&count!=1)) return fail(e,ErrorCode::BlockCount,14,"message block count");
  if(u32(p+16)||u32(p+20)) return fail(e,ErrorCode::Reserved,16,"header flags/reserved");
  if(n>ARB1_MAX_MESSAGE_BYTES||n<64+48*count) return fail(e,ErrorCode::Length,8,"message size bound");
  total=n; success(e); return true;
}

bool decode(const uint8_t* p,size_t size,Message& out,DecodeError* e) {
  size_t total=0;
  if(!inspect_frame_size(p,size,total,e)) return false;
  if(total!=size) return fail(e,ErrorCode::Length,8,"exact frame length required");
  Message result;
  Header& h=result.header; h.major=u16(p+4); h.minor=u16(p+6); h.type=u16(p+12); h.flags=u32(p+16);
  h.bridge_epoch=u64(p+24); h.message_sequence=u64(p+32); h.sent_mono_ns=u64(p+40); copy_id(p+48,h.boot_id);
  size_t count=u16(p+14), table_end=64+48*count;
  struct Descriptor { uint16_t kind,schema; Metadata metadata; size_t offset,length; };
  std::vector<Descriptor> descriptors; descriptors.reserve(count);
  std::set<uint16_t> known;
  std::vector<std::pair<size_t,size_t>> intervals;
  for(size_t i=0;i<count;++i) {
    const uint8_t* q=p+64+48*i;
    Descriptor d; d.kind=u16(q); d.schema=u16(q+2); d.metadata.flags=u32(q+4);
    d.metadata.sample_sequence=u64(q+8); d.metadata.received_mono_ns=u64(q+16); d.metadata.source_stamp_ns=i64(q+24); d.metadata.source_age_ns=u64(q+32);
    d.offset=u32(q+40); d.length=u32(q+44);
    if(d.metadata.flags&~uint32_t(ARB1_BLOCK_KNOWN_FLAGS)) return fail(e,ErrorCode::Reserved,68+48*i,"descriptor flags reserved");
    if(d.offset%8) return fail(e,ErrorCode::Alignment,104+48*i,"payload offset alignment");
    if(d.offset<table_end||d.offset>size||d.length>size-d.offset) return fail(e,ErrorCode::Length,104+48*i,"payload bounds/overflow");
    if(known_kind(d.kind)&&!known.insert(d.kind).second) return fail(e,ErrorCode::Duplicate,64+48*i,"duplicate known kind");
    bool supported=known_kind(d.kind)&&d.schema==ARB1_SCHEMA_V1;
    if(supported) {
      const uint32_t clock_flags=ARB1_BLOCK_SOURCE_ROS_TIME|ARB1_BLOCK_SOURCE_SYSTEM_TIME;
      if((d.metadata.flags&clock_flags)==clock_flags)
        return fail(e,ErrorCode::Reserved,68+48*i,"mutually exclusive source clocks");
      if((d.metadata.flags&ARB1_BLOCK_STATIC_TF)&&d.kind!=ARB1_KIND_TF)
        return fail(e,ErrorCode::Reserved,68+48*i,"STATIC_TF on non-TF block");
      if((d.metadata.flags&(ARB1_BLOCK_VALID|clock_flags))&&
         (d.kind<ARB1_KIND_MAP||d.kind>ARB1_KIND_PATH))
        return fail(e,ErrorCode::Reserved,68+48*i,"source validity/clock on non-state block");
    }
    if(!supported&&(d.metadata.flags&ARB1_BLOCK_REQUIRED)) return fail(e,ErrorCode::RequiredUnknown,64+48*i,"required unknown kind/schema");
    if(supported) {
      if((h.type==ARB1_MSG_HELLO && (d.kind!=ARB1_KIND_HELLO || !(d.metadata.flags&ARB1_BLOCK_REQUIRED))) ||
         (h.type==ARB1_MSG_ERROR && (d.kind!=ARB1_KIND_ERROR || !(d.metadata.flags&ARB1_BLOCK_REQUIRED))) ||
         (h.type==ARB1_MSG_DRIVE_INTENT && (d.kind!=ARB1_KIND_DRIVE_INTENT || !(d.metadata.flags&ARB1_BLOCK_REQUIRED))) ||
         (h.type==ARB1_MSG_SNAPSHOT && (d.kind<ARB1_KIND_MAP || d.kind>ARB1_KIND_PATH))) return fail(e,ErrorCode::MessageType,64+48*i,"block/message type mismatch");
    } else if(h.type!=ARB1_MSG_SNAPSHOT) return fail(e,ErrorCode::MessageType,64+48*i,"typed message requires supported block");
    intervals.emplace_back(d.offset,d.offset+d.length); descriptors.push_back(d);
  }
  std::sort(intervals.begin(),intervals.end());
  size_t end=table_end;
  for(const auto& interval:intervals) {
    if(interval.first<end) return fail(e,ErrorCode::Overlap,interval.first,"overlapping payloads");
    for(size_t i=end;i<interval.first;++i) if(p[i]) return fail(e,ErrorCode::Reserved,i,"nonzero alignment gap");
    end=interval.second;
  }
  if(end!=size) return fail(e,ErrorCode::Length,end,"unclaimed trailing bytes");
  for(const auto& d:descriptors) {
    Block b; b.metadata=d.metadata;
    if(known_kind(d.kind)&&d.schema==ARB1_SCHEMA_V1) {
      if(!decode_payload(d.kind,p+d.offset,d.length,h,b.payload,e,d.offset)) return false;
    } else b.payload=Unknown{d.kind,d.schema,Bytes(p+d.offset,p+d.offset+d.length)};
    result.blocks.push_back(std::move(b));
  }
  out=std::move(result); success(e); return true;
}

bool encode(const Message& m,Bytes& out,DecodeError* e) {
  if(m.blocks.size()>ARB1_MAX_BLOCKS) return fail(e,ErrorCode::BlockCount,14,"block count limit");
  Bytes b(64+48*m.blocks.size()); b[0]='A';b[1]='R';b[2]='B';b[3]='1';
  put(b,4,m.header.major,2);put(b,6,m.header.minor,2);put(b,12,m.header.type,2);put(b,14,m.blocks.size(),2);put(b,16,m.header.flags,4);
  put(b,24,m.header.bridge_epoch,8);put(b,32,m.header.message_sequence,8);put(b,40,m.header.sent_mono_ns,8);put_id(b,48,m.header.boot_id);
  for(size_t i=0;i<m.blocks.size();++i) {
    const Block& block=m.blocks[i]; Bytes payload; uint16_t schema=0;
    if(!encode_payload(block.payload,payload,schema,e)) return false;
    size_t start=(b.size()+7)&~size_t(7);
    if(start>ARB1_MAX_MESSAGE_BYTES||payload.size()>ARB1_MAX_MESSAGE_BYTES-start) return fail(e,ErrorCode::Bound,start,"message size limit");
    b.resize(start+payload.size()); std::copy(payload.begin(),payload.end(),b.begin()+start);
    size_t at=64+48*i; put(b,at,block_kind(block.payload),2);put(b,at+2,schema,2);put(b,at+4,block.metadata.flags,4);
    put(b,at+8,block.metadata.sample_sequence,8);put(b,at+16,block.metadata.received_mono_ns,8);
    uint64_t stamp;std::memcpy(&stamp,&block.metadata.source_stamp_ns,8);put(b,at+24,stamp,8);put(b,at+32,block.metadata.source_age_ns,8);
    put(b,at+40,start,4);put(b,at+44,payload.size(),4);
  }
  put(b,8,b.size(),4);
  Message checked;
  if(!decode(b.data(),b.size(),checked,e)) return false;
  out=std::move(b);success(e);return true;
}

bool executable(const DriveIntent& v) {
  return v.valid_for_ms>0 && v.flags==0 && finite({v.linear_mps,v.angular_rps});
}
}  // namespace arb1
