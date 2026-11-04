#include "follow_geometry.h"
#include "../vendor/picojson.h"
#include <cmath>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace robotphone {
namespace {
constexpr double degrees = 57.2957795130823208768;
const picojson::value *field(const picojson::object &o, const char *key) {
    auto i = o.find(key); return i == o.end() ? nullptr : &i->second;
}
bool number(const picojson::object &o, const char *key, double &out) {
    auto v = field(o,key); if (!v || !v->is<double>()) return false;
    out = v->get<double>(); return std::isfinite(out);
}
bool boolean(const picojson::object &o, const char *key, bool &out) {
    auto v = field(o,key); if (!v || !v->is<bool>()) return false;
    out = v->get<bool>(); return true;
}
bool finiteEye(const BearingEye &e) {
    return std::isfinite(e.fx_px) && std::isfinite(e.cx_px) && std::isfinite(e.yaw_right_deg) &&
        std::isfinite(e.lateral_right_m) && std::isfinite(e.min_u_px) &&
        std::isfinite(e.max_u_px) && std::isfinite(e.max_error_deg);
}
}
bool validateBearingConfig(const BearingConfig &c, std::string &error) {
    error.clear();
    if (!std::isfinite(c.lookahead_m) || c.lookahead_m < .2 || c.lookahead_m > 10) {
        error = "invalid_lookahead_m"; return false;
    }
    for (int i = 0; i < 2; ++i) {
        const auto &e = c.eyes[i];
        if (!finiteEye(e) || std::fabs(e.yaw_right_deg) >= 75 || std::fabs(e.lateral_right_m) > 1 ||
            e.min_u_px < 0 || e.max_u_px > 959 || e.min_u_px >= e.max_u_px ||
            e.max_error_deg < 0 || e.max_error_deg > 1) {
            error = "invalid_eye_parameters_" + std::to_string(i); return false;
        }
        if (c.enabled && !e.calibrated) { error = "eye_not_calibrated_" + std::to_string(i); return false; }
        if (e.calibrated && (e.fx_px <= 0 || e.fx_px > 100000 ||
            e.cx_px <= e.min_u_px || e.cx_px >= e.max_u_px || e.max_u_px-e.min_u_px < 160)) {
            error = "invalid_intrinsics_" + std::to_string(i); return false;
        }
    }
    return true;
}
bool loadBearingConfig(const std::string &path, BearingConfig &config, std::string &error) {
    config = BearingConfig(); error.clear();
    int fd = open(path.c_str(), O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);
    if (fd < 0) { error = "open_geometry: " + std::string(std::strerror(errno)); return false; }
    struct stat st{};
    if (fstat(fd,&st) || !S_ISREG(st.st_mode) || st.st_size <= 0 || st.st_size > 16384) {
        close(fd); error = "invalid_geometry_file"; return false;
    }
    std::string data(static_cast<size_t>(st.st_size),'\0'); size_t at = 0;
    while (at < data.size()) {
        ssize_t n = read(fd,&data[at],data.size()-at);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) { close(fd); error = "read_geometry_failed"; return false; }
        at += static_cast<size_t>(n);
    }
    close(fd);
    try {
        picojson::value root; std::string parse_error;
        auto end = picojson::parse(root,data.data(),data.data()+data.size(),&parse_error);
        while (end != data.data()+data.size() && (*end==' ' || *end=='\t' || *end=='\r' || *end=='\n')) ++end;
        if (!parse_error.empty() || end != data.data()+data.size() || !root.is<picojson::object>()) {
            error = "invalid_geometry_json"; return false;
        }
        const auto &o = root.get<picojson::object>(); BearingConfig next; double schema=0,width=0;
        auto eyes = field(o,"eyes");
        if (!number(o,"schema",schema) || schema != 1 ||
            !number(o,"source_eye_width_px",width) || width != 960 ||
            !boolean(o,"enabled",next.enabled) || !number(o,"lookahead_m",next.lookahead_m) ||
            !eyes || !eyes->is<picojson::array>() || eyes->get<picojson::array>().size() != 2) {
            error = "invalid_geometry_schema"; return false;
        }
        for (int i=0; i<2; ++i) {
            const auto &v=eyes->get<picojson::array>()[i];
            if (!v.is<picojson::object>()) { error="invalid_eye_object"; return false; }
            const auto &e=v.get<picojson::object>(); auto &n=next.eyes[i];
            if (!boolean(e,"calibrated",n.calibrated) || !number(e,"fx_px",n.fx_px) ||
                !number(e,"cx_px",n.cx_px) || !number(e,"yaw_right_deg",n.yaw_right_deg) ||
                !number(e,"lateral_right_m",n.lateral_right_m) || !number(e,"min_u_px",n.min_u_px) ||
                !number(e,"max_u_px",n.max_u_px) || !number(e,"max_error_deg",n.max_error_deg)) {
                error="missing_eye_parameters_"+std::to_string(i); return false;
            }
        }
        if (!validateBearingConfig(next,error)) return false;
        config=next; return true;
    } catch (...) { error="geometry_parse_exception"; return false; }
}
BearingResult targetBearing(const BearingConfig &c,int eye,double u,double yaw_change_left_deg) {
    BearingResult r; r.u_px=std::isfinite(u)?u:0;
    if (!c.enabled) { r.reason="follow_geometry_disabled"; return r; }
    std::string error;
    if (!validateBearingConfig(c,error)) { r.reason=error; return r; }
    if (eye<0 || eye>1) { r.reason="invalid_camera_eye"; return r; }
    const auto &e=c.eyes[eye];
    if (!std::isfinite(u) || !std::isfinite(yaw_change_left_deg) || std::fabs(yaw_change_left_deg)>180) {
        r.reason="nonfinite_or_invalid_bearing_input"; return r;
    }
    if (u<e.min_u_px || u>e.max_u_px) { r.reason="outside_calibrated_span"; return r; }
    r.camera_deg=std::atan((u-e.cx_px)/e.fx_px)*degrees+e.yaw_right_deg;
    // A leftward turn since capture moves the same fixed target to image right.
    double current=r.camera_deg+yaw_change_left_deg;
    if (std::fabs(r.camera_deg)>=75 || std::fabs(current)>=75) { r.reason="bearing_too_large"; return r; }
    // The lookahead is a configured virtual forward plane, never a measured depth.
    // This is an approximate lateral-offset correction, not stereo reconstruction.
    r.turn_deg=std::atan(std::tan(current/degrees)+e.lateral_right_m/c.lookahead_m)*degrees;
    if (!std::isfinite(r.turn_deg) || std::fabs(r.turn_deg)>=75) { r.reason="turn_too_large"; return r; }
    r.valid=true; r.reason="virtual_lookahead"; return r;
}
}
