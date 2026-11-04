#include "../sonar_diagnostics.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + " " #x); } while (0)
using Object = picojson::object;
Object diagnostic(const navwire::Sensors& s, bool have = true, double received = 100., double now = 100.008) {
    const auto original = s;
    const auto text = robot_nav::sonar_diagnostics_json(s, have, received, now, robot_nav::Config{});
    picojson::value value;
    CHECK(picojson::parse(value, text).empty());
    CHECK(value.is<Object>());
    CHECK(original.valid == s.valid && original.fault == s.fault && original.distance_mm == s.distance_mm && original.age_ms == s.age_ms);
    return value.get<Object>();
}
Object channel(const Object& root, unsigned index) {
    return root.at("channels").get<picojson::array>().at(index).get<Object>();
}
bool close(double a, double b) { return std::abs(a - b) < 1e-7; }
navwire::Sensors sample() {
    navwire::Sensors s; s.valid = 7; s.distance_mm = {{1349, 1024, 2322}}; s.age_ms = {{191, 96, 143}};
    return s;
}
void test_actual_reported_invalid_with_fresh_retained_range() {
    auto s = sample(); s.valid = 6; s.fault = 1;
    const auto root = diagnostic(s);
    CHECK(root.at("distance_semantics").get<std::string>() == "last_accepted_observation_not_raw_uart");
    const auto front = channel(root, 0);
    CHECK(front.at("name").get<std::string>() == "front");
    CHECK(front.at("status").get<std::string>() == "fpga_invalid");
    CHECK(!front.at("valid").get<bool>() && front.at("fault").get<bool>());
    CHECK(!front.at("navigation_usable").get<bool>());
    CHECK(front.at("last_accepted_mm").get<double>() == 1349);
    CHECK(front.at("source_age_ms").get<double>() == 191);
    CHECK(close(front.at("total_age_ms").get<double>(), 199));
    for (unsigned i : {1, 2}) {
        const auto c = channel(root, i);
        CHECK(c.at("status").get<std::string>() == "valid");
        CHECK(c.at("navigation_usable").get<bool>());
    }
    s.valid = 3; s.fault = 4; s.distance_mm = {{1456, 1516, 2348}}; s.age_ms = {{16, 46, 189}};
    const auto right = channel(diagnostic(s, true, 100, 100.007), 2);
    CHECK(right.at("name").get<std::string>() == "right");
    CHECK(right.at("status").get<std::string>() == "fpga_invalid");
    CHECK(right.at("last_accepted_mm").get<double>() == 2348);
    CHECK(close(right.at("total_age_ms").get<double>(), 196));
}
void test_missing_source_packet_and_clocks() {
    auto s = sample();
    auto root = diagnostic(s, false);
    CHECK(!root.at("packet_received").get<bool>() && root.at("packet_age_ms").is<picojson::null>());
    for (unsigned i = 0; i < 3; ++i) {
        const auto c = channel(root, i);
        CHECK(c.at("status").get<std::string>() == "not_received");
        CHECK(c.at("last_accepted_mm").is<picojson::null>());
        CHECK(c.at("source_age_ms").is<picojson::null>());
        CHECK(c.at("total_age_ms").is<picojson::null>());
        CHECK(!c.at("navigation_usable").get<bool>());
    }
    s.age_ms[0] = UINT16_MAX;
    s.distance_mm[0] = 0;
    auto c = channel(diagnostic(s), 0);
    CHECK(c.at("status").get<std::string>() == "not_observed");
    CHECK(c.at("last_accepted_mm").is<picojson::null>() && c.at("source_age_ms").is<picojson::null>());
    CHECK(!c.at("source_age_saturated").get<bool>());
    s.distance_mm[0] = 1451;
    c = channel(diagnostic(s), 0);
    CHECK(c.at("status").get<std::string>() == "source_expired");
    CHECK(c.at("last_accepted_mm").get<double>() == 1451);
    CHECK(c.at("source_age_saturated").get<bool>());
    CHECK(c.at("source_age_at_least_ms").get<double>() == 65535);
    CHECK(c.at("source_age_ms").is<picojson::null>() && c.at("total_age_ms").is<picojson::null>());
    for (double bad : {99., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        root = diagnostic(sample(), true, 100., bad); c = channel(root, 0);
        CHECK(c.at("status").get<std::string>() == "host_time_invalid");
        CHECK(!c.at("navigation_usable").get<bool>() && c.at("total_age_ms").is<picojson::null>());
        CHECK(root.at("packet_age_ms").is<picojson::null>());
    }
}
void test_distinguish_expiry_from_fpga_invalid() {
    auto s = sample(); s.valid = 6; s.fault = 1; s.age_ms[0] = 900;
    auto c = channel(diagnostic(s), 0);
    CHECK(c.at("status").get<std::string>() == "source_expired");
    CHECK(c.at("fault").get<bool>() && !c.at("valid").get<bool>());
    c = channel(diagnostic(s, true, 100., 100.251), 0);
    CHECK(c.at("status").get<std::string>() == "packet_expired");
    s = sample(); s.age_ms.fill(500);
    c = channel(diagnostic(s, true, 100., 100.25), 0);
    CHECK(c.at("status").get<std::string>() == "valid");
    c = channel(diagnostic(s, true, 100., 100.25001), 0);
    CHECK(c.at("status").get<std::string>() == "packet_expired");
    s.age_ms[0] = 743;
    c = channel(diagnostic(s, true, 100., 100.008), 0);
    CHECK(c.at("status").get<std::string>() == "source_expired");
}
void test_far_is_only_valid_actual_range() {
    auto s = sample();
    for (uint16_t mm : {uint16_t(30), uint16_t(4000), uint16_t(4001), uint16_t(4500)}) {
        s.distance_mm[0] = mm;
        const auto c = channel(diagnostic(s), 0);
        CHECK(c.at("status").get<std::string>() == "valid");
        CHECK(c.at("last_accepted_mm").get<double>() == mm);
    }
    for (uint16_t mm : {uint16_t(0), uint16_t(29), uint16_t(4501), uint16_t(UINT16_MAX)}) {
        s.distance_mm[0] = mm;
        const auto c = channel(diagnostic(s), 0);
        CHECK(c.at("status").get<std::string>() == "outside_configured_range");
        CHECK(!c.at("navigation_usable").get<bool>());
        if (mm == 0 || mm == UINT16_MAX) CHECK(c.at("last_accepted_mm").is<picojson::null>());
    }
    s = sample(); s.valid = 7; s.fault = 4;
    CHECK(channel(diagnostic(s), 2).at("status").get<std::string>() == "fpga_invalid");
}
}
int main() {
    try {
        test_actual_reported_invalid_with_fresh_retained_range(); test_missing_source_packet_and_clocks();
        test_distinguish_expiry_from_fpga_invalid(); test_far_is_only_valid_actual_range();
        std::cout << "sonar diagnostics checks=" << checks << " PASS\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
