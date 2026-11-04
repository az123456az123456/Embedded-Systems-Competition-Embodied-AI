#pragma once
#include "navcore.h"
#include "protocol.h"
#include "vendor/picojson.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace robot_nav {
// Read-only diagnostics of an SNS1 snapshot. SNS1 retains the last ACCEPTED
// range after invalid UART output; it does not carry that output, checksum
// failures, or their cause. In particular, fpga_invalid does NOT mean no_echo,
// raw zero, raw FFFF, or an observation of free space beyond four metres.
// These diagnostics never feed the navigation validity or motion decisions.
inline std::string sonar_diagnostics_json(const navwire::Sensors& sample,
                                         bool have_sample, double received_s,
                                         double now_s, const Config& config) {
    picojson::object result;
    const bool host_time_ok = std::isfinite(received_s) && std::isfinite(now_s) &&
                              now_s >= received_s;
    const double packet_age_s = host_time_ok ? now_s - received_s : 0.;
    result["distance_semantics"] = picojson::value(std::string("last_accepted_observation_not_raw_uart"));
    result["packet_received"] = picojson::value(have_sample);
    if (have_sample && host_time_ok)
        result["packet_age_ms"] = picojson::value(packet_age_s * 1000.);
    else result["packet_age_ms"] = picojson::value();
    result["navigation_source_timeout_ms"] = picojson::value(config.sonar_timeout_s * 1000.);
    result["navigation_packet_timeout_ms"] = picojson::value(config.sensor_timeout_s * 1000.);
    const char* names[] = {"front", "left", "right"};
    picojson::array channels;
    for (unsigned i = 0; i < 3; ++i) {
        picojson::object channel;
        const bool reported_valid = have_sample && (sample.valid & (1U << i));
        const bool reported_fault = have_sample && (sample.fault & (1U << i));
        const bool source_age_known = have_sample && sample.age_ms[i] != UINT16_MAX;
        const bool retained_range_known = have_sample && sample.distance_mm[i] != 0 &&
                                           sample.distance_mm[i] != UINT16_MAX;
        // In v25, FFFF age is shared by startup/no observation and age
        // saturation after 65.535 s. A retained nonzero range distinguishes
        // the latter; do not erase that last observation from diagnostics.
        const bool source_age_saturated = have_sample && !source_age_known && retained_range_known;
        const double distance_m = sample.distance_mm[i] / 1000.;
        const bool in_range = retained_range_known && distance_m >= config.sonar_min_m &&
                              distance_m <= config.sonar_max_m;
        const double total_age_s = packet_age_s + sample.age_ms[i] / 1000.;
        std::string status;
        if (!have_sample) status = "not_received";
        else if (!host_time_ok) status = "host_time_invalid";
        else if (!source_age_known) status = source_age_saturated ? "source_expired" : "not_observed";
        else if (packet_age_s > config.sensor_timeout_s) status = "packet_expired";
        else if (total_age_s > config.sonar_timeout_s) status = "source_expired";
        else if (!reported_valid || reported_fault) status = "fpga_invalid";
        else if (!in_range) status = "outside_configured_range";
        else status = "valid";
        channel["channel"] = picojson::value(double(i));
        channel["name"] = picojson::value(std::string(names[i]));
        channel["valid"] = picojson::value(reported_valid);
        channel["fault"] = picojson::value(reported_fault);
        channel["status"] = picojson::value(status);
        channel["source_age_saturated"] = picojson::value(source_age_saturated);
        channel["navigation_usable"] = picojson::value(status == "valid");
        channel["last_accepted_mm"] = retained_range_known ?
            picojson::value(double(sample.distance_mm[i])) : picojson::value();
        channel["source_age_ms"] = source_age_known ?
            picojson::value(double(sample.age_ms[i])) : picojson::value();
        channel["source_age_at_least_ms"] = source_age_saturated ?
            picojson::value(double(UINT16_MAX)) : picojson::value();
        channel["total_age_ms"] = source_age_known && host_time_ok ?
            picojson::value(total_age_s * 1000.) : picojson::value();
        channels.emplace_back(channel);
    }
    result["channels"] = picojson::value(channels);
    return picojson::value(result).serialize();
}
} // namespace robot_nav
