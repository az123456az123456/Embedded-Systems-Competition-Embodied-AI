#pragma once
#include "gateway.h"
#include "imu_alignment.h"
#include "navcore.h"
#include "protocol.h"
#include <deque>
#include <optional>
#include <string>

namespace robot_nav {
// Serialized by the caller's navigation/core mutex. No networking or motor I/O.
class NavigationInputs {
public:
    explicit NavigationInputs(Config config,double yaw_sign=1);
    void reset();
    void update_imu(const robotnav::GatewayTelemetry& sample,double now);
    void enqueue(const navwire::Sensors& sample,double kernel_received_s);
    void drain(Navigator& nav,double now);
    const std::string& wait_reason() const { return wait_; }
    const std::string& fault_reason() const { return fault_; }
    std::string diagnostics_json() const;
private:
    struct Pending {navwire::Sensors sample;double received_s;};
    Config config_;
    double yaw_sign_;
    ImuAlignment alignment_;
    std::deque<Pending> pending_;
    std::optional<double> initial_yaw_,host_offset_lower_;
    std::deque<std::pair<double,double>> host_anchors_;
    std::optional<uint32_t> last_sensor_sequence_;
    bool established_=false,notified_=false,startup_reset_pending_=false;
    std::string wait_="waiting_imu_source",fault_;
    uint64_t imu_accepted_=0,imu_duplicates_=0,imu_ignored_=0,sensor_queued_=0;
    uint64_t sensor_ingested_=0,startup_dropped_=0,sensor_ignored_=0,transport_waits_=0;
    void latch(const std::string& reason);
    void clear_startup_imu();
};
} // namespace robot_nav
