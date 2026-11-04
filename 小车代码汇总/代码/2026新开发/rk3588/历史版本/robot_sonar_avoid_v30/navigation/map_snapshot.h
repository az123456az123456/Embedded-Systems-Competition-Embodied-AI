#ifndef ROBOT_NAV_MAP_SNAPSHOT_H
#define ROBOT_NAV_MAP_SNAPSHOT_H
#include "navcore.h"
#include <mutex>
#include <string>
#include <deque>

namespace robot_nav {
// Caller holds the navigation state lock. Grid rows start at world y=0;
// UI clients flip the vertical axis, never the world coordinates themselves.
std::string map_snapshot_json(const Navigator&, double now_s,
                              double sensor_age_s, bool reset_required,
                              const std::string& session_id);

// HTTP readers do not take the motor/control/navigation state locks or call
// the dashboard backend. Only a short immutable snapshot copy is synchronized.
class MapSnapshotCache {
public:
    void publish(std::string json, double now_s);
    std::string read(double now_s) const;
    bool validate_goal(const std::string& session, uint32_t revision,
                       double x, double y, double now_s, std::string& error) const;
private:
    mutable std::mutex mutex_;
    std::string json_;
    double updated_s_=0;
    std::deque<std::pair<double,std::string>> history_;
};
}
#endif
