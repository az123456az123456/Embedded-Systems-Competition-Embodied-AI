// Exercise the actual production Shared state and gates without entering main,
// constructing a network socket, starting a server, or issuing a motor command.
#define main navigation_production_main_not_executed
#include "../main.cc"
#undef main
#include <cassert>

int main(){
    Config c;applyFixedProfile(c);c.configured=true;c.drive_mps_per_full_pwm=.6;
    Shared shared(c,true);std::string error;
    assert(!shared.command("start",0,0,error)&&error=="verified_wheel_layout_required");
    robotnav::GatewayTelemetry t;t.available=true;t.link_ok=true;t.motor_ack_age_ms=10;t.left_mask=12;
    assert(shared.synchronize_wheel_layout(t));
    assert(shared.have_wheel_layout&&!shared.reset_required&&shared.origin_epoch==1);
    assert(shared.nav.config().left_wheel_mask==12);
    assert(!shared.synchronize_wheel_layout(t)&&shared.origin_epoch==1);
    auto old_status=shared.status();picojson::value status;assert(picojson::parse(status,old_status).empty());
    assert(status.get("encoder_ticks_per_turn").get<double>()==60527.7);
    assert(status.get("wheel_layout_verified").get<bool>());
    assert(status.get("left_wheel_mask").get<double>()==12);
    for(unsigned invalid:{0U,1U,7U,15U,16U}){t.left_mask=invalid;assert(!shared.synchronize_wheel_layout(t));assert(shared.nav.config().left_wheel_mask==12);}
    t.left_mask=10;t.motor_ack_age_ms=301;assert(!shared.synchronize_wheel_layout(t));
    t.motor_ack_age_ms=10;t.link_ok=false;assert(!shared.synchronize_wheel_layout(t));
    t.link_ok=true;t.available=false;assert(!shared.synchronize_wheel_layout(t));
    t.available=true;assert(shared.synchronize_wheel_layout(t));
    assert(shared.reset_required&&shared.origin_epoch==2&&!shared.have_sensors);
    assert(!shared.command("start",0,0,error)&&error.find("wheel_mapping_change")!=std::string::npos);
    assert(shared.command("reset-map",0,0,error)&&!shared.reset_required&&shared.origin_epoch==3);
    assert(shared.have_wheel_layout&&shared.nav.config().left_wheel_mask==10);
    assert(!shared.command("start",0,0,error)&&error=="waiting_imu_source");
    Shared observe(c,false);assert(observe.synchronize_wheel_layout(t));
    assert(!observe.command("start",0,0,error)&&error.find("observe_only")!=std::string::npos);
    Shared live(c,true);t.left_mask=12;assert(live.synchronize_wheel_layout(t));
    auto now=navwire::monotonicSeconds();robot_nav::SensorFrame f;f.timestamp_s=now-.05;f.imu_valid=true;f.sonar_m={2.,2.,2.};
    live.nav.ingest(f);f.timestamp_s=now;assert(live.nav.ingest(f));
    // Only live forward sonar observations: starting must not depend on a
    // manually observed circle behind/around the chassis.
    assert(live.start_problem(now).empty());
    assert(live.command("start",0,0,error)&&live.nav.enabled());
    assert(live.nav.tick(now).left==0);
    live.nav.stop();
    assert(live.nav.grid().import_observations(std::vector<int8_t>(4096,0),{-3.2,-3.2},now));
    assert(live.start_problem(now).empty());
    picojson::value ready;assert(picojson::parse(ready,live.status()).empty());
    assert(ready.get("can_start").get<bool>()&&ready.get("start_mode").get<std::string>()=="explore");
    assert(live.command("start",0,0,error)&&live.nav.enabled()&&!live.nav.goal());
    live.nav.stop();live.nav.set_goal(1.05,.05);assert(live.command("start",0,0,error));
    assert(live.nav.goal()&&live.nav.goal()->first==1.05);
    assert(live.command("stop",0,0,error)&&!live.nav.enabled());
    // Persistent disagreement needs explicit reset; preserve first context.
    Shared fault(c,true);fault.have_wheel_layout=true;
    robot_nav::SensorFrame bad;double stamp=navwire::monotonicSeconds();
    bad.timestamp_s=stamp;bad.device_timestamp_s=1.;bad.imu_valid=true;bad.sonar_m={2.,2.,2.};
    fault.nav.ingest(bad,stamp); // first frame establishes the count baseline
    robotnav::GatewayTelemetry before;before.available=true;before.armed=true;
    before.received_ms=uint64_t(stamp*1000);before.reason="manual_reverse";
    before.left_mask=10;before.invert_mask=15;before.left_duty=-.2;before.right_duty=-.2;
    before.ack_left_duty=-.2;before.ack_right_duty=-.2;before.motor_ack_age_ms=3;before.motor_ack_sequence=77;
    robot_nav::Control prior;prior.state="STOPPED";prior.reason="manual_takeover";
    for(unsigned i=1;i<=25&&!fault.nav.localization_diagnostics().latched;++i){
        bad.timestamp_s=stamp+i*.05;bad.device_timestamp_s=1.+i*.05;
        bad.counts[0]=uint32_t(-int32_t(std::llround(i*.05*.2*c.encoder_ticks_per_turn/c.wheel_circumference_m)));
        fault.nav.ingest(bad,bad.timestamp_s);
    }
    assert(fault.nav.localization_diagnostics().first_reason=="encoder_wheel_disagreement");
    fault.retain_fault_context_locked(before,prior,false,bad.timestamp_s);
    picojson::value failed;assert(picojson::parse(failed,fault.status()).empty());
    assert(failed.get("reset_required").get<bool>()&&!failed.get("can_start").get<bool>());
    assert(failed.get("readiness_reason").get<std::string>()=="encoder_wheel_disagreement");
    assert(failed.get("recovery_action").get<std::string>()=="stop_inspect_localization_then_reset_map");
    auto context=failed.get("localization_fault_context");
    assert(context.get("backend_reason").get<std::string>()=="manual_reverse");
    assert(context.get("ack_sequence").get<double>()==77);
    assert(!context.get("navigation_was_enabled").get<bool>());
    const auto original=fault.nav.localization_diagnostics().timestamp_s;
    before.reason="disarmed";before.armed=false;before.motor_ack_sequence=99;
    fault.nav.stop("operator_stop");fault.retain_fault_context_locked(before,fault.nav.command(),false,bad.timestamp_s+10.);
    assert(picojson::parse(failed,fault.status()).empty());
    assert(failed.get("localization_fault_context")==context);
    assert(fault.nav.localization_diagnostics().timestamp_s==original);
    assert(!fault.command("straight",0,0,error)&&error=="encoder_wheel_disagreement");
    assert(!fault.nav.enabled());
    picojson::value map;
    assert(picojson::parse(map,robot_nav::map_snapshot_json(fault.nav,bad.timestamp_s,0.,fault.needs_localization_reset(),"fault")).empty());
    assert(map.get("availability_reason").get<std::string>()=="reset_map_required");
    const unsigned epoch=fault.origin_epoch;
    assert(fault.command("reset-map",0,0,error));
    assert(!fault.needs_localization_reset()&&!fault.nav.enabled()&&fault.origin_epoch==epoch+1);
    assert(fault.localization_fault_context.is<picojson::null>());
    assert(fault.nav.trajectory().empty()&&!fault.nav.frame());
    assert(!fault.command("straight",0,0,error)); // Needs new aligned observations.
    Shared passive(c,true);robotnav::GatewayTelemetry diag;t.left_mask=10;
    diag.available=true;diag.link_ok=true;diag.motor_ack_age_ms=1;diag.left_mask=10;diag.invert_mask=15;
    diag.velocity_protocol_supported=true;diag.softcore_ready=false;diag.motor_ack_protocol=4;diag.received_ms=10000;
    diag.softcore_reason="softcore_cpu_trap";
    diag.softcore_diagnostics_json="{\"flags\":7,\"observed_imu_sequence\":1234,\"sample_mono_ms\":9900,\"heartbeat_advance_mono_ms\":9800}";
    assert(passive.synchronize_wheel_layout(diag));
    assert(picojson::parse(status,passive.status_locked(10.)).empty());
    auto core=status.get("softcore_control");assert(!core.get("ready").get<bool>());
    assert(core.get("reason").get<std::string>()=="softcore_cpu_trap");
    assert(core.get("diagnostics").get("packet_age_ms").get<double>()==100);
    assert(core.get("diagnostics").get("observed_imu_sequence").get<double>()==1234);
    assert(picojson::parse(status,passive.status_locked(11.)).empty());
    assert(!status.get("softcore_control").get("diagnostics").get("packet_fresh").get<bool>());
    assert(status.get("softcore_control").get("diagnostics").get("packet_age_ms").get<double>()==1100);
    std::cout<<"PASS production runtime: verified ACK grouping, startup baseline, stale/invalid ACK rejection, mapping-change map reset, control gates, passive NAV1 age and profile status\n";
}
