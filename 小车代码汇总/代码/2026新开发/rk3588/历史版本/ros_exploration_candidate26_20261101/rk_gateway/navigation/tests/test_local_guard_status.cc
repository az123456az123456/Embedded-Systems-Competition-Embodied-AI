// Reuse the actual fixture builder; neither included main is executed.
#define main local_map_test_main_not_executed
#include "test_local_rotation_map.cc"
#undef main
#define main navigation_production_main_not_executed
#include "../main.cc"
#undef main
int main(){try{
 Fixture f("tests/fixtures/local-rotation-map-20261027.json");Shared shared(f.config,true);shared.nav=f.make();
 const double now=navwire::monotonicSeconds();shared.nav.frame_->timestamp_s=now;
 ck(shared.nav.grid().import_observations(f.cells,f.origin,now),"recorded TTL-classified snapshot imported at test observation time");
 ck(shared.nav.bypass_rotation_clear(wrap(f.pose.yaw+.012),now),"recorded map rotation is clear in production state");
 shared.nav.measured_wheel_mps_={.21,.20,.22,.20};shared.nav.measured_sample_dt_s_=.05;
 shared.nav.straight_assist_=true;shared.nav.last_motion_mps_=.30;shared.nav.command_.linear_mps=.10;shared.nav.command_.angular_rps=.02;
 shared.nav.stopped_requested_s_=now-.8;shared.nav.stationary_since_=now-.35;
 shared.have_sensors=true;shared.received_s=now;shared.sensors.valid=7;shared.sensors.distance_mm={4000,4000,4000};
 const std::string bytes=shared.status_locked(now);picojson::value j;ck(picojson::parse(j,bytes).empty(),"actual production status JSON parses");
 const auto& g=j.get("local_guard");ck(g.get("evaluated").get<bool>()&&!g.get("blocked").get<bool>(),"status reports actual last guard decision");
 ck(g.get("cell_x").get<double>()==-1&&g.get("cell_y").get<double>()==-1,"sonar-only guard does not fabricate map-cell authority");
 ck(g.get("reason").get<std::string>()=="fresh_sonar_rotation_clear_map_not_authority","guard decision names fresh sonar authority");
 ck(g.get("sample_mono_ms").get<double>()==now*1000.,"last check timestamp exposed, not implicit current authorization");
 ck(g.get("scope").get<std::string>().find("map_not_authority")!=std::string::npos,"diagnostics explicitly exclude map authority");
 const auto& m=j.get("motion_gate");ck(m.get("frame_available").get<bool>()&&m.get("source_usable").get<bool>(),"actual aligned frame source health represented");
 ck(m.get("measured_sample_dt_s").get<double>()==.05,"instantaneous SNS1 interval exposed independently of encoder fault window");
 ck(m.get("measured_wheel_mps").get<picojson::array>()[2].get<double>()==.22,"actual measured wheel speed exposed");
 ck(m.get("wheel_speed_semantics").get<std::string>().find("not_250ms")!=std::string::npos,"instantaneous speed cannot be mistaken for disagreement window");
 ck(m.get("safety_speed_mps").get<double>()==.30&&m.get("target_linear_mps").get<double>()==.10,"safety and command speeds distinguished");
 ck(m.get("front_stop_distance_m").get<double>()>.65&&m.get("front_resume_distance_m").get<double>()>.69,"unaltered dynamic .85reaction+.30latency stopping/resume thresholds visible");
 ck(m.get("front_blocked").get<bool>()&&!m.get("forward_clear").get<bool>(),"computed active-frame gates visible");
 ck(std::abs(m.get("stop_wait_elapsed_s").get<double>()-.8)<1e-8&&std::abs(m.get("stationary_elapsed_s").get<double>()-.35)<1e-8,"stop/stationary clock durations exposed");
 ck(j.get("sonar_m").get<picojson::array>()[0].get<double>()==4.0&&m.get("sonar_m").get<picojson::array>()[0].get<double>()==.41,"latest UI packet and actual aligned control-frame ranges not conflated");
 ck(m.get("range_source").get<std::string>()=="aligned_range_fallback","without an observed latest SNS1 sample the aligned fallback provenance is explicit");
 ck(j.get("avoidance_policy").get("turn_selection").get<std::string>()=="away_from_close_side_else_larger_sonar"&&!j.get("avoidance_policy").get("local_map_veto").get<bool>(),"production policy exposes away/larger-sonar and map-free local semantics");
 ck(bytes.size()+4600<16384,"full map evidence still fits state-edge diagnostic payload budget");
 std::cout<<"PASS v32 production guard/motion status JSON "<<checks<<" checks; no HTTP/server/motor IO\n";
 }catch(const std::exception&e){std::cerr<<"FAIL guard status "<<e.what()<<'\n';return 1;}}
