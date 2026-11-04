// Production Gateway/HTTP servers. The backend is a bounded loopback fixture;
// no Wi-Fi radio, Android OS or FPGA execution is claimed by this test.
#define main previous_gateway_concurrency_main
#include "test_gateway_concurrency.cc"
#undef main

int main() {
 unsigned checks=0;auto check=[&](bool value){assert(value);++checks;};
 Backend backend;unsigned bp=port(),gp=port();backend.start(bp);
 const std::string token="/tmp/gateway-v30-wait-"+std::to_string(getpid());
 {std::ofstream out(token);out<<"012345\n";}
 std::atomic<unsigned> map_reads{0},state_reads{0},cancels{0};
 std::atomic<bool> waiting{true};
 robotnav::GatewayCallbacks cb;
 cb.navigation_command=[](const std::string&,double,double,std::string&){return true;};
 cb.navigation_cancel=[&](const std::string&){++cancels;};
 cb.navigation_state=[&]{++state_reads;return std::string("{\"can_start\":true,\"straight_assist_active\":true,\"state\":\"")+(waiting?"AVOID_WAIT":"CRUISE")+"\",\"reason\":\"waiting_front_clear\"}";};
 cb.navigation_map=[&]{++map_reads;return "{\"schema\":1,\"available\":true,\"revision\":123,\"cells\":[0,0,0]}";};
 robotnav::GatewayOptions o;o.backend_port=bp;o.port=gp;o.bind_address="127.0.0.1";o.token_file=token;
 robotnav::Gateway g(o,cb);std::string error;check(g.start(error));
 check(request(gp,"POST","/api/connect","{}").code==200);
 std::uint64_t sequence=0;
 auto control=[&](const std::string& op,const std::string& extra="") {return request(gp,"POST","/api/control","{\"session\":\"session\",\"seq\":"+std::to_string(++sequence)+",\"op\":\""+op+"\""+extra+"}");};
 check(control("arm").code==200);check(control("drive",",\"throttle\":0,\"steer\":0").code==200);
 check(g.requestNavigation("straight",0,0,error));const auto generation=g.generation();
 std::atomic<bool> done{false};std::atomic<unsigned> failed{0},responses{0};
 std::thread poller([&]{while(!done){g.pollState();std::this_thread::sleep_for(milliseconds(50));}});
 std::vector<std::thread> clients;
 for(unsigned c=0;c<4;++c)clients.emplace_back([&,c]{while(!done){auto r=request(gp,"GET",c%2?"/api/navigation/map":"/api/state");if(r.code==200)++responses;else ++failed;std::this_thread::sleep_for(milliseconds(8));}});
 const auto began=ms();std::uint64_t max_idle_latency=0;unsigned updates=0;
 // Forty seconds exceeds every existing phone/owner/preparation timeout many
 // times. No encoder progress and zero duty are intentionally held throughout.
 while(ms()-began<40000){
  const auto started=ms();check(control("idle").code==200);max_idle_latency=std::max(max_idle_latency,ms()-started);
  check(g.navigationActive());check(g.telemetry().armed);check(g.applyNav(0,0,generation));++updates;
  std::this_thread::sleep_for(milliseconds(50));
 }
 check(responses>300);check(!failed);check(max_idle_latency<200);check(cancels==0);
 // State changes are observations, never new authorization. Re-entering CRUISE
 // with a real current owner works; later reads without that owner cannot move.
 waiting=false;check(control("idle").code==200);check(g.applyNav(.1,.1,generation));
 std::this_thread::sleep_for(milliseconds(390));
 for(unsigned i=0;i<10&&g.navigationActive();++i)std::this_thread::sleep_for(milliseconds(10));
 check(!g.navigationActive());check(cancels>0);
 {std::lock_guard<std::mutex> lock(backend.mutex);check(!backend.armed&&backend.duty==0);}
 check(!g.applyNav(.2,.2,generation));
 done=true;poller.join();for(auto& t:clients)t.join();
 // A valid STOP while already stopped preserves the transport/session, but it
 // never keeps an old route capability usable. A new explicit ARM is needed.
 check(control("stop").code==200);check(request(gp,"GET","/api/state").code==200);
 check(control("arm").code==200);check(!g.navigationActive());check(control("stop").code==200);
 g.stop();backend.server.stop();unlink(token.c_str());
 std::printf("PASS %u real-HTTP WAIT liveness checks duration_ms=%llu zero_velocity_updates=%u concurrent_read_responses=%u max_idle_latency_ms=%llu: WAIT/zero speed retain session, reads never renew real owner, expired owner disarms, STOP and explicit ARM recover without reconnect\n",checks,(unsigned long long)(ms()-began),updates,responses.load(),(unsigned long long)max_idle_latency);
}
