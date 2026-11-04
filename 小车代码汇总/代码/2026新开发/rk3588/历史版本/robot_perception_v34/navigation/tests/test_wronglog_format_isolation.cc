// Fail a producer-side allocation BEFORE the common C ABI. Logging must never
// turn that failure into a navigation/control exception or skip a real stop.
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <new>
#define private public
#include "../gateway.h"
#undef private
#define main navigation_main_not_executed
#include "../main.cc"
#undef main
#include "../gateway.cc"

static thread_local bool fail_next_allocation=false;
void* operator new(std::size_t n){if(fail_next_allocation){fail_next_allocation=false;throw std::bad_alloc();}if(void* p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void* p) noexcept {std::free(p);}
void operator delete[](void* p) noexcept {std::free(p);}
void operator delete(void* p,std::size_t) noexcept {std::free(p);}
void operator delete[](void* p,std::size_t) noexcept {std::free(p);}
namespace {
unsigned checks=0;
void check(bool value,const char* reason){++checks;if(!value)throw std::runtime_error(reason);}
template<class Function>void fail_log(Function function){fail_next_allocation=true;function();check(!fail_next_allocation,"injected formatting allocation was exercised");}
}
int main(){
    try{
        Config config;applyFixedProfile(config);config.configured=true;config.drive_mps_per_full_pwm=.6;Shared shared(config,true);
        const auto state=shared.nav.state(),reason=shared.nav.reason();const auto enabled=shared.nav.enabled();
        static_assert(noexcept(shared.log_event_locked("test","test")),"log event boundary");
        static_assert(noexcept(shared.log_snapshot()),"snapshot boundary");
        static_assert(noexcept(shared.observe_diagnostics_locked()),"transition boundary");
        static_assert(noexcept(runtime_log_config(config,1,true)),"config logging boundary");
        fail_log([&]{shared.log_event_locked("test","test");});
        fail_log([&]{shared.log_snapshot();});
        fail_log([&]{check(runtime_log_config(config,1,true).empty(),"failed config formatting has empty safe fallback");});
        fail_log([]{navigation_log_exit(SIGTERM);});
        fail_log([]{navigation_log_exception("A long diagnostic message that requires allocation");});
        check(shared.nav.state()==state&&shared.nav.reason()==reason&&shared.nav.enabled()==enabled,"formatting failures leave original navigator unchanged");
        shared.nav.invalidate_localization("test_sensor_fault",navwire::monotonicSeconds());fail_log([&]{shared.observe_diagnostics_locked();});
        check(!shared.nav.enabled()&&shared.nav.reason()=="test_sensor_fault","observation formatting cannot erase an actual fault");
        robotnav::Gateway::Impl gateway({},{});
        static_assert(noexcept(gateway.diagnostics()),"gateway snapshot formatting boundary");
        static_assert(noexcept(gateway.event("test","test")),"gateway event formatting boundary");
        const auto generation=gateway.generation_id;
        fail_log([&]{check(gateway.diagnostics().empty(),"gateway diagnostic fallback");});
        fail_log([&]{gateway.event("long_diagnostic_reason_requiring_an_allocation","test");});
        fail_log([&]{gateway.snapshot();});
        robotphone::JsonObject input;input["client_reason"]={robotphone::JsonValue::String,"app_background"};const std::string stop="stop";
        static_assert(noexcept(gateway.client_diagnostic(input,stop,1,"admitted")),"client diagnostic boundary");
        fail_log([&]{gateway.client_diagnostic(input,stop,1,"admitted");});
        input["op"]={robotphone::JsonValue::String,std::string(80,'z')};fail_log([&]{gateway.command_diagnostic_started(input);});
        robotnav::Reply reply;reply.status=409;reply.body="{\"error\":\"failure\"}";const std::string path="/api/navigation";
        fail_log([&]{gateway.rejected_reply_diagnostic(reply,path);});
        fail_log([&]{gateway.rejected_auth_diagnostic(401);});
        check(!gateway.active&&gateway.generation_id==generation&&gateway.control_epoch==1,"diagnostic failures cannot mutate gateway control state");
        gateway.active=true;fail_log([&]{gateway.invalidate("phone_disarm");});
        check(!gateway.active&&gateway.generation_id==generation+1&&gateway.cancel_reason=="phone_disarm","real invalidation still executes after logging allocation failure");
        std::printf("PASS %u producer formatting isolation checks: bad_alloc before C ABI is contained; original faults/state preserved; real disarm invalidation still executes\n",checks);return 0;
    }catch(const std::exception& e){fail_next_allocation=false;std::fprintf(stderr,"FAIL producer formatting isolation: %s\n",e.what());return 1;}
}
