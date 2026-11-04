// Deterministic scheduling of the actual service controlLoop. Link-time wraps
// stop it immediately before its first state_mutex acquisition; no production
// hook, socket, hardware, or probabilistic sleep is used to arrange the race.
#ifndef PHONE_SERVICE_UNDER_TEST_PATH
#define PHONE_SERVICE_UNDER_TEST_PATH "../phone/phone_service.cc"
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#include PHONE_SERVICE_UNDER_TEST_PATH
#pragma GCC diagnostic pop
#include <condition_variable>
#include <iostream>
#include <pthread.h>

namespace {
std::atomic<uint64_t> fake_now{1000};
std::mutex schedule_mutex;
std::condition_variable schedule_cv;
pthread_mutex_t *control_mutex=nullptr;
std::atomic<bool> *loop_stop=nullptr;
bool at_control_lock=false,allow_control_lock=false;
thread_local bool control_thread_under_test=false;
thread_local bool intercept_next_lock=false;
thread_local unsigned control_clock_calls=0;
std::atomic<uint64_t> loop_sample{0};
int checks=0;
void check(bool okay,const char *label){++checks;if(!okay){std::cerr<<"FAIL "<<label<<'\n';std::exit(1);}}
}
extern "C" int __real_pthread_mutex_lock(pthread_mutex_t *);
extern "C" int __real_pthread_mutex_unlock(pthread_mutex_t *);
extern "C" int __wrap_pthread_mutex_lock(pthread_mutex_t *mutex){
 if(control_thread_under_test&&intercept_next_lock&&mutex==control_mutex){
  std::unique_lock<std::mutex> lock(schedule_mutex);
  at_control_lock=true;schedule_cv.notify_all();
  schedule_cv.wait(lock,[]{return allow_control_lock;});
  intercept_next_lock=false;
 }
 return __real_pthread_mutex_lock(mutex);
}
extern "C" int __wrap_pthread_mutex_unlock(pthread_mutex_t *mutex){
 if(control_thread_under_test&&mutex==control_mutex)loop_stop->store(true);
 return __real_pthread_mutex_unlock(mutex);
}
extern "C" uint64_t __wrap_imu_clock_ms(){
 const uint64_t now=fake_now.load();
 if(control_thread_under_test&&++control_clock_calls==2)loop_sample=now;
 return now;
}

namespace {
struct CaseResult {State state;uint64_t sample;};
CaseResult runCase(const char *token,uint64_t resume_at,bool moving){
 phone_options options{};options.demo=1;options.left_mask=10;options.invert_mask=15;
 options.pair_token_file=token;options.camera_width=1920;options.camera_height=1080;
 options.camera_view="both";
 auto service=std::make_unique<phone_service>(options);
 service->controller.updateMotorLink(1000,true);
 service->controller.updateArcProtocol(true);service->controller.updateAngleProtocol(true);
 check(service->controller.arm(1,1000),"fixture arms actual controller");
 if(moving)check(service->controller.driveWheels(2,.2,.2,1000),"fixture establishes actual movement lease");
 fake_now=1000;loop_sample=0;at_control_lock=allow_control_lock=false;
 control_mutex=service->state_mutex.native_handle();loop_stop=&service->stopped;
 std::unique_lock<std::mutex> state_lock(service->state_mutex);
 service->stopped=false;
 std::thread worker([&]{control_thread_under_test=true;intercept_next_lock=true;control_clock_calls=0;service->controlLoop();control_thread_under_test=false;});
 {std::unique_lock<std::mutex> lock(schedule_mutex);schedule_cv.wait(lock,[]{return at_control_lock;});}
 // This is the real stateJson() used by GET /api/state, while holding the
 // service's actual state mutex. It advances the controller to 1027 ms.
 fake_now=1027;
 const std::string response=service->stateJson();
 check(response.find("\"armed\":true")!=std::string::npos,"newer HTTP state tick remains armed");
 check(service->controller.state().reason!="clock_reversed","HTTP tick is monotonic");
 fake_now=resume_at;state_lock.unlock();
 {std::lock_guard<std::mutex> lock(schedule_mutex);allow_control_lock=true;schedule_cv.notify_all();}
 worker.join();
 return {service->controller.state(),loop_sample.load()};
}
}
int main(int argc,char **argv){
 const bool legacy=argc==2&&std::string(argv[1])=="--expect-legacy";
 char dir[]="/tmp/acg138-clock-race-XXXXXX";check(mkdtemp(dir)!=nullptr,"temporary pairing fixture");
 const std::string token=std::string(dir)+"/pair-code";
 {std::ofstream file(token);file<<"123456\n";}
 // Repeated schedules are identical, not attempts to win a timing race.
 for(unsigned repeat=0;repeat<5;++repeat){
  auto idle=runCase(token.c_str(),1027,false);
  if(legacy){check(idle.sample==1000,"old loop captured time before blocked mutex");check(!idle.state.armed&&idle.state.reason=="clock_reversed","old actual loop falsely disarms healthy idle after HTTP tick");}
  else{check(idle.sample==1027,"fixed loop samples time after acquiring mutex");check(idle.state.armed&&idle.state.mode==Mode::Idle,"zero-speed healthy wait retains enable");}
  auto moving=runCase(token.c_str(),1027,true);
  if(legacy)check(!moving.state.armed&&moving.state.reason=="clock_reversed","old race also affects active lease");
  else check(moving.state.armed&&moving.state.reason=="navigation_wheels","fresh movement lease survives same schedule");
 }
 if(!legacy){
  auto expired=runCase(token.c_str(),1400,true);
  check(!expired.state.armed&&expired.state.reason=="lease_expired","actual loop still disarms expired movement lease");
  auto reversed=runCase(token.c_str(),990,false);
  check(!reversed.state.armed&&reversed.state.reason=="clock_reversed","actual loop still disarms genuine clock reversal");
 }
 // Actual acknowledgement freshness mechanism and Controller keep their
 // safety behavior; demo mode above deliberately requires no raw socket.
 MotorAckWindow window;window.noteSent(7,1000,10,15,1);
 MotorAck ack;ack.sequence=7;ack.left_mask=10;ack.invert_mask=15;ack.configured=true;ack.armed=true;ack.protocol=1;
 check(window.accept(ack,1005),"real ACK window accepts matching recent acknowledgement");
 Config config;config.left_motor_mask=10;
 Controller controller(config);controller.updateMotorLink(1005,window.fresh(1005));
 check(controller.arm(1,1005),"ACK-backed controller arms");
 check(window.fresh(1300),"ACK valid through its command's exact 300 ms boundary");
 check(!window.fresh(1301),"ACK expires after its command's 300 ms boundary");
 controller.updateMotorLink(1301,window.fresh(1301));controller.tick(1301);
 check(!controller.state().armed&&controller.state().reason=="motor_link_not_ready","expired ACK still disarms controller");
 unlink(token.c_str());rmdir(dir);
 std::cout<<"PASS actual control-loop clock schedule "<<(legacy?"legacy reproducer":"fixed regression")<<": "<<checks<<" checks\n";
}
