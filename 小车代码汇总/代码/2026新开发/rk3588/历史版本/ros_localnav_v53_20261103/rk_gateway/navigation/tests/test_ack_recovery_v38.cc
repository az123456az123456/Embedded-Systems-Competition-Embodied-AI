// Reuse the real HTTP fixture, not Gateway internals or a fabricated ACK.
#define main previous_gateway_straight_main
#include "test_gateway_straight.cc"
#undef main
int main(){
 for(bool soft_wait:{false,true}){
  Fixture f;f.prepare();check(f.start().status==200);const auto generation=f.gateway->generation();check(f.gateway->navigationActive());
  {std::lock_guard<std::mutex> l(f.backend.mutex);f.backend.ack_protocol=soft_wait?4:1;f.backend.softcore_ready=!soft_wait;f.backend.softcore_reason=soft_wait?"softcore_waiting_calibration_or_fault":"ready";}
  f.gateway->pollState();const auto cancels=f.cancellations.load();
  for(unsigned k=0;k<5;++k){check(f.gateway->applyNav(.2,.1,generation));check(f.gateway->navigationActive()&&f.gateway->generation()==generation&&f.cancellations==cancels);std::lock_guard<std::mutex> l(f.backend.mutex);check(f.backend.armed&&f.backend.left==0&&f.backend.right==0);}
  {std::lock_guard<std::mutex> l(f.backend.mutex);f.backend.ack_protocol=4;f.backend.softcore_ready=true;f.backend.softcore_reason="ready";}
  f.gateway->pollState();check(f.gateway->applyNav(.2,.1,generation));
  {std::lock_guard<std::mutex> l(f.backend.mutex);check(f.backend.armed&&f.backend.left==.2&&f.backend.right==.1);}
  check(f.cancellations==cancels);check(f.control("stop").status==200);check(!f.gateway->applyNav(.2,.1,generation));
 }
 {
  Fixture f;f.prepare();check(f.start().status==200);const auto generation=f.gateway->generation();
  {std::lock_guard<std::mutex> l(f.backend.mutex);f.backend.ack_protocol=1;}
  f.gateway->pollState();check(f.gateway->applyNav(.2,.1,generation));
  std::this_thread::sleep_for(milliseconds(2020));f.gateway->pollState();
  check(!f.gateway->applyNav(.2,.1,generation));check(!f.gateway->navigationActive());
  {std::lock_guard<std::mutex> l(f.backend.mutex);check(f.backend.armed&&f.backend.left==0&&f.backend.right==0);}
  check(f.control("stop").status==200);
 }
 {
  Fixture f;f.prepare();check(f.start().status==200);const auto generation=f.gateway->generation();
  {std::lock_guard<std::mutex> l(f.backend.mutex);f.backend.softcore_ready=false;f.backend.softcore_reason="softcore_cpu_trap";}
  f.gateway->pollState();check(!f.gateway->applyNav(.2,.1,generation));check(!f.gateway->navigationActive());
  {std::lock_guard<std::mutex> l(f.backend.mutex);check(f.backend.armed&&f.backend.left==0&&f.backend.right==0);}
  check(f.control("stop").status==200);
 }
 std::printf("PASS %u v38 real HTTP ACK recovery checks: ACK1/readiness transient sends zero, intent retained, ACK4 resumes, bounded2s and CPU fault stop motion, explicit STOP wins\n",checks);
}
