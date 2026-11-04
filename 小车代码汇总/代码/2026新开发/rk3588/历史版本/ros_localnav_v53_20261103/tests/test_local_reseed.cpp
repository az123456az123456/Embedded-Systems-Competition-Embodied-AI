#include "../../../fpga/robot_navigation_v51_local_pose_sonar50/verification/stage3_plan_device.hpp"
struct LocalDevice:Device {
 void send(const n::Wire&w) override {ne_message m{};if(ne_decode(w.data(),258,&m))throw std::runtime_error("bad request");Device::send(w);
  if(m.h.opcode==NE_INFO&&!rx.empty()){auto&b=rx.back();ne_put32(b.data()+128,127);ne_put16(b.data()+256,ne_crc16(b.data(),256));}
 }
};
int main(){LocalDevice d;n::Client c(d,5);c.handshake(123,1,"map");c.upload(n::Bitmap{},{0,0,1});d.exec.state=NV_CANCELLED;d.exec.guard.pose_seq=900000;d.exec.guard.pose_source_stamp_ns=1000000000;d.exec.guard.pose_source_clock=1;d.exec.guard.pose_localization=1;d.state();c.pump();n::PoseSample p;p.x_mm=p.y_mm=50;p.source_stamp_ns=1100000000;bool ok=c.stream_pose(p)&&d.exec.guard.pose_seq==900001;std::cout<<"{\"case\":\"new_task_seed_exceeds_FPGA_local_ledger\",\"pass\":"<<(ok?"true":"false")<<",\"actual_seq\":"<<d.exec.guard.pose_seq<<"}\n";return ok?0:1;}
