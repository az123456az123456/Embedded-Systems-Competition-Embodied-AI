#include "acg138_fpga_map_bridge/rear_clearance.hpp"
#include "acg138_fpga_map_bridge/nav3_wire.hpp"
extern "C" {
#include "/home/tan/qiansai/fpga/robot_navigation_v53_obstacle_reverse/source/common/nav_exec_wire.h"
}
#include <iostream>
#include <cmath>
#include <cstring>
#include <random>
int main(){using namespace acg138;namespace n=nav3;unsigned failures=0,count=0;auto ck=[&](const char*name,bool ok){++count;failures+=!ok;std::cout<<"{\"case\":\""<<name<<"\",\"pass\":"<<(ok?"true":"false")<<"}\n";};
 constexpr double pi=3.141592653589793;std::vector<float> scan(1440,1.0f);auto q=rear_clearance(scan,0,pi/720,.05,12,0);ck("observed_rear_clear",q.observed&&q.travel_mm>=600);
 for(int i=0;i<1440;++i){scan[i]=std::numeric_limits<float>::infinity();}q=rear_clearance(scan,0,pi/720,.05,12,0);ck("no_return_not_free",!q.observed&&!q.travel_mm);
 scan.assign(1440,1.0f);for(int i=700;i<736;++i)scan[i]=NAN;q=rear_clearance(scan,0,pi/720,.05,12,0);ck("one_blind_sector_not_free",!q.observed);
 scan.assign(1440,1.0f);scan[720]=.3f;q=rear_clearance(scan,0,pi/720,.05,12,0);ck("near_rear_obstacle_blocks",q.observed&&!q.travel_mm);
 scan.assign(1440,1.0f);scan[0]=.3f;q=rear_clearance(scan,0,pi/720,.05,12,pi);ck("sensor_yaw_180_selects_real_rear",q.observed&&!q.travel_mm);
 n::Header h;h.opcode=0x31;h.session_id=17;h.transport_seq=1;h.run_generation=2;h.goal_id[0]=1;h.plan_id=h.map_epoch=h.map_revision=h.localization_epoch=1;h.payload_bytes=60;
 n::Payload p{};ne_put32(p.data(),200);ne_put32(p.data()+4,1);ne_put32(p.data()+12,1);ne_put64(p.data()+16,1);ne_put64(p.data()+24,1000000000);ne_put32(p.data()+32,1);ne_put32(p.data()+44,350);ne_put64(p.data()+48,1);ne_put32(p.data()+56,3);
 n::Wire wire{};ck("CXX_rear_encode",n::encode(h,p,wire)==n::Error::Ok);ne_message m{};ck("FPGA_C_rear_decode",ne_decode(wire.data(),258,&m)==0&&m.h.opcode==NE_REAR_CLEAR);unsigned char encoded[258];ck("byte_exact_round_trip",ne_encode(&m.h,m.payload,encoded)==0&&!std::memcmp(encoded,wire.data(),258));
 scan.assign(500,1.0f);for(unsigned i=0;i<500;i+=5)scan[i]=NAN;
 q=rear_clearance(scan,-pi,2*pi/500,.05,12,0);ck("actual_500_bin_C1_with_sparse_dropouts",q.observed&&q.travel_mm>=600);
 scan.assign(500,1.0f);for(unsigned i=0;i<7;++i)scan[i]=NAN;
 q=rear_clearance(scan,-pi,2*pi/500,.05,12,0);ck("four_degree_blind_sector_refuses_rear",!q.observed);
 std::mt19937 gen(20261103);unsigned mismatches=0;
 for(unsigned i=0;i<2048;++i){auto b=wire;unsigned k=gen()%256;b[k]^=1u<<(gen()%8);ne_put16(b.data()+256,ne_crc16(b.data(),256));ne_message cm{};n::Message cp{};bool a=ne_decode(b.data(),258,&cm)==0;bool c=n::decode(b,cp)==n::Error::Ok;mismatches+=a!=c;}
 ck("2048_mutations_C_CXX_admission_agree",mismatches==0);
 std::cout<<"{\"summary\":true,\"cases\":"<<count<<",\"failed\":"<<failures<<"}\n";return failures?1:0;}
