// Native execution of the unchanged production C filter, not a rewritten model.
#include <cstdint>
extern "C" {
#include "../softcore/firmware/layered_control.c"
}
extern "C" void kf_step(int now,int m0,int m1,int m2,int q0,int q1,int q2,int t0,int t1,int t2,int online,int*f0,int*f1,int*f2,int*s0,int*s1,int*s2,int*valid){
 static lc_state state;static bool initialized=false;lc_input in={};lc_output out={};
 if(!initialized){lc_init(&state);initialized=true;}
 in.now_ms=uint32_t(now);in.left_mask=10;in.flags=LC_CONFIGURED;in.sonar_online=uint8_t(online);
 in.sonar_mm[0]=uint16_t(m0);in.sonar_mm[1]=uint16_t(m1);in.sonar_mm[2]=uint16_t(m2);
 in.sonar_sequence[0]=uint32_t(q0);in.sonar_sequence[1]=uint32_t(q1);in.sonar_sequence[2]=uint32_t(q2);
 in.sonar_stamp_ms[0]=uint32_t(t0);in.sonar_stamp_ms[1]=uint32_t(t1);in.sonar_stamp_ms[2]=uint32_t(t2);
 lc_update(&state,&in,&out);*f0=out.sonar_mm[0];*f1=out.sonar_mm[1];*f2=out.sonar_mm[2];
 *s0=int(out.sonar_sequence[0]);*s1=int(out.sonar_sequence[1]);*s2=int(out.sonar_sequence[2]);*valid=int((out.flags>>8)&7);
}
