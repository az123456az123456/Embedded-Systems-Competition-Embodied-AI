#include "box_stability.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace box_stability;
int main(){
 Filter f;vision_object boxes[2]={{100,100,160,220,.8,1,0,"person"},{500,100,560,220,.8,1,1,"person"}};
 vision_display_object out[VISION_OBJECTS];
 f.update(1,1000,boxes,2,1050);assert(f.copy(out,128,1050)==2);
 // A complete empty observation is a true miss, not permission to clear display.
 f.update(2,1100,boxes,0,1150);assert(f.copy(out,128,1150)==2&&out[0].held);
 assert(out[0].source_ms==1000&&out[0].frame_id==1);
 auto moved=boxes[0];moved.left+=5;moved.right+=5;moved.score=.4;
 f.update(3,1200,&moved,1,1250);assert(f.copy(out,128,1250)==2);
 assert(out[0].object.left>100&&out[0].object.left<105&&!out[0].held);
 assert(out[0].object.score>.7&&out[1].held); // opposite eye persists
 // Repeated publication of exactly the same NPU result cannot renew a hold.
 f.update(3,1200,&moved,1,1750);assert(f.copy(out,128,1801)==1);
 assert(f.copy(out,128,2001)==0);
 f.update(4,2050,boxes,2,2100);assert(f.copy(out,128,2100)==2);
 f.update(5,2150,boxes,2,2200,false);assert(f.copy(out,128,2200)==0);
 f.update(6,2300,boxes,2,2300);f.update(7,2250,boxes,2,2250);assert(f.copy(out,128,2250)==0);
 // Duplicate-nearby objects never let one old track consume two observations.
 f.clear();boxes[1]=boxes[0];boxes[1].left+=35;boxes[1].right+=35;
 f.update(1,1000,boxes,2,1000);f.update(2,1100,boxes,2,1100);assert(f.copy(out,128,1100)==2);
 assert(out[0].source_index!=out[1].source_index);
 // Stale source input does not resurrect a box after its fixed retention limit.
 f.clear();f.update(1,1000,boxes,2,1801);assert(f.copy(out,128,1801)==0);
 std::puts("PASS box stability: dropouts, stereo persistence, smooth geometry/confidence, fixed source expiry, disable/reset, one-to-one display matching");
}
