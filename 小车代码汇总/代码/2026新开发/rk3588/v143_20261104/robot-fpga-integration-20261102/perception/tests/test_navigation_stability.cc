#include "navigation_viewport.h"
#include "navigation_enhance.h"
#include <cassert>
#include <cstring>
#include <iostream>
int main(){
 navigation_viewport v{};navigation_viewport_update(&v,-3.2,-3.2,6.4,"a",1000);
 assert(v.x==-3.2&&v.span==6.4);
 navigation_viewport_update(&v,-3.23,-3.2,6.43,"a",1010);
 assert(v.x==-3.2&&v.span==6.4); // No jump on map arrival.
 navigation_viewport_update(&v,-3.23,-3.2,6.43,"a",1160);
 assert(v.span>6.4&&v.span<v.ts);
 navigation_viewport_update(&v,-3.23,-3.2,6.43,"a",1310);
 double x=v.x,y=v.y,span=v.span;
 for(int i=0;i<100;i++){navigation_viewport_update(&v,-3.2+i%2*.03,-3.2,6.4,"a",1400+i*10);assert(v.x==x&&v.y==y&&v.span==span);}
 navigation_viewport_update(&v,-1,-1,2,"b",3000);assert(v.x==-1&&v.span==2);
 char cells[]="?????..?.#?..?.??????"; // Use explicit 5x5 below.
 (void)cells;const char *input="?????" "?...." "?.#.." "?...." "?????";
 uint8_t result[25];navigation_enhance(input,5,5,result);
 for(int i=0;i<25;i++){if(input[i]=='#')assert(result[i]==255);if(input[i]=='?')assert(result[i]==0);if(input[i]=='.')assert(result[i]>=55);}
 assert(result[2*5+3]>55); // Gaussian edge, original occupancy string unchanged.
 assert(input[12]=='#');
 std::cout<<"{\"viewport_growth_eased\":true,\"100_refreshes_stable\":true,\"new_session_reset\":true,\"obstacles_unknown_preserved\":true}\n";
}
