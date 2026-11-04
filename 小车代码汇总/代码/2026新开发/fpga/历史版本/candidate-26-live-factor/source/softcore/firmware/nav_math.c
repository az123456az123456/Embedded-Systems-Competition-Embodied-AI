#include "nav_math.h"
#include <limits.h>
static const int32_t angles[]={785398,463648,244979,124355,62419,31240,15624,7812,3906,1953,977,488,244,122,61,31,15,8,4,2,1};
uint32_t nm_isqrt(uint32_t n)
{
 uint32_t r=0,b=UINT32_C(1)<<30;
 while(b>n)b>>=2;
 while(b){if(n>=r+b){n-=r+b;r=(r>>1)+b;}else r>>=1;b>>=2;}return r;
}
uint32_t nm_distance(int32_t dx,int32_t dy)
{
 uint32_t x=dx<0?0u-(uint32_t)dx:(uint32_t)dx,y=dy<0?0u-(uint32_t)dy:(uint32_t)dy;
 if(x>32767u||y>32767u)return UINT32_MAX;
 return nm_isqrt(x*x+y*y);
}
int32_t nm_wrap(int32_t a){a%=NM_TAU_URAD;if(a>=NM_PI_URAD)a-=NM_TAU_URAD;if(a< -NM_PI_URAD)a+=NM_TAU_URAD;return a;}
int32_t nm_atan2(int32_t y,int32_t x)
{
 int32_t z=0,n;unsigned i;
 if(!x&&!y)return 0;
 /* The navigation contract bounds local coordinates to a 6.4 m window. */
 if(x>32767||x< -32767||y>32767||y< -32767)return 0;
 if(x<0){z=y>=0?NM_PI_URAD:-NM_PI_URAD;x=-x;y=-y;}
 x*=4096;y*=4096;
 for(i=0;i<sizeof(angles)/sizeof(angles[0]);++i){
  if(y>0){n=x+(y>>i);y-=x>>i;z+=angles[i];x=n;}
  else if(y<0){n=x-(y>>i);y+=x>>i;z-=angles[i];x=n;}
 }
 return nm_wrap(z);
}
void nm_sincos(int32_t yaw,int32_t *s,int32_t *c)
{
 int32_t x=652032874,y=0,z=nm_wrap(yaw),n,sign=1;unsigned i;
 if(!z){*s=0;*c=32768;return;}
 if(z== -NM_PI_URAD){*s=0;*c=-32768;return;}
 if(z==NM_HALF_PI_URAD){*s=32768;*c=0;return;}
 if(z== -NM_HALF_PI_URAD){*s=-32768;*c=0;return;}
 if(z>NM_HALF_PI_URAD){z-=NM_PI_URAD;sign=-1;}else if(z< -NM_HALF_PI_URAD){z+=NM_PI_URAD;sign=-1;}
 for(i=0;i<sizeof(angles)/sizeof(angles[0]);++i){
  if(z>=0){n=x-(y>>i);y+=x>>i;z-=angles[i];x=n;}
  else{n=x+(y>>i);y-=x>>i;z+=angles[i];x=n;}
 }
 x*=sign;y*=sign;x=(x+(x<0?-16384:16384))/32768;y=(y+(y<0?-16384:16384))/32768;if(x>32768)x=32768;if(x< -32768)x=-32768;if(y>32768)y=32768;if(y< -32768)y=-32768;
 *s=y;*c=x;
}
int32_t nm_project(int32_t dx,int32_t dy,int32_t yaw)
{
 int32_t s,c;if(dx>32767||dx< -32767||dy>32767||dy< -32767)return 0;
 nm_sincos(yaw,&s,&c);return (dx*c+dy*s)/32768;
}
