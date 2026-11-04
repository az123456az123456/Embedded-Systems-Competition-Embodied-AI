#include "nav_obstacle.h"
#include "nav_math.h"
static int floor100(int32_t x){return x<0?(x-99)/100:x/100;}
unsigned nav_obstacle_mark(struct nav_exec *e,unsigned sensor,uint16_t range_mm)
{
 int32_t bs,bc,dx,dy,bx,by,px,py,lx,ly,sx,sy;int x,y,x0,x1,y0,y1;uint16_t start=UINT16_MAX;unsigned changed=0;
 struct astar_context *p=&e->server->planner;
 if(sensor>2u||range_mm<30u||range_mm>1000u||!e->active.valid||astar_busy(p))return 0;
 if(!nav_exec_cell(&e->active,e->pose.x_mm,e->pose.y_mm,&start))return 0;
 /* Work in the validated local window, so a global origin near INT32 limits
  * cannot overflow when the sensor ray projects beyond an edge. */
 sx=(int32_t)((uint32_t)e->pose.x_mm-(uint32_t)e->active.origin_x_mm);
 sy=(int32_t)((uint32_t)e->pose.y_mm-(uint32_t)e->active.origin_y_mm);
 /* +150mm robot-radius inflation and50mm probe/mapping uncertainty. Distance
  * below is to the grid square, not just its centre; no corner holes. */
 nm_sincos(e->pose.yaw_urad,&bs,&bc);
 bx=sensor==0?145+(int32_t)range_mm:0;
 by=sensor==0?0:(sensor==1?113+(int32_t)range_mm:-113-(int32_t)range_mm);
 px=sx+(bx*bc-by*bs)/32768;
 py=sy+(bx*bs+by*bc)/32768;
 x0=floor100(px-250);x1=floor100(px+250);
 y0=floor100(py-250);y1=floor100(py+250);
 if(x0<0)x0=0;
 if(y0<0)y0=0;
 if(x1>63)x1=63;
 if(y1>63)y1=63;
 for(y=y0;y<=y1;++y)for(x=x0;x<=x1;++x){
  dx=x*100+50-px;dy=y*100+50-py;
  dx=dx<0?-dx:dx;dy=dy<0?-dy:dy;dx=dx>50?dx-50:0;dy=dy>50?dy-50:0;
  if(dx>200||dy>200||dx*dx+dy*dy>40000)continue;
  lx=x*100+50-sx;
  ly=y*100+50-sy;
  bx=(lx*bc+ly*bs)/32768;by=(-lx*bs+ly*bc)/32768;
  /* Inflation cannot imprison the physically occupied start footprint. This
   * skips ONLY new overlay inflation, never a committed or prior obstacle.
   * Whole-square inside body is deliberately stricter than centre inclusion. */
  {int32_t half=((bc<0?-bc:bc)+(bs<0?-bs:bs))*50/32768;
   if(bx-half>=-194&&bx+half<=159&&(by<0?-by:by)+half<=150)continue;}
  /* Do not let the uncertainty halo overwrite the vehicle origin. */
  if((unsigned)(y*64+x)==start)continue;
  changed+=(unsigned)astar_mark_dynamic(p,(unsigned)x,(unsigned)y);
 }
 return changed;
}
