#include "nav_track.h"
#include "nav_math.h"

void nav_track_init(struct nav_track *t)
{t->nav_epoch=0;t->plan=0;t->last_ms=0;t->speed_mmps=0;t->initialized=0;t->final_leg=0;t->stopping=0;}
static int point(const struct nav_exec *e,unsigned index,int32_t *x,int32_t *y,uint16_t *cell)
{
 if(!astar_path_get(&e->server->planner,index,cell))return 0;
 *x=e->active.origin_x_mm+(int32_t)((*cell%64u)*100u+50u);
 *y=e->active.origin_y_mm+(int32_t)((*cell/64u)*100u+50u);return 1;
}
static int free_cell(const struct astar_context *p,int x,int y)
{return !astar_is_occupied(p,(unsigned)x,(unsigned)y);}
static int free_touch(const struct astar_context *p,int x,int y,int edge_x,int edge_y)
{return free_cell(p,x,y)&&(!edge_x||free_cell(p,x-1,y))&&(!edge_y||free_cell(p,x,y-1))&&(!(edge_x&&edge_y)||free_cell(p,x-1,y-1));}
int nav_track_line_clear(const struct astar_context *p,const struct nv_map *m,
                         int32_t x0,int32_t y0,int32_t x1,int32_t y1)
{
 uint16_t a,b;int x,y,ex,ey,sx,sy,edge_x,edge_y;uint32_t dx,dy,nx,ny,steps=0;
 if(!nav_exec_cell(m,x0,y0,&a)||!nav_exec_cell(m,x1,y1,&b))return 0;
 x=a%64u;y=a/64u;ex=b%64u;ey=b/64u;
 sx=x1>x0?1:-1;sy=y1>y0?1:-1;
 dx=x1>=x0?(uint32_t)x1-(uint32_t)x0:(uint32_t)x0-(uint32_t)x1;
 dy=y1>=y0?(uint32_t)y1-(uint32_t)y0:(uint32_t)y0-(uint32_t)y1;
 edge_x=!dx&&((uint32_t)x0-(uint32_t)m->origin_x_mm)%100u==0;
 edge_y=!dy&&((uint32_t)y0-(uint32_t)m->origin_y_mm)%100u==0;
 if(!free_touch(p,x,y,edge_x,edge_y))return 0;
 /* Distances to the first crossed grid boundary, in local mm. A negative ray
  * beginning exactly on a boundary immediately touches the previous cell. */
 nx=sx>0?100u-((uint32_t)x0-(uint32_t)m->origin_x_mm)%100u:((uint32_t)x0-(uint32_t)m->origin_x_mm)%100u;
 ny=sy>0?100u-((uint32_t)y0-(uint32_t)m->origin_y_mm)%100u:((uint32_t)y0-(uint32_t)m->origin_y_mm)%100u;
 while(x!=ex||y!=ey){
  uint32_t vx=nx*dy,vy=ny*dx;
  if(++steps>128u)return 0;
  if(!dy||(dx&&vx<vy)){x+=sx;nx+=100u;}
  else if(!dx||vx>vy){y+=sy;ny+=100u;}
  else{
   if(!free_cell(p,x+sx,y)||!free_cell(p,x,y+sy))return 0;
   x+=sx;y+=sy;nx+=100u;ny+=100u;
  }
  if(!free_touch(p,x,y,edge_x,edge_y))return 0;
 }
 return 1;
}
static int within(int32_t dx,int32_t dy)
{return dx*dx+dy*dy<=6400;}
static void output(struct nav_exec *e,struct nav_track *t,uint32_t now,int32_t v,int32_t w)
{
 struct nv_intent i;i.nav_epoch=e->nav_epoch;i.run=e->task.run;i.plan=e->task.plan;
 i.pose_seq=e->guard.pose_seq;i.permit_seq=e->guard.permit_seq;i.linear_mmps=v;i.angular_mradps=w;
 if(!nav_exec_submit_intent(e,&i,now))t->speed_mmps=0;
}
static int hold(struct nav_exec *e,struct nav_track *t,uint32_t now,int result)
{t->speed_mmps=0;output(e,t,now,0,0);return result;}
/* Four-neighbour A* guarantees each remaining complete segment is100mm. The
 * final segment goes from the last cell centre to the actual requested goal.
 * Geometry is never inferred from encoder rotations. */
static int remaining(struct nav_exec *e,struct nav_track *t,int32_t x,int32_t y,uint32_t *distance)
{
 int32_t lx,ly;uint16_t cell;
 if(!point(e,e->path_count-1u,&lx,&ly,&cell))return 0;
 if(t->final_leg){*distance=nm_distance(e->goal_x_mm-e->pose.x_mm,e->goal_y_mm-e->pose.y_mm);return 1;}
 *distance=nm_distance(x-e->pose.x_mm,y-e->pose.y_mm)+
   (e->path_count-1u-e->cursor)*100u+nm_distance(e->goal_x_mm-lx,e->goal_y_mm-ly);return 1;
}
/* Stop at the first not-yet-reached corner, not at every collinear waypoint.
 * Looking ahead affects only braking; it never skips or changes the cursor. */
static int brake_distance(struct nav_exec *e,struct nav_track *t,uint32_t first,uint32_t *distance)
{
 unsigned k=e->cursor,ahead=0;uint16_t a,b,previous;int direction=0,next;int32_t lx,ly;
 *distance=first;if(t->final_leg)return 1;
 if(!astar_path_get(&e->server->planner,k,&a))return 0;
 if(k){if(!astar_path_get(&e->server->planner,k-1u,&previous))return 0;direction=(int)a-previous;}
 while(k+1u<e->path_count&&ahead++<8u){
  if(!astar_path_get(&e->server->planner,k+1u,&b))return 0;
  next=(int)b-a;
  if((next!=1&&next!=-1&&next!=64&&next!=-64)||((next==1||next==-1)&&a/64u!=b/64u))return 0;
  if(direction&&next!=direction)return 1;
  direction=next;*distance+=100u;a=b;++k;
 }
 if(k+1u<e->path_count)return 1;
 if(!point(e,e->path_count-1u,&lx,&ly,&a))return 0;
 *distance+=nm_distance(e->goal_x_mm-lx,e->goal_y_mm-ly);return 1;
}
int nav_track_update(struct nav_exec *e,struct nav_track *t,struct nv_stop *stop,
                     const struct nv_stop_sample *sample)
{
 uint32_t now,dist,brake,limit;uint16_t cell,current,goalcell;int32_t x,y,dx,dy,error,w,v;int fresh,due,corner_approach=0;
 if(!e||!t||!stop||!sample)return NV_TRACK_INVALID;
 now=sample->now_ms;
 nav_exec_poll(e,now);
 if(e->state!=NV_FOLLOWING||!nav_exec_motion_allowed(e,now)){nav_track_init(t);return NV_TRACK_WAIT;}
 if(!e->plan_ready||e->path_count!=astar_path_length(&e->server->planner)||e->planned_revision!=astar_path_revision(&e->server->planner)||e->cursor>=e->path_count)
  return hold(e,t,now,NV_TRACK_INVALID);
 if(!nav_exec_cell(&e->active,e->pose.x_mm,e->pose.y_mm,&current)||!nav_exec_cell(&e->active,e->goal_x_mm,e->goal_y_mm,&goalcell))
  return hold(e,t,now,NV_TRACK_OUTSIDE);
 fresh=!t->initialized||t->nav_epoch!=e->nav_epoch||t->plan!=e->task.plan;
 if(fresh){nav_track_init(t);t->initialized=1;t->nav_epoch=e->nav_epoch;t->plan=e->task.plan;t->last_ms=now-20u;t->final_leg=(uint8_t)(e->path_count==1u);}
 if(!point(e,e->cursor,&x,&y,&cell))return hold(e,t,now,NV_TRACK_INVALID);
 if(cell==goalcell&&t->final_leg){x=e->goal_x_mm;y=e->goal_y_mm;}
 dx=x-e->pose.x_mm;dy=y-e->pose.y_mm;
 /* Do not jump an unreached turn. Even within its80mm radius, retain this
  * waypoint if the proposed next leg would cut a blocked corner. */
 if(!t->final_leg&&within(dx,dy)){
  if(e->cursor+1u<e->path_count){
   int32_t nx,ny;uint16_t next;
   if(!point(e,e->cursor+1u,&nx,&ny,&next))return hold(e,t,now,NV_TRACK_INVALID);
   if(nav_track_line_clear(&e->server->planner,&e->active,e->pose.x_mm,e->pose.y_mm,nx,ny)){++e->cursor;x=nx;y=ny;cell=next;}
   else corner_approach=1;
  }else{t->final_leg=1;x=e->goal_x_mm;y=e->goal_y_mm;}
  dx=x-e->pose.x_mm;dy=y-e->pose.y_mm;
 }
 if(!nav_track_line_clear(&e->server->planner,&e->active,e->pose.x_mm,e->pose.y_mm,x,y))return hold(e,t,now,NV_TRACK_BLOCKED);
 if(!remaining(e,t,x,y,&e->remaining_mm))return hold(e,t,now,NV_TRACK_INVALID);
 if(t->final_leg&&within(e->goal_x_mm-e->pose.x_mm,e->goal_y_mm-e->pose.y_mm)){
  if(!t->stopping){t->stopping=1;nv_stop_begin(stop,now);}
  hold(e,t,now,NV_TRACK_STOPPING);
  if(nv_stop_update(stop,sample)&&nav_exec_arrive(e,now)==NE_OK)return NV_TRACK_ARRIVED;
  return NV_TRACK_STOPPING;
 }
 t->stopping=0;
 due=(uint32_t)(now-t->last_ms)>=20u;
 /* Recompute geometry from a newly admitted source before the PI release.
  * This refreshes a decision, never an already computed motor result. */
 if(!due&&e->intent.pose_seq==e->guard.pose_seq&&e->intent.permit_seq==e->guard.permit_seq)return NV_TRACK_WAIT;
 if(due)t->last_ms=now;
 dist=nm_distance(dx,dy);if(!brake_distance(e,t,dist,&brake))return hold(e,t,now,NV_TRACK_INVALID);
 error=nm_wrap(nm_atan2(dy,dx)-e->pose.yaw_urad);w=(error/1000)*2;if(w>1000)w=1000;if(w< -1000)w=-1000;
 v=error< -785398||error>785398?0:300-(300*(error<0?-error:error))/785398;
 limit=nm_isqrt(800u*(brake>(corner_approach?0u:80u)?brake-(corner_approach?0u:80u):0u));if((uint32_t)v>limit)v=(int32_t)limit;
 if(!v&&(error< -785398||error>785398))t->speed_mmps=0;
 else if(due){if(v>t->speed_mmps+5)t->speed_mmps+=5;
 else if(v<t->speed_mmps-8)t->speed_mmps-=8;
 else t->speed_mmps=(int16_t)v;}
 if(t->speed_mmps<0)t->speed_mmps=0;
 output(e,t,now,t->speed_mmps,w);return NV_TRACK_COMMAND;
}
