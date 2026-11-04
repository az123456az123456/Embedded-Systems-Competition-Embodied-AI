#include "dashboard.h"
#include "display_transform.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
static unsigned area(const struct dashboard *d,int x,int y,int w,int h){unsigned hash=2166136261u;for(int yy=y;yy<y+h;yy++)for(int xx=x;xx<x+w;xx++)hash=(hash^d->pixels[yy*VIEW_W+xx])*16777619u;return hash;}
static void save(struct dashboard *d,const char *name){char path[128];snprintf(path,sizeof(path),"build/navigation-%s.ppm",name);assert(!dashboard_ppm(d,path));struct display_transform t;assert(!display_transform_init(&t,760,1280,1280,760,90));snprintf(path,sizeof(path),"build/navigation-%s-90.ppm",name);assert(!display_transform_ppm(&t,d->pixels,path));}
int main(void){
 struct dashboard *d=malloc(sizeof(*d)+32);assert(d);dashboard_init(d);memset((char *)d+sizeof(*d),0xa6,32);
 struct imu_state imu;imu_init(&imu);imu.have_packet=imu.initialized=1;imu.received_ms=10000;imu.sample.flags=1;imu.sample.accel[2]=1;
 d->vision.have_frame=1;d->vision.timestamp_ms=10000;for(int i=0;i<VISION_W*VISION_H;i++)d->vision.pixels[i]=0xff82a9b3;
 struct sensor_sample sensors={.sequence=1,.uptime_ms=50,.window_ms=50,.valid_mask=7,.distance_mm={1953,600,4500}};assert(sensor_update(&d->sensors,&sensors,10000));
 dashboard_render(d,&imu,10000,"");unsigned right=area(d,952,20,300,720),sonar=area(d,48,639,864,82);save(d,"camera");
 assert(dashboard_action(110,65)==5&&dashboard_action(250,65)==6&&dashboard_action(10,10)==4);
 /* Apply the same inverse transform used by both pointer and touch handlers. */
 for(int rotation=0;rotation<=90;rotation+=90){struct display_transform t;double x=250,y=65,sx,sy;assert(!display_transform_init(&t,rotation?760:1280,rotation?1280:760,1280,760,rotation));double px=rotation?760-y:y;double py=rotation?x:y;if(!rotation)px=x;assert(display_transform_point(&t,px,py,&sx,&sy));assert(dashboard_action(sx,sy)==6);}
 d->navigation_selected=1;dashboard_render(d,&imu,10000,"");save(d,"waiting");unsigned waiting=area(d,48,137,864,486);
 struct navigation_map *m=&d->navigation;m->loaded=m->available=m->pose_valid=1;m->width=m->height=64;m->resolution_m=.1;m->updated_monotonic_ms=10000;m->robot_radius_m=.18;m->origin[0]=m->origin[1]=-3.2;strcpy(m->session_id,"test-session");memset(m->cells,'?',4096);
 for(int y=8;y<55;y++)for(int x=8;x<55;x++)m->cells[y*64+x]='.';
 for(int y=9;y<49;y++)m->cells[y*64+44]='#';
 for(int x=20;x<44;x++)m->cells[48*64+x]='#';
 m->path_count=4;double path[4][2]={{0,0},{-1.5,0},{-1.5,2},{.7,2}};memcpy(m->path,path,sizeof(path));m->trajectory_count=3;double track[3][2]={{-1.2,-.8},{-.3,-.8},{0,0}};memcpy(m->trajectory,track,sizeof(track));m->has_goal=1;m->goal[0]=.7;m->goal[1]=2;m->pose[2]=.4;
 dashboard_render(d,&imu,10000,"");save(d,"live");assert(area(d,48,137,864,486)!=waiting);assert(area(d,952,20,300,720)==right&&area(d,48,639,864,82)==sonar);
 /* Lower-left is y=0; a marked upper/right wall must not be flipped. */
 assert(d->pixels[(149+112)*VIEW_W+264+257]==0xff35404b);
 unsigned live=area(d,48,137,864,486);dashboard_render(d,&imu,12001,"");save(d,"stale");assert(area(d,48,137,864,486)!=live);
 m->available=0;dashboard_render(d,&imu,10000,"");save(d,"unavailable");assert(area(d,48,137,864,486)!=live);
 m->available=1;m->pose_valid=0;dashboard_render(d,&imu,10000,"");save(d,"invalid-pose");
 d->navigation_selected=0;dashboard_render(d,&imu,10000,"");assert(d->pixels[137*VIEW_W+48]==0xff82a9b3);assert(d->pixels[622*VIEW_W+911]==0xff82a9b3);
 for(int i=0;i<32;i++)assert(((unsigned char *)d)[sizeof(*d)+i]==0xa6);
 puts("PASS navigation UI: camera/tab roundtrip, y-up map, waiting/live/stale/invalid pose, 0/90 touch coordinates, preserved IMU/acceleration/sonar and uncropped camera, bounds");free(d);
}
