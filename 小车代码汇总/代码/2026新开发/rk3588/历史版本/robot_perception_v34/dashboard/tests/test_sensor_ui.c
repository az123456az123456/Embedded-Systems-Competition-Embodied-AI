#include "dashboard.h"
#include "display_transform.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
static unsigned row_hash(const struct dashboard *d){
    unsigned hash=2166136261u;
    for(int y=639;y<721;y++)for(int x=48;x<912;x++)hash=(hash^d->pixels[y*VIEW_W+x])*16777619u;
    return hash;
}
static void camera_corners(const struct dashboard *d){
    const int x[]={0,VISION_W-1,0,VISION_W-1},y[]={0,0,VISION_H-1,VISION_H-1};
    for(unsigned i=0;i<4;i++)assert(d->pixels[(137+y[i])*VIEW_W+48+x[i]]==d->vision.pixels[y[i]*VISION_W+x[i]]);
}
static void save(struct dashboard *d,const char *name){
    char path[160];snprintf(path,sizeof(path),"build/v125-sonar-%s.ppm",name);assert(!dashboard_ppm(d,path));
    struct display_transform rotate;assert(!display_transform_init(&rotate,VIEW_H,VIEW_W,VIEW_W,VIEW_H,90));
    snprintf(path,sizeof(path),"build/v125-sonar-%s-90.ppm",name);assert(!display_transform_ppm(&rotate,d->pixels,path));
}
int main(void){
    struct dashboard *d=malloc(sizeof(*d)+32);assert(d);dashboard_init(d);
    memset((unsigned char *)d+sizeof(*d),0xa6,32);
    struct imu_state imu;imu_init(&imu);imu.initialized=imu.have_packet=1;imu.received_ms=10000;imu.sample.flags=1;imu.sample.accel[2]=1;
    for(int y=0;y<VISION_H;y++)for(int x=0;x<VISION_W;x++)
        d->vision.pixels[y*VISION_W+x]=0xff000000|((unsigned)(30+x/4)<<16)|((unsigned)(40+y/4)<<8)|90;
    const unsigned corners[]={0xffff5b55,0xff5bc674,0xff6c96ff,0xffffcf62};
    for(unsigned corner=0;corner<4;corner++)for(unsigned y=0;y<24;y++)for(unsigned x=0;x<24;x++)
        d->vision.pixels[((corner&2)?VISION_H-24+y:y)*VISION_W+((corner&1)?VISION_W-24+x:x)]=corners[corner];
    d->vision.have_frame=1;d->vision.timestamp_ms=10000;
    dashboard_render(d,&imu,10000,"unused");camera_corners(d);unsigned waiting=row_hash(d);save(d,"waiting");
    struct sensor_sample sample={.sequence=1,.uptime_ms=50,.window_ms=50,.valid_mask=7,.distance_mm={1953,600,4500}};
    assert(sensor_update(&d->sensors,&sample,10000));dashboard_render(d,&imu,10000,"unused");camera_corners(d);
    unsigned valid=row_hash(d);assert(valid!=waiting);save(d,"valid");
    /* Both actual valid measurements above 4 m share the same presentation.
       The exact 4 m boundary retains its numeric value. */
    d->sensors.sample.distance_mm[2]=4001;
    dashboard_render(d,&imu,10000,"unused");assert(row_hash(d)==valid);
    d->sensors.sample.distance_mm[2]=4000;
    dashboard_render(d,&imu,10000,"unused");assert(row_hash(d)!=valid);
    d->sensors.sample.distance_mm[2]=4500;
    sample.sequence++;sample.uptime_ms+=50;sample.valid_mask=2;sample.fault_mask=5;sample.age_ms[0]=UINT16_MAX;
    assert(sensor_update(&d->sensors,&sample,10050));dashboard_render(d,&imu,10050,"unused");camera_corners(d);
    assert(row_hash(d)!=waiting&&row_hash(d)!=valid);save(d,"mixed");
    d->vision.timestamp_ms=12000;imu.received_ms=12000;
    dashboard_render(d,&imu,12000,"unused");unsigned expired=row_hash(d);save(d,"expired");
    /* An expired row must not reveal any retained distance, including zero or
       the last good value. Camera continues with every corner unmodified. */
    d->sensors.sample.distance_mm[0]=30;d->sensors.sample.distance_mm[1]=4499;d->sensors.sample.distance_mm[2]=1000;
    dashboard_render(d,&imu,12000,"unused");assert(row_hash(d)==expired);camera_corners(d);
    for(unsigned i=0;i<32;i++)assert(((unsigned char *)d)[sizeof(*d)+i]==0xa6);
    puts("PASS sonar UI: metre distance row, independent mixed channel status, expired/unknown never display retained range, complete camera corners, normal/90 bounds and original three-card layout");
    free(d);return 0;
}
