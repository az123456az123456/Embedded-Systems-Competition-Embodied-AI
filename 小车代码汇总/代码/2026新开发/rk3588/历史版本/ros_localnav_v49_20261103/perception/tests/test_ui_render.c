#define _POSIX_C_SOURCE 200809L
#include "dashboard.h"
#include "display_transform.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int in_rect(int x,int y,int left,int top,int w,int h) {
    return x>=left&&x<left+w&&y>=top&&y<top+h;
}
static int is_content(int x,int y) {
    return in_rect(x,y,28,20,904,723)||in_rect(x,y,952,20,300,403)||in_rect(x,y,952,440,300,303);
}
static void assert_restored_ui(const struct dashboard *d) {
    unsigned colored[3]={0};
    const unsigned colors[]={0x007aff,0x34a880,0xd58b2a};
    for(int y=0;y<VIEW_H;y++)for(int x=0;x<VIEW_W;x++) {
        unsigned p=d->pixels[y*VIEW_W+x];
        if(!is_content(x,y))assert(p==0xfff5f5f7);
        if(in_rect(x,y,952,440,300,300))
            for(int i=0;i<3;i++)if((p&0xffffff)==colors[i])colored[i]++;
    }
    for(int i=0;i<3;i++)assert(colored[i]>10);
}
static unsigned curve_hash(const struct dashboard *d) {
    unsigned sum=2166136261u;
    for(int axis=0;axis<3;axis++)for(int y=500+axis*73;y<554+axis*73;y++)
        for(int x=1131;x<1235;x++)sum=(sum^d->pixels[y*VIEW_W+x])*16777619u;
    return sum;
}
static void camera_pattern(struct vision_snapshot *v) {
    for(int y=0;y<VISION_H;y++)for(int x=0;x<VISION_W;x++) {
        unsigned r=35+x*130/VISION_W,g=55+y*110/VISION_H,b=65+(x+y)*60/(VISION_W+VISION_H);
        v->pixels[y*VISION_W+x]=0xff000000|(r<<16)|(g<<8)|b;
    }
    const unsigned colors[]={0xffff5b55,0xff5bc674,0xff6c96ff,0xffffcf62};
    for(int corner=0;corner<4;corner++)for(int dy=0;dy<28;dy++)for(int dx=0;dx<28;dx++) {
        int x=(corner&1)?VISION_W-28+dx:dx,y=(corner&2)?VISION_H-28+dy:dy;
        v->pixels[y*VISION_W+x]=colors[corner];
    }
}
static void camera_corners(const struct dashboard *d) {
    const int x[]={0,VISION_W-1,0,VISION_W-1},y[]={0,0,VISION_H-1,VISION_H-1};
    for(int i=0;i<4;i++)assert(d->pixels[(137+y[i])*VIEW_W+48+x[i]]==d->vision.pixels[y[i]*VISION_W+x[i]]);
}
int main(int argc,char **argv) {
    unsigned char *memory=malloc(sizeof(struct dashboard)+32);assert(memory);
    struct dashboard *d=(struct dashboard *)memory;
    dashboard_init(d);memset(memory+sizeof(*d),0xa7,32);
    struct imu_state imu;imu_init(&imu);
    /* Former nav/calibration/zero/close coordinates no longer trigger an action. */
    const double removed[][2]={{500,728},{700,728},{980,728},{1170,728},{1240,45}};
    for(size_t i=0;i<sizeof(removed)/sizeof(*removed);i++)assert(dashboard_action(removed[i][0],removed[i][1])==0);
    assert(dashboard_action(10,100)==4&&dashboard_action(940,400)==4);
    assert(dashboard_action(NAN,10)==0&&dashboard_action(20,INFINITY)==0);
    assert(dashboard_action(-1,100)==0&&dashboard_action(1280,100)==0);
    camera_pattern(&d->vision);
    imu.have_packet=imu.initialized=1;imu.received_ms=9999;imu.sample.flags=1;
    for(int i=0;i<120;i++) {
        imu.sample.accel[0]=sin(i*.15);imu.sample.accel[1]=cos(i*.12);
        imu.sample.accel[2]=1+sin(i*.19)*.35;dashboard_history(d,&imu,10000);
    }
    for(int state=0;state<10;state++) {
        imu.have_packet=state>0;imu.initialized=state>1;imu.received_ms=9999;
        imu.sample.flags=1;imu.sample.ag_age=0;
        imu.calibrating=state==2;imu.roll=-179.9;imu.pitch=89.9;imu.yaw=-135.4;
        imu.sample.accel[0]=-16;imu.sample.accel[1]=16;imu.sample.accel[2]=1;
        if(state==4){imu.yaw=NAN;imu.sample.accel[0]=INFINITY;}
        if(state==5){imu.roll=INFINITY;imu.sample.accel[1]=NAN;}
        if(state==6){imu.roll=-99999.9;imu.pitch=1e30;imu.yaw=-1e30;imu.sample.accel[0]=1e30;}
        d->vision.have_frame=state>1;d->vision.timestamp_ms=state==7?1000:9999;
        d->vision.count=0;
        if(state==8) {
            d->vision.count=5;d->vision.boxes_delayed=1;
            d->vision.objects[0]=(struct vision_object){.left=30,.top=30,.right=300,.bottom=300,.score=.8,.class_id=0,.eye=0};
            strcpy(d->vision.objects[0].name,"person");
            d->vision.objects[1]=(struct vision_object){.left=NAN,.top=4,.right=250,.bottom=460,.score=.8};
            d->vision.objects[2]=(struct vision_object){.left=60,.top=100,.right=30,.bottom=40,.score=.9};
            d->vision.objects[3]=(struct vision_object){.left=1,.top=4,.right=250,.bottom=460,.score=INFINITY};
            d->vision.objects[4]=(struct vision_object){.left=-200,.top=-200,.right=1200,.bottom=900,.score=.7};
            strcpy(d->vision.objects[4].name,"frame-edge");
        }
        if(state==9){imu.have_packet=0;d->vision.have_frame=0;}
        dashboard_render(d,&imu,10000,"removed-interface-information");
        assert_restored_ui(d);
        for(int i=0;i<32;i++)assert(memory[sizeof(*d)+i]==0xa7);
        if(d->vision.have_frame&&state!=7&&state!=8)camera_corners(d);
        if(state==0)assert(!dashboard_ppm(d,"build/v125-rk-waiting.ppm"));
        if(state==3) {
            assert(!dashboard_ppm(d,"build/v125-rk-normal.ppm"));
            struct display_transform rotation;
            assert(!display_transform_init(&rotation,VIEW_H,VIEW_W,VIEW_W,VIEW_H,90));
            assert(!display_transform_ppm(&rotation,d->pixels,"build/v125-rk-90.ppm"));
        }
        if(state==6)assert(!dashboard_ppm(d,"build/v125-rk-extreme.ppm"));
        if(state==7)assert(!dashboard_ppm(d,"build/v125-rk-stale.ppm"));
        if(state==8)assert(!dashboard_ppm(d,"build/v125-rk-boxes.ppm"));
    }
    /* Motion-compensated coordinates align to this preview and draw solid;
       uncorrected old coordinates retain the dated/dashed presentation. */
    d->vision.have_frame=1;d->vision.timestamp_ms=9999;d->vision.count=1;
    d->vision.boxes_delayed=1;
    d->vision.objects[0]=(struct vision_object){.left=50,.top=50,.right=300,.bottom=260,.score=.8,.class_id=0,.eye=0};
    strcpy(d->vision.objects[0].name,"person");
    d->vision.motion_valid[0]=0;dashboard_render(d,&imu,10000,"unused");
    unsigned old_gap=d->pixels[(137+50)*VIEW_W+48+50+10];
    d->vision.motion_valid[0]=1;dashboard_render(d,&imu,10000,"unused");
    unsigned aligned_line=d->pixels[(137+50)*VIEW_W+48+50+10];
    assert(old_gap!=aligned_line);
    assert(!dashboard_ppm(d,"build/v125-rk-motion.ppm"));
    /* A plotted curve must react to history while current numeric readings
       remain identical; NaN gaps must also remove those curve segments. */
    imu.have_packet=imu.initialized=1;imu.sample.flags=1;imu.received_ms=9999;
    imu.sample.accel[0]=0;imu.sample.accel[1]=0;imu.sample.accel[2]=1;
    d->vision.have_frame=1;d->vision.timestamp_ms=9999;d->vision.count=0;
    d->head=d->count=0;dashboard_render(d,&imu,10000,"unused");
    unsigned empty_curve=curve_hash(d);
    for(int i=0;i<120;i++) {
        imu.sample.accel[0]=sin(i*.15);imu.sample.accel[1]=cos(i*.12);
        imu.sample.accel[2]=1+sin(i*.19)*.35;
        dashboard_history(d,&imu,10000);
    }
    imu.sample.accel[0]=0;imu.sample.accel[1]=0;imu.sample.accel[2]=1;
    dashboard_render(d,&imu,10000,"unused");
    unsigned live_curve=curve_hash(d);assert(empty_curve!=live_curve);
    assert(!dashboard_ppm(d,"build/v125-rk-curves.ppm"));
    for(int i=0;i<120;i++)for(int axis=0;axis<3;axis++)d->history[i][axis]=NAN;
    dashboard_render(d,&imu,10000,"unused");assert(curve_hash(d)==empty_curve);
    for(int i=0;i<32;i++)assert(memory[sizeof(*d)+i]==0xa7);
    if(argc>1&&!strcmp(argv[1],"--benchmark")) {
        imu.have_packet=imu.initialized=1;imu.sample.flags=1;imu.received_ms=9999;
        imu.roll=-179.9;imu.pitch=89.9;imu.yaw=-135.4;
        imu.sample.accel[0]=-16;imu.sample.accel[1]=16;imu.sample.accel[2]=1;
        d->vision.have_frame=1;d->vision.timestamp_ms=9999;d->vision.count=0;
        clock_t started=clock();
        for(int frame=0;frame<60;frame++) {
            imu.yaw+=.1;dashboard_render(d,&imu,10000,"unused");
        }
        printf("UI render local CPU %.2fms/frame (60 fresh large-number frames; no file I/O or assertion scans; not board FPS)\n",1000.*(clock()-started)/CLOCKS_PER_SEC/60);
    }
    puts("PASS restored left-camera/right-stacked cards and original colors, live XYZ history curves and NaN gaps, per-box aligned solid/stale dashed borders, full camera corners, removed navigation/actions, adaptive numbers, stale/NaN/extreme states, normal/90 render bounds");
    free(memory);return 0;
}
