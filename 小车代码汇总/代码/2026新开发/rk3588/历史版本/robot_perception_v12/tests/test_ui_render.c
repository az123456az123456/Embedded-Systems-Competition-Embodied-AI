#define _POSIX_C_SOURCE 200809L
#include "dashboard.h"
#include "display_transform.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

void reference_dashboard_init(struct dashboard *);
void reference_dashboard_render(struct dashboard *, const struct imu_state *, uint64_t, const char *);
int reference_display_transform_init(struct display_transform *,int,int,int,int,int);
void reference_display_transform_blit(const struct display_transform *,uint32_t *,const uint32_t *,uint32_t);

static unsigned random_state = 20261019;
static unsigned next_random(void) {
    random_state = random_state * 1664525u + 1013904223u;
    return random_state;
}
static double milliseconds(void) {
    struct timespec t;
    assert(!clock_gettime(CLOCK_MONOTONIC,&t));
    return t.tv_sec*1000.0+t.tv_nsec/1000000.0;
}
static void prepare(struct dashboard *d, struct imu_state *s, unsigned mode) {
    memset(s,0,sizeof(*s));
    s->q[0]=1; s->sample.flags=1; s->sample.temperature=26.8;
    s->have_packet=mode!=0; s->initialized=mode!=1;
    s->received_ms=mode==2?7000:10000;
    s->calibrating=mode>=3 && mode<=6;
    s->calibration_status=(enum imu_calibration_status)(mode%4);
    s->calibration=137; s->calibration_samples=138;
    s->bias_valid=mode>=7;
    s->roll=(int)(mode*37%360)-180;
    s->pitch=(int)(mode*11%180)-90;
    s->yaw=(int)(mode*43%360)-180;
    for(int axis=0;axis<3;axis++) {
        s->sample.accel[axis]=((int)(next_random()%300)-150)/100.0;
        s->sample.gyro[axis]=((int)(next_random()%300)-150)/100.0;
        s->bias[axis]=(axis-1)*.132;
    }
    if(mode==8) s->roll=s->pitch=s->yaw=NAN;
    d->head=mode%120; d->count=mode==0?0:(mode==1?17:120);
    d->demo=mode==9;
    for(int i=0;i<120;i++) for(int axis=0;axis<3;axis++)
        d->history[i][axis]=((i+mode)%29==0)?NAN:sin((i+axis*30+mode)/11.0)*9;
    struct vision_snapshot *v=&d->vision;
    memset(v,0,sizeof(*v));
    v->have_frame=mode>=2; v->timestamp_ms=mode==3?7000:10000;
    v->demo=mode==10; v->count=mode<5?0:(int)((mode-4)*5%129);
    v->capture_fps=15.13f; v->preview_fps=14.96f; v->fps=12.05f;
    v->detection_enabled=mode!=11; v->detection_available=mode!=12;
    v->boxes_delayed=mode%2; v->detection_age_ms=208;
    snprintf(v->status,sizeof(v->status),"%s",mode==13?"AI error: test failure":"Waiting for camera");
    snprintf(v->source,sizeof(v->source),"1920x1080 NV12 / both");
    for(size_t i=0;i<(size_t)VISION_W*VISION_H;i++)v->pixels[i]=0xff000000|next_random()%0x1000000;
    for(int i=0;i<v->count;i++) {
        struct vision_object *o=&v->objects[i];
        o->left=((int)(next_random()%1100)-150)/1.3f;
        o->top=((int)(next_random()%700)-100)/1.3f;
        o->right=o->left+next_random()%500/1.3f;
        o->bottom=o->top+next_random()%500/1.3f;
        o->score=next_random()%101/100.0f;
        o->class_id=i; o->eye=i%3-1;
        snprintf(o->name,sizeof(o->name),"%s",i%2?"person":"very long object name for clipping");
        if(i%23==22)o->left=NAN;
    }
}
static void same_pixels(const uint32_t *a,const uint32_t *b,size_t count,const char *name,unsigned mode) {
    for(size_t i=0;i<count;i++) if(a[i]!=b[i]) {
        fprintf(stderr,"FAIL %s mode=%u pixel=%zu old=%08x new=%08x\n",name,mode,i,a[i],b[i]);
        abort();
    }
}
int main(int argc,char **argv) {
    struct dashboard *old=malloc(sizeof(*old)),*fresh=malloc(sizeof(*fresh));
    assert(old && fresh);
    reference_dashboard_init(old); dashboard_init(fresh);
    struct imu_state s;
    for(unsigned mode=0;mode<40;mode++) {
        prepare(old,&s,mode); memcpy(fresh,old,sizeof(*fresh));
        reference_dashboard_render(old,&s,10000,"eth1");
        dashboard_render(fresh,&s,10000,"eth1");
        same_pixels(old->pixels,fresh->pixels,(size_t)VIEW_W*VIEW_H,"full dashboard",mode);
    }
    puts("PASS full dashboard byte equality: 40 waiting/stale/live/demo/calibration/error states, clipped/fractional/dashed detections, history and all yaw directions");
    const int sizes[][2]={{1280,760},{760,1280},{1920,1080},{1080,1920},{800,475},{475,800},{1,1},{1001,777}};
    for(size_t k=0;k<sizeof(sizes)/sizeof(sizes[0]);k++) for(int rotation=0;rotation<360;rotation+=90) {
        struct display_transform a,b;
        int w=sizes[k][0],h=sizes[k][1];
        assert(!reference_display_transform_init(&a,w,h,VIEW_W,VIEW_H,rotation));
        assert(!display_transform_init(&b,w,h,VIEW_W,VIEW_H,rotation));
        size_t count=(size_t)w*h;
        uint32_t *pa=malloc(count*sizeof(*pa)),*pb=malloc(count*sizeof(*pb)); assert(pa && pb);
        reference_display_transform_blit(&a,pa,old->pixels,0xff123456);
        display_transform_blit(&b,pb,old->pixels,0xff123456);
        same_pixels(pa,pb,count,"display transform",(unsigned)(k*360+rotation));
        free(pa);free(pb);
    }
    puts("PASS display byte equality: 8 physical display sizes x 4 rotations, including identity memcpy");
    if(argc>1 && !strcmp(argv[1],"--benchmark")) {
        const unsigned count=30;
        double before=0,after=0;
        prepare(old,&s,7); old->vision.count=3;
        memcpy(fresh,old,sizeof(*fresh));
        /* Alternate execution order to reduce warm-cache/order bias. These
           measurements are this machine's CPU only, never a RK3588 FPS claim. */
        for(unsigned i=0;i<count;i++) {
            double start=milliseconds();
            if(i%2)dashboard_render(fresh,&s,10000,"eth1");
            else reference_dashboard_render(old,&s,10000,"eth1");
            double middle=milliseconds();
            if(i%2)reference_dashboard_render(old,&s,10000,"eth1");
            else dashboard_render(fresh,&s,10000,"eth1");
            double end=milliseconds();
            before+=i%2?end-middle:middle-start;
            after+=i%2?middle-start:end-middle;
        }
        same_pixels(old->pixels,fresh->pixels,(size_t)VIEW_W*VIEW_H,"benchmark dashboard",7);
        printf("BENCH complete dashboard render (live IMU, 3 detections, full history): old_ms=%.3f new_ms=%.3f samples=%u speedup=%.2fx (local CPU only)\n",before/count,after/count,count,before/after);
    }
    free(old);free(fresh);
    return 0;
}
