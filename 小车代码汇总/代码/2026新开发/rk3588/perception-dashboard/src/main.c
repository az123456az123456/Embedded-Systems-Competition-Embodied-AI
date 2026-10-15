#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include "dashboard.h"
#include <wayland-client.h>
#include "xdg-shell-client-protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <linux/if_packet.h>
struct buffer { struct wl_buffer *wl; uint32_t *pixels; size_t bytes; int busy; };
struct app {
    struct wl_display *display; struct wl_compositor *compositor; struct wl_shm *shm;
    struct xdg_wm_base *wm; struct wl_registry *registry; struct wl_surface *surface;
    struct xdg_surface *xdg; struct xdg_toplevel *top; struct wl_seat *seat;
    struct wl_pointer *pointer; struct wl_touch *touch; struct wl_surface *cursor_surface; struct wl_callback *callback;
    struct buffer buffers[2],cursor; struct dashboard dashboard; struct imu_state imu;
    struct vision_worker *vision;
    int width,height,pending_w,pending_h,configured,closed,fullscreen;
    double px,py,scale,ox,oy;
};
static volatile sig_atomic_t stop;
static void signal_stop(int s) { (void)s; stop=1; }
static void release(void *data,struct wl_buffer *b) { (void)b; ((struct buffer *)data)->busy=0; }
static const struct wl_buffer_listener buffer_listener={release};
static int buffer_create(struct app *a,struct buffer *b,int width,int height,uint32_t format) {
    char filename[]="/tmp/imu-display-XXXXXX";
    int fd=mkstemp(filename); if(fd<0) return -1;
    unlink(filename); fcntl(fd,F_SETFD,FD_CLOEXEC);
    b->bytes=(size_t)width*height*4;
    if(ftruncate(fd,(off_t)b->bytes)) { close(fd); return -1; }
    b->pixels=mmap(NULL,b->bytes,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
    if(b->pixels==MAP_FAILED) { b->pixels=NULL; close(fd); return -1; }
    struct wl_shm_pool *pool=wl_shm_create_pool(a->shm,fd,(int)b->bytes);
    b->wl=wl_shm_pool_create_buffer(pool,0,width,height,width*4,format);
    wl_shm_pool_destroy(pool); close(fd);
    wl_buffer_add_listener(b->wl,&buffer_listener,b); return 0;
}
static void buffer_destroy(struct buffer *b) {
    if(b->wl) wl_buffer_destroy(b->wl);
    if(b->pixels) munmap(b->pixels,b->bytes);
    memset(b,0,sizeof(*b));
}
static void frame_done(void *data,struct wl_callback *cb,uint32_t t) {
    (void)t; struct app *a=data; wl_callback_destroy(cb); a->callback=NULL;
}
static const struct wl_callback_listener frame_listener={frame_done};
static void configure(void *data,struct xdg_surface *s,uint32_t serial) {
    struct app *a=data; xdg_surface_ack_configure(s,serial); a->configured=1;
}
static const struct xdg_surface_listener surface_listener={configure};
static void top_configure(void *data,struct xdg_toplevel *t,int32_t w,int32_t h,struct wl_array *states) {
    (void)t;(void)states; struct app *a=data;
    if(w>0 && h>0) { a->pending_w=w; a->pending_h=h; }
}
static void top_close(void *data,struct xdg_toplevel *t) { (void)t; ((struct app *)data)->closed=1; }
static const struct xdg_toplevel_listener top_listener={.configure=top_configure,.close=top_close};
static void ping(void *data,struct xdg_wm_base *wm,uint32_t serial) { (void)data; xdg_wm_base_pong(wm,serial); }
static const struct xdg_wm_base_listener wm_listener={ping};
static void click(struct app *a,double x,double y,uint32_t serial) {
    if(a->scale<=0) return;
    x=(x-a->ox)/a->scale; y=(y-a->oy)/a->scale;
    if(x<0 || x>=VIEW_W || y<0 || y>=VIEW_H) return;
    switch(dashboard_action(x,y)) {
        case 1: if(imu_fresh(&a->imu,imu_clock_ms()) && a->imu.initialized) imu_zero_yaw(&a->imu); break;
        case 2: imu_calibrate(&a->imu); break;
        case 3: a->closed=1; break;
        case 4: if(!a->fullscreen) xdg_toplevel_move(a->top,a->seat,serial); break;
    }
}
static void pointer_enter(void *data,struct wl_pointer *p,uint32_t serial,struct wl_surface *s,wl_fixed_t x,wl_fixed_t y) {
    (void)s; struct app *a=data; a->px=wl_fixed_to_double(x); a->py=wl_fixed_to_double(y);
    if(!a->cursor_surface && a->shm && a->compositor) {
        if(buffer_create(a,&a->cursor,24,32,WL_SHM_FORMAT_ARGB8888)) return;
        for(int j=0;j<32;j++) for(int i=0;i<24;i++) {
            int inside=j>=2 && j<=26 && i>=2 && i<=2+j*.65;
            int edge=i<=3 || j<=3 || i>=1+j*.65 || j>=25;
            a->cursor.pixels[j*24+i]=inside?(edge?0xffffffff:0xff193542):0;
        }
        a->cursor_surface=wl_compositor_create_surface(a->compositor);
        wl_surface_attach(a->cursor_surface,a->cursor.wl,0,0);
        wl_surface_damage(a->cursor_surface,0,0,24,32); wl_surface_commit(a->cursor_surface);
    }
    if(a->cursor_surface) wl_pointer_set_cursor(p,serial,a->cursor_surface,2,2);
}
static void pointer_leave(void *d,struct wl_pointer *p,uint32_t s,struct wl_surface *w) {(void)d;(void)p;(void)s;(void)w;}
static void pointer_motion(void *data,struct wl_pointer *p,uint32_t t,wl_fixed_t x,wl_fixed_t y) {
    (void)p;(void)t; struct app *a=data; a->px=wl_fixed_to_double(x); a->py=wl_fixed_to_double(y);
}
static void pointer_button(void *data,struct wl_pointer *p,uint32_t serial,uint32_t time,uint32_t button,uint32_t state) {
    (void)p;(void)time; struct app *a=data;
    if(button==0x110 && state==WL_POINTER_BUTTON_STATE_PRESSED) click(a,a->px,a->py,serial);
}
static void pointer_axis(void *d,struct wl_pointer *p,uint32_t t,uint32_t ax,wl_fixed_t v) {(void)d;(void)p;(void)t;(void)ax;(void)v;}
static const struct wl_pointer_listener pointer_listener={.enter=pointer_enter,.leave=pointer_leave,.motion=pointer_motion,.button=pointer_button,.axis=pointer_axis};
static void touch_down(void *data,struct wl_touch *t,uint32_t serial,uint32_t time,struct wl_surface *s,int32_t id,wl_fixed_t x,wl_fixed_t y) {
    (void)t;(void)time;(void)s; if(id==0) click(data,wl_fixed_to_double(x),wl_fixed_to_double(y),serial);
}
static void touch_up(void *d,struct wl_touch *t,uint32_t s,uint32_t time,int32_t id) {(void)d;(void)t;(void)s;(void)time;(void)id;}
static void touch_motion(void *d,struct wl_touch *t,uint32_t time,int32_t id,wl_fixed_t x,wl_fixed_t y) {(void)d;(void)t;(void)time;(void)id;(void)x;(void)y;}
static void touch_frame(void *d,struct wl_touch *t) {(void)d;(void)t;}
static void touch_cancel(void *d,struct wl_touch *t) {(void)d;(void)t;}
static const struct wl_touch_listener touch_listener={.down=touch_down,.up=touch_up,.motion=touch_motion,.frame=touch_frame,.cancel=touch_cancel};
static void seat_caps(void *data,struct wl_seat *seat,uint32_t caps) {
    struct app *a=data;
    if((caps&WL_SEAT_CAPABILITY_POINTER) && !a->pointer) {
        a->pointer=wl_seat_get_pointer(seat); wl_pointer_add_listener(a->pointer,&pointer_listener,a);
    } else if(!(caps&WL_SEAT_CAPABILITY_POINTER) && a->pointer) { wl_pointer_destroy(a->pointer); a->pointer=NULL; }
    if((caps&WL_SEAT_CAPABILITY_TOUCH) && !a->touch) {
        a->touch=wl_seat_get_touch(seat); wl_touch_add_listener(a->touch,&touch_listener,a);
    } else if(!(caps&WL_SEAT_CAPABILITY_TOUCH) && a->touch) { wl_touch_destroy(a->touch); a->touch=NULL; }
}
static const struct wl_seat_listener seat_listener={.capabilities=seat_caps};
static void global(void *data,struct wl_registry *r,uint32_t name,const char *interface,uint32_t version) {
    (void)version; struct app *a=data;
    if(!strcmp(interface,"wl_compositor")) a->compositor=wl_registry_bind(r,name,&wl_compositor_interface,1);
    else if(!strcmp(interface,"wl_shm")) a->shm=wl_registry_bind(r,name,&wl_shm_interface,1);
    else if(!strcmp(interface,"xdg_wm_base")) { a->wm=wl_registry_bind(r,name,&xdg_wm_base_interface,1); xdg_wm_base_add_listener(a->wm,&wm_listener,a); }
    else if(!strcmp(interface,"wl_seat") && !a->seat) { a->seat=wl_registry_bind(r,name,&wl_seat_interface,1); wl_seat_add_listener(a->seat,&seat_listener,a); }
}
static void global_remove(void *data,struct wl_registry *r,uint32_t name) {(void)data;(void)r;(void)name;}
static const struct wl_registry_listener registry_listener={global,global_remove};
static int display_open(struct app *a) {
    a->display=wl_display_connect(NULL);
    if(!a->display) { fprintf(stderr,"Cannot connect to Weston. Run ./run-perception.sh in the existing desktop session.\n"); return -1; }
    a->registry=wl_display_get_registry(a->display); wl_registry_add_listener(a->registry,&registry_listener,a);
    if(wl_display_roundtrip(a->display)<0 || !a->wm || !a->compositor || !a->shm) {
        fprintf(stderr,"Desktop needs Wayland wl_shm and xdg-shell.\n"); return -1;
    }
    a->surface=wl_compositor_create_surface(a->compositor);
    a->xdg=xdg_wm_base_get_xdg_surface(a->wm,a->surface); xdg_surface_add_listener(a->xdg,&surface_listener,a);
    a->top=xdg_surface_get_toplevel(a->xdg); xdg_toplevel_add_listener(a->top,&top_listener,a);
    xdg_toplevel_set_title(a->top,"视觉与姿态"); xdg_toplevel_set_app_id(a->top,"acg138.perception-dashboard");
    xdg_toplevel_set_min_size(a->top,800,475);
    if(a->fullscreen) xdg_toplevel_set_fullscreen(a->top,NULL);
    wl_surface_commit(a->surface); return 0;
}
static int present(struct app *a) {
    if(!a->configured || a->callback) return 0;
    if(a->pending_w!=a->width || a->pending_h!=a->height) {
        if(a->pending_w<1 || a->pending_h<1 || a->pending_w>4096 || a->pending_h>4096) {
            fprintf(stderr,"Unsupported desktop window size\n"); return -1;
        }
        a->width=a->pending_w; a->height=a->pending_h;
        a->scale=fmin(a->width/(double)VIEW_W,a->height/(double)VIEW_H);
        a->ox=(a->width-VIEW_W*a->scale)/2; a->oy=(a->height-VIEW_H*a->scale)/2;
        for(int i=0;i<2;i++) { buffer_destroy(&a->buffers[i]); if(buffer_create(a,&a->buffers[i],a->width,a->height,WL_SHM_FORMAT_XRGB8888)) return -1; }
    }
    struct buffer *b=NULL;
    for(int i=0;i<2;i++) if(!a->buffers[i].busy) { b=&a->buffers[i]; break; }
    if(!b) return 0;
    for(int y=0;y<a->height;y++) for(int x=0;x<a->width;x++) {
        int sx=(x-a->ox)/a->scale,sy=(y-a->oy)/a->scale;
        b->pixels[y*a->width+x]=(x<a->ox || y<a->oy || (unsigned)sx>=VIEW_W || (unsigned)sy>=VIEW_H)?
            0xfff4f7fa:a->dashboard.pixels[sy*VIEW_W+sx];
    }
    a->callback=wl_surface_frame(a->surface); wl_callback_add_listener(a->callback,&frame_listener,a);
    wl_surface_attach(a->surface,b->wl,0,0); wl_surface_damage(a->surface,0,0,a->width,a->height);
    wl_surface_commit(a->surface); b->busy=1; return 0;
}
static void cleanup(struct app *a) {
    vision_stop(a->vision);
    if(a->callback) wl_callback_destroy(a->callback);
    for(int i=0;i<2;i++) buffer_destroy(&a->buffers[i]);
    if(a->cursor_surface) wl_surface_destroy(a->cursor_surface);
    buffer_destroy(&a->cursor);
    if(a->touch) wl_touch_destroy(a->touch);
    if(a->pointer) wl_pointer_destroy(a->pointer);
    if(a->seat) wl_seat_destroy(a->seat);
    if(a->top) xdg_toplevel_destroy(a->top);
    if(a->xdg) xdg_surface_destroy(a->xdg);
    if(a->surface) wl_surface_destroy(a->surface);
    if(a->wm) xdg_wm_base_destroy(a->wm);
    if(a->shm) wl_shm_destroy(a->shm);
    if(a->compositor) wl_compositor_destroy(a->compositor);
    if(a->registry) wl_registry_destroy(a->registry);
    if(a->display) wl_display_disconnect(a->display);
    free(a);
}
static void demo_sample(struct app *a,uint64_t now,uint64_t elapsed) {
    double t=elapsed/1000., r=.22*sin(t*.7),p=.16*cos(t*.5);
    struct imu_sample s={.flags=15,.ag_age=0,.temperature=26.4,.sequence=(uint32_t)(elapsed/10+1),.timestamp=(uint32_t)elapsed,.uptime=(uint32_t)elapsed};
    s.accel[0]=-sin(p); s.accel[1]=sin(r)*cos(p); s.accel[2]=cos(r)*cos(p);
    s.gyro[0]=.22*.7*cos(t*.7)*180/IMU_PI; s.gyro[1]=-.16*.5*sin(t*.5)*180/IMU_PI;
    imu_update(&a->imu,&s,now); a->imu.calibrating=0;
}
static int number(const char *s,int low,int high,int *out) {
    char *end=NULL; errno=0; long n=strtol(s,&end,10);
    if(errno || !*s || *end || n<low || n>high) return 0;
    *out=(int)n; return 1;
}
static int fraction(const char *s,float *out) {
    char *end=NULL; errno=0; float n=strtof(s,&end);
    if(errno || !*s || *end || !isfinite(n) || n<=0 || n>=1) return 0;
    *out=n; return 1;
}
static void usage(const char *name) {
    printf("Usage: %s [--model FILE.rknn] [--labels FILE.txt] [--camera /dev/videoX]\n"
           "  [--view both|left|right|single] [--confidence 0.35] [--nms 0.45]\n"
           "  [--interface NAME] [--width 1920] [--height 1080] [--fullscreen]\n"
           "  [--demo] [--seconds N] [--preview FILE.ppm [--waiting]]\n",name);
}
int main(int argc,char **argv) {
    struct app *a=calloc(1,sizeof(*a)); if(!a) return 1;
    a->pending_w=VIEW_W; a->pending_h=VIEW_H;
    const char *iface=NULL,*preview=NULL; int demo=0,timeout=0,waiting_preview=0;
    struct vision_options options={.model="model/model.rknn",.labels="model/coco_80_labels_list.txt",
        .device="/dev/video0",.view="both",.width=1920,.height=1080,.confidence=.35f,.nms=.45f};
    for(int i=1;i<argc;i++) {
        if(!strcmp(argv[i],"--help")) { usage(argv[0]); free(a); return 0; }
        if(!strcmp(argv[i],"--demo")) { demo=1; continue; }
        if(!strcmp(argv[i],"--fullscreen")) { a->fullscreen=1; continue; }
        if(!strcmp(argv[i],"--waiting")) { waiting_preview=1; continue; }
        if(i+1>=argc) { usage(argv[0]); free(a); return 2; }
        const char *key=argv[i],*value=argv[++i]; int valid=1;
        if(!strcmp(key,"--interface")) iface=value;
        else if(!strcmp(key,"--model")) options.model=value;
        else if(!strcmp(key,"--labels")) options.labels=value;
        else if(!strcmp(key,"--camera")) options.device=value;
        else if(!strcmp(key,"--view")) {
            valid=!strcmp(value,"both")||!strcmp(value,"left")||!strcmp(value,"right")||!strcmp(value,"single"); options.view=value;
        }
        else if(!strcmp(key,"--confidence")) valid=fraction(value,&options.confidence);
        else if(!strcmp(key,"--nms")) valid=fraction(value,&options.nms);
        else if(!strcmp(key,"--width")) valid=number(value,2,4096,&options.width);
        else if(!strcmp(key,"--height")) valid=number(value,2,2160,&options.height);
        else if(!strcmp(key,"--seconds")) valid=number(value,1,86400,&timeout);
        else if(!strcmp(key,"--preview")) { preview=value; demo=1; }
        else valid=0;
        if(!valid) { fprintf(stderr,"Invalid option: %s %s\n",key,value);usage(argv[0]);free(a);return 2; }
    }
    dashboard_init(&a->dashboard); imu_init(&a->imu); a->dashboard.demo=demo;
    if(preview) {
        if(!waiting_preview) for(int i=0;i<=600;i++) {
            demo_sample(a,i*10,i*10); if(i%5==0) dashboard_history(&a->dashboard,&a->imu,i*10);
        }
        if(!waiting_preview) { vision_demo(&a->dashboard.vision,6000); a->dashboard.vision.timestamp_ms=6000; }
        dashboard_render(&a->dashboard,&a->imu,6000,"DEMO");
        int result=dashboard_ppm(&a->dashboard,preview); free(a); return result?1:0;
    }
    int fd=-1,result=0; char name[16]="DEMO";
    if(!demo) {
        fd=imu_open_socket(iface,name);
        if(fd<0) { perror("IMU network (root / CAP_NET_RAW required; check --interface)"); strcpy(name,"NO IMU"); }
    }
    if(display_open(a)) { if(fd>=0) close(fd); cleanup(a); return 1; }
    if(!demo) {
        a->vision=vision_start(&options);
        if(!a->vision) { if(fd>=0) close(fd); cleanup(a); return 1; }
    }
    signal(SIGINT,signal_stop); signal(SIGTERM,signal_stop);
    uint64_t start=imu_clock_ms(),last_render=0,last_history=0,last_demo=0;
    while(!stop && !a->closed) {
        uint64_t now=imu_clock_ms();
        if(timeout && now-start>=(uint64_t)timeout*1000) break;
        if(demo && now-last_demo>=10) { demo_sample(a,now,now-start); last_demo=now; }
        if(now-last_history>=50) { dashboard_history(&a->dashboard,&a->imu,now); last_history=now; }
        if(a->configured && !a->callback && now-last_render>=33) {
            if(demo) vision_demo(&a->dashboard.vision,now-start);
            else vision_snapshot_copy(a->vision,&a->dashboard.vision);
            now=imu_clock_ms();
            dashboard_render(&a->dashboard,&a->imu,now,name);
            if(present(a)<0) { perror("display buffer"); result=1; break; } last_render=now;
        }
        if(wl_display_dispatch_pending(a->display)<0) { result=1; break; }
        int flushed=wl_display_flush(a->display);
        if(flushed<0 && errno!=EAGAIN) { result=1; break; }
        struct pollfd fds[2]={{wl_display_get_fd(a->display),POLLIN|(flushed<0?POLLOUT:0),0},{fd,POLLIN,0}};
        int n=poll(fds,fd>=0?2:1,10);
        if(n<0) { if(errno==EINTR) continue; result=1; break; }
        if(fds[0].revents&(POLLERR|POLLHUP|POLLNVAL)) { result=1; break; }
        if((fds[0].revents&POLLIN) && wl_display_dispatch(a->display)<0) { result=1; break; }
        if(fds[1].revents&(POLLERR|POLLHUP|POLLNVAL)) { fprintf(stderr,"Network interface failed\n"); result=1; break; }
        if(fds[1].revents&POLLIN) for(int k=0;k<128;k++) {
            unsigned char bytes[2048]; struct sockaddr_ll from; socklen_t length=sizeof(from);
            ssize_t got=recvfrom(fd,bytes,sizeof(bytes),0,(struct sockaddr *)&from,&length);
            if(got<0) { if(errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) { perror("receive"); result=1; a->closed=1; } break; }
            struct imu_sample s;
            if(from.sll_pkttype!=PACKET_OUTGOING && imu_decode(bytes,(size_t)got,&s)) imu_update(&a->imu,&s,imu_clock_ms());
        }
    }
    if(fd>=0) close(fd);
    cleanup(a); return result;
}
