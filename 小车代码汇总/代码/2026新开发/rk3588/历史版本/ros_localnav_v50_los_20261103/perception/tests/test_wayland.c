/* Headless protocol harness, not a physical display/compositor timing test. */
#define _POSIX_C_SOURCE 200809L
#include <wayland-server.h>
#include "xdg-shell-server-protocol.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
static struct wl_display *display;
static struct wl_resource *xdg,*top,*buffer,*callback,*touch,*main_surface;
static int navigation_test,nav_seen,camera_seen;
static uint32_t serial; static int acked,frames,resized,fullscreen_requested,rotation,target_frames=10;
static void destroy(struct wl_client *c,struct wl_resource *r) {(void)c; wl_resource_destroy(r);}
static void ignored(struct wl_client *c,struct wl_resource *r,int32_t x,int32_t y,int32_t w,int32_t h) {(void)c;(void)r;(void)x;(void)y;(void)w;(void)h;}
static void configure(int w,int h) {
    struct wl_array states; wl_array_init(&states);
    xdg_toplevel_send_configure(top,w,h,&states); serial=wl_display_next_serial(display);
    xdg_surface_send_configure(xdg,serial); acked=0;
}
static void attach(struct wl_client *c,struct wl_resource *r,struct wl_resource *b,int32_t x,int32_t y) {(void)c;(void)r;(void)x;(void)y; buffer=b;}
static void frame(struct wl_client *c,struct wl_resource *r,uint32_t id) {(void)r; assert(!callback); callback=wl_resource_create(c,&wl_callback_interface,1,id);}
static unsigned canvas_pixel(unsigned *pixels,int x,int y) {
    return rotation==90?pixels[x*760+(759-y)]:pixels[y*1280+x];
}
static void touch_tab(int nav) {
    assert(touch&&main_surface);double x=nav?250:110,y=65;
    double px=rotation==90?760-y:x,py=rotation==90?x:y;
    uint32_t s=wl_display_next_serial(display);
    wl_touch_send_down(touch,s,frames*33,main_surface,0,wl_fixed_from_double(px),wl_fixed_from_double(py));
    wl_touch_send_up(touch,s,frames*33+1,0);wl_touch_send_frame(touch);
}
static void commit(struct wl_client *c,struct wl_resource *r) {
    (void)c;(void)r;
    if(!serial) { configure(rotation%180?760:1280,rotation%180?1280:760); return; }
    if(!buffer) return;
    assert(acked);
    struct wl_shm_buffer *b=wl_shm_buffer_get(buffer); assert(b);
    wl_shm_buffer_begin_access(b);
    assert(wl_shm_buffer_get_format(b)==WL_SHM_FORMAT_XRGB8888);
    unsigned *pixels=wl_shm_buffer_get_data(b);
    if(wl_shm_buffer_get_width(b)==(rotation%180?760:1280)) {
        assert(wl_shm_buffer_get_height(b)==(rotation%180?1280:760));
        assert(pixels[0]==0xfff5f5f7);
        if(navigation_test&&frames>=2&&frames<=3) {
            assert(canvas_pixel(pixels,200,50)==0xff202124);
            assert(canvas_pixel(pixels,58,147)==0xfff7f8fa);nav_seen=1;
        }
        if(navigation_test&&frames>=5&&frames<=7) {
            assert(canvas_pixel(pixels,62,50)==0xff202124);
            assert(canvas_pixel(pixels,200,50)!=0xff202124);camera_seen=1;
        }
    } else {
        assert(wl_shm_buffer_get_width(b)==(rotation%180?544:800) && wl_shm_buffer_get_height(b)==(rotation%180?800:544));
        assert(pixels[0]==0xfff5f5f7); /* Letterbox outside the 1280:760 canvas. */
        resized=1;
    }
    wl_shm_buffer_end_access(b);
    wl_buffer_send_release(buffer); buffer=NULL;
    if(callback) { wl_callback_send_done(callback,frames*33); wl_resource_destroy(callback); callback=NULL; }
    frames++;
    if(navigation_test&&frames==1)touch_tab(1);
    if(navigation_test&&frames==4)touch_tab(0);
    if(frames==(navigation_test?8:3)) configure(rotation%180?544:800,rotation%180?800:544);
    if(frames==target_frames) xdg_toplevel_send_close(top);
}
static const struct wl_surface_interface surface_impl={.destroy=destroy,.attach=attach,.damage=ignored,.frame=frame,.commit=commit};
static void surface_create(struct wl_client *c,struct wl_resource *r,uint32_t id) {
    (void)r; struct wl_resource *s=wl_resource_create(c,&wl_surface_interface,1,id); wl_resource_set_implementation(s,&surface_impl,NULL,NULL);
}
static const struct wl_compositor_interface compositor_impl={.create_surface=surface_create};
static void bind_compositor(struct wl_client *c,void *data,uint32_t version,uint32_t id) {
    (void)data;(void)version; struct wl_resource *r=wl_resource_create(c,&wl_compositor_interface,1,id); wl_resource_set_implementation(r,&compositor_impl,NULL,NULL);
}
static void title(struct wl_client *c,struct wl_resource *r,const char *v) {(void)c;(void)r;assert(v);}
static void min_size(struct wl_client *c,struct wl_resource *r,int32_t w,int32_t h) {(void)c;(void)r;assert(w==(rotation%180?475:800)&&h==(rotation%180?800:475));}
static void fullscreen(struct wl_client *c,struct wl_resource *r,struct wl_resource *o) {(void)c;(void)r;(void)o;fullscreen_requested=1;}
static const struct xdg_toplevel_interface top_impl={.destroy=destroy,.set_title=title,.set_app_id=title,.set_min_size=min_size,.set_fullscreen=fullscreen};
static void get_top(struct wl_client *c,struct wl_resource *r,uint32_t id) {
    (void)r; top=wl_resource_create(c,&xdg_toplevel_interface,1,id); wl_resource_set_implementation(top,&top_impl,NULL,NULL);
}
static void ack(struct wl_client *c,struct wl_resource *r,uint32_t s) {(void)c;(void)r;assert(s==serial);acked=1;}
static const struct xdg_surface_interface xdg_impl={.destroy=destroy,.get_toplevel=get_top,.ack_configure=ack};
static void get_xdg(struct wl_client *c,struct wl_resource *r,uint32_t id,struct wl_resource *surface) {
    (void)r;main_surface=surface; xdg=wl_resource_create(c,&xdg_surface_interface,1,id); wl_resource_set_implementation(xdg,&xdg_impl,NULL,NULL);
}
static void pong(struct wl_client *c,struct wl_resource *r,uint32_t s) {(void)c;(void)r;(void)s;}
static const struct xdg_wm_base_interface wm_impl={.destroy=destroy,.get_xdg_surface=get_xdg,.pong=pong};
static void bind_wm(struct wl_client *c,void *data,uint32_t version,uint32_t id) {
    (void)data;(void)version; struct wl_resource *r=wl_resource_create(c,&xdg_wm_base_interface,1,id); wl_resource_set_implementation(r,&wm_impl,NULL,NULL);
}
static void get_touch(struct wl_client *c,struct wl_resource *r,uint32_t id) {
    (void)r;touch=wl_resource_create(c,&wl_touch_interface,1,id);
}
static const struct wl_seat_interface seat_impl={.get_touch=get_touch};
static void bind_seat(struct wl_client *c,void *data,uint32_t version,uint32_t id) {
    (void)data;(void)version;struct wl_resource *r=wl_resource_create(c,&wl_seat_interface,1,id);
    wl_resource_set_implementation(r,&seat_impl,NULL,NULL);wl_seat_send_capabilities(r,WL_SEAT_CAPABILITY_TOUCH);
}
static void client_gone(struct wl_listener *listener,void *data) {(void)listener;(void)data;wl_display_terminate(display);}
static struct wl_listener gone={.notify=client_gone};
static void client_new(struct wl_listener *listener,void *data) {(void)listener;wl_client_add_destroy_listener(data,&gone);}
static struct wl_listener new_client={.notify=client_new};
int main(void) {
    navigation_test=getenv("PERCEPTION_NAVIGATION_TEST")!=NULL;
    const char *requested=getenv("PERCEPTION_TEST_ROTATION");rotation=requested?atoi(requested):0;
    const char *target=getenv("PERCEPTION_TEST_FRAMES");if(target&&atoi(target)>=10)target_frames=atoi(target);
    display=wl_display_create(); assert(display);
    assert(wl_display_add_socket(display,"imu-test")==0); assert(wl_display_init_shm(display)==0);
    wl_global_create(display,&wl_compositor_interface,1,NULL,bind_compositor);
    wl_global_create(display,&xdg_wm_base_interface,1,NULL,bind_wm);
    if(navigation_test)wl_global_create(display,&wl_seat_interface,1,NULL,bind_seat);
    wl_display_add_client_created_listener(display,&new_client);
    puts("READY");fflush(stdout);wl_display_run(display);
    assert(frames>=10 && resized);
    if(navigation_test) {assert(nav_seen&&camera_seen);puts("PASS navigation live Wayland touch: camera -> map -> camera, selected tabs, transformed hit targets");}
    const char *expected=getenv("PERCEPTION_EXPECT_FULLSCREEN");
    assert(fullscreen_requested==(expected?atoi(expected):1));
    wl_display_destroy_clients(display);wl_display_destroy(display);
    puts("PASS Wayland: configure/ack, shared-memory format/pixels, double-buffer release, frame callback, resize and close");
}
