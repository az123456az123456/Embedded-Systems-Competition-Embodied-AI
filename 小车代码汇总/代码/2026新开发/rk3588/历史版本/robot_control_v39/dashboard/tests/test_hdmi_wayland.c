/* A headless protocol regression, not an HDMI electrical/hardware test.
 * Both output geometries deliberately have generic model strings: only the
 * xdg-output protocol identifies HDMI when wl_output is capped at version 3.
 * No wl_seat or input events are ever advertised by this compositor. */
#define _POSIX_C_SOURCE 200809L
#include <wayland-server.h>
#include "xdg-shell-server-protocol.h"
#include "xdg-output-server-protocol.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct output { const char *name; int index; };
static struct output outputs[]={{"DSI-1",0},{"HDMI-A-1",1}};
static struct wl_display *display;
static struct wl_resource *xdg,*top,*buffer;
static struct wl_resource *callbacks[16];
static unsigned callback_count,callback_requests,frames,callbacks_sent;
static unsigned activations,activations_after_callback,fullscreen_requests;
static unsigned empty_commits;
static struct wl_event_source *tick;
static uint32_t serial;
static int acked,configured,selected=-1,rotation,expect_fullscreen=1;
static int delay_configure,hold_first,never_callback,no_outputs;
static int unfocused_default;
static int closed;
static uint64_t started_ms,buffer_ms;

static uint64_t now_ms(void) {
    struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);
    return (uint64_t)t.tv_sec*1000+(uint64_t)t.tv_nsec/1000000;
}
static void destroy(struct wl_client *client,struct wl_resource *resource) {
    (void)client;wl_resource_destroy(resource);
}
static void configure(void) {
    struct wl_array states;wl_array_init(&states);
    xdg_toplevel_send_configure(top,rotation%180?760:1280,rotation%180?1280:760,&states);
    serial=wl_display_next_serial(display);xdg_surface_send_configure(xdg,serial);
    configured=1;acked=0;wl_array_release(&states);
}
static void ignored_damage(struct wl_client *c,struct wl_resource *r,
                           int32_t x,int32_t y,int32_t w,int32_t h) {
    (void)c;(void)r;(void)x;(void)y;(void)w;(void)h;
}
static void ignored_region(struct wl_client *c,struct wl_resource *r,struct wl_resource *region) {
    (void)c;(void)r;(void)region;
}
static void attach(struct wl_client *c,struct wl_resource *r,struct wl_resource *b,int32_t x,int32_t y) {
    (void)c;(void)r;(void)x;(void)y;buffer=b;
}
static void frame(struct wl_client *c,struct wl_resource *r,uint32_t id) {
    (void)r;assert(callback_count<16);
    callbacks[callback_count++]=wl_resource_create(c,&wl_callback_interface,1,id);
    callback_requests++;
    /* Once startup succeeded, every submitted buffer must wait for its frame
     * callback. Startup retries alone may replace an unsignalled callback. */
    if(callbacks_sent)assert(callback_count==1);
}
static void commit(struct wl_client *c,struct wl_resource *r) {
    (void)c;(void)r;
    if(!buffer) {
        empty_commits++;
        /* Reproduce a desktop with a visible HDMI background but no focused
         * input output: a NULL-output fullscreen request never obtains its
         * initial configure. An explicit HDMI request maps immediately. */
        if(!configured&&!delay_configure&&(!unfocused_default||selected==1))configure();
        return;
    }
    assert(configured&&acked);
    struct wl_shm_buffer *shm=wl_shm_buffer_get(buffer);assert(shm);
    assert(wl_shm_buffer_get_format(shm)==WL_SHM_FORMAT_XRGB8888);
    assert(wl_shm_buffer_get_width(shm)==(rotation%180?760:1280));
    assert(wl_shm_buffer_get_height(shm)==(rotation%180?1280:760));
    wl_shm_buffer_begin_access(shm);
    assert(((uint32_t *)wl_shm_buffer_get_data(shm))[0]==0xfff5f5f7);
    wl_shm_buffer_end_access(shm);
    wl_buffer_send_release(buffer);buffer=NULL;buffer_ms=now_ms();frames++;
    if(callbacks_sent)assert(frames<=callbacks_sent+2);
}
static const struct wl_surface_interface surface_impl={
    .destroy=destroy,.attach=attach,.damage=ignored_damage,.frame=frame,
    .set_opaque_region=ignored_region,.set_input_region=ignored_region,.commit=commit
};
static void create_surface(struct wl_client *c,struct wl_resource *r,uint32_t id) {
    (void)r;struct wl_resource *surface=wl_resource_create(c,&wl_surface_interface,1,id);
    wl_resource_set_implementation(surface,&surface_impl,NULL,NULL);
}
static const struct wl_compositor_interface compositor_impl={.create_surface=create_surface};
static void bind_compositor(struct wl_client *c,void *data,uint32_t version,uint32_t id) {
    (void)data;(void)version;struct wl_resource *r=wl_resource_create(c,&wl_compositor_interface,1,id);
    wl_resource_set_implementation(r,&compositor_impl,NULL,NULL);
}
static void title(struct wl_client *c,struct wl_resource *r,const char *value) {
    (void)c;(void)r;
    if(strstr(value,"requests=activate")) {
        activations++;
        if(callbacks_sent)activations_after_callback++;
    }
}
static void min_size(struct wl_client *c,struct wl_resource *r,int32_t w,int32_t h) {
    (void)c;(void)r;assert(w==(rotation%180?475:800)&&h==(rotation%180?800:475));
}
static void fullscreen(struct wl_client *c,struct wl_resource *r,struct wl_resource *output) {
    (void)c;(void)r;fullscreen_requests++;
    struct output *o=output?wl_resource_get_user_data(output):NULL;
    selected=o?o->index:-1;
}
static const struct xdg_toplevel_interface top_impl={
    .destroy=destroy,.set_title=title,.set_app_id=title,
    .set_min_size=min_size,.set_fullscreen=fullscreen
};
static void get_top(struct wl_client *c,struct wl_resource *r,uint32_t id) {
    (void)r;top=wl_resource_create(c,&xdg_toplevel_interface,1,id);
    wl_resource_set_implementation(top,&top_impl,NULL,NULL);
}
static void ack(struct wl_client *c,struct wl_resource *r,uint32_t value) {
    (void)c;(void)r;assert(value==serial);acked=1;
}
static const struct xdg_surface_interface xdg_impl={.destroy=destroy,.get_toplevel=get_top,.ack_configure=ack};
static void get_xdg(struct wl_client *c,struct wl_resource *r,uint32_t id,struct wl_resource *surface) {
    (void)r;(void)surface;xdg=wl_resource_create(c,&xdg_surface_interface,1,id);
    wl_resource_set_implementation(xdg,&xdg_impl,NULL,NULL);
}
static void pong(struct wl_client *c,struct wl_resource *r,uint32_t value) {(void)c;(void)r;(void)value;}
static const struct xdg_wm_base_interface wm_impl={.destroy=destroy,.get_xdg_surface=get_xdg,.pong=pong};
static void bind_wm(struct wl_client *c,void *data,uint32_t version,uint32_t id) {
    (void)data;(void)version;struct wl_resource *r=wl_resource_create(c,&xdg_wm_base_interface,1,id);
    wl_resource_set_implementation(r,&wm_impl,NULL,NULL);
}
static const struct wl_output_interface output_impl={.release=destroy};
static void bind_output(struct wl_client *c,void *data,uint32_t version,uint32_t id) {
    struct output *o=data;struct wl_resource *r=wl_resource_create(c,&wl_output_interface,version,id);
    wl_resource_set_implementation(r,&output_impl,o,NULL);
    wl_output_send_geometry(r,o->index*1280,0,300,180,WL_OUTPUT_SUBPIXEL_UNKNOWN,"Generic","Panel",WL_OUTPUT_TRANSFORM_NORMAL);
    wl_output_send_mode(r,WL_OUTPUT_MODE_CURRENT|WL_OUTPUT_MODE_PREFERRED,1280,760,60000);
    if(version>=2) {wl_output_send_scale(r,1);wl_output_send_done(r);}
}
static const struct zxdg_output_v1_interface xdg_output_impl={.destroy=destroy};
static void get_xdg_output(struct wl_client *c,struct wl_resource *r,uint32_t id,struct wl_resource *output) {
    struct output *o=wl_resource_get_user_data(output);
    uint32_t version=wl_resource_get_version(r);
    struct wl_resource *xo=wl_resource_create(c,&zxdg_output_v1_interface,version,id);
    wl_resource_set_implementation(xo,&xdg_output_impl,o,NULL);
    zxdg_output_v1_send_logical_position(xo,o->index*1280,0);
    zxdg_output_v1_send_logical_size(xo,1280,760);
    if(version>=2) {zxdg_output_v1_send_name(xo,o->name);zxdg_output_v1_send_description(xo,"headless connector fixture");}
    if(version<3)zxdg_output_v1_send_done(xo);else wl_output_send_done(output);
}
static const struct zxdg_output_manager_v1_interface manager_impl={.destroy=destroy,.get_xdg_output=get_xdg_output};
static void bind_manager(struct wl_client *c,void *data,uint32_t version,uint32_t id) {
    (void)data;struct wl_resource *r=wl_resource_create(c,&zxdg_output_manager_v1_interface,version,id);
    wl_resource_set_implementation(r,&manager_impl,NULL,NULL);
}
static int timer(void *data) {
    (void)data;uint64_t elapsed=now_ms()-started_ms;
    if(top&&!configured&&delay_configure&&elapsed>=1200)configure();
    if(callback_count&&!never_callback&&now_ms()-buffer_ms>=25&&
       (!hold_first||callbacks_sent||activations>=2)) {
        for(unsigned i=0;i<callback_count;i++) {
            wl_callback_send_done(callbacks[i],(uint32_t)now_ms());
            wl_resource_destroy(callbacks[i]);
        }
        callback_count=0;callbacks_sent++;
        if(frames>=10&&!closed) {xdg_toplevel_send_close(top);closed=1;}
    }
    assert(elapsed<8000);
    wl_event_source_timer_update(tick,10);return 0;
}
static void gone(struct wl_listener *listener,void *data) {(void)listener;(void)data;wl_display_terminate(display);}
static struct wl_listener client_destroyed={.notify=gone};
static void client_created(struct wl_listener *listener,void *data) {(void)listener;wl_client_add_destroy_listener(data,&client_destroyed);}
static struct wl_listener client_listener={.notify=client_created};
int main(void) {
    const char *mode=getenv("HDMI_TEST_MODE");
    delay_configure=mode&&!strcmp(mode,"configure-delay");
    hold_first=mode&&!strcmp(mode,"recover");
    never_callback=mode&&!strcmp(mode,"bounded");
    no_outputs=mode&&!strcmp(mode,"no-outputs");
    unfocused_default=mode&&(!strcmp(mode,"baseline")||!strcmp(mode,"no-focus"));
    const char *value=getenv("HDMI_TEST_ROTATION");rotation=value?atoi(value):0;
    value=getenv("HDMI_EXPECT_FULLSCREEN");if(value)expect_fullscreen=atoi(value);
    display=wl_display_create();assert(display);
    assert(wl_display_add_socket(display,"imu-test")==0);
    assert(wl_display_init_shm(display)==0);
    wl_global_create(display,&wl_compositor_interface,1,NULL,bind_compositor);
    wl_global_create(display,&xdg_wm_base_interface,1,NULL,bind_wm);
    if(!no_outputs) {
        wl_global_create(display,&wl_output_interface,3,&outputs[0],bind_output);
        wl_global_create(display,&wl_output_interface,3,&outputs[1],bind_output);
        wl_global_create(display,&zxdg_output_manager_v1_interface,3,NULL,bind_manager);
    }
    wl_display_add_client_created_listener(display,&client_listener);
    tick=wl_event_loop_add_timer(wl_display_get_event_loop(display),timer,NULL);assert(tick);
    started_ms=now_ms();wl_event_source_timer_update(tick,10);
    puts("READY");fflush(stdout);wl_display_run(display);
    if(!(mode&&!strcmp(mode,"missing"))) {
        if(!(mode&&!strcmp(mode,"baseline")))assert(frames>0);
        assert(activations<=3&&activations_after_callback==0);
        assert((fullscreen_requests>0)==expect_fullscreen);
        if(expect_fullscreen&&!no_outputs&&!(mode&&!strcmp(mode,"baseline"))) {
            const char *expected=getenv("HDMI_EXPECT_OUTPUT");
            assert(selected==(expected&&!strcmp(expected,"DSI-1")?0:1));
        }
        if(mode&&!strcmp(mode,"baseline")) {assert(frames==0&&callbacks_sent==0&&selected==-1);}
        else if(never_callback) {
            assert(callbacks_sent==0&&activations==3&&fullscreen_requests==3);
            assert(frames==1&&callback_requests==1&&empty_commits==3);
        }
        else {assert(frames>=10&&callbacks_sent>=10);if(hold_first)assert(activations>=2);}
    }
    printf("RESULT {\"frames\":%u,\"callbacks\":%u,\"requests\":%u,\"activations\":%u,\"late_activations\":%u,\"output\":%d,\"fullscreen_requests\":%u,\"empty_commits\":%u}\n",
           frames,callbacks_sent,callback_requests,activations,activations_after_callback,selected,fullscreen_requests,empty_commits);
    wl_display_destroy_clients(display);wl_display_destroy(display);return 0;
}
