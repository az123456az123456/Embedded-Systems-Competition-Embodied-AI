#include "display_output.h"
#include "xdg-output-client-protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct display_output {
    struct display_output *next;
    struct wl_output *wl;
    struct zxdg_output_v1 *xdg;
    uint32_t global_name,version;
    int x,y,width,height;
    char name[128],description[256],make[64],model[128];
};
static void copy(char *dest,size_t length,const char *source) {
    snprintf(dest,length,"%s",source?source:"");
}
static void geometry(void *data,struct wl_output *wl,int32_t x,int32_t y,
                     int32_t pw,int32_t ph,int32_t subpixel,const char *make,
                     const char *model,int32_t transform) {
    (void)wl;(void)pw;(void)ph;(void)subpixel;(void)transform;
    struct display_output *o=data;o->x=x;o->y=y;
    copy(o->make,sizeof(o->make),make);copy(o->model,sizeof(o->model),model);
}
static void mode(void *data,struct wl_output *wl,uint32_t flags,int32_t width,
                 int32_t height,int32_t refresh) {
    (void)wl;(void)refresh;struct display_output *o=data;
    if(flags&WL_OUTPUT_MODE_CURRENT) {o->width=width;o->height=height;}
}
static void done(void *data,struct wl_output *wl) {(void)data;(void)wl;}
static void scale(void *data,struct wl_output *wl,int32_t value) {(void)data;(void)wl;(void)value;}
static void name(void *data,struct wl_output *wl,const char *value) {
    (void)wl;struct display_output *o=data;copy(o->name,sizeof(o->name),value);
}
static void description(void *data,struct wl_output *wl,const char *value) {
    (void)wl;struct display_output *o=data;copy(o->description,sizeof(o->description),value);
}
static const struct wl_output_listener output_listener={
    .geometry=geometry,.mode=mode,.done=done,.scale=scale,.name=name,.description=description
};
static void logical_position(void *data,struct zxdg_output_v1 *xdg,int32_t x,int32_t y) {
    (void)xdg;struct display_output *o=data;o->x=x;o->y=y;
}
static void logical_size(void *data,struct zxdg_output_v1 *xdg,int32_t width,int32_t height) {
    (void)xdg;struct display_output *o=data;o->width=width;o->height=height;
}
static void xdg_done(void *data,struct zxdg_output_v1 *xdg) {(void)data;(void)xdg;}
static void xdg_name(void *data,struct zxdg_output_v1 *xdg,const char *value) {
    (void)xdg;struct display_output *o=data;copy(o->name,sizeof(o->name),value);
}
static void xdg_description(void *data,struct zxdg_output_v1 *xdg,const char *value) {
    (void)xdg;struct display_output *o=data;copy(o->description,sizeof(o->description),value);
}
static const struct zxdg_output_v1_listener xdg_listener={
    .logical_position=logical_position,.logical_size=logical_size,.done=xdg_done,
    .name=xdg_name,.description=xdg_description
};
static void attach_xdg(struct display_outputs *outputs,struct display_output *o) {
    if(outputs->manager && !o->xdg) {
        o->xdg=zxdg_output_manager_v1_get_xdg_output(outputs->manager,o->wl);
        if(o->xdg)zxdg_output_v1_add_listener(o->xdg,&xdg_listener,o);
    }
}
void display_outputs_global(struct display_outputs *outputs,struct wl_registry *registry,
                            uint32_t global_name,const char *interface,uint32_t version) {
    if(!strcmp(interface,"wl_output")) {
        struct display_output *o=calloc(1,sizeof(*o));
        if(!o) {fprintf(stderr,"DISPLAY output discovery: allocation failed\n");return;}
        o->global_name=global_name;o->version=version<4?version:4;
        o->wl=wl_registry_bind(registry,global_name,&wl_output_interface,o->version);
        if(!o->wl) {free(o);return;}
        wl_output_add_listener(o->wl,&output_listener,o);
        /* Preserve announcement order for compositors with no connector names. */
        struct display_output **tail=&outputs->head;while(*tail)tail=&(*tail)->next;*tail=o;
        attach_xdg(outputs,o);
    } else if(!strcmp(interface,"zxdg_output_manager_v1") && !outputs->manager) {
        outputs->manager_name=global_name;
        outputs->manager=wl_registry_bind(registry,global_name,
            &zxdg_output_manager_v1_interface,version<3?version:3);
        for(struct display_output *o=outputs->head;o;o=o->next)attach_xdg(outputs,o);
    }
}
static void destroy_output(struct display_output *o) {
    if(o->xdg)zxdg_output_v1_destroy(o->xdg);
    if(o->wl) {
        if(o->version>=WL_OUTPUT_RELEASE_SINCE_VERSION)wl_output_release(o->wl);
        else wl_output_destroy(o->wl);
    }
    free(o);
}
void display_outputs_remove(struct display_outputs *outputs,uint32_t global_name) {
    struct display_output **p=&outputs->head;
    while(*p) {
        struct display_output *o=*p;
        if(o->global_name==global_name) {
            *p=o->next;if(outputs->selected==o)outputs->selected=NULL;
            destroy_output(o);return;
        }
        p=&o->next;
    }
    if(outputs->manager && outputs->manager_name==global_name) {
        zxdg_output_manager_v1_destroy(outputs->manager);outputs->manager=NULL;
    }
}
static int hdmi(const struct display_output *o) {
    return !strncmp(o->name,"HDMI",4) || !strncmp(o->name,"hdmi",4);
}
int display_outputs_choose(struct display_outputs *outputs,const char *requested) {
    int automatic=!requested || !*requested || !strcmp(requested,"auto");
    outputs->selected=NULL;
    for(struct display_output *o=outputs->head;o;o=o->next) {
        fprintf(stderr,"DISPLAY available_output=%s mode=%dx%d position=%d,%d make=%s model=%s\n",
            o->name[0]?o->name:"unnamed",o->width,o->height,o->x,o->y,o->make,o->model);
        if(!automatic && !strcmp(requested,o->name))outputs->selected=o;
        else if(automatic && (!outputs->selected || (hdmi(o)&&!hdmi(outputs->selected))))outputs->selected=o;
    }
    if(!automatic && !outputs->selected) {
        fprintf(stderr,"DISPLAY requested output '%s' is unavailable; set PERCEPTION_OUTPUT=auto or an available connector name\n",requested);
        return -1;
    }
    fprintf(stderr,"DISPLAY output=%s selection=%s input_focus_required=0\n",
        display_outputs_name(outputs),automatic?"auto-hdmi-preferred":"explicit");
    return 0;
}
struct wl_output *display_outputs_selected(const struct display_outputs *outputs) {
    return outputs->selected?outputs->selected->wl:NULL;
}
const char *display_outputs_name(const struct display_outputs *outputs) {
    if(!outputs->selected)return "compositor-default";
    return outputs->selected->name[0]?outputs->selected->name:"unnamed";
}
void display_outputs_destroy(struct display_outputs *outputs) {
    while(outputs->head) {
        struct display_output *o=outputs->head;outputs->head=o->next;destroy_output(o);
    }
    if(outputs->manager)zxdg_output_manager_v1_destroy(outputs->manager);
    memset(outputs,0,sizeof(*outputs));
}
