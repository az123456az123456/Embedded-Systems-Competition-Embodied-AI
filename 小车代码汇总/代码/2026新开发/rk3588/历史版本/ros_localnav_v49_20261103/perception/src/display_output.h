#ifndef DISPLAY_OUTPUT_H
#define DISPLAY_OUTPUT_H
#include <stddef.h>
#include <stdint.h>
#include <wayland-client.h>

struct display_output;
struct zxdg_output_manager_v1;
struct display_outputs {
    struct display_output *head;
    struct display_output *selected;
    struct zxdg_output_manager_v1 *manager;
    uint32_t manager_name;
};

/* Discover outputs independently of pointer/touch focus. xdg-output supplies
 * connector names on this board, whose wl_output global is version 3. */
void display_outputs_global(struct display_outputs *outputs,struct wl_registry *registry,
                            uint32_t name,const char *interface,uint32_t version);
void display_outputs_remove(struct display_outputs *outputs,uint32_t name);
int display_outputs_choose(struct display_outputs *outputs,const char *requested);
struct wl_output *display_outputs_selected(const struct display_outputs *outputs);
const char *display_outputs_name(const struct display_outputs *outputs);
void display_outputs_destroy(struct display_outputs *outputs);
#endif
