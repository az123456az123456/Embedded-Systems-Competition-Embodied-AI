#ifndef DISPLAY_TRANSFORM_H
#define DISPLAY_TRANSFORM_H
#include <stdint.h>
#define DISPLAY_MAX_SIZE 4096
struct display_transform {
    int width,height,source_w,source_h,rotation;
    double scale,ox,oy;
    int xmap[DISPLAY_MAX_SIZE],ymap[DISPLAY_MAX_SIZE];
};
int display_transform_init(struct display_transform *,int width,int height,int source_w,int source_h,int rotation);
/* Convert Wayland surface coordinates to the original dashboard coordinates. */
int display_transform_point(const struct display_transform *,double x,double y,double *sx,double *sy);
void display_transform_blit(const struct display_transform *,uint32_t *dst,const uint32_t *src,uint32_t background);
int display_transform_ppm(const struct display_transform *,const uint32_t *src,const char *path);
#endif
