#ifndef CAMERA_RATE_H
#define CAMERA_RATE_H
#include <string.h>
#ifndef CAMERA_DEFAULT_FPS
#define CAMERA_DEFAULT_FPS 15
#endif
#if CAMERA_DEFAULT_FPS != 15 && CAMERA_DEFAULT_FPS != 20 && CAMERA_DEFAULT_FPS != 30
#error CAMERA_DEFAULT_FPS must be 15, 20 or 30
#endif
/* This is the output stream interval, never a source of synthetic frames.
 * A matching sensor image is needed to change the real RAW arrival rate. */
static int camera_target_fps_valid(unsigned fps) {
    return fps==15 || fps==20 || fps==30;
}
static int camera_target_fps_parse(const char *value,unsigned *fps) {
    if(!value){*fps=CAMERA_DEFAULT_FPS;return 0;}
    if(!strcmp(value,"15"))*fps=15;
    else if(!strcmp(value,"20"))*fps=20;
    else if(!strcmp(value,"30"))*fps=30;
    else return -1;
    return 0;
}
#endif
