#ifndef CAMERA_PIXELS_H
#define CAMERA_PIXELS_H
#include <stdint.h>
#include <string.h>
static void nv12_crop(const uint8_t *src,unsigned stride,unsigned h,
                      unsigned x,unsigned w,uint8_t *dst) {
    for(unsigned y=0;y<h;y++)memcpy(dst+y*w,src+y*stride+x,w);
    for(unsigned y=0;y<h/2;y++)memcpy(dst+w*h+y*w,src+stride*h+y*stride+x,w);
}
/* RGB565 LE -> NV12, BT.601 limited range. Even crop geometry required. */
static void rgb565_nv12(const uint8_t *src,unsigned stride,unsigned x0,
                       unsigned w,unsigned h,uint8_t *dst) {
    for(unsigned y=0;y<h;y+=2)for(unsigned x=0;x<w;x+=2) {
        int rs=0,gs=0,bs=0;
        for(unsigned dy=0;dy<2;dy++)for(unsigned dx=0;dx<2;dx++) {
            unsigned i=2*((y+dy)*stride+x0+x+dx);
            unsigned v=src[i]|((unsigned)src[i+1]<<8);
            int r=(v>>11)*255/31,g=((v>>5)&63)*255/63,b=(v&31)*255/31;
            dst[(y+dy)*w+x+dx]=(uint8_t)(((66*r+129*g+25*b+128)>>8)+16);
            rs+=r;gs+=g;bs+=b;
        }
        int r=(rs+2)/4,g=(gs+2)/4,b=(bs+2)/4;
        unsigned uv=w*h+(y/2)*w+x;
        dst[uv]=(uint8_t)(((-38*r-74*g+112*b+128)>>8)+128);
        dst[uv+1]=(uint8_t)(((112*r-94*g-18*b+128)>>8)+128);
    }
}
#endif
