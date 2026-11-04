#include "display_transform.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int display_transform_init(struct display_transform *t,int w,int h,int sw,int sh,int rotation) {
    if(w<1 || h<1 || w>DISPLAY_MAX_SIZE || h>DISPLAY_MAX_SIZE || sw<1 || sh<1 ||
       (rotation!=0 && rotation!=90 && rotation!=180 && rotation!=270))return -1;
    int rw=rotation%180?sh:sw,rh=rotation%180?sw:sh;
    t->width=w;t->height=h;t->source_w=sw;t->source_h=sh;t->rotation=rotation;
    t->scale=fmin(w/(double)rw,h/(double)rh);
    t->ox=(w-rw*t->scale)/2;t->oy=(h-rh*t->scale)/2;
    for(int x=0;x<w;x++) {
        double u=(x+.5-t->ox)/t->scale;
        t->xmap[x]=u>=0 && u<rw?(int)u:-1;
    }
    for(int y=0;y<h;y++) {
        double v=(y+.5-t->oy)/t->scale;
        t->ymap[y]=v>=0 && v<rh?(int)v:-1;
    }
    return 0;
}
int display_transform_ppm(const struct display_transform *t,const uint32_t *src,const char *path) {
    uint32_t *pixels=malloc((size_t)t->width*t->height*sizeof(*pixels));
    if(!pixels)return -1;
    display_transform_blit(t,pixels,src,0xfff5f5f7);
    FILE *f=fopen(path,"wb");
    if(!f){free(pixels);return -1;}
    int failed=fprintf(f,"P6\n%d %d\n255\n",t->width,t->height)<0;
    for(size_t i=0;i<(size_t)t->width*t->height && !failed;i++) {
        unsigned char rgb[3]={(unsigned char)(pixels[i]>>16),(unsigned char)(pixels[i]>>8),(unsigned char)pixels[i]};
        if(fwrite(rgb,1,sizeof(rgb),f)!=sizeof(rgb))failed=1;
    }
    if(fclose(f))failed=1;
    free(pixels);return failed?-1:0;
}
int display_transform_point(const struct display_transform *t,double x,double y,double *sx,double *sy) {
    if(t->scale<=0 || !isfinite(x) || !isfinite(y))return 0;
    double u=(x-t->ox)/t->scale,v=(y-t->oy)/t->scale;
    int rw=t->rotation%180?t->source_h:t->source_w;
    int rh=t->rotation%180?t->source_w:t->source_h;
    if(u<0 || u>=rw || v<0 || v>=rh)return 0;
    // Hit-test the same source pixel used by nearest-neighbor painting. This
    // also avoids floating-point tie ambiguity on reversed integer boundaries.
    int iu=(int)u,iv=(int)v;
    switch(t->rotation) {
        case 90:*sx=iv+.5;*sy=t->source_h-1-iu+.5;break;
        case 180:*sx=t->source_w-1-iu+.5;*sy=t->source_h-1-iv+.5;break;
        case 270:*sx=t->source_w-1-iv+.5;*sy=iu+.5;break;
        default:*sx=iu+.5;*sy=iv+.5;break;
    }
    return 1;
}
void display_transform_blit(const struct display_transform *t,uint32_t *dst,const uint32_t *src,uint32_t background) {
    if(t->rotation==0 && t->width==t->source_w && t->height==t->source_h) {
        memcpy(dst,src,(size_t)t->width*t->height*sizeof(*dst));
        return;
    }
    // Small tiles keep transposed reads close in cache for 90/270 degree views.
    for(int by=0;by<t->height;by+=16)for(int bx=0;bx<t->width;bx+=16) {
        int end_y=by+16<t->height?by+16:t->height,end_x=bx+16<t->width?bx+16:t->width;
        for(int y=by;y<end_y;y++)for(int x=bx;x<end_x;x++) {
            int u=t->xmap[x],v=t->ymap[y],sx,sy;
            uint32_t color=background;
            if(u>=0 && v>=0) {
                switch(t->rotation) {
                    case 90:sx=v;sy=t->source_h-1-u;break;
                    case 180:sx=t->source_w-1-u;sy=t->source_h-1-v;break;
                    case 270:sx=t->source_w-1-v;sy=u;break;
                    default:sx=u;sy=v;break;
                }
                color=src[(size_t)sy*t->source_w+sx];
            }
            dst[(size_t)y*t->width+x]=color;
        }
    }
}
