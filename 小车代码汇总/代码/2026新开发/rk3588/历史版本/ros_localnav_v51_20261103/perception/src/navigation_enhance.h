#ifndef NAVIGATION_ENHANCE_H
#define NAVIGATION_ENHANCE_H
#include <stdint.h>
/* Map Enhancer Wizard V2 GaussianBlur(3x3, sigma=0) equivalent weights.
 * Display-only adaptation: keep observed obstacles white and unknown black.
 * Deliberately omit its binary threshold / erosion / geometry optimization. */
static int nav_reflect(int x,int n){if(n<=1)return 0;return x<0?-x:x>=n?2*n-2-x:x;}
static unsigned nav_cell_gray(char c){return c=='#'?255:c=='.'?55:0;}
static void navigation_enhance(const char *src,int w,int h,uint8_t *dst){
    for(int y=0;y<h;y++)for(int x=0;x<w;x++){
        int i=y*w+x;if(src[i]!='.'){dst[i]=(uint8_t)nav_cell_gray(src[i]);continue;}
        unsigned total=0;
        for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++)
            total+=(dx?1:2)*(dy?1:2)*nav_cell_gray(src[nav_reflect(y+dy,h)*w+nav_reflect(x+dx,w)]);
        unsigned g=(total+8)/16;dst[i]=(uint8_t)(g<55?55:g);
    }
}
#endif
