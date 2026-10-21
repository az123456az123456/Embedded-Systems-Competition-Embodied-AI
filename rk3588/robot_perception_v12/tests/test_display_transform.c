#include "display_transform.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void) {
    struct display_transform t;
    const uint32_t source[]={1,2,3,4,5,6};uint32_t result[64];
    const uint32_t expected[4][6]={{1,2,3,4,5,6},{4,1,5,2,6,3},{6,5,4,3,2,1},{3,6,2,5,1,4}};
    for(int i=0;i<4;i++) {
        int width=i%2?2:3,height=i%2?3:2;
        assert(!display_transform_init(&t,width,height,3,2,i*90));
        display_transform_blit(&t,result,source,99);
        for(int y=0;y<height;y++)for(int x=0;x<width;x++) {
            double sx,sy;assert(result[y*width+x]==expected[i][y*width+x]);
            assert(display_transform_point(&t,x+.5,y+.5,&sx,&sy));
            assert(source[(int)sy*3+(int)sx]==result[y*width+x]);
        }
        // Every pixel's touch coordinate must select the same source pixel.
        for(int w=2;w<17;w++)for(int h=2;h<19;h++) {
            uint32_t frame[17*19];assert(!display_transform_init(&t,w,h,3,2,i*90));
            display_transform_blit(&t,frame,source,99);
            for(int y=0;y<h;y++)for(int x=0;x<w;x++) {
                double sx,sy;int valid=display_transform_point(&t,x+.5,y+.5,&sx,&sy);
                if(!valid)assert(frame[y*w+x]==99);
                else {
                    if(frame[y*w+x]!=source[(int)sy*3+(int)sx])
                        fprintf(stderr,"rotation=%d size=%dx%d pixel=%d,%d map=%d,%d point=%.17g,%.17g\n",i*90,w,h,x,y,t.xmap[x],t.ymap[y],sx,sy);
                    assert(frame[y*w+x]==source[(int)sy*3+(int)sx]);
                }
            }
        }
    }
    // Known button centers, across landscape, portrait and resized windows.
    const double buttons[3][2]={{1023,729},{1180,729},{1215,35}};
    const int sizes[3][2]={{1920,1080},{1080,1920},{800,544}};
    for(int rotation=0;rotation<360;rotation+=90)for(int size=0;size<3;size++) {
        assert(!display_transform_init(&t,sizes[size][0],sizes[size][1],1280,760,rotation));
        for(int b=0;b<3;b++) {
            double x=buttons[b][0],y=buttons[b][1],u=x,v=y,sx,sy;
            if(rotation==90){u=760-y;v=x;}
            if(rotation==180){u=1280-x;v=760-y;}
            if(rotation==270){u=y;v=1280-x;}
            assert(display_transform_point(&t,t.ox+u*t.scale,t.oy+v*t.scale,&sx,&sy));
            assert(fabs(sx-x)<.51 && fabs(sy-y)<.51);
        }
        double sx,sy;assert(!display_transform_point(&t,-1,-1,&sx,&sy));
        assert(!display_transform_point(&t,NAN,0,&sx,&sy));
    }
    assert(display_transform_init(&t,0,1,3,2,0));
    assert(display_transform_init(&t,1,1,3,2,45));
    assert(display_transform_init(&t,4097,1,3,2,0));
    puts("PASS 0/90/180/270 pixels, scaling/letterbox, portrait/landscape/resize, pointer/touch button coordinates and invalid bounds");
}
