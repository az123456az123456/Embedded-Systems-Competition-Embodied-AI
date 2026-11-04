#ifndef NAVIGATION_RENDER_H
#define NAVIGATION_RENDER_H
/* Display-only rasterization. Original occupancy and planning masks are immutable.
 * The destination is the actual viewport pixel size, including 1.5K displays. */
#include "navigation_map.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#define NAV_SKY 0x65caffu
#define NAV_RED 0xf04452u
static double nav_gray(const char *cells,const uint8_t *gray,int w,int h,int x,int y,int live){
    if(x<0||y<0||x>=w||y>=h)return 0;
    if(gray)return gray[(size_t)y*w+x]*(live?1.:.67);
    char c=cells[(size_t)y*w+x];return c=='#'?(live?255:170):c=='.'?(live?51:39):0;
}
static double nav_sample(const char *cells,const uint8_t *gray,int w,int h,double x,double y,int live){
    int ix=(int)floor(x),iy=(int)floor(y);double fx=x-ix,fy=y-iy;
    return (nav_gray(cells,gray,w,h,ix,iy,live)*(1-fx)+nav_gray(cells,gray,w,h,ix+1,iy,live)*fx)*(1-fy)+
           (nav_gray(cells,gray,w,h,ix,iy+1,live)*(1-fx)+nav_gray(cells,gray,w,h,ix+1,iy+1,live)*fx)*fy;
}
static void nav_dot(uint32_t *dst,int stride,int side,int x,int y,int radius,unsigned color){
    for(int j=-radius;j<=radius;j++)for(int i=-radius;i<=radius;i++)
        if(i*i+j*j<=radius*radius&&x+i>=0&&x+i<side&&y+j>=0&&y+j<side)
            dst[(y+j)*stride+x+i]=0xff000000u|color;
}
static void nav_bounds(const struct navigation_map *m,double *ox,double *oy,double *span){
    *ox=m->origin[0];*oy=m->origin[1];*span=m->width*m->resolution_m;
    if(!m->render_width||!m->render_height)return;
    double c=cos(m->render_origin[2]),s=sin(m->render_origin[2]);
    double minx=m->render_origin[0],maxx=minx,miny=m->render_origin[1],maxy=miny;
    for(int x=0;x<=1;x++)for(int y=0;y<=1;y++){
        double a=x*m->render_width*m->render_resolution,b=y*m->render_height*m->render_resolution;
        double wx=m->render_origin[0]+c*a-s*b,wy=m->render_origin[1]+s*a+c*b;
        minx=fmin(minx,wx);maxx=fmax(maxx,wx);miny=fmin(miny,wy);maxy=fmax(maxy,wy);
    }
    *ox=minx;*oy=miny;*span=fmax(maxx-minx,maxy-miny);
}
static void nav_view(const struct navigation_map *m,double *ox,double *oy,double *span){
    if(m->display_span>0){*ox=m->display_x;*oy=m->display_y;*span=m->display_span;return;}
    nav_bounds(m,ox,oy,span);
}
static void nav_position(const struct navigation_map *m,int side,const double *p,double *x,double *y){
    double ox,oy,span;nav_view(m,&ox,&oy,&span);double scale=side/span;*x=(p[0]-ox)*scale;*y=side-(p[1]-oy)*scale;
}
static int nav_clip(double p,double q,double *lo,double *hi){
    if(p==0)return q>=0;
    double t=q/p;if(p<0){if(t>*hi)return 0;if(t>*lo)*lo=t;}else{if(t<*lo)return 0;if(t<*hi)*hi=t;}return 1;
}
static void nav_segment(uint32_t *dst,int stride,int side,double x,double y,double xx,double yy,unsigned color,int radius){
    double dx=xx-x,dy=yy-y,lo=0,hi=1;
    if(!nav_clip(-dx,x,&lo,&hi)||!nav_clip(dx,side-1-x,&lo,&hi)||!nav_clip(-dy,y,&lo,&hi)||!nav_clip(dy,side-1-y,&lo,&hi))return;
    xx=x+dx*hi;yy=y+dy*hi;x+=dx*lo;y+=dy*lo;int n=(int)ceil(fmax(fabs(xx-x),fabs(yy-y)));
    for(int i=0;i<=n;i++){double t=n?(double)i/n:0;nav_dot(dst,stride,side,(int)lround(x+(xx-x)*t),(int)lround(y+(yy-y)*t),radius,color);}
}
static void nav_path(const struct navigation_map *m,uint32_t *dst,int stride,int side,const double p[][2],unsigned n,unsigned color,int radius){
    for(unsigned i=1;i<n;i++){double x,y,xx,yy;nav_position(m,side,p[i-1],&x,&y);nav_position(m,side,p[i],&xx,&yy);nav_segment(dst,stride,side,x,y,xx,yy,color,radius);}
}
static void navigation_paint(const struct navigation_map *m,uint32_t *dst,int stride,int side,uint64_t now){
    if(!m->loaded||side<=0||m->width<=0||m->resolution_m<=0)return;
    int live=navigation_map_fresh(m,now),detail=m->render_width&&m->render_height;
    int w=detail?(int)m->render_width:64,h=detail?(int)m->render_height:64;
    const char *cells=detail?m->render_cells:m->cells;
    double res=detail?m->render_resolution:m->resolution_m;
    double ox=detail?m->render_origin[0]:m->origin[0],oy=detail?m->render_origin[1]:m->origin[1],yaw=detail?m->render_origin[2]:0;
    double vx,vy,span;nav_view(m,&vx,&vy,&span);
    double c=cos(yaw),s=sin(yaw),pixel=span/side;
    /* Smooth only the display colours. Supersampling limits aliasing when the
       source map is larger than the screen, without obstacle-winning 64x64 pooling. */
    int samples=(int)ceil(pixel/res);if(samples<1)samples=1;if(samples>4)samples=4;
    for(int y=0;y<side;y++)for(int x=0;x<side;x++){
        double v=0;
        for(int j=0;j<samples;j++)for(int i=0;i<samples;i++){
            double dx=vx+(x+(i+.5)/samples)*pixel-ox;
            double dy=vy+(side-y-(j+.5)/samples)*pixel-oy;
            v+=nav_sample(cells,detail&&m->display_gray_ready?m->display_gray:NULL,w,h,(c*dx+s*dy)/res-.5,(-s*dx+c*dy)/res-.5,live);
        }
        unsigned g=(unsigned)lround(v/(samples*samples));dst[y*stride+x]=0xff000000u|(g<<16)|(g<<8)|g;
    }
    int radius=(int)fmax(1,lround(side/300.));
    nav_path(m,dst,stride,side,m->trajectory,m->trajectory_count,0x81959e,1);
    nav_path(m,dst,stride,side,m->path,m->path_count,NAV_SKY,radius);
    if(m->has_goal){double x,y;nav_position(m,side,m->goal,&x,&y);if(x>=-20&&x<side+20&&y>=-20&&y<side+20){
        nav_dot(dst,stride,side,(int)lround(x),(int)lround(y),radius*3+2,NAV_SKY);
        nav_dot(dst,stride,side,(int)lround(x),(int)lround(y),radius+2,0xffffff);}}
    if(live&&m->pose_valid){double x,y;nav_position(m,side,m->pose,&x,&y);
        if(x>=-40&&x<side+40&&y>=-40&&y<side+40){
            /* User-confirmed forward icon correction, display only. Never
               modify ROS TF, goal coordinates or motor/FPGA direction. */
            double fx=-cos(m->pose[2]),fy=sin(m->pose[2]),r=fmax(8,side*.019);
            for(int dy=-(int)ceil(r);dy<=(int)ceil(r);dy++)for(int dx=-(int)ceil(r);dx<=(int)ceil(r);dx++){
                double along=dx*fx+dy*fy,across=-dx*fy+dy*fx;
                if(along>=-.65*r&&along<=r&&fabs(across)<=.65*(r-along)/1.65)
                    nav_dot(dst,stride,side,(int)lround(x)+dx,(int)lround(y)+dy,0,NAV_RED);
            }
        }
    }
}
#endif
