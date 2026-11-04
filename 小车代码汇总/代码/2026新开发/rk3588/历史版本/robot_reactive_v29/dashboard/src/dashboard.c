#include "dashboard.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "font_atlas.h"

#define BG 0xf5f5f7
#define INK 0x1d1d1f
#define MUTED 0x86868b
#define LINE 0xe9e9ee
#define TEAL 0x007aff
#define GREEN 0x34a880
#define GOLD 0xd58b2a
#define CAM_X 48
#define CAM_Y 137
#define CAMERA_X 28
#define CAMERA_Y 20
#define CAMERA_W 904
#define CAMERA_H 720
#define POSE_X 952
#define POSE_Y 20
#define POSE_W 300
#define POSE_H 400
#define ACCEL_X 952
#define ACCEL_Y 440
#define ACCEL_W 300
#define ACCEL_H 300

static unsigned char *atlas;
static double clamp(double x, double lo, double hi) { return fmax(lo, fmin(hi, x)); }
static void pixel(struct dashboard *d, int x, int y, unsigned c, double a) {
    if ((unsigned)x >= VIEW_W || (unsigned)y >= VIEW_H || a <= 0) return;
    if (a >= 1) {
        d->pixels[y * VIEW_W + x] = 0xff000000 | (c & 0xffffff);
        return;
    }
    a = fmin(1, a);
    unsigned old = d->pixels[y * VIEW_W + x];
    unsigned r = ((c >> 16) & 255) * a + ((old >> 16) & 255) * (1-a);
    unsigned g = ((c >> 8) & 255) * a + ((old >> 8) & 255) * (1-a);
    unsigned b = (c & 255) * a + (old & 255) * (1-a);
    d->pixels[y * VIEW_W + x] = 0xff000000 | (r << 16) | (g << 8) | b;
}
static void box(struct dashboard *d, double x, double y, double w, double h,
                double r, unsigned c) {
    const int left = fmax(0, floor(x)), right = fmin(VIEW_W, ceil(x+w));
    const unsigned solid = 0xff000000 | (c & 0xffffff);
    for (int j = fmax(0, floor(y)); j < fmin(VIEW_H, ceil(y+h)); j++) {
        double dy = fmax(fabs(j+.5-(y+h/2))-(h/2-r), 0);
        int fill_left = right, fill_right = right;
        /* Only the proven opaque strips take this path. Keep the original
           distance/alpha calculation around every rounded edge and corner. */
        if (r >= .5 && dy == 0) {
            fill_left = fmax(left, ceil(x+1));
            fill_right = fmin(right, floor(x+w-1));
        } else if (r >= .5 && dy < r-.5) {
            fill_left = fmax(left, ceil(x+r));
            fill_right = fmin(right, floor(x+w-r));
        }
        if (fill_left > fill_right) fill_left = fill_right = right;
        for (int i = left; i < fill_left; i++) {
            double dx = fmax(fabs(i+.5-(x+w/2))-(w/2-r), 0);
            pixel(d, i, j, c, clamp(r+.5-hypot(dx,dy), 0, 1));
        }
        for (int i = fill_left; i < fill_right; i++) d->pixels[j*VIEW_W+i] = solid;
        for (int i = fill_right; i < right; i++) {
            double dx = fmax(fabs(i+.5-(x+w/2))-(w/2-r), 0);
            pixel(d, i, j, c, clamp(r+.5-hypot(dx,dy), 0, 1));
        }
    }
}
static void line(struct dashboard *d, double x, double y, double xx, double yy,
                 double width, unsigned c) {
    double vx = xx-x, vy = yy-y, len = vx*vx+vy*vy;
    for (int j = fmax(0, floor(fmin(y,yy)-width)); j <= fmin(VIEW_H-1, ceil(fmax(y,yy)+width)); j++)
        for (int i = fmax(0, floor(fmin(x,xx)-width)); i <= fmin(VIEW_W-1, ceil(fmax(x,xx)+width)); i++) {
            double t = len ? clamp(((i+.5-x)*vx+(j+.5-y)*vy)/len, 0, 1) : 0;
            pixel(d, i, j, c, clamp(width/2+.5-hypot(i+.5-x-t*vx,j+.5-y-t*vy), 0, 1));
        }
}
static unsigned utf8(const char **p) {
    const unsigned char *s = (const unsigned char *)*p;
    unsigned c = *s++; int n = 0;
    if (c >= 0xf0) { c &= 7; n = 3; }
    else if (c >= 0xe0) { c &= 15; n = 2; }
    else if (c >= 0xc0) { c &= 31; n = 1; }
    while (n-- && *s && (*s & 0xc0) == 0x80) c = (c << 6) | (*s++ & 63);
    *p = (const char *)s; return c;
}
static int glyph(unsigned cp) {
    for (int k = 0; k < FONT_COUNT; k++) if (font_glyphs[k].cp == cp) return k;
    return -1;
}
static double text_width(double size, const char *s) {
    double width = 0;
    while (*s) { int k = glyph(utf8(&s)); if (k >= 0) width += font_glyphs[k].advance*size/48.; }
    return width;
}
static void text(struct dashboard *d, double x, double baseline, double size,
                 unsigned c, const char *s) {
    double scale = size/48.;
    while (*s) {
        int k = glyph(utf8(&s)); if (k < 0) continue;
        const unsigned char *p = atlas + k*4096;
        double left = x-2*scale, top = baseline-52*scale;
        for (int j = fmax(0, floor(top)); j < fmin(VIEW_H, ceil(top+64*scale)); j++)
            for (int i = fmax(0, floor(left)); i < fmin(VIEW_W, ceil(left+64*scale)); i++) {
                double sx = (i+.5-left)/scale-.5, sy = (j+.5-top)/scale-.5;
                int a = floor(sx), b = floor(sy);
                if (a < 0 || b < 0 || a >= 63 || b >= 63) continue;
                double fx = sx-a, fy = sy-b;
                double v = p[b*64+a]*(1-fx)*(1-fy) + p[b*64+a+1]*fx*(1-fy) +
                           p[(b+1)*64+a]*(1-fx)*fy + p[(b+1)*64+a+1]*fx*fy;
                pixel(d, i, j, c, v/255.);
            }
        x += font_glyphs[k].advance*scale;
    }
}
static void text_fit(struct dashboard *d, double x, double baseline, double size,
                     unsigned c, double width, const char *s) {
    char clipped[256]; size_t used = 0; const char *p = s;
    double advance = 0, dots = text_width(size, "...");
    if (text_width(size, s) <= width) { text(d,x,baseline,size,c,s); return; }
    while (*p) {
        const char *start = p; int k = glyph(utf8(&p));
        double next = k >= 0 ? font_glyphs[k].advance*size/48. : 0;
        size_t len = (size_t)(p-start);
        if (advance+next+dots > width || used+len+4 >= sizeof(clipped)) break;
        memcpy(clipped+used,start,len); used += len; advance += next;
    }
    memcpy(clipped+used,"...",4); text(d,x,baseline,size,c,clipped);
}
static void card(struct dashboard *d,int x,int y,int w,int h) {
    box(d,x,y+3,w,h,18,0xeaeaf0);box(d,x,y,w,h,18,0xffffff);
}
static double fitted_size(double maximum, double width, const char *label) {
    double measured=text_width(maximum,label);
    return measured>width ? floor(maximum*width/measured) : maximum;
}
static void centered_text(struct dashboard *d, double center, double baseline,
                          double maximum, double width, unsigned color,
                          const char *label) {
    double size=fmax(8,fitted_size(maximum,width,label));
    text(d,center-text_width(size,label)/2,baseline,size,color,label);
}
static int vision_fresh(const struct vision_snapshot *v, uint64_t now) {
    return v->have_frame && now >= v->timestamp_ms && now-v->timestamp_ms <= 1000;
}
static void camera_wait(struct dashboard *d, int stale) {
    /* Preserve a received frame as a visibly paused image; never present old boxes as live. */
    for (int y = 0; y < VISION_H; y++) for (int x = 0; x < VISION_W; x++) {
        if (stale) pixel(d,CAM_X+x,CAM_Y+y,0x163237,.72);
        else {
            unsigned c = (x%72 == 0 || y%72 == 0) ? 0x26464b : 0x203e43;
            d->pixels[(CAM_Y+y)*VIEW_W+CAM_X+x] = 0xff000000|c;
        }
    }
    int cx = CAM_X+VISION_W/2, cy = CAM_Y+VISION_H/2-22;
    box(d,cx-29,cy-20,58,42,10,0x527579);
    box(d,cx-24,cy-15,48,32,7,0x24474c);
    box(d,cx-9,cy-8,18,18,9,0x7ea7aa);
    line(d,cx+29,cy-10,cx+41,cy-18,3,0x7ea7aa);
    line(d,cx+41,cy-18,cx+41,cy+20,3,0x7ea7aa);
    line(d,cx+41,cy+20,cx+29,cy+12,3,0x7ea7aa);
    const char *label = stale ? "画面已暂停" : "等待摄像头";
    text(d,cx-text_width(21,label)/2,cy+63,21,0xe0eeee,label);
    const char *hint = stale ? "等待新的画面" : "连接后将自动显示";
    text(d,cx-text_width(13,hint)/2,cy+91,13,0x9db8ba,hint);
}
static void detections(struct dashboard *d) {
    struct vision_snapshot *v = &d->vision;
    const unsigned colors[] = {0x41d6bd,0x7fb9ff,0xf5c879,0xc6b2ef};
    int count = (int)clamp(v->count,0,VISION_OBJECTS);
    for (int i = 0; i < count; i++) {
        struct vision_object *o = &v->objects[i];
        if (!isfinite(o->left) || !isfinite(o->top) || !isfinite(o->right) ||
            !isfinite(o->bottom) || !isfinite(o->score) || o->right <= o->left || o->bottom <= o->top) continue;
        double x = CAM_X+clamp(o->left,2,VISION_W-3), y = CAM_Y+clamp(o->top,2,VISION_H-3);
        double r = CAM_X+clamp(o->right,2,VISION_W-3), b = CAM_Y+clamp(o->bottom,2,VISION_H-3);
        if (r <= x || b <= y) continue;
        unsigned c = colors[(unsigned)o->class_id%4];
        if(v->boxes_delayed && !v->motion_valid[i]) {
            /* Dashed boxes are dated detection results, not tracked current positions. */
            for(double xx=x;xx<r;xx+=14){line(d,xx,y,fmin(r,xx+8),y,2,c);line(d,xx,b,fmin(r,xx+8),b,2,c);}
            for(double yy=y;yy<b;yy+=14){line(d,x,yy,x,fmin(b,yy+8),2,c);line(d,r,yy,r,fmin(b,yy+8),2,c);}
        } else {
            line(d,x,y,r,y,2,c); line(d,x,y,x,b,2,c); line(d,r,y,r,b,2,c); line(d,x,b,r,b,2,c);
        }
        const char *eye = o->eye == 0 ? "L  " : o->eye == 1 ? "R  " : "";
        char label[100]; snprintf(label,sizeof(label),"%s%.63s  %.0f%%",eye,o->name,clamp(o->score,0,1)*100);
        double w = fmin(230,text_width(13,label)+18), ty = fmax(CAM_Y,y-28);
        double tx = fmin(x,CAM_X+VISION_W-w);
        box(d,tx,ty,w,26,5,c); text_fit(d,tx+8,ty+18,13,0x153a3c,w-16,label);
    }
}
struct car_pose { double cx,cy,c,s; };
static void car_line(struct dashboard *d,const struct car_pose *p,
                     double x,double y,double xx,double yy,double width,unsigned color) {
    line(d,p->cx+x*p->c+y*p->s,p->cy-x*p->s+y*p->c,
         p->cx+xx*p->c+yy*p->s,p->cy-xx*p->s+yy*p->c,width,color);
}
static void car_path(struct dashboard *d,const struct car_pose *p,
                     const double points[][2],size_t count,double width,unsigned color) {
    for(size_t i=0;i<count;i++) {
        size_t next=(i+1)%count;
        car_line(d,p,points[i][0],points[i][1],points[next][0],points[next][1],width,color);
    }
}
static void car_top_view(struct dashboard *d, double yaw, int fresh) {
    const double cx=1102, cy=300, radius=68;
    fresh=fresh && isfinite(yaw);
    double angle=fresh?remainder(yaw,360)*IMU_PI/180:0;
    const struct car_pose pose={cx,cy,cos(angle),sin(angle)};
    unsigned body=fresh?INK:0xa5b5b6, accent=fresh?TEAL:0xb3c2c3;
    box(d,973,210,258,180,14,0xf6f9f8);
    text(d,985,231,11,MUTED,"俯视");
    text(d,985,245,8,MUTED,"TOP VIEW");
    text(d,1192,231,9,MUTED,"YAW");
    /* A fixed reference ring; this is relative heading, not magnetic north. */
    for (int i=0;i<96;i++) {
        double a=i*2*IMU_PI/96,b=(i+1)*2*IMU_PI/96;
        line(d,cx+radius*cos(a),cy+radius*sin(a),cx+radius*cos(b),cy+radius*sin(b),1,0xe0e9e7);
    }
    for (int i=0;i<4;i++) {
        double a=i*IMU_PI/2, inner=i==3?53:56;
        line(d,cx+inner*cos(a),cy+inner*sin(a),cx+radius*cos(a),cy+radius*sin(a),
             i==3?2:1.3,i==3?0x92bbb5:0xc3d4d0);
    }
    for (int i=-49;i<=49;i+=10) {
        line(d,cx+i,cy,cx+i+3,cy,1,0xe5edeb);
        line(d,cx,cy+i,cx,cy+i+3,1,0xe5edeb);
    }
    /* Four separate wheels and a chamfered chassis, all in one yaw transform. */
    for (int side=-1;side<=1;side+=2) for (int axle=-1;axle<=1;axle+=2) {
        double x=side*28.5,y=axle*23;
        const double wheel[][2]={{x-4.5,y-11},{x+4.5,y-11},{x+4.5,y+11},{x-4.5,y+11}};
        car_line(d,&pose,side*21,y,side*24,y,1.7,accent);
        car_path(d,&pose,wheel,4,2.2,body);
    }
    const double chassis[][2]={{-15,-39},{15,-39},{21,-29},{21,30},
                               {15,38},{-15,38},{-21,30},{-21,-29}};
    const double cabin[][2]={{-12,-15},{12,-15},{15,13},{-15,13}};
    car_path(d,&pose,chassis,8,2.2,body);
    car_path(d,&pose,cabin,4,1.5,accent);
    car_line(d,&pose,-13,-9,13,-9,1,accent);
    car_line(d,&pose,-8,28,8,28,1.6,accent);
    car_line(d,&pose,-12,-32,-7,-32,2.7,accent);
    car_line(d,&pose,7,-32,12,-32,2.7,accent);
    /* Small nose arrow makes forward unambiguous at every heading. */
    car_line(d,&pose,0,-42,0,-51,2.2,accent);
    car_line(d,&pose,-4,-47,0,-52,2.2,accent);
    car_line(d,&pose,0,-52,4,-47,2.2,accent);
    if (!fresh) {
        box(d,cx-55,366,110,20,7,0xf6f9f8);
        text(d,cx-text_width(12,"等待有效数据")/2,380,12,MUTED,"等待有效数据");
    }
}
static void sonar_readings(struct dashboard *d,uint64_t now) {
    const char *directions[]={"前方","左侧","右侧"};
    const unsigned colors[]={TEAL,GREEN,GOLD};
    for(unsigned i=0;i<3;i++) {
        int x=CAM_X+(int)i*294;
        box(d,x,639,276,82,12,0xf7f9fc);
        box(d,x+16,654,5,15,2.5,colors[i]);
        text(d,x+30,668,17,MUTED,directions[i]);
        char value[32];
        enum sensor_distance_status status=sensor_distance_text(&d->sensors,i,now,value,sizeof(value));
        text(d,x+16,707,30,status==SENSOR_VALID?INK:MUTED,value);
        const char *hint=status==SENSOR_WAITING?(d->sensors.have_sample?"等待测距":"未连接"):status==SENSOR_STALE?"数据过期":status==SENSOR_FAULT?"测距异常":"";
        text_fit(d,x+155,705,14,MUTED,105,hint);
    }
}
/* Map drawing stays within the existing camera region. No camera/NPU state is
   changed by the tab: even while viewing navigation, phone video stays live. */
static void map_solid(struct dashboard *d,int x,int y,int w,int h,unsigned color) {
    int right=fmin(CAM_X+VISION_W,x+w),bottom=fmin(CAM_Y+VISION_H,y+h);
    x=fmax(CAM_X,x);y=fmax(CAM_Y,y);
    for(int yy=y;yy<bottom;yy++)for(int xx=x;xx<right;xx++)d->pixels[yy*VIEW_W+xx]=0xff000000|color;
}
static int map_clip(double p,double q,double *lo,double *hi) {
    if(p==0)return q>=0;
    double t=q/p;
    if(p<0){if(t>*hi)return 0;if(t>*lo)*lo=t;}
    else {if(t<*lo)return 0;if(t<*hi)*hi=t;}
    return 1;
}
static void map_line(struct dashboard *d,double x,double y,double xx,double yy,
                     int left,int top,int size,unsigned color,int width) {
    double dx=xx-x,dy=yy-y,lo=0,hi=1;
    if(!map_clip(-dx,x-left,&lo,&hi)||!map_clip(dx,left+size-1-x,&lo,&hi)||
       !map_clip(-dy,y-top,&lo,&hi)||!map_clip(dy,top+size-1-y,&lo,&hi))return;
    xx=x+dx*hi;yy=y+dy*hi;x+=dx*lo;y+=dy*lo;
    unsigned steps=(unsigned)ceil(fmax(fabs(xx-x),fabs(yy-y)));
    for(unsigned i=0;i<=steps;i++) {
        double t=steps?(double)i/steps:0;
        int px=(int)lround(x+(xx-x)*t),py=(int)lround(y+(yy-y)*t);
        for(int j=-width/2;j<=width/2;j++)for(int k=-width/2;k<=width/2;k++)
            if(px+k>=left&&px+k<left+size&&py+j>=top&&py+j<top+size)
                d->pixels[(py+j)*VIEW_W+px+k]=0xff000000|color;
    }
}
static void map_point(const struct navigation_map *m,int left,int top,int size,
                      const double point[2],double *x,double *y) {
    double scale=size/(m->width*m->resolution_m);
    *x=left+(point[0]-m->origin[0])*scale;
    *y=top+size-(point[1]-m->origin[1])*scale;
}
static void map_route(struct dashboard *d,const double points[][2],unsigned n,
                      int left,int top,int size,unsigned color,int width) {
    for(unsigned i=1;i<n;i++) {
        double x,y,xx,yy;map_point(&d->navigation,left,top,size,points[i-1],&x,&y);
        map_point(&d->navigation,left,top,size,points[i],&xx,&yy);
        map_line(d,x,y,xx,yy,left,top,size,color,width);
    }
}
static void navigation_view(struct dashboard *d,uint64_t now) {
    const struct navigation_map *m=&d->navigation;
    const int left=264,top=149,size=464;
    const int live=navigation_map_fresh(m,now);
    map_solid(d,CAM_X,CAM_Y,VISION_W,VISION_H,0xf7f8fa);
    text(d,68,179,22,INK,"局部地图");
    if(!m->loaded) {
        centered_text(d,CAM_X+VISION_W/2,330,27,640,MUTED,"等待导航地图");
        centered_text(d,CAM_X+VISION_W/2,372,17,740,MUTED,"启动导航程序后自动显示");
        return;
    }
    const char *status=live?"实时更新":
        !strcmp(m->reason,"map_file_unavailable")?"导航服务未连接":
        (!m->available||!m->pose_valid)?"定位未就绪":"地图已暂停";
    text(d,68,212,17,live?GREEN:GOLD,status);
    char label[64];snprintf(label,sizeof(label),"%.1f × %.1f m",m->width*m->resolution_m,m->height*m->resolution_m);
    text(d,68,247,16,MUTED,label);
    if(!live) {text(d,68,282,15,MUTED,"仅显示历史地图");text(d,68,306,15,MUTED,"等待有效定位");}
    const unsigned free_color=live?0xffffff:0xf3f3f3,blocked=live?0x35404b:0x93989c,unknown=0xe3e6eb;
    for(int py=0;py<size;py++) {
        int gy=63-py*64/size;
        for(int px=0;px<size;px++) {
            char c=m->cells[gy*64+px*64/size];
            d->pixels[(top+py)*VIEW_W+left+px]=0xff000000|(c=='.'?free_color:c=='#'?blocked:unknown);
        }
    }
    for(int k=0;k<=8;k++) {
        int g=k*(size-1)/8;
        map_line(d,left+g,top,left+g,top+size-1,left,top,size,0xd9dde2,1);
        map_line(d,left,top+g,left+size-1,top+g,left,top,size,0xd9dde2,1);
    }
    map_route(d,m->trajectory,m->trajectory_count,left,top,size,live?0x9ebbb1:0xb7babb,1);
    map_route(d,m->path,m->path_count,left,top,size,live?TEAL:0x9eaab6,3);
    if(m->has_goal) {
        double x,y;map_point(m,left,top,size,m->goal,&x,&y);
        map_line(d,x-6,y-6,x+6,y+6,left,top,size,GOLD,3);
        map_line(d,x-6,y+6,x+6,y-6,left,top,size,GOLD,3);
    }
    if(live) {
        double x,y;map_point(m,left,top,size,m->pose,&x,&y);
        double a=m->pose[2],fx=cos(a),fy=-sin(a),sx=-fy,sy=fx;
        double radius=clamp(m->robot_radius_m*size/(64*m->resolution_m),8,22);
        /* Arrow nose points along +X at yaw=0, +Y at counter-clockwise yaw. */
        double ax=x+radius*fx,ay=y+radius*fy;
        double bx=x-radius*.65*fx+radius*.65*sx,by=y-radius*.65*fy+radius*.65*sy;
        double cx=x-radius*.65*fx-radius*.65*sx,cy=y-radius*.65*fy-radius*.65*sy;
        map_line(d,ax,ay,bx,by,left,top,size,0x111111,3);
        map_line(d,bx,by,cx,cy,left,top,size,0x111111,3);
        map_line(d,cx,cy,ax,ay,left,top,size,0x111111,3);
        snprintf(label,sizeof(label),"X  %.2f m",m->pose[0]);text(d,68,382,17,INK,label);
        snprintf(label,sizeof(label),"Y  %.2f m",m->pose[1]);text(d,68,415,17,INK,label);
        snprintf(label,sizeof(label),"%.1f°",m->pose[2]*180/IMU_PI);text(d,68,448,25,TEAL,label);
    }
    box(d,752,176,16,16,3,blocked);text(d,780,190,16,MUTED,"障碍");
    box(d,752,215,16,16,3,unknown);text(d,780,229,16,MUTED,"未知");
    box(d,752,254,16,16,3,free_color);text(d,780,268,16,MUTED,"可通行");
    line(d,752,308,770,308,3,TEAL);text(d,780,314,16,MUTED,"规划路线");
    line(d,752,347,770,347,2,0x9ebbb1);text(d,780,353,16,MUTED,"行驶轨迹");
    text(d,752,409,14,MUTED,"FPGA A*");
    text(d,752,435,14,MUTED,"局部建图");
}
static void navigation_tabs(struct dashboard *d) {
    box(d,48,38,284,52,14,0xf0f1f3);
    for(int tab=0;tab<2;tab++) {
        int x=52+tab*140,selected=d->navigation_selected==tab;
        if(selected)box(d,x,42,136,44,11,0x202124);
        centered_text(d,x+68,72,22,122,selected?0xffffff:INK,tab?"导航":"摄像头");
    }
}

void dashboard_init(struct dashboard *d) {
    memset(d,0,sizeof(*d));
    if(!atlas) {
        atlas=malloc(FONT_COUNT*4096);
        if(!atlas){fprintf(stderr,"font allocation failed\n");exit(1);}
        size_t at=0;
        for(size_t i=0;i<sizeof(font_rle);i+=2) {
            if(at+font_rle[i]>FONT_COUNT*4096)abort();
            memset(atlas+at,font_rle[i+1],font_rle[i]);at+=font_rle[i];
        }
        if(at!=FONT_COUNT*4096)abort();
    }
}
void dashboard_history(struct dashboard *d,const struct imu_state *s,uint64_t now) {
    for(int i=0;i<3;i++)d->history[d->head][i]=imu_fresh(s,now)?s->sample.accel[i]*IMU_G:NAN;
    d->head=(d->head+1)%120;if(d->count<120)d->count++;
}
int dashboard_action(double x,double y) {
    if(!isfinite(x)||!isfinite(y)||x<0||x>=VIEW_W||y<0||y>=VIEW_H)return 0;
    if(y>=38&&y<90&&x>=48&&x<332)return x<190?5:6;
    /* Remaining content is read-only; empty margins allow window dragging. */
    if(x<CAMERA_X||x>=POSE_X+POSE_W||y<20||y>=740||
       (x>=CAMERA_X+CAMERA_W&&x<POSE_X)||
       (x>=POSE_X&&y>=POSE_Y+POSE_H&&y<ACCEL_Y))return 4;
    return 0;
}
void dashboard_render(struct dashboard *d,const struct imu_state *s,uint64_t now,const char *iface) {
    (void)iface;
    const int fresh=imu_fresh(s,now),attitude=fresh&&s->initialized;
    const int vfresh=vision_fresh(&d->vision,now);
    const unsigned axes[]={TEAL,GREEN,GOLD};
    char buffer[128];
    for(int i=0;i<VIEW_W*VIEW_H;i++)d->pixels[i]=0xff000000|BG;
    card(d,CAMERA_X,CAMERA_Y,CAMERA_W,CAMERA_H);
    navigation_tabs(d);
    /* The preview remains an exact 864x486 copy. Restoring the original
       left-camera/right-stack arrangement never crops or rescales source data. */
    if(!d->navigation_selected && d->vision.have_frame)
        for(int y=0;y<VISION_H;y++)memcpy(&d->pixels[(CAM_Y+y)*VIEW_W+CAM_X],&d->vision.pixels[y*VISION_W],VISION_W*4);
    if(d->navigation_selected)navigation_view(d,now);
    else if(vfresh)detections(d);else camera_wait(d,d->vision.have_frame);
    sonar_readings(d,now);

    card(d,POSE_X,POSE_Y,POSE_W,POSE_H);
    text(d,973,61,25,INK,"姿态角");
    const char *names[]={"横滚","俯仰","偏航"};
    const char *english[]={"ROLL","PITCH","YAW"};
    const double angles[]={s->roll,s->pitch,s->yaw};
    for(int i=0;i<3;i++) {
        double cx=1012+i*88;
        centered_text(d,cx,98,17,82,MUTED,names[i]);
        if(attitude&&isfinite(angles[i])) {
            if(fabs(angles[i])<100000)snprintf(buffer,sizeof(buffer),"%+.1f°",angles[i]);
            else snprintf(buffer,sizeof(buffer),"%+.1e°",angles[i]);
        } else strcpy(buffer,"—");
        centered_text(d,cx,144,30,82,attitude?axes[i]:MUTED,buffer);
        centered_text(d,cx,171,12,82,MUTED,english[i]);
    }
    line(d,973,190,1231,190,1,LINE);
    car_top_view(d,attitude?s->yaw:0,attitude);

    card(d,ACCEL_X,ACCEL_Y,ACCEL_W,ACCEL_H);
    text(d,973,481,25,INK,"加速度");
    text(d,1174,480,15,MUTED,"m/s²");
    for(int axis=0;axis<3;axis++) {
        int y=535+axis*73;
        box(d,974,y-10,7,7,3.5,axes[axis]);
        snprintf(buffer,sizeof(buffer),"%c",'X'+axis);
        text(d,990,y,17,axes[axis],buffer);
        if(fresh&&isfinite(s->sample.accel[axis])) {
            double value=s->sample.accel[axis]*IMU_G;
            if(isfinite(value)&&fabs(value)<100000)snprintf(buffer,sizeof(buffer),"%+.2f",value);
            else if(isfinite(value))snprintf(buffer,sizeof(buffer),"%+.1e",value);
            else strcpy(buffer,"—");
        } else strcpy(buffer,"—");
        centered_text(d,1068,y+3,30,118,fresh?INK:MUTED,buffer);
        const double left=1135,right=1231,mid=y-8;
        box(d,left-4,mid-27,right-left+8,54,7,0xf7f9fc);
        line(d,left,mid,right,mid,1,LINE);
        /* Original 120-sample rolling curve per axis, with gaps for missing
           samples. Clamp only the plot amplitude, never the numerical values. */
        for(int i=1;i<d->count;i++) {
            double a=d->history[(d->head-d->count+i-1+120)%120][axis];
            double b=d->history[(d->head-d->count+i+120)%120][axis];
            if(isfinite(a)&&isfinite(b))
                line(d,left+(120-d->count+i-1)*(right-left)/119,
                     clamp(mid-a/IMU_G*10,mid-24,mid+24),
                     left+(120-d->count+i)*(right-left)/119,
                     clamp(mid-b/IMU_G*10,mid-24,mid+24),1.8,axes[axis]);
        }
        if(axis<2)line(d,973,y+32,1231,y+32,1,LINE);
    }
}
int dashboard_ppm(const struct dashboard *d,const char *path) {
    FILE *f=fopen(path,"wb");if(!f)return -1;
    fprintf(f,"P6\n%d %d\n255\n",VIEW_W,VIEW_H);
    for(int i=0;i<VIEW_W*VIEW_H;i++) {
        unsigned p=d->pixels[i];unsigned char b[]={p>>16,p>>8,p};
        if(fwrite(b,1,3,f)!=3){fclose(f);return -1;}
    }
    return fclose(f);
}
