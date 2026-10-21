/* Frozen v10 renderer for byte-for-byte comparison and local CPU benchmark. */
#define dashboard_init reference_dashboard_init
#define dashboard_history reference_dashboard_history
#define dashboard_action reference_dashboard_action
#define dashboard_render reference_dashboard_render
#define dashboard_ppm reference_dashboard_ppm
#include "dashboard.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "font_atlas.h"

#define BG 0xf2f6f6
#define INK 0x193b41
#define MUTED 0x7b9194
#define LINE 0xe8eff0
#define TEAL 0x138d89
#define GREEN 0x6a997e
#define GOLD 0xc48c4a
#define CAM_X 48
#define CAM_Y 154

static unsigned char *atlas;
static double clamp(double x, double lo, double hi) { return fmax(lo, fmin(hi, x)); }
static void pixel(struct dashboard *d, int x, int y, unsigned c, double a) {
    if ((unsigned)x >= VIEW_W || (unsigned)y >= VIEW_H || a <= 0) return;
    a = fmin(1, a);
    unsigned old = d->pixels[y * VIEW_W + x];
    unsigned r = ((c >> 16) & 255) * a + ((old >> 16) & 255) * (1-a);
    unsigned g = ((c >> 8) & 255) * a + ((old >> 8) & 255) * (1-a);
    unsigned b = (c & 255) * a + (old & 255) * (1-a);
    d->pixels[y * VIEW_W + x] = 0xff000000 | (r << 16) | (g << 8) | b;
}
static void box(struct dashboard *d, double x, double y, double w, double h,
                double r, unsigned c) {
    for (int j = fmax(0, floor(y)); j < fmin(VIEW_H, ceil(y+h)); j++)
        for (int i = fmax(0, floor(x)); i < fmin(VIEW_W, ceil(x+w)); i++) {
            double dx = fmax(fabs(i+.5-(x+w/2))-(w/2-r), 0);
            double dy = fmax(fabs(j+.5-(y+h/2))-(h/2-r), 0);
            pixel(d, i, j, c, clamp(r+.5-hypot(dx,dy), 0, 1));
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
static void card(struct dashboard *d, int x, int y, int w, int h) {
    box(d,x,y+3,w,h,18,0xe8eeee); box(d,x,y,w,h,18,0xffffff);
}
static void pill(struct dashboard *d, int x, int y, int w, unsigned color, const char *label) {
    box(d,x,y,w,28,14,0xeaf4f2); box(d,x+12,y+10,7,7,3.5,color);
    text_fit(d,x+26,y+19,12,color,w-35,label);
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
        double x = CAM_X+clamp(o->left,0,VISION_W-1), y = CAM_Y+clamp(o->top,0,VISION_H-1);
        double r = CAM_X+clamp(o->right,0,VISION_W-1), b = CAM_Y+clamp(o->bottom,0,VISION_H-1);
        if (r <= x || b <= y) continue;
        unsigned c = colors[(unsigned)o->class_id%4];
        if(v->boxes_delayed) {
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
struct car_pose { double cx, cy, c, s; };
static void car_line(struct dashboard *d, const struct car_pose *p,
                     double x, double y, double xx, double yy,
                     double width, unsigned color) {
    /* Screen Y points down: positive IMU yaw turns the nose to the left. */
    line(d,p->cx+x*p->c+y*p->s,p->cy-x*p->s+y*p->c,
         p->cx+xx*p->c+yy*p->s,p->cy-xx*p->s+yy*p->c,width,color);
}
static void car_path(struct dashboard *d, const struct car_pose *p,
                     const double points[][2], size_t count,
                     double width, unsigned color) {
    for (size_t i=0;i<count;i++) {
        size_t next=(i+1)%count;
        car_line(d,p,points[i][0],points[i][1],points[next][0],points[next][1],width,color);
    }
}
static void car_top_view(struct dashboard *d, double yaw, int fresh) {
    const double cx=1102, cy=333, radius=60;
    fresh=fresh && isfinite(yaw);
    double angle=fresh?remainder(yaw,360)*IMU_PI/180:0;
    const struct car_pose pose={cx,cy,cos(angle),sin(angle)};
    unsigned body=fresh?INK:0xa5b5b6, accent=fresh?TEAL:0xb3c2c3;
    box(d,973,261,258,142,14,0xf6f9f8);
    text(d,985,282,11,MUTED,"俯视");
    text(d,985,296,8,MUTED,"TOP VIEW");
    text(d,1192,282,9,MUTED,"YAW");
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
        box(d,cx-55,377,110,20,7,0xf6f9f8);
        text(d,cx-text_width(12,"等待有效数据")/2,391,12,MUTED,"等待有效数据");
    }
}
void dashboard_init(struct dashboard *d) {
    memset(d,0,sizeof(*d));
    if (!atlas) {
        atlas = malloc(FONT_COUNT*4096);
        if (!atlas) { fprintf(stderr,"font allocation failed\n"); exit(1); }
        size_t at = 0;
        for (size_t i=0; i<sizeof(font_rle); i+=2) {
            if (at+font_rle[i] > FONT_COUNT*4096) abort();
            memset(atlas+at,font_rle[i+1],font_rle[i]); at += font_rle[i];
        }
        if (at != FONT_COUNT*4096) abort();
    }
}
void dashboard_history(struct dashboard *d, const struct imu_state *s, uint64_t now) {
    for (int i=0;i<3;i++) d->history[d->head][i] = imu_fresh(s,now) ? s->sample.accel[i]*IMU_G : NAN;
    d->head = (d->head+1)%120; if (d->count<120) d->count++;
}
int dashboard_action(double x, double y) {
    if (x >= 1222 && x < 1260 && y >= 25 && y < 72) return 3;
    if (y >= 709 && y < 749) {
        if (x >= 1110 && x < 1252) return 1;
        if (x >= 952 && x < 1094) return 2;
    }
    if (y >= 0 && y < 93) return 4;
    return 0;
}
void dashboard_render(struct dashboard *d, const struct imu_state *s, uint64_t now, const char *iface) {
    (void)iface;
    int fresh = imu_fresh(s,now), attitude = fresh && s->initialized;
    int vfresh = vision_fresh(&d->vision,now);
    int count = vfresh ? (int)clamp(d->vision.count,0,VISION_OBJECTS) : 0;
    int demo = d->demo || d->vision.demo;
    char buffer[256]; unsigned axes[] = {TEAL,GREEN,GOLD};
    for (int i=0;i<VIEW_W*VIEW_H;i++) d->pixels[i] = 0xff000000|BG;
    box(d,28,30,6,31,3,TEAL); text(d,48,57,27,INK,"视觉与姿态");
    text(d,49,81,12,MUTED,"ROBOT PERCEPTION");
    if (demo) { box(d,253,35,75,26,13,0xf8ead4); text(d,266,53,12,GOLD,"演示数据"); }
    pill(d,946,35,112,vfresh?TEAL:GOLD,vfresh?"视觉已连接":"视觉等待");
    pill(d,1072,35,134,fresh?TEAL:GOLD,fresh?"姿态已连接":"姿态等待");
    line(d,1233,42,1246,55,1.7,MUTED); line(d,1233,55,1246,42,1.7,MUTED);

    card(d,28,104,904,590);
    text(d,48,135,18,INK,"实时画面");
    if (vfresh) {
        snprintf(buffer,sizeof(buffer),"CAP %.1f / VIEW %.1f / AI %.1f",fmax(0,d->vision.capture_fps),fmax(0,d->vision.preview_fps),fmax(0,d->vision.fps));
        text_fit(d,620,134,12,MUTED,290,buffer);
        text_fit(d,166,133,12,MUTED,440,d->vision.source);
    } else text_fit(d,166,133,12,MUTED,640,d->vision.have_frame?"画面已超时，正在等待更新":
                   d->vision.status[0]?d->vision.status:"等待摄像头连接");
    if (d->vision.have_frame)
        for (int y=0;y<VISION_H;y++) memcpy(&d->pixels[(CAM_Y+y)*VIEW_W+CAM_X],&d->vision.pixels[y*VISION_W],VISION_W*4);
    if (vfresh) detections(d); else camera_wait(d,d->vision.have_frame);
    box(d,48,657,7,7,3.5,vfresh?TEAL:GOLD);
    if (vfresh && d->vision.detection_enabled && !d->vision.detection_available)
        snprintf(buffer,sizeof(buffer),"检测结果等待更新");
    else if (vfresh) snprintf(buffer,sizeof(buffer),"检测结果 %d 项",count);
    else snprintf(buffer,sizeof(buffer),"%s",d->vision.have_frame?"画面已超时，等待更新":"等待连接");
    text(d,64,670,14,vfresh?INK:MUTED,buffer);
    if (vfresh && strncmp(d->vision.status,"AI error:",9)==0) {
        text_fit(d,225,670,12,GOLD,666,d->vision.status);
    } else if (vfresh && d->vision.boxes_delayed && d->vision.detection_available) {
        snprintf(buffer,sizeof(buffer),"检测结果 %.0f ms 前",d->vision.detection_age_ms);
        text_fit(d,225,670,12,MUTED,666,buffer);
    } else if (vfresh && count) {
        size_t used = 0; buffer[0] = 0;
        for (int i=0;i<count && i<8;i++) {
            const char *name = d->vision.objects[i].name;
            int len = snprintf(buffer+used,sizeof(buffer)-used,"%s%.63s",i?"  /  ":"",name);
            if (len < 0 || (size_t)len >= sizeof(buffer)-used) break;
            used += (size_t)len;
        }
        text_fit(d,225,670,13,MUTED,666,buffer);
    } else if (vfresh) text(d,225,670,13,MUTED,"当前画面暂无目标");

    card(d,952,104,300,336); text(d,973,135,18,INK,"姿态角");
    text(d,1160,134,11,MUTED,"ATTITUDE");
    const char *names[] = {"横滚","俯仰","偏航"};
    const char *english[] = {"ROLL","PITCH","YAW"};
    double angles[] = {s->roll,s->pitch,s->yaw};
    for (int i=0;i<3;i++) {
        int x=973+i*88;
        text(d,x,169,13,MUTED,names[i]);
        if (attitude && isfinite(angles[i])) snprintf(buffer,sizeof(buffer),"%+.1f°",angles[i]);
        else strcpy(buffer,"—");
        text(d,x-1,204,23,attitude?axes[i]:MUTED,buffer);
        text(d,x,225,10,MUTED,english[i]);
    }
    line(d,973,245,1231,245,1,LINE);
    car_top_view(d,attitude?s->yaw:0,attitude);
    text(d,973,422,11,MUTED,"偏航为相对角，可能随时间漂移");

    card(d,952,460,300,234); text(d,973,492,18,INK,"加速度");
    text(d,1142,491,11,MUTED,"含重力 · m/s²");
    for (int axis=0;axis<3;axis++) {
        int y=530+axis*56;
        box(d,974,y-11,7,7,3.5,axes[axis]);
        snprintf(buffer,sizeof(buffer),"%c",'X'+axis); text(d,990,y,14,axes[axis],buffer);
        if (fresh && isfinite(s->sample.accel[axis])) snprintf(buffer,sizeof(buffer),"%+.2f",s->sample.accel[axis]*IMU_G);
        else strcpy(buffer,"—");
        text(d,1014,y+2,24,INK,buffer);
        for (int i=1;i<d->count;i++) {
            double a=d->history[(d->head-d->count+i-1+120)%120][axis];
            double b=d->history[(d->head-d->count+i+120)%120][axis];
            if (isfinite(a) && isfinite(b)) line(d,1137+(120-d->count+i-1)*94./119,
                clamp(y-7-a/IMU_G*10,y-22,y+10),1137+(120-d->count+i)*94./119,
                clamp(y-7-b/IMU_G*10,y-22,y+10),1.6,axes[axis]);
        }
        if (axis<2) line(d,973,y+20,1231,y+20,1,LINE);
    }

    if (!fresh) snprintf(buffer,sizeof(buffer),"%s",s->have_packet?"姿态数据已超时，等待更新":"等待 IMU 数据连接");
    else if (s->calibrating && s->calibration_status==IMU_CAL_MOTION)
        snprintf(buffer,sizeof(buffer),"角速度或振动较大，请保持静止后校准");
    else if (s->calibrating && s->calibration_status==IMU_CAL_NOISE)
        snprintf(buffer,sizeof(buffer),"振动较大，请保持静止后校准");
    else if (s->calibrating) snprintf(buffer,sizeof(buffer),"请保持静止 · 零偏校准 %u%% · %u 个采样",s->calibration/2,s->calibration_samples);
    else snprintf(buffer,sizeof(buffer),"零偏校准完成 · 姿态数据正常 · %.1f°C",s->sample.temperature);
    text_fit(d,30,728,13,MUTED,860,buffer);
    if (demo) text(d,30,748,11,GOLD,"当前显示演示数据");
    else if(fresh && s->bias_valid) {
        snprintf(buffer,sizeof(buffer),"BIAS [%+.3f %+.3f %+.3f] / CORR [%+.3f %+.3f %+.3f] dps",
            s->bias[0],s->bias[1],s->bias[2],s->sample.gyro[0]-s->bias[0],
            s->sample.gyro[1]-s->bias[1],s->sample.gyro[2]-s->bias[2]);
        text_fit(d,30,748,11,MUTED,860,buffer);
    }
    else text(d,30,748,11,MUTED,"校准时请保持设备静止；航向归零不改变横滚和俯仰。");
    box(d,952,709,142,40,11,0xe3edeb); text(d,993,735,14,INK,"重新校准");
    box(d,1110,709,142,40,11,TEAL); text(d,1151,735,14,0xffffff,"航向归零");
}
int dashboard_ppm(const struct dashboard *d,const char *path) {
    FILE *f=fopen(path,"wb"); if (!f) return -1;
    fprintf(f,"P6\n%d %d\n255\n",VIEW_W,VIEW_H);
    for (int i=0;i<VIEW_W*VIEW_H;i++) {
        unsigned p=d->pixels[i]; unsigned char b[]={p>>16,p>>8,p};
        if (fwrite(b,1,3,f)!=3) { fclose(f); return -1; }
    }
    return fclose(f);
}
