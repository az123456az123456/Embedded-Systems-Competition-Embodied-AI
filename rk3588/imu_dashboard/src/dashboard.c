#include "dashboard.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "font_atlas.h"
#define BG 0xf4f7fa
#define INK 0x142936
#define MUTED 0x6e818e
#define LINE 0xe8eef2
#define CYAN 0x0e9ea9
#define GREEN 0x699678
#define GOLD 0xc88a3d
static unsigned char *atlas;
static double clamp(double x,double a,double b) { return fmax(a,fmin(b,x)); }
static void pixel(struct dashboard *d,int x,int y,unsigned c,double a) {
    if((unsigned)x>=VIEW_W || (unsigned)y>=VIEW_H || a<=0) return;
    unsigned old=d->pixels[y*VIEW_W+x];
    unsigned r=((c>>16)&255)*a+((old>>16)&255)*(1-a);
    unsigned g=((c>>8)&255)*a+((old>>8)&255)*(1-a);
    unsigned b=(c&255)*a+(old&255)*(1-a);
    d->pixels[y*VIEW_W+x]=0xff000000|(r<<16)|(g<<8)|b;
}
static void box(struct dashboard *d,double x,double y,double w,double h,double r,unsigned c) {
    for(int j=(int)y;j<(int)ceil(y+h);j++) for(int i=(int)x;i<(int)ceil(x+w);i++) {
        double dx=fmax(fabs(i+.5-(x+w/2))-(w/2-r),0);
        double dy=fmax(fabs(j+.5-(y+h/2))-(h/2-r),0);
        pixel(d,i,j,c,clamp(r+.5-hypot(dx,dy),0,1));
    }
}
static void line(struct dashboard *d,double x,double y,double xx,double yy,double width,unsigned c) {
    double vx=xx-x,vy=yy-y,len=vx*vx+vy*vy;
    for(int j=(int)floor(fmin(y,yy)-width);j<=ceil(fmax(y,yy)+width);j++)
        for(int i=(int)floor(fmin(x,xx)-width);i<=ceil(fmax(x,xx)+width);i++) {
            double t=len?clamp(((i+.5-x)*vx+(j+.5-y)*vy)/len,0,1):0;
            pixel(d,i,j,c,clamp(width/2+.5-hypot(i+.5-x-t*vx,j+.5-y-t*vy),0,1));
        }
}
static unsigned utf8(const char **p) {
    const unsigned char *s=(const unsigned char *)*p; unsigned c=*s++; int n=0;
    if(c>=0xf0) { c&=7; n=3; } else if(c>=0xe0) { c&=15; n=2; } else if(c>=0xc0) { c&=31; n=1; }
    while(n-- && *s) c=(c<<6)|(*s++&63);
    *p=(const char *)s; return c;
}
static void text(struct dashboard *d,double x,double baseline,double size,unsigned c,const char *s) {
    double scale=size/48.;
    while(*s) {
        unsigned cp=utf8(&s); int k;
        for(k=0;k<FONT_COUNT;k++) if(font_glyphs[k].cp==cp) break;
        if(k==FONT_COUNT) continue;
        const unsigned char *p=atlas+k*4096;
        double left=x-2*scale,top=baseline-52*scale;
        for(int j=fmax(0,floor(top));j<fmin(VIEW_H,ceil(top+64*scale));j++)
            for(int i=fmax(0,floor(left));i<fmin(VIEW_W,ceil(left+64*scale));i++) {
                double sx=(i+.5-left)/scale-.5,sy=(j+.5-top)/scale-.5;
                int a=floor(sx),b=floor(sy);
                if(a<0 || b<0 || a>=63 || b>=63) continue;
                double fx=sx-a,fy=sy-b;
                double v=p[b*64+a]*(1-fx)*(1-fy)+p[b*64+a+1]*fx*(1-fy)+
                         p[(b+1)*64+a]*(1-fx)*fy+p[(b+1)*64+a+1]*fx*fy;
                pixel(d,i,j,c,v/255.);
            }
        x+=font_glyphs[k].advance*scale;
    }
}
static void card(struct dashboard *d,int x,int y,int w,int h) {
    box(d,x,y+3,w,h,18,0xeaf0f4); box(d,x,y,w,h,18,0xffffff);
}
static void horizon(struct dashboard *d,double roll,double pitch,int fresh) {
    double c=cos(roll*IMU_PI/180),s=sin(roll*IMU_PI/180);
    for(int y=178;y<388;y++) for(int x=50;x<442;x++) {
        double dx=fmax(fabs(x+.5-246)-180,0),dy=fmax(fabs(y+.5-283)-89,0);
        double alpha=clamp(16.5-hypot(dx,dy),0,1);
        double ry=(x-246)*s+(y-283)*c-pitch*1.75;
        unsigned col=fresh?(ry<0?0xe1f2f7:0xece8df):0xeef2f5;
        pixel(d,x,y,col,alpha);
        if(fresh && fabs(ry)<1) pixel(d,x,y,0x85b8c4,alpha);
        if(fresh) for(int deg=-30;deg<=30;deg+=10) if(deg && fabs(ry+deg*1.75)<.7) {
            double rx=(x-246)*c-(y-283)*s;
            if(fabs(rx)<(deg%20==0?35:22)) pixel(d,x,y,0x96a8ab,alpha*.7);
        }
    }
    line(d,195,283,232,283,3,INK); line(d,260,283,297,283,3,INK);
    line(d,232,283,246,293,3,INK); line(d,246,293,260,283,3,INK);
    line(d,246,192,241,200,2,CYAN); line(d,246,192,251,200,2,CYAN);
    if(!fresh) { box(d,167,307,158,38,12,0xffffff); text(d,182,332,17,MUTED,"等待有效数据"); }
}
void dashboard_init(struct dashboard *d) {
    memset(d,0,sizeof(*d));
    if(!atlas) {
        atlas=malloc(FONT_COUNT*4096);
        if(!atlas) { fprintf(stderr,"font allocation failed\n"); exit(1); }
        size_t at=0;
        for(size_t i=0;i<sizeof(font_rle);i+=2) {
            if(at+font_rle[i]>FONT_COUNT*4096) abort();
            memset(atlas+at,font_rle[i+1],font_rle[i]); at+=font_rle[i];
        }
        if(at!=FONT_COUNT*4096) abort();
    }
}
void dashboard_history(struct dashboard *d,const struct imu_state *s,uint64_t now) {
    for(int i=0;i<3;i++) d->history[d->head][i]=imu_fresh(s,now)?s->sample.accel[i]*IMU_G:NAN;
    d->head=(d->head+1)%120; if(d->count<120) d->count++;
}
int dashboard_action(double x,double y) {
    if(x>=942 && y<77) return 3;
    if(y>=621 && y<661) { if(x>=828 && x<=972) return 1; if(x>=659 && x<811) return 2; }
    if(y<100) return 4;
    return 0;
}
void dashboard_render(struct dashboard *d,const struct imu_state *s,uint64_t now,const char *iface) {
    int fresh=imu_fresh(s,now),attitude=fresh&&s->initialized;
    for(int i=0;i<VIEW_W*VIEW_H;i++) d->pixels[i]=0xff000000|BG;
    box(d,28,29,7,31,3,CYAN); text(d,48,57,28,INK,"IMU 姿态仪");
    text(d,49,84,13,MUTED,"MPU9250  /  LIVE MOTION");
    unsigned status=fresh?CYAN:GOLD;
    box(d,762,32,163,36,18,fresh?0xe3f3ef:0xf9eddf);
    box(d,777,46,8,8,4,status);
    text(d,795,56,15,status,d->demo?"演示数据":fresh?"数据已连接":s->have_packet?"数据已超时":"等待连接");
    line(d,950,43,964,57,1.8,MUTED); line(d,950,57,964,43,1.8,MUTED);
    card(d,28,112,436,338); text(d,50,148,18,INK,"实时姿态"); text(d,344,148,12,MUTED,"ATTITUDE");
    horizon(d,attitude?s->roll:0,attitude?s->pitch:0,attitude);
    text(d,50,422,14,MUTED,attitude?"机体坐标 · 横滚 / 俯仰":"等待新采样，姿态显示暂停");
    const char *labels[]={"横滚","俯仰","偏航"}; const char *en[]={"ROLL","PITCH","YAW"};
    double angles[]={s->roll,s->pitch,s->yaw}; unsigned colors[]={CYAN,GREEN,GOLD};
    char buffer[128];
    for(int i=0;i<3;i++) {
        int x=488+i*164; card(d,x,112,156,154);
        text(d,x+18,147,17,INK,labels[i]); text(d,x+18,169,11,MUTED,en[i]);
        if(attitude) snprintf(buffer,sizeof(buffer),"%+.1f°",angles[i]); else strcpy(buffer,"—");
        text(d,x+14,216,28,attitude?colors[i]:MUTED,buffer);
        text(d,x+18,246,12,MUTED,i==2?"相对航向":"角度 / deg");
    }
    card(d,488,284,484,166); text(d,508,320,18,INK,"三轴加速度");
    text(d,783,320,13,MUTED,"含重力 · m/s²");
    for(int i=0;i<3;i++) {
        int x=508+i*154; box(d,x,346,8,8,4,colors[i]);
        snprintf(buffer,sizeof(buffer),"%c",'X'+i); text(d,x+16,358,16,colors[i],buffer);
        if(fresh) snprintf(buffer,sizeof(buffer),"%+.2f",s->sample.accel[i]*IMU_G); else strcpy(buffer,"—");
        text(d,x,396,30,INK,buffer);
        if(fresh) snprintf(buffer,sizeof(buffer),"%+.3f g",s->sample.accel[i]); else strcpy(buffer,"无有效采样");
        text(d,x,425,13,MUTED,buffer);
    }
    card(d,28,472,944,132); text(d,50,505,17,INK,"加速度趋势"); text(d,193,505,13,MUTED,"近 6 秒");
    for(int i=0;i<3;i++) { box(d,827+i*44,492,7,7,3,colors[i]); snprintf(buffer,sizeof(buffer),"%c",'X'+i); text(d,840+i*44,503,12,MUTED,buffer); }
    for(int i=-1;i<=1;i++) line(d,75,554-i*26,949,554-i*26,1,LINE);
    text(d,45,532,10,MUTED,"+g"); text(d,51,558,10,MUTED,"0"); text(d,45,584,10,MUTED,"−g");
    for(int i=1;i<d->count;i++) for(int a=0;a<3;a++) {
        double y0=d->history[(d->head-d->count+i-1+120)%120][a];
        double y1=d->history[(d->head-d->count+i+120)%120][a];
        if(isfinite(y0) && isfinite(y1)) line(d,75+(120-d->count+i-1)*874./119,
            clamp(554-y0/IMU_G*26,520,589),75+(120-d->count+i)*874./119,
            clamp(554-y1/IMU_G*26,520,589),2,colors[a]);
    }
    if(s->calibrating) snprintf(buffer,sizeof(buffer),"请保持静止 · 零偏校准 %u%%",s->calibration/2);
    else snprintf(buffer,sizeof(buffer),"%s  ·  %.1f°C  ·  采样 #%u",iface,s->sample.temperature,s->sample.sequence);
    text(d,30,637,14,MUTED,buffer);
    text(d,30,659,12,MUTED,"偏航为相对角，会随时间漂移；归零不改变横滚与俯仰。");
    box(d,659,620,152,42,12,0xe7edf2); text(d,684,647,15,INK,"重新校准");
    box(d,828,620,144,42,12,CYAN); text(d,850,647,15,0xffffff,"航向归零");
}
int dashboard_ppm(const struct dashboard *d,const char *path) {
    FILE *f=fopen(path,"wb"); if(!f) return -1;
    fprintf(f,"P6\n%d %d\n255\n",VIEW_W,VIEW_H);
    for(int i=0;i<VIEW_W*VIEW_H;i++) { unsigned p=d->pixels[i]; unsigned char b[]={p>>16,p>>8,p}; if(fwrite(b,1,3,f)!=3) { fclose(f); return -1; } }
    return fclose(f);
}
