#ifndef NAVIGATION_VIEWPORT_H
#define NAVIGATION_VIEWPORT_H
#include <math.h>
#include <stdint.h>
#include <string.h>
/* Display-only extent. Never contracts within a map session; growth is eased. */
struct navigation_viewport {
    int valid;char session[129];uint64_t since;
    double x,y,span,sx,sy,ss,tx,ty,ts;
};
static inline void navigation_viewport_update(struct navigation_viewport *v,double x,double y,double span,const char *session,uint64_t now){
    if(!isfinite(x)||!isfinite(y)||!isfinite(span)||span<=0)return;
    if(!v->valid||strcmp(v->session,session)){
        v->valid=1;strncpy(v->session,session,128);v->session[128]=0;
        v->x=v->sx=v->tx=x;v->y=v->sy=v->ty=y;v->span=v->ss=v->ts=span;v->since=now;return;
    }
    double u=now>=v->since?fmin(1.,(now-v->since)/300.):0;u=u*u*(3-2*u);
    v->x=v->sx+(v->tx-v->sx)*u;v->y=v->sy+(v->ty-v->sy)*u;v->span=v->ss+(v->ts-v->ss)*u;
    if(x>=v->tx-1e-6&&y>=v->ty-1e-6&&x+span<=v->tx+v->ts+1e-6&&y+span<=v->ty+v->ts+1e-6)return;
    double loX=fmin(v->tx,x),loY=fmin(v->ty,y),hiX=fmax(v->tx+v->ts,x+span),hiY=fmax(v->ty+v->ts,y+span);
    double size=fmax(hiX-loX,hiY-loY),pad=fmax(.25,size*.125);
    v->sx=v->x;v->sy=v->y;v->ss=v->span;v->since=now;
    v->ts=size+2*pad;v->tx=(loX+hiX-v->ts)/2;v->ty=(loY+hiY-v->ts)/2;
}
#endif
