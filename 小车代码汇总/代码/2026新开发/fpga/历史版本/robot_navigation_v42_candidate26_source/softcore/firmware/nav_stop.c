#include "nav_stop.h"
#include "layered_control.h"

static int32_t relative_yaw(int32_t yaw,int32_t origin)
{
    int32_t d=yaw-origin;
    if(d>=180000)d-=360000;
    if(d< -180000)d+=360000;
    return d;
}
static int ordered(uint32_t current,uint32_t previous)
{return current!=previous && (uint32_t)(current-previous)<UINT32_C(0x80000000);}
static uint32_t magnitude_delta(int32_t current,int32_t previous)
{
    uint32_t d=(uint32_t)current-(uint32_t)previous;
    return (d&UINT32_C(0x80000000))?~d+1u:d;
}
static void reset_evidence(struct nv_stop *s,unsigned reason)
{s->seen=0;s->count=0;s->next=0;s->imu_samples=0;s->reason=(uint8_t)reason;}
void nv_stop_begin(struct nv_stop *s,uint32_t now)
{
    uint8_t *p=(uint8_t *)s;unsigned i;
    for(i=0;i<sizeof(*s);++i)p[i]=0;
    s->requested_ms=now;s->active=1;
}
static void seed(struct nv_stop *s,const struct nv_stop_sample *v)
{
    unsigned i;
    s->seen=1;s->count=0;s->next=0;s->imu_samples=1;
    s->encoder_ms=v->encoder_ms;s->encoder_seq=v->encoder_seq;
    s->imu_ms=v->imu_ms;s->imu_seq=v->imu_seq;s->imu_start_ms=v->imu_ms;
    s->slice_start_ms=v->encoder_ms;s->yaw_origin=v->yaw_mdeg;
    s->yaw_min=0;s->yaw_max=0;s->reason=NV_STOP_WAIT;
    for(i=0;i<4;++i){s->previous_counts[i]=v->counts[i];s->pending_ticks[i]=0;}
}
int nv_stop_update(struct nv_stop *s,const struct nv_stop_sample *v)
{
    uint32_t ticks[4],dt=0,end_ms;int32_t lo,hi,yaw;unsigned i,k;
    if(!s||!v||!s->active)return 0;
    if(!v->encoder_valid||!v->imu_valid||!v->encoder_seq||!v->imu_seq){reset_evidence(s,NV_STOP_NO_SOURCE);return 0;}
    if(v->now_ms-v->encoder_ms>250u||v->now_ms-v->imu_ms>250u||
       v->yaw_mdeg< -180000||v->yaw_mdeg>180000){reset_evidence(s,NV_STOP_SOURCE_AGE);return 0;}
    if((int32_t)(v->encoder_ms-s->requested_ms)<=0||
       (int32_t)(v->imu_ms-s->requested_ms)<=0){reset_evidence(s,NV_STOP_BEFORE_REQUEST);return 0;}
    if(!s->seen){seed(s,v);return 0;}
    if((v->encoder_seq==s->encoder_seq&&v->encoder_ms!=s->encoder_ms)||
       (v->imu_seq==s->imu_seq&&v->imu_ms!=s->imu_ms)||
       (v->encoder_seq!=s->encoder_seq&&(!ordered(v->encoder_seq,s->encoder_seq)||!ordered(v->encoder_ms,s->encoder_ms)))||
       (v->imu_seq!=s->imu_seq&&(!ordered(v->imu_seq,s->imu_seq)||!ordered(v->imu_ms,s->imu_ms)))){
        reset_evidence(s,NV_STOP_SOURCE_ORDER);return 0;
    }
    if(v->encoder_seq!=s->encoder_seq){
        if(v->encoder_ms-s->encoder_ms>40u){seed(s,v);s->reason=NV_STOP_GAP;return 0;}
        for(i=0;i<4;++i){
            uint32_t add=magnitude_delta(v->counts[i],s->previous_counts[i]);
            if(add>UINT32_MAX-s->pending_ticks[i]){seed(s,v);s->reason=NV_STOP_WHEELS_MOVING;return 0;}
            s->pending_ticks[i]+=add;s->previous_counts[i]=v->counts[i];
        }
        s->encoder_ms=v->encoder_ms;s->encoder_seq=v->encoder_seq;
    }
    if(v->imu_seq!=s->imu_seq){
        s->imu_ms=v->imu_ms;s->imu_seq=v->imu_seq;
        if(s->imu_samples<2u)++s->imu_samples;
        yaw=relative_yaw(v->yaw_mdeg,s->yaw_origin);
        /* A stop can be requested while the body still rotates. Rebase after
         * a large displacement, so a later stationary heading opposite that
         * old origin cannot oscillate across relative +/-180 degrees forever.
         * A new complete measured window is required after rebasing. */
        if(yaw>90000||yaw< -90000){seed(s,v);return 0;}
        if(yaw<s->yaw_min)s->yaw_min=yaw;
        if(yaw>s->yaw_max)s->yaw_max=yaw;
    }
    if(s->encoder_ms-s->slice_start_ms>=20u){
        struct nv_stop_slice *slice=s->slices+s->next;
        slice->start_ms=s->slice_start_ms;slice->end_ms=s->encoder_ms;
        slice->yaw_min=s->yaw_min;slice->yaw_max=s->yaw_max;
        for(i=0;i<4;++i){slice->ticks[i]=s->pending_ticks[i];s->pending_ticks[i]=0;}
        s->next=(uint8_t)((s->next+1u)%6u);if(s->count<6u)++s->count;
        s->slice_start_ms=s->encoder_ms;
        s->yaw_min=relative_yaw(v->yaw_mdeg,s->yaw_origin);s->yaw_max=s->yaw_min;
    }
    for(i=0;i<4;++i)ticks[i]=s->pending_ticks[i];
    lo=s->yaw_min;hi=s->yaw_max;
    end_ms=(int32_t)(s->encoder_ms-s->imu_ms)<0?s->encoder_ms:s->imu_ms;
    for(k=0;k<s->count;++k){
        const struct nv_stop_slice *slice=s->slices+(s->next+5u-k)%6u;
        for(i=0;i<4;++i){
            if(slice->ticks[i]>UINT32_MAX-ticks[i]){s->reason=NV_STOP_WHEELS_MOVING;return 0;}
            ticks[i]+=slice->ticks[i];
        }
        if(slice->yaw_min<lo)lo=slice->yaw_min;
        if(slice->yaw_max>hi)hi=slice->yaw_max;
        /* Only the common source-time window counts. Later encoder motion is
         * still included in ticks (conservative), but a frozen IMU cannot use
         * encoder time or host receipt time to manufacture 80ms of evidence. */
        dt=((int32_t)(slice->start_ms-s->imu_start_ms)>=0&&
            (int32_t)(end_ms-slice->start_ms)>=0)?end_ms-slice->start_ms:0;
        if(dt>=80u)break;
    }
    if(dt<80u||s->imu_samples<2u){s->reason=NV_STOP_WAIT;return 0;}
    for(i=0;i<4;++i){
        if(ticks[i]>UINT32_MAX/LC_TICK_NM||ticks[i]*LC_TICK_NM>15000u*dt){s->reason=NV_STOP_WHEELS_MOVING;return 0;}
    }
    if(v->gyro_mdps>4584||v->gyro_mdps< -4584||hi-lo>(int32_t)((4584u*dt+999u)/1000u+58u)){
        s->reason=NV_STOP_IMU_MOVING;return 0;
    }
    s->reason=NV_STOP_CONFIRMED;return 1;
}
