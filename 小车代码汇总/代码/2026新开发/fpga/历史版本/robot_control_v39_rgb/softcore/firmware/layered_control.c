#include "layered_control.h"

static int32_t absolute(int32_t x)
{ return x >= 0 ? x : x == INT32_MIN ? INT32_MAX : -x; }
static int32_t limit(int32_t x, int32_t lo, int32_t hi)
{ return x < lo ? lo : x > hi ? hi : x; }
static int32_t angle(int32_t x)
{ if (x > 180000) x -= 360000; if (x < -180000) x += 360000; return x; }
static int32_t signed16(uint32_t v)
{ return (v & 0x8000u) ? (int32_t)(v & 0xffffu) - 65536 : (int32_t)(v & 0xffffu); }
static int32_t count_difference(int32_t a, int32_t b)
{ uint32_t d = (uint32_t)a - (uint32_t)b; return d <= INT32_MAX ? (int32_t)d : -1 - (int32_t)(UINT32_MAX-d); }
static void zero_duties(struct lc_state *s, struct lc_output *o)
{ unsigned i; for (i=0;i<4;++i) { o->duty[i]=0; s->integral_q8[i]=0; } }
int32_t lc_forward_component_mmps(const struct lc_state *s)
{
    /* fast_step validates every delta first. Each accepted wheel is bounded
     * below1.6m/s; the four-term signed sum cannot overflow. Never use absolute
     * single-wheel speed here: that would misclassify in-place turning. */
    return (s->wheel_mmps[0]+s->wheel_mmps[1]+
            s->wheel_mmps[2]+s->wheel_mmps[3])/4;
}
uint32_t lc_front_stop_mm(int32_t forward_mmps, uint32_t source_age_ms)
{
    uint32_t v, latency, reaction_distance, braking_distance;
    if(forward_mmps<=0)return LC_FRONT_STOP_BASE_MM;
    if(source_age_ms>LC_SONAR_TIMEOUT_MS || forward_mmps>2000)return UINT16_MAX;
    v=(uint32_t)forward_mmps;
    latency=source_age_ms+LC_STOP_REACTION_MS+LC_STOP_SENSOR_LATENCY_MS;
    /* Bounds above keep every product below4e6, using RV32M32bit division.
     * Round UP so integer truncation cannot shrink the stopping distance. */
    reaction_distance=(v*latency+999u)/1000u;
    braking_distance=(v*v+2u*LC_STOP_BRAKE_MMPS2-1u)/(2u*LC_STOP_BRAKE_MMPS2);
    return LC_FRONT_STOP_BASE_MM+reaction_distance+braking_distance;
}
static int actuation_allowed(const struct lc_state *s,const struct lc_input *in,int32_t yaw_rate)
{
    unsigned i;int32_t forward=lc_forward_component_mmps(s);
    if((in->sonar_online&7u)!=7u)return 0;
    for(i=0;i<3;++i)if(in->now_ms-in->sonar_stamp_ms[i]>LC_SONAR_TIMEOUT_MS)return 0;
    /* Include requested acceleration and real forward coasting. Opposing
     * wheel velocities in a pure turn cancel, so no forward formula applies
     * to a stationary turn. The directional side floor still applies. */
    if(in->linear_mmps>forward)forward=in->linear_mmps;
    if(forward>0 && in->sonar_mm[0]<=lc_front_stop_mm(forward,
            in->now_ms-in->sonar_stamp_ms[0]))return 0;
    if(yaw_rate>0 && in->sonar_mm[1]<=LC_SIDE_STOP_HARD_MM)return 0;
    if(yaw_rate<0 && in->sonar_mm[2]<=LC_SIDE_STOP_HARD_MM)return 0;
    return 1;
}

void lc_init(struct lc_state *s)
{
    unsigned i; uint8_t *p=(uint8_t *)s;
    for (i=0;i<sizeof(*s);++i) p[i]=0;
    s->imu_reason=LC_IMU_WAIT_PACKET;
}
static void calibration_reset(struct lc_state *s)
{
    unsigned i; s->calibration_count=0;
    for (i=0;i<3;++i) { s->calibration_sum[i]=0; s->calibration_accel_sum[i]=0; }
}
static uint32_t square_root(uint32_t n)
{
    uint32_t root=0,bit=UINT32_C(1)<<30;
    while(bit>n)bit>>=2;
    while(bit) {
        if(n>=root+bit) {n-=root+bit;root=(root>>1)+bit;}
        else root>>=1;
        bit>>=2;
    }
    return root;
}
static int32_t project_vertical(const int32_t v[3],const int32_t gravity_q12[3])
{
    /* |v_i|<=256138mdps; unit-vector L1<=sqrt(3)*4096, so the complete
     * 32bit signed accumulator remains below1.82e9 even at gyro full scale. */
    return (v[0]*gravity_q12[0]+v[1]*gravity_q12[1]+v[2]*gravity_q12[2])/4096;
}
static int imu_update(struct lc_state *s, const struct lc_input *in)
{
    uint32_t seq=in->imu_words[2], stamp=in->imu_words[3], dt;
    int32_t g[3], a[3], magnitude, rate; uint32_t norm;
    unsigned i; uint8_t quiet_reason=0;
    if (in->imu_words[0]!=UINT32_C(0x494d5539) || (in->imu_words[1]>>16)!=0x0192u) {
        s->imu_reason=in->imu_words[0]?LC_IMU_BAD_FORMAT:LC_IMU_WAIT_PACKET;return 0;
    }
    if(!(in->imu_words[1]&4u)) {s->imu_reason=LC_IMU_SENSOR_NOT_INITIALIZED;return 0;}
    if(!(in->imu_words[1]&1u) || (in->imu_words[14]>>16)>100u) {
        s->imu_reason=LC_IMU_SOURCE_STALE;return 0;
    }
    if((uint32_t)(in->now_ms-in->imu_arrival_ms)>100u) {
        s->imu_reason=LC_IMU_RECEIVE_STALE;return 0;
    }
    if (s->imu_seen && seq==s->imu_sequence) {
        if(s->calibrated)s->imu_reason=LC_IMU_READY;
        return 1;
    }
    dt=s->imu_seen ? stamp-s->imu_stamp_ms : 0;
    if (s->imu_seen && ((int32_t)(seq-s->imu_sequence)<=0 || dt>100u || !dt)) {
        /* A restarted source or a missing interval invalidates the reference.
         * Never integrate a large gap or silently copy a new angle reference. */
        s->calibrated=0; calibration_reset(s); s->active_previous=0; dt=0;
        s->source_reset_seen=1;s->imu_reason=LC_IMU_SOURCE_RESTART;
    }
    s->imu_seen=1; s->imu_sequence=seq; s->imu_stamp_ms=stamp;
    a[0]=signed16(in->imu_words[4]>>16); a[1]=signed16(in->imu_words[4]);
    a[2]=signed16(in->imu_words[5]>>16);
    g[0]=signed16(in->imu_words[6]>>16); g[1]=signed16(in->imu_words[6]);
    g[2]=signed16(in->imu_words[7]>>16);
    norm=(uint32_t)(a[0]*a[0])+(uint32_t)(a[1]*a[1])+(uint32_t)(a[2]*a[2]);
    if (norm<UINT32_C(200000000) || norm>UINT32_C(355000000)) quiet_reason=LC_CAL_GRAVITY_RANGE;
    /* Local yaw is planar integration about the STATIC calibrated vertical,
     * handling a rigid upside-down/tilted IMU without assuming boardZ is up.
     * The car remains approximately level; full attitude remains on RK.
     * Calibrate only at rest: a nonzero requested motion blocks calibration. */
    if ((in->flags&LC_ARMED) && (in->linear_mmps || in->yaw_rate_mradps)) quiet_reason=LC_CAL_REQUESTED_MOTION;
    for (i=0;i<4;++i)
        if (absolute(count_difference(in->counts[i],s->count_previous[i]))>4) quiet_reason=LC_CAL_ENCODER_MOVING;
    for (i=0;i<3;++i) {
        if (absolute(g[i])>786) quiet_reason=LC_CAL_GYRO_OFFSET_RANGE; /* unchanged acquisition bound ±6deg/s */
        if (s->calibration_count &&
            (g[i]<s->calibration_min[i]-65 || g[i]>s->calibration_max[i]+65 ||
             s->calibration_max[i]-s->calibration_min[i]>131)) quiet_reason=LC_CAL_GYRO_UNSTABLE;
    }
    if (!s->calibrated) {
        if (quiet_reason) {calibration_reset(s);s->imu_reason=quiet_reason;}
        else {
            s->imu_reason=LC_CAL_COLLECTING;
            if (!s->calibration_count) {
                s->calibration_started_ms=stamp;
                for (i=0;i<3;++i) s->calibration_min[i]=s->calibration_max[i]=g[i];
            }
            for (i=0;i<3;++i) {
                s->calibration_sum[i]+=g[i];
                s->calibration_accel_sum[i]+=a[i];
                if(g[i]<s->calibration_min[i])s->calibration_min[i]=g[i];
                if(g[i]>s->calibration_max[i])s->calibration_max[i]=g[i];
            }
            ++s->calibration_count;
            if(s->calibration_count>=32u && stamp-s->calibration_started_ms>=2000u) {
                int32_t mean_accel[3];uint32_t gravity_norm;
                for(i=0;i<3;++i) {
                    s->gyro_bias_vector_mdps[i]=(s->calibration_sum[i]*1000)/(131*(int32_t)s->calibration_count);
                    mean_accel[i]=s->calibration_accel_sum[i]/(int32_t)s->calibration_count;
                }
                gravity_norm=square_root((uint32_t)(mean_accel[0]*mean_accel[0])+
                    (uint32_t)(mean_accel[1]*mean_accel[1])+(uint32_t)(mean_accel[2]*mean_accel[2]));
                /* Every accepted norm is near1g, but guard the average if
                 * physically moved between opposite attitudes during calibration. */
                if(gravity_norm<12000u || gravity_norm>20000u) {
                    calibration_reset(s);s->imu_reason=LC_CAL_GRAVITY_RANGE;return 1;
                }
                for(i=0;i<3;++i)s->gravity_q12[i]=mean_accel[i]*4096/(int32_t)gravity_norm;
                s->gyro_bias_mdps=project_vertical(s->gyro_bias_vector_mdps,s->gravity_q12);
                s->calibrated=1; s->yaw_mdeg=0; s->yaw_remainder=0;
                s->target_yaw_mdeg=0; s->target_remainder=0;
                s->imu_reason=LC_IMU_READY;
            }
        }
    }
    for(i=0;i<3;++i)g[i]=(g[i]*1000)/131-s->gyro_bias_vector_mdps[i];
    rate=s->calibrated?project_vertical(g,s->gravity_q12):0;
    /* Planar yaw deadband remains narrow: retain actual slow turns. */
    if(s->calibrated && absolute(rate)<=30)rate=0;
    s->gyro_mdps=rate;
    if (s->calibrated && dt) {
        magnitude=s->yaw_remainder+rate*(int32_t)dt;
        s->yaw_mdeg=angle(s->yaw_mdeg+magnitude/1000);
        s->yaw_remainder=magnitude%1000;
    }
    return 1;
}
static void sonar_update(struct lc_state *s,const struct lc_input *in,unsigned i)
{
    struct lc_kalman *k=&s->sonar[i];
    uint32_t seq=in->sonar_sequence[i], dt, p, gain;
    uint16_t z=in->sonar_mm[i]; int32_t residual;
    if (!(in->sonar_online&(1u<<i)) || !seq || z<30u || z==UINT16_MAX ||
        in->now_ms-in->sonar_stamp_ms[i]>LC_SONAR_TIMEOUT_MS) { k->valid=0; return; }
    if(k->seen && seq==k->sequence)return;
    dt=k->seen?in->sonar_stamp_ms[i]-k->stamp_ms:0;
    if(z>4500u)z=4500u;
    residual=(int32_t)z-k->x_q8/256;
    /* Near obstacles bypass estimation; sudden changes while rotating start a
     * new scene. For a large far jump while straight, require two consecutive
     * far frames, keeping the FIRST candidate unavailable rather than freshening
     * the previous wall estimate. No prediction is ever a new observation. */
    if(!k->seen || dt>500u || z<=350u || residual<=-50 ||
       absolute(s->gyro_mdps)>15000 || (residual>200 && k->far_pending)) {
        k->x_q8=(int32_t)z*256; k->variance=400; k->valid=1; k->far_pending=0;
    } else if(residual>200) {
        k->far_mm=z; k->far_pending=1; k->valid=0;
    } else {
        k->far_pending=0; p=k->variance+limit((int32_t)dt,10,1000);
        if(p>30000u)p=30000u;
        gain=(p*32768u)/(p+400u); /* R=(20mm)^2, Q≈dt mm² */
        k->x_q8+=(residual*(int32_t)gain)/128;
        k->variance=(p*(32768u-gain))/32768u; k->valid=1;
    }
    k->seen=1; k->sequence=seq; k->stamp_ms=in->sonar_stamp_ms[i];
}
static void fast_step(struct lc_state *s,const struct lc_input *in,struct lc_output *o,int imu_ready)
{
    unsigned i; uint32_t dt=in->now_ms-s->last_step_ms;
    int active=(in->flags&(LC_PROTOCOL_MOT4|LC_CONFIGURED|LC_ARMED|LC_WATCHDOG|LC_CPU_TRAP))==7u;
    int32_t targets[2], w, error, gyro_mradps, e, u, iq, delta, top;
    if(!dt)return;
    if(dt>s->max_step_ms)s->max_step_ms=dt;
    if(dt>25u)++s->late_count;
    ++s->steps; s->last_step_ms=in->now_ms;
    for(i=0;i<4;++i) {
        delta=count_difference(in->counts[i],s->count_previous[i]);
        s->count_previous[i]=in->counts[i];
        if(absolute(delta)>(int32_t)(dt*450u+32u) || dt>40u) { imu_ready=0; s->wheel_mmps[i]=0; }
        else s->wheel_mmps[i]=(delta*LC_TICK_NM)/((int32_t)dt*1000);
        if(i==0 || i==2)s->wheel_mmps[i]=-s->wheel_mmps[i];
    }
    if(!active || !imu_ready || !actuation_allowed(s,in,in->yaw_rate_mradps) ||
       (!in->linear_mmps && !in->yaw_rate_mradps) ||
       in->left_mask==0 || in->left_mask==15) {
        zero_duties(s,o); s->active_previous=0; s->target_yaw_mdeg=s->yaw_mdeg;
        s->target_remainder=0; return;
    }
    if(!s->active_previous || s->epoch!=in->epoch) {
        zero_duties(s,o); s->target_yaw_mdeg=s->yaw_mdeg; s->target_remainder=0;
    }
    s->active_previous=1; s->epoch=in->epoch;
    /* yaw trajectory in mdeg: mrad * 180/pi ≈ mrad * 57296/1000. */
    top=s->target_remainder+((int32_t)in->yaw_rate_mradps*57296/1000)*(int32_t)dt;
    s->target_yaw_mdeg=angle(s->target_yaw_mdeg+top/1000);
    s->target_remainder=top%1000;
    error=limit(angle(s->target_yaw_mdeg-s->yaw_mdeg),-30000,30000);
    s->target_yaw_mdeg=angle(s->yaw_mdeg+error);
    gyro_mradps=(s->gyro_mdps*1745)/100000;
    w=limit((int32_t)in->yaw_rate_mradps+error*7/100-
            (gyro_mradps-in->yaw_rate_mradps)*3/10,-1000,1000);
    if(!actuation_allowed(s,in,w)) {
        zero_duties(s,o);s->active_previous=0;s->target_yaw_mdeg=s->yaw_mdeg;
        s->target_remainder=0;return;
    }
    /* v_left=v-w*track/2, CCW positive. 180635um track, rounded mm/s. */
    delta=w*(int32_t)LC_TRACK_UM/2000000;
    targets[0]=in->linear_mmps-delta; targets[1]=in->linear_mmps+delta;
    top=absolute(targets[0]); if(absolute(targets[1])>top)top=absolute(targets[1]);
    if(top>300) { targets[0]=targets[0]*300/top; targets[1]=targets[1]*300/top; }
    for(i=0;i<4;++i) {
        int32_t target=targets[(in->left_mask&(1u<<i))?0:1];
        if(!target) { o->duty[i]=0;s->integral_q8[i]=0;continue; }
        if((o->duty[i]>0 && target<0)||(o->duty[i]<0 && target>0))s->integral_q8[i]=0;
        e=limit(target-s->wheel_mmps[i],-600,600);
        iq=limit(s->integral_q8[i]+e*8,-160*256,160*256);
        /* PI: Kp=.5 duty/(mm/s), Ki=.03125 per20ms; 2x feedforward is a
         * bounded starting estimate, not a measured motor calibration. */
        u=target*2+(e*128+iq)/256;
        if((u<=LC_MAX_DUTY || e<0) && (u>=-LC_MAX_DUTY || e>0))s->integral_q8[i]=iq;
        u=limit(u,-LC_MAX_DUTY,LC_MAX_DUTY);
        if(target>0 && u<0)u=0;
        if(target<0 && u>0)u=0;
        o->duty[i]=(int16_t)u;
    }
}
int lc_update(struct lc_state *s,const struct lc_input *in,struct lc_output *o)
{
    unsigned i;int fresh,ready,commit=0;
    if(!s->initialized) {
        lc_init(s);s->initialized=1;s->last_step_ms=in->now_ms;
        for(i=0;i<4;++i)s->count_previous[i]=in->counts[i];
        zero_duties(s,o);commit=1;
    }
    fresh=imu_update(s,in);ready=fresh&&s->calibrated;
    for(i=0;i<3;++i) {
        sonar_update(s,in,i);
        o->sonar_mm[i]=(uint16_t)(s->sonar[i].x_q8/256);
        o->sonar_sequence[i]=s->sonar[i].sequence;
    }
    /* Stop packets and any loss of authority bypass the20ms PI deadline. */
    if((in->flags&31u)!=7u || (!in->linear_mmps&&!in->yaw_rate_mradps) ||
       !ready || !actuation_allowed(s,in,in->yaw_rate_mradps)) {
        for(i=0;i<4;++i)if(o->duty[i])commit=1;
        zero_duties(s,o);s->active_previous=0;
    }
    if(in->now_ms-s->last_step_ms>=LC_PERIOD_MS) {fast_step(s,in,o,ready);commit=1;}
    o->flags=1u|(ready?2u:0u);
    o->flags|=(uint32_t)s->imu_reason<<LC_REASON_SHIFT;
    if(s->calibrated)o->flags|=LC_MOUNT_VALID;
    if(s->calibrated && s->gravity_q12[2]<0)o->flags|=LC_MOUNT_Z_DOWN;
    if(s->source_reset_seen)o->flags|=LC_SOURCE_RESTART_SEEN;
    for(i=0;i<3;++i)if(s->sonar[i].valid && in->sonar_online&(1u<<i) &&
        in->now_ms-s->sonar[i].stamp_ms<=LC_SONAR_TIMEOUT_MS)o->flags|=1u<<(8+i);
    o->yaw_mdeg=(uint32_t)s->yaw_mdeg;o->bias_mdps=(uint32_t)s->gyro_bias_mdps;
    o->calibration_progress=s->calibrated?100u:s->calibration_count?
        (uint32_t)limit((int32_t)(s->imu_stamp_ms-s->calibration_started_ms)/20,0,99):0;
    o->late_count=s->late_count;
    return commit;
}
