#include "sensor_data.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned checks;
#define CHECK(x) do{++checks;assert(x);}while(0)
int main(void){
    struct sensor_state s;sensor_init(&s);char text[32];double distance=999;
    CHECK(sensor_distance_text(&s,0,1000,text,sizeof(text))==SENSOR_WAITING&&!strcmp(text,"-- m"));
    struct sensor_sample v={.sequence=1,.uptime_ms=50,.window_ms=50,.valid_mask=7,
        .distance_mm={3999,4000,4001},.age_ms={0,0,0}};
    CHECK(sensor_update(&s,&v,1000));
    CHECK(sensor_distance_text(&s,0,1000,text,sizeof(text))==SENSOR_VALID&&!strcmp(text,"4.00 m"));
    CHECK(sensor_distance_text(&s,1,1000,text,sizeof(text))==SENSOR_VALID&&!strcmp(text,"4.00 m"));
    CHECK(sensor_distance_text(&s,2,1000,text,sizeof(text))==SENSOR_VALID&&!strcmp(text,">4 m"));
    CHECK(sensor_distance(&s,2,1000,&distance)==SENSOR_VALID&&distance==4.001);
    v.sequence++;v.uptime_ms+=50;v.distance_mm[0]=4500;v.distance_mm[1]=30;
    CHECK(sensor_update(&s,&v,1050));
    CHECK(sensor_distance_text(&s,0,1050,text,sizeof(text))==SENSOR_VALID&&!strcmp(text,">4 m"));
    CHECK(sensor_distance(&s,0,1050,&distance)==SENSOR_VALID&&distance==4.5);
    CHECK(sensor_distance_text(&s,1,1050,text,sizeof(text))==SENSOR_VALID&&!strcmp(text,"0.03 m"));
    // Unknown/sentinel values are not numeric observations and never become clear space.
    for(unsigned invalid=0;invalid<3;++invalid){
        struct sensor_sample bad=v;bad.sequence++;bad.uptime_ms+=50;
        bad.distance_mm[0]=(const unsigned[]){0,29,65535}[invalid];
        CHECK(!sensor_update(&s,&bad,1100)&&s.sample.sequence==v.sequence);
    }
    // Real SNS1 raw-before-filter values must not make the entire packet
    // disappear. Retain the raw observation, censor only control metres,
    // and keep both host/source age and fault gates independent.
    for(unsigned raw_index=0;raw_index<8;++raw_index){
        const uint16_t raw=(const uint16_t[]){4501,4559,4600,4799,5936,6001,32000,65534}[raw_index];
        struct sensor_state far;sensor_init(&far);
        struct sensor_sample observed={.sequence=1,.uptime_ms=50,.window_ms=50,.valid_mask=7,
            .distance_mm={raw,1000,2000},.age_ms={17,0,0}};
        CHECK(sensor_update(&far,&observed,1000));
        CHECK(far.sample.distance_mm[0]==raw&&far.sample.age_ms[0]==17&&far.received_ms==1000);
        CHECK(sensor_distance(&far,0,1000,&distance)==SENSOR_VALID&&distance==4.5);
        CHECK(sensor_distance_text(&far,0,1000,text,sizeof(text))==SENSOR_VALID&&!strcmp(text,">4 m"));
        CHECK(sensor_distance(&far,0,1984,&distance)==SENSOR_STALE&&distance==0);
        CHECK(sensor_distance(&far,0,2001,&distance)==SENSOR_STALE&&distance==0);
        observed.sequence++;observed.uptime_ms+=50;observed.valid_mask=6;observed.fault_mask=1;
        CHECK(sensor_update(&far,&observed,1050));
        CHECK(sensor_distance(&far,0,1050,&distance)==SENSOR_FAULT&&distance==0);
        observed.sequence++;observed.uptime_ms+=50;observed.age_ms[0]=UINT16_MAX;
        CHECK(sensor_update(&far,&observed,1100));
        CHECK(sensor_distance(&far,0,1100,&distance)==SENSOR_WAITING&&distance==0);
    }
    v.sequence++;v.uptime_ms+=50;v.valid_mask=6;v.fault_mask=1;v.age_ms[0]=10;
    CHECK(sensor_update(&s,&v,1100));
    CHECK(sensor_distance_text(&s,0,1100,text,sizeof(text))==SENSOR_FAULT&&!strcmp(text,"-- m"));
    CHECK(sensor_distance(&s,0,1100,&distance)==SENSOR_FAULT&&distance==0);
    v.sequence++;v.uptime_ms+=50;v.age_ms[0]=UINT16_MAX;
    CHECK(sensor_update(&s,&v,1150));
    CHECK(sensor_distance_text(&s,0,1150,text,sizeof(text))==SENSOR_WAITING&&!strcmp(text,"-- m"));
    CHECK(sensor_distance_text(&s,2,2151,text,sizeof(text))==SENSOR_STALE&&!strcmp(text,"-- m"));
    CHECK(sensor_distance_text(&s,3,1150,text,sizeof(text))==SENSOR_WAITING&&!strcmp(text,"-- m"));
    CHECK(sensor_distance_text(&s,2,1150,NULL,0)==SENSOR_VALID);
    char small[2]={0,1};CHECK(sensor_distance_text(&s,2,1150,small,sizeof(small))==SENSOR_VALID&&small[0]=='>'&&small[1]==0);
    printf("PASS sonar display: %u checks (4m boundaries, raw range retained, faults/stale/invalid never clear)\n",checks);
    return 0;
}
