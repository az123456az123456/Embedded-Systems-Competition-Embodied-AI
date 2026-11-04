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
    // Values outside the documented measuring range never become clear space.
    for(unsigned invalid=0;invalid<3;++invalid){
        struct sensor_sample bad=v;bad.sequence++;bad.uptime_ms+=50;
        bad.distance_mm[0]=(const unsigned[]){0,4501,65535}[invalid];
        CHECK(!sensor_update(&s,&bad,1100)&&s.sample.sequence==v.sequence);
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
