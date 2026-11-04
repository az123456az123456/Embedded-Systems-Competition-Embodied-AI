#include "sensor_data.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned checks;
#define CHECK(x) do{checks++;assert(x);}while(0)
static void put16(unsigned char *p,unsigned v){p[0]=v>>8;p[1]=v;}
static void put32(unsigned char *p,uint32_t v){p[0]=v>>24;p[1]=v>>16;p[2]=v>>8;p[3]=v;}
static void seal(unsigned char *b){put16(b+14+64,sensor_crc16(b+14,64));}
static void fixture(unsigned char *b){
    memset(b,0,84);memset(b,255,6);
    const unsigned char src[]={2,0xac,0x13,0x80,0,1,0x88,0xb8};memcpy(b+6,src,8);
    unsigned char *p=b+14;memcpy(p,"SNS1",4);p[4]=1;p[5]=7;p[7]=5;
    put32(p+8,123);put32(p+12,6150);
    put32(p+16,1000);put32(p+20,(uint32_t)-2000);put32(p+24,0x80000000);put32(p+28,0x7fffffff);
    put32(p+32,10);put32(p+36,(uint32_t)-20);put32(p+40,0);put32(p+44,30);
    put16(p+48,1953);put16(p+50,600);put16(p+52,4500);
    put16(p+54,50);put16(p+56,200);put16(p+58,1000);put16(p+60,50);p[62]=10;seal(b);
}
static void fcs(unsigned char *b){
    uint32_t c=UINT32_MAX;for(unsigned i=0;i<80;i++){c^=b[i];for(unsigned j=0;j<8;j++)c=(c>>1)^((c&1)?0xedb88320u:0);}
    c=~c;for(unsigned i=0;i<4;i++)b[80+i]=(unsigned char)(c>>(i*8));
}
int main(void){
    CHECK(sensor_crc16((const unsigned char *)"123456789",9)==0x29b1);
    unsigned char b[84],bad[84];struct sensor_sample sample,other;fixture(b);
    CHECK(sensor_decode(b,80,&sample));
    CHECK(sample.sequence==123&&sample.uptime_ms==6150&&sample.invert_mask==5&&sample.illegal_mask==10);
    CHECK(sample.position[0]==1000&&sample.position[1]==-2000&&sample.position[2]==INT32_MIN&&sample.position[3]==INT32_MAX);
    CHECK(sample.delta[0]==10&&sample.delta[1]==-20&&sample.delta[2]==0&&sample.delta[3]==30);
    CHECK(sample.distance_mm[0]==1953&&sample.distance_mm[1]==600&&sample.distance_mm[2]==4500&&sample.window_ms==50);
    for(unsigned i=0;i<80;i++){memcpy(bad,b,84);bad[i]^=1;CHECK(!sensor_decode(bad,80,&other));}
    for(unsigned n=0;n<84;n++)if(n!=80)CHECK(!sensor_decode(b,n,&other));
    CHECK(!sensor_decode(NULL,80,&other)&&!sensor_decode(b,80,NULL));
    fcs(b);CHECK(sensor_decode(b,84,&other));b[82]^=1;CHECK(!sensor_decode(b,84,&other));
    const unsigned offsets[]={4,5,6,7,63,62};const unsigned values[]={2,8,7,16,1,16};
    for(unsigned i=0;i<sizeof(offsets)/sizeof(*offsets);i++){fixture(bad);bad[14+offsets[i]]=values[i];seal(bad);CHECK(!sensor_decode(bad,80,&other));}
    fixture(bad);put16(bad+14+48,29);seal(bad);CHECK(!sensor_decode(bad,80,&other));
    fixture(bad);put16(bad+14+50,65535);seal(bad);CHECK(!sensor_decode(bad,80,&other));
    fixture(bad);put16(bad+14+54,1001);seal(bad);CHECK(!sensor_decode(bad,80,&other));
    fixture(bad);put16(bad+14+60,0);seal(bad);CHECK(!sensor_decode(bad,80,&other));
    memset(bad+14+32,0,16);seal(bad);CHECK(sensor_decode(bad,80,&other)&&other.window_ms==0);
    CHECK(other.position[0]==1000&&other.distance_mm[0]==1953);
    put16(bad+14+60,1);seal(bad);CHECK(!sensor_decode(bad,80,&other));
    fixture(bad);bad[14+5]=0;bad[14+6]=7;put16(bad+14+54,65535);seal(bad);CHECK(sensor_decode(bad,80,&other));
    for(unsigned raw_index=0;raw_index<8;++raw_index){
        const uint16_t raw=(const uint16_t[]){4501,4559,4600,4799,5936,6001,32000,65534}[raw_index];
        fixture(bad);put16(bad+14+48,raw);seal(bad);
        CHECK(sensor_decode(bad,80,&other)&&other.distance_mm[0]==raw);
        bad[14+48]^=1;CHECK(!sensor_decode(bad,80,&other));
        fixture(bad);put16(bad+14+48,raw);bad[14+6]=1;seal(bad);
        CHECK(!sensor_decode(bad,80,&other)); // valid/fault overlap cannot be repaired by saturation
    }
    struct sensor_state state;sensor_init(&state);double metres=999;
    CHECK(sensor_distance(&state,0,1000,&metres)==SENSOR_WAITING&&metres==0);
    CHECK(sensor_update(&state,&sample,1000));CHECK(sensor_distance(&state,0,1000,&metres)==SENSOR_VALID&&metres==1.953);
    CHECK(sensor_distance(&state,1,1800,&metres)==SENSOR_VALID&&metres==.6);
    CHECK(sensor_distance(&state,1,1801,&metres)==SENSOR_STALE&&metres==0);
    CHECK(sensor_distance(&state,2,1000,&metres)==SENSOR_VALID&&metres==4.5);
    CHECK(sensor_distance(&state,2,1001,&metres)==SENSOR_STALE&&metres==0);
    CHECK(sensor_distance(&state,0,2001,&metres)==SENSOR_STALE&&metres==0);
    CHECK(sensor_distance(&state,3,1000,&metres)==SENSOR_WAITING);
    CHECK(!sensor_link_fresh(&state,999));
    CHECK(!sensor_update(&state,&sample,1100)&&state.received_ms==1000);
    struct sensor_sample old=sample;old.sequence--;old.uptime_ms-=50;CHECK(!sensor_update(&state,&old,1100)&&state.received_ms==1000);
    struct sensor_sample next=sample;next.sequence++;next.uptime_ms+=50;CHECK(sensor_update(&state,&next,1050));
    CHECK(!sensor_update(&state,&next,1049));
    next.sequence++;next.uptime_ms+=5000;CHECK(!sensor_update(&state,&next,1100));
    sensor_init(&state);sample.sequence=UINT32_MAX;sample.uptime_ms=UINT32_MAX-25;CHECK(sensor_update(&state,&sample,1000));
    sample.sequence=0;sample.uptime_ms=24;CHECK(sensor_update(&state,&sample,1050));
    sample.sequence=1;sample.uptime_ms=74;CHECK(sensor_update(&state,&sample,1100));
    sensor_init(&state);sample.sequence=200;sample.uptime_ms=10000;CHECK(sensor_update(&state,&sample,1000));
    next=sample;next.sequence=1;next.uptime_ms=50;CHECK(!sensor_update(&state,&next,1300)&&state.received_ms==1000);
    CHECK(!sensor_update(&state,&next,1320)&&state.received_ms==1000);
    next.sequence=2;next.uptime_ms=100;CHECK(sensor_update(&state,&next,1350)&&state.restarts==1);
    old=sample;old.sequence++;old.uptime_ms+=50;CHECK(!sensor_update(&state,&old,1370)&&state.sample.sequence==2);
    /* A board may reboot while its network is unavailable for a minute. A
       fixed two-second "new boot" ceiling would lock it out indefinitely. */
    sensor_init(&state);sample.sequence=20000;sample.uptime_ms=1000000;
    CHECK(sensor_update(&state,&sample,1000));
    next=sample;next.sequence=1000;next.uptime_ms=50000;
    CHECK(!sensor_update(&state,&next,61000));
    next.sequence++;next.uptime_ms+=50;
    CHECK(sensor_update(&state,&next,61050)&&state.restarts==1);
    next.sequence++;next.uptime_ms+=50;next.valid_mask=0;next.fault_mask=7;next.age_ms[0]=UINT16_MAX;
    CHECK(sensor_update(&state,&next,61100));CHECK(sensor_distance(&state,0,61100,&metres)==SENSOR_WAITING);
    CHECK(sensor_distance(&state,1,61100,&metres)==SENSOR_FAULT&&metres==0);
    sensor_init(&state);fixture(bad);CHECK(sensor_decode(bad,80,&sample));
    CHECK(sensor_update(&state,&sample,1000));
    next=sample;next.window_ms=0;memset(next.delta,0,sizeof(next.delta));next.sequence++;
    CHECK(sensor_update(&state,&next,1015)&&state.received_ms==1000);
    next.sequence++;CHECK(!sensor_update(&state,&next,1101)&&state.received_ms==1000);
    next.uptime_ms++;CHECK(sensor_update(&state,&next,1102)&&state.received_ms==1102);
    next.sequence++;next.delta[2]=1;CHECK(!sensor_update(&state,&next,1103));
    printf("PASS sensor protocol/state: %u checks (SNS1 CRC/FCS, strict frame/masks/range, signed counts, units, independent age expiry, duplicate/order/wrap and conservative two-frame reboot)\n",checks);
    return 0;
}
