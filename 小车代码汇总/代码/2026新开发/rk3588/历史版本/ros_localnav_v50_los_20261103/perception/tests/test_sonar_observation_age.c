/* A filtered distance may come from the preceding UART frame. Frequent SNS
   snapshots carry that observation's increasing age, not a new measurement. */
#include "sensor_data.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    struct sensor_state s; sensor_init(&s);
    struct sensor_sample v={.sequence=1,.uptime_ms=100,.window_ms=50,
        .valid_mask=7,.distance_mm={500,1200,4500},.age_ms={0,0,0}};
    assert(sensor_update(&s,&v,10000));
    for(unsigned elapsed=50;elapsed<=1000;elapsed+=50) {
        v.sequence++;v.uptime_ms+=50;
        for(unsigned ch=0;ch<3;++ch)v.age_ms[ch]=elapsed;
        assert(sensor_update(&s,&v,10000+elapsed));
        double metres=0;
        assert(sensor_distance(&s,0,10000+elapsed,&metres)==SENSOR_VALID);
        assert(metres==.5);
    }
    // A still-live transport cannot renew an old physical observation.
    assert(sensor_link_fresh(&s,11001));
    assert(sensor_distance(&s,0,11001,NULL)==SENSOR_STALE);
    // Confirmed farther pair: minimum 1200 came from the previous frame,
    // 100 ms earlier. That age must survive decoding/UI freshness checks.
    v.sequence++;v.uptime_ms+=50;v.distance_mm[0]=1200;
    v.age_ms[0]=100;v.age_ms[1]=0;v.age_ms[2]=0;
    assert(sensor_update(&s,&v,11050));
    assert(sensor_distance(&s,0,11950,NULL)==SENSOR_VALID);
    assert(sensor_distance(&s,0,11951,NULL)==SENSOR_STALE);
    assert(sensor_distance(&s,1,11951,NULL)==SENSOR_VALID);
    // Invalid/unknown values must never clear obstacles as "far away".
    v.sequence++;v.uptime_ms+=50;v.valid_mask=6;v.fault_mask=1;
    assert(sensor_update(&s,&v,12000));
    assert(sensor_distance(&s,0,12000,NULL)==SENSOR_FAULT);
    puts("PASS sonar observation age: live transport cannot renew old range; previous-frame age retained; unknown never far");
    return 0;
}
