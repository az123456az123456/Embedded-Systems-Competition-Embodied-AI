#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "ui_schedule.h"

/* A 10 ms IMU cadence interleaves with a 17 ms render/present cost.
 * A fixed 10 ms idle poll misses each 33 ms deadline until the 40 ms tick.
 * Count scheduled renders, not generated video frames or hardware FPS. */
static unsigned simulate(int adaptive) {
    uint64_t now=0,last=0;
    unsigned renders=0;
    while(now<1000) {
        if(!renders || now-last>=UI_RENDER_INTERVAL_MS) {
            last=now;renders++;now+=17;
        }
        int timeout=adaptive?ui_poll_timeout_ms(now,last,1,0):10;
        uint64_t next_imu=(now/10+1)*10;
        uint64_t next_poll=now+(unsigned)timeout;
        now=next_imu<next_poll?next_imu:next_poll;
    }
    return renders;
}
int main(void) {
    assert(UI_RENDER_INTERVAL_MS==33);
    assert(ui_poll_timeout_ms(30,0,1,0)==3);
    assert(ui_poll_timeout_ms(32,0,1,0)==1);
    assert(ui_poll_timeout_ms(33,0,1,0)==1);
    assert(ui_poll_timeout_ms(99,0,1,0)==1);
    /* Before configuration or while waiting for the compositor callback, do
     * not convert a missed render deadline into an unnecessary 1 ms poll. */
    assert(ui_poll_timeout_ms(99,0,0,0)==10);
    assert(ui_poll_timeout_ms(99,0,1,1)==10);
    assert(ui_poll_timeout_ms(99,0,0,1)==10);
    assert(ui_poll_timeout_ms(10,20,1,0)==10);
    assert(ui_poll_timeout_ms(UINT64_MAX,UINT64_MAX-32,1,0)==1);
    for(uint64_t elapsed=0;elapsed<1000;elapsed++) {
        int timeout=ui_poll_timeout_ms(1000+elapsed,1000,1,0);
        assert(timeout>=1 && timeout<=10);
    }
    /* Read the clock after the 17 ms rendering work, then after any dispatch;
     * a stale value from before that work would request an excessive wait. */
    assert(ui_poll_timeout_ms(30,0,1,0)<ui_poll_timeout_ms(13,0,1,0));
    unsigned fixed=simulate(0),adaptive=simulate(1);
    assert(fixed==25 && adaptive>=30 && adaptive<=31);
    printf("PASS UI deadline scheduling: fixed=%u/s adaptive=%u/s in 10ms IMU simulation; 33ms render limit, callback/configuration gate, late-clock and 1..10ms bounds\n",fixed,adaptive);
    return 0;
}
