#ifndef UI_SCHEDULE_H
#define UI_SCHEDULE_H
#include <stdint.h>

#define UI_RENDER_INTERVAL_MS 33u
#define UI_IDLE_POLL_MS 10

/* The render limit stays at ~30 Hz. Shorten the idle wait near its deadline
 * rather than rounding every 33 ms frame to the next 10 ms IMU/poll tick.
 * now_ms must be read after rendering and Wayland dispatch, not before them. */
static inline int ui_poll_timeout_ms(uint64_t now_ms,uint64_t last_render_ms,
                                     int configured,int callback_pending) {
    if(!configured || callback_pending) return UI_IDLE_POLL_MS;
    uint64_t elapsed=now_ms>=last_render_ms?now_ms-last_render_ms:0;
    if(elapsed>=UI_RENDER_INTERVAL_MS) return 1;
    unsigned remaining=UI_RENDER_INTERVAL_MS-(unsigned)elapsed;
    return remaining<UI_IDLE_POLL_MS?(int)remaining:UI_IDLE_POLL_MS;
}
#endif
