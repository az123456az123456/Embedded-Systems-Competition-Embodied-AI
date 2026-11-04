#ifndef ROBOT_LAYERED_MMIO_H
#define ROBOT_LAYERED_MMIO_H
#include "layered_control.h"
#include "nav_stop.h"
extern struct lc_state layered_state;
void layered_service_init(void);
void layered_service(void);
/* One actual hardware snapshot every2ms. Raw encoder positions are direct
 * encoder_4ch counters latched with DEVICE_MS, not cached50ms SNS1 values. */
int layered_snapshot(void);
const struct lc_input *layered_raw(void);
const struct lc_output *layered_output(void);
/* This is the unchanged production PI/PD/Kalman implementation. Navigation
 * qualification selects commands; while uncalibrated it observes zero requests. */
int layered_step(int nav_selected,int32_t v_mmps,int32_t w_mradps,uint32_t epoch);
void layered_publish_legacy(int nav_selected);
void layered_stop_sample(struct nv_stop_sample *sample);
#endif
