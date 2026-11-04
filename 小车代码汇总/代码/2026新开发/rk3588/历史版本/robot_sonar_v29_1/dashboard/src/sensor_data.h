#ifndef ACG138_SENSOR_DATA_H
#define ACG138_SENSOR_DATA_H
#include <stddef.h>
#include <stdint.h>
#define SENSOR_ETHERTYPE 0x88b8
#define SENSOR_MAX_AGE_MS 1000u
#define SENSOR_FRAME_BYTES 80u
#define SENSOR_RANGE_MAX_MM 4500u
#define SENSOR_DISPLAY_LIMIT_MM 4000u
/* Front, left, right throughout; E1..E4 correspond to M1..M4. */
struct sensor_sample {
    uint32_t sequence, uptime_ms;
    int32_t position[4], delta[4];
    uint16_t distance_mm[3], age_ms[3], window_ms;
    uint8_t valid_mask, fault_mask, invert_mask, illegal_mask;
};
struct sensor_state {
    struct sensor_sample sample;
    uint64_t received_ms;
    uint64_t restart_seen_ms;
    uint32_t restart_sequence, restart_uptime;
    unsigned have_sample, restart_pending, restarts;
};
enum sensor_distance_status {
    SENSOR_WAITING, SENSOR_VALID, SENSOR_STALE, SENSOR_FAULT
};
uint16_t sensor_crc16(const unsigned char *data, size_t size);
int sensor_decode(const unsigned char *frame, size_t size, struct sensor_sample *out);
void sensor_init(struct sensor_state *state);
int sensor_update(struct sensor_state *state, const struct sensor_sample *sample, uint64_t now_ms);
int sensor_link_fresh(const struct sensor_state *state, uint64_t now_ms);
enum sensor_distance_status sensor_distance(const struct sensor_state *state, unsigned channel,
                                           uint64_t now_ms, double *metres);
/* Presentation only. A valid reading above 4 m is displayed as ">4 m";
   navigation/follow callers retain the original measured distance and age. */
enum sensor_distance_status sensor_distance_text(const struct sensor_state *state, unsigned channel,
                                                uint64_t now_ms, char *text, size_t size);
int sensor_open_socket(const char *interface_name);
#endif
