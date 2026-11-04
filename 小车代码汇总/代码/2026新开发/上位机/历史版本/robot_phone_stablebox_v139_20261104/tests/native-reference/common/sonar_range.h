#ifndef ACG138_SONAR_RANGE_H
#define ACG138_SONAR_RANGE_H
#include <stdint.h>
/* SNS1 carries checksum-accepted A02YYUW numeric observations. The v29.1
   FPGA may publish a new RAW_MODE value before its 20 ms CPU filter commit;
   the committed estimate then censors that same numeric observation at
   4500 mm (layered_control.c). Mirror that established firmware contract:
   valid numeric 30..65534 mm is bounded to 4500 mm for control. This is NOT
   a sensor-range claim. Packet checksum, valid/fault flags, source age and
   packet age independently govern usability. Never map zero/FFFF, a fault,
   missing source or stale observation into free space or refresh its stamp. */
#define ROBOT_SONAR_MIN_MM 30u
#define ROBOT_SONAR_RANGE_MM 4500u
#define ROBOT_SONAR_RAW_NUMERIC_MAX_MM 65534u
static inline int robot_sonar_range_supported(uint16_t raw_mm) {
    return raw_mm >= ROBOT_SONAR_MIN_MM && raw_mm <= ROBOT_SONAR_RAW_NUMERIC_MAX_MM;
}
static inline uint16_t robot_sonar_range_censored(uint16_t raw_mm) {
    return raw_mm > ROBOT_SONAR_RANGE_MM ? ROBOT_SONAR_RANGE_MM : raw_mm;
}
#endif
