#ifndef NAV_MATH_H
#define NAV_MATH_H
#include <stdint.h>
#define NM_PI_URAD 3141593
#define NM_HALF_PI_URAD 1570796
#define NM_TAU_URAD 6283186
uint32_t nm_isqrt(uint32_t n);
/* Inputs must be local mm differences <=32767; returns UINT32_MAX otherwise. */
uint32_t nm_distance(int32_t dx,int32_t dy);
int32_t nm_wrap(int32_t angle_urad);
int32_t nm_atan2(int32_t y,int32_t x);
void nm_sincos(int32_t yaw_urad,int32_t *s_q15,int32_t *c_q15);
int32_t nm_project(int32_t dx_mm,int32_t dy_mm,int32_t yaw_urad);
#endif
