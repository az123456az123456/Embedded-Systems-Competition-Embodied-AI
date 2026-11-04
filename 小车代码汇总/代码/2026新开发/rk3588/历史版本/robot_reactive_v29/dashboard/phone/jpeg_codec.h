#ifndef ROBOT_PHONE_JPEG_CODEC_H
#define ROBOT_PHONE_JPEG_CODEC_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int jpeg_from_argb(const uint32_t *,int width,int height,int quality,void (*write)(void *,void *,int),void *context);
const char *jpeg_codec_name(void);
#ifdef __cplusplus
}
#endif
#endif
