#ifndef PERCEPTION_VISION_H
#define PERCEPTION_VISION_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define VISION_W 864
#define VISION_H 486
#define VISION_OBJECTS 128
struct vision_object {
    float left, top, right, bottom, score;
    int class_id, eye;
    char name[64];
};
struct vision_snapshot {
    uint32_t pixels[VISION_W * VISION_H];
    uint64_t timestamp_ms;
    int have_frame, count, demo;
    float fps, inference_ms;
    struct vision_object objects[VISION_OBJECTS];
    char status[160], source[64];
};
struct vision_options {
    const char *model, *device, *labels, *view;
    int width, height;
    float confidence, nms;
};
struct vision_worker;
struct vision_worker *vision_start(const struct vision_options *);
void vision_snapshot_copy(struct vision_worker *, struct vision_snapshot *);
void vision_stop(struct vision_worker *);
void vision_demo(struct vision_snapshot *, uint64_t elapsed_ms);
#ifdef __cplusplus
}
#endif
#endif
