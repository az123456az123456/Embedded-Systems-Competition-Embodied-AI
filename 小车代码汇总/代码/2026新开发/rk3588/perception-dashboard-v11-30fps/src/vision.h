#ifndef PERCEPTION_VISION_H
#define PERCEPTION_VISION_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define VISION_W 864
#define VISION_H 486
#define VISION_OBJECTS 128
#define VISION_BOX_MAX_AGE_MS 350
struct vision_object {
    float left, top, right, bottom, score;
    int class_id, eye;
    char name[64];
};
struct vision_snapshot {
    uint32_t pixels[VISION_W * VISION_H];
    uint64_t timestamp_ms;
    uint64_t frame_id, detection_frame_id, detection_timestamp_ms, result_timestamp_ms;
    int have_frame, count, demo;
    int npu_workers, npu_pinned, inflight_peak;
    uint64_t completed_pairs, discarded_pairs;
    float fps, inference_ms, capture_fps, frame_age_ms;
    float preview_fps, preview_ms, detection_age_ms;
    int detection_enabled, boxes_delayed, detection_available;
    struct vision_object objects[VISION_OBJECTS];
    char status[160], source[64];
};
struct vision_options {
    const char *model, *device, *labels, *view;
    int width, height;
    float confidence, nms;
    int no_detect, serial_eyes, sync_preview;
    int npu_workers; // 0=default (3), 1/2/3=worker limit; serial_eyes forces 1
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
