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
#define VISION_PAIR_PARTIAL 1
#define VISION_PAIR_COMPLETE 2
#define VISION_PAIR_WARN_MS 50
#define VISION_PAIR_CONTROL_TIMEOUT_MS 100
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
    // source_objects are the unmodified NPU detections in the same index order.
    // objects may be aligned to the preview, but control freshness/IDs continue
    // to use the original detection frame and sensor timestamp above.
    int source_count, motion_enabled, motion_compensated;
    float tracking_ms;
    unsigned char motion_valid[VISION_OBJECTS];
    struct vision_object source_objects[VISION_OBJECTS];
    // Public boxes belong to one source frame; the mask identifies ready eyes.
    // Missing eyes are absent, never filled with previous-frame detections.
    uint64_t eye_frame_id[2], eye_source_timestamp_ms[2], eye_result_timestamp_ms[2];
    unsigned int eye_complete_mask;
    uint64_t completed_eye_jobs, published_pairs, out_of_order_pairs;
    uint64_t expired_pairs, pending_expired_pairs, capture_superseded;
    float eye_completion_fps;
    // Physical-camera coordinate contract for follow-bearing consumers only.
    // Public NPU/preview boxes remain in VISION_W x VISION_H tensor-view pixels.
    int image_geometry_kind; // 0=legacy full eye, 1=FPGA letterbox640
    int tensor_eye_width,tensor_eye_height,physical_eye_width,physical_eye_height;
    int tensor_content_x,tensor_content_y,tensor_content_width,tensor_content_height;
    int preview_content_y,preview_content_height;
    uint32_t camera_source_sequence;
    uint16_t camera_source_epoch;
    uint64_t detection_update_id;
    int pair_status, counterpart_timeout, control_eligible;
    float counterpart_wait_ms;
    unsigned int eye_requested_mask;
    uint32_t detection_source_sequence;
    uint16_t detection_source_epoch;
    // Complete control observations have their own latest-only channel so an
    // early next-frame partial cannot hide a completed pair from the controller.
    uint64_t control_update_id, control_frame_id, control_timestamp_ms;
    uint32_t control_source_sequence;uint16_t control_source_epoch;
    unsigned int control_complete_mask;
    int control_count, control_available;
    struct vision_object control_objects[VISION_OBJECTS];
    uint64_t partial_events, partial_publications, observation_updates;
    uint64_t pair_warn_timeouts, pair_control_timeouts;
    uint64_t capture_age_expired, unstarted_age_expired;

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
uint64_t vision_snapshot_generation(struct vision_worker *);
void vision_observation_copy(struct vision_worker *, struct vision_snapshot *);
void vision_stop(struct vision_worker *);
void vision_demo(struct vision_snapshot *, uint64_t elapsed_ms);
#ifdef __cplusplus
}
#endif
#endif
