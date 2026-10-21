#ifndef PERCEPTION_DETECTOR_H
#define PERCEPTION_DETECTOR_H

#include <memory>
#include <string>
#include <vector>

struct Detection {
    float left, top, right, bottom, score;
    int class_id;
};

// Wall times of SDK calls, not hardware-only NPU profiling. One record per eye.
struct DetectorTiming {
    double preprocess_ms=0, input_ms=0, run_ms=0, output_ms=0;
    double postprocess_ms=0, release_ms=0, total_ms=0;
};

class RknnDetector {
public:
    RknnDetector();
    ~RknnDetector();
    bool open(const std::string &model, float confidence, float nms,
              int class_count, std::string &error);
    bool infer(const unsigned char *rgb, int width, int height, int stride,
               std::vector<Detection> &results, std::string &error,
               DetectorTiming *timing=nullptr);
    // -1=runtime auto, 0/1/2=one RK3588 NPU core. Optional runtime capability.
    bool set_core(int core, std::string &error);

    RknnDetector(const RknnDetector &) = delete;
    RknnDetector &operator=(const RknnDetector &) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif
