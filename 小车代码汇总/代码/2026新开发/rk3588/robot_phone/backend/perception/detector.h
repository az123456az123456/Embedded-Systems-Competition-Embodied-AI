#ifndef PERCEPTION_DETECTOR_H
#define PERCEPTION_DETECTOR_H

#include <memory>
#include <string>
#include <vector>

struct Detection {
    float left, top, right, bottom, score;
    int class_id;
};

class RknnDetector {
public:
    RknnDetector();
    ~RknnDetector();
    bool open(const std::string &model, float confidence, float nms,
              int class_count, std::string &error);
    bool infer(const unsigned char *rgb, int width, int height, int stride,
               std::vector<Detection> &results, std::string &error);

    RknnDetector(const RknnDetector &) = delete;
    RknnDetector &operator=(const RknnDetector &) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif
