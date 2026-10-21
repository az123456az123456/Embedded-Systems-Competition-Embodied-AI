#ifndef PERCEPTION_DETECTOR_MATH_H
#define PERCEPTION_DETECTOR_MATH_H

#include "detector.h"
#include <cstddef>

namespace detector_math {

struct Letterbox {
    int source_w, source_h, target_w, target_h;
    int resized_w, resized_h, pad_x, pad_y;
    float scale_x, scale_y;
};

struct TensorShape {
    std::vector<int> dims;
    bool nchw;
    TensorShape() : nchw(true) {}
    TensorShape(std::initializer_list<int> shape) : dims(shape), nchw(true) {}
};

struct Head {
    const float *boxes;
    const float *classes;
    const float *score_sum;
    int grid_w, grid_h, dfl_len, class_count;
};

bool make_letterbox(int source_w, int source_h, int target_w, int target_h,
                    Letterbox &box, std::string &error);
bool preprocess_rgb(const unsigned char *rgb, int width, int height, int stride,
                    int target_w, int target_h, std::vector<unsigned char> &out,
                    Letterbox &box, std::string &error);
bool validate_contract(const std::vector<TensorShape> &outputs, int model_w,
                       int model_h, int class_count, std::string &error);
bool dfl_expectation(const float *logits, int bins, int element_stride, float &value);
void class_aware_nms(std::vector<Detection> &candidates, float threshold,
                     std::size_t max_results, std::vector<Detection> &results);
bool decode_heads(const std::vector<Head> &heads, const Letterbox &box,
                  float confidence, float nms, std::vector<Detection> &results,
                  std::string &error, std::size_t max_candidates = 3000,
                  std::size_t max_results = 128);

} // namespace detector_math
#endif
