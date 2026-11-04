#include "detector_math.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>

namespace detector_math {
namespace {
bool valid_dimension(int value) { return value > 0 && value <= 16384; }
bool valid_threshold(float value) {
    return std::isfinite(value) && value >= 0.0f && value <= 1.0f;
}
float clamp(float value, float low, float high) {
    return std::max(low, std::min(high, value));
}
bool valid_detection(const Detection &d) {
    return std::isfinite(d.left) && std::isfinite(d.top) &&
           std::isfinite(d.right) && std::isfinite(d.bottom) &&
           std::isfinite(d.score) && d.right > d.left && d.bottom > d.top &&
           d.score >= 0.0f && d.score <= 1.0f && d.class_id >= 0;
}
float overlap(const Detection &a, const Detection &b) {
    const float w = std::max(0.0f, std::min(a.right, b.right) - std::max(a.left, b.left));
    const float h = std::max(0.0f, std::min(a.bottom, b.bottom) - std::max(a.top, b.top));
    const float intersection = w * h;
    const float total = (a.right-a.left)*(a.bottom-a.top) +
                        (b.right-b.left)*(b.bottom-b.top) - intersection;
    return total > 0.0f ? intersection / total : 0.0f;
}
struct MinScoreFirst {
    bool operator()(const Detection &a, const Detection &b) const { return a.score > b.score; }
};
} // namespace

bool make_letterbox(int source_w, int source_h, int target_w, int target_h,
                    Letterbox &box, std::string &error) {
    error.clear();
    if (!valid_dimension(source_w) || !valid_dimension(source_h) ||
        !valid_dimension(target_w) || !valid_dimension(target_h) ||
        static_cast<std::size_t>(target_w) * target_h > 64u * 1024u * 1024u) {
        error = "Invalid image or model dimensions";
        return false;
    }
    const double scale = std::min(static_cast<double>(target_w)/source_w,
                                  static_cast<double>(target_h)/source_h);
    box.source_w = source_w; box.source_h = source_h;
    box.target_w = target_w; box.target_h = target_h;
    box.resized_w = std::max(1, std::min(target_w, static_cast<int>(std::lround(source_w*scale))));
    box.resized_h = std::max(1, std::min(target_h, static_cast<int>(std::lround(source_h*scale))));
    box.pad_x = (target_w-box.resized_w)/2;
    box.pad_y = (target_h-box.resized_h)/2;
    // Rounding the resized dimensions changes each scale slightly. Use the
    // actual per-axis scales again when projecting boxes back into this eye.
    box.scale_x = static_cast<float>(box.resized_w)/source_w;
    box.scale_y = static_cast<float>(box.resized_h)/source_h;
    return true;
}

bool preprocess_rgb(const unsigned char *rgb, int width, int height, int stride,
                    int target_w, int target_h, std::vector<unsigned char> &out,
                    Letterbox &box, std::string &error) {
    if (!make_letterbox(width, height, target_w, target_h, box, error)) return false;
    if (!rgb || stride < width*3) {
        error = "RGB input is missing or its row stride is too small";
        return false;
    }
    out.assign(static_cast<std::size_t>(target_w)*target_h*3, 114);
    std::vector<int> x0(box.resized_w), x1(box.resized_w);
    std::vector<unsigned> wx(box.resized_w);
    for (int x = 0; x < box.resized_w; ++x) {
        const float src_x = clamp((x+0.5f)/box.scale_x-0.5f, 0, width-1);
        x0[x] = static_cast<int>(src_x);
        x1[x] = std::min(x0[x]+1, width-1);
        wx[x] = static_cast<unsigned>(std::lround((src_x-x0[x])*2048));
    }
    for (int y = 0; y < box.resized_h; ++y) {
        const float src_y = clamp((y+0.5f)/box.scale_y-0.5f, 0, height-1);
        const int y0 = static_cast<int>(src_y), y1 = std::min(y0+1, height-1);
        const unsigned wy = static_cast<unsigned>(std::lround((src_y-y0)*2048));
        const unsigned char *row0 = rgb+static_cast<std::size_t>(y0)*stride;
        const unsigned char *row1 = rgb+static_cast<std::size_t>(y1)*stride;
        unsigned char *dst = out.data() + (static_cast<std::size_t>(y+box.pad_y)*target_w+box.pad_x)*3;
        for (int x = 0; x < box.resized_w; ++x) {
            for (int c = 0; c < 3; ++c) {
                // 11-bit bilinear weights: <=1 RGB level from the float path,
                // with no per-channel floating rounding calls in the hot loop.
                const unsigned top = row0[x0[x]*3+c]*(2048-wx[x]) + row0[x1[x]*3+c]*wx[x];
                const unsigned bottom = row1[x0[x]*3+c]*(2048-wx[x]) + row1[x1[x]*3+c]*wx[x];
                dst[x*3+c] = static_cast<unsigned char>((top*(2048-wy)+bottom*wy+(1u<<21))>>22);
            }
        }
    }
    return true;
}

bool validate_contract(const std::vector<TensorShape> &outputs, int model_w,
                       int model_h, int class_count, std::string &error) {
    error.clear();
    if (!valid_dimension(model_w) || !valid_dimension(model_h) ||
        static_cast<std::size_t>(model_w)*model_h > 64u*1024u*1024u ||
        class_count < 1 || class_count > 10000) {
        error = "Invalid model geometry or requested class count";
        return false;
    }
    if (outputs.size() != 6 && outputs.size() != 9) {
        error = "Model must expose 6 or 9 Rockchip YOLOv8 outputs (three DFL/class branches); standard [1,84,8400] export is unsupported";
        return false;
    }
    const std::size_t per_branch = outputs.size()/3;
    int previous_w = 0, previous_h = 0, previous_bins = 0;
    for (int branch = 0; branch < 3; ++branch) {
        const std::size_t idx = branch*per_branch;
        for (std::size_t j = 0; j < per_branch; ++j) {
            const TensorShape &s = outputs[idx+j];
            if (!s.nchw || s.dims.size() != 4 || s.dims[0] != 1 ||
                !valid_dimension(s.dims[2]) || !valid_dimension(s.dims[3])) {
                error = "YOLOv8 outputs must be batch-one NCHW tensors with valid spatial dimensions";
                return false;
            }
        }
        const std::vector<int> &b = outputs[idx].dims, &c = outputs[idx+1].dims;
        if (b[1] < 8 || b[1] > 256 || b[1]%4 || c[1] != class_count ||
            b[2] != c[2] || b[3] != c[3]) {
            std::ostringstream message;
            message << "YOLOv8 branch " << branch << " has invalid DFL/class shape; expected " << class_count << " classes";
            error = message.str();
            return false;
        }
        if (per_branch == 3) {
            const std::vector<int> &s = outputs[idx+2].dims;
            if (s[1] != 1 || s[2] != b[2] || s[3] != b[3]) {
                error = "YOLOv8 score-sum output must have shape [1,1,H,W] matching its branch";
                return false;
            }
        }
        const int stride = 8 << branch;
        if (model_w != b[3]*stride || model_h != b[2]*stride ||
            (branch && (previous_w != b[3]*2 || previous_h != b[2]*2 || previous_bins != b[1]/4))) {
            error = "YOLOv8 output grids must use strides 8, 16 and 32 with consistent DFL bins";
            return false;
        }
        previous_w = b[3]; previous_h = b[2]; previous_bins = b[1]/4;
    }
    return true;
}

bool dfl_expectation(const float *logits, int bins, int element_stride, float &value) {
    if (!logits || bins < 2 || bins > 64 || element_stride < 1) return false;
    float maximum = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < bins; ++i) {
        const float v = logits[static_cast<std::size_t>(i)*element_stride];
        if (!std::isfinite(v)) return false;
        maximum = std::max(maximum, v);
    }
    double denominator = 0, numerator = 0;
    for (int i = 0; i < bins; ++i) {
        const double weight = std::exp(static_cast<double>(logits[static_cast<std::size_t>(i)*element_stride]-maximum));
        denominator += weight;
        numerator += weight*i;
    }
    value = static_cast<float>(numerator/denominator);
    return std::isfinite(value);
}

void class_aware_nms(std::vector<Detection> &candidates, float threshold,
                     std::size_t max_results, std::vector<Detection> &results) {
    results.clear();
    if (!valid_threshold(threshold)) return;
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                     [](const Detection &d) { return !valid_detection(d); }), candidates.end());
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const Detection &a, const Detection &b) { return a.score > b.score; });
    for (const Detection &candidate : candidates) {
        bool suppressed = false;
        for (const Detection &kept : results) {
            if (candidate.class_id == kept.class_id && overlap(candidate, kept) > threshold) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed && results.size() < max_results) results.push_back(candidate);
        if (results.size() == max_results) break;
    }
}

bool decode_heads(const std::vector<Head> &heads, const Letterbox &box,
                  float confidence, float nms, std::vector<Detection> &results,
                  std::string &error, std::size_t max_candidates, std::size_t max_results) {
    results.clear(); error.clear();
    if (heads.size() != 3 || !valid_threshold(confidence) || !valid_threshold(nms) ||
        !max_candidates || max_candidates > 10000 || !max_results || max_results > 1000 ||
        !std::isfinite(box.scale_x) || !std::isfinite(box.scale_y) ||
        box.scale_x <= 0 || box.scale_y <= 0 || !valid_dimension(box.source_w) || !valid_dimension(box.source_h)) {
        error = "Invalid detection thresholds, head count or letterbox geometry";
        return false;
    }
    std::vector<TensorShape> shapes;
    const bool has_sum = heads[0].score_sum != NULL;
    for (const Head &h : heads) {
        if (!h.boxes || !h.classes || (h.score_sum != NULL) != has_sum ||
            h.dfl_len < 2 || h.dfl_len > 64) {
            error = "Missing or inconsistent YOLOv8 output buffers";
            return false;
        }
        shapes.push_back(TensorShape{1,h.dfl_len*4,h.grid_h,h.grid_w});
        shapes.push_back(TensorShape{1,h.class_count,h.grid_h,h.grid_w});
        if (has_sum) shapes.push_back(TensorShape{1,1,h.grid_h,h.grid_w});
    }
    if (!validate_contract(shapes, box.target_w, box.target_h, heads[0].class_count, error)) return false;
    std::priority_queue<Detection,std::vector<Detection>,MinScoreFirst> best;
    for (const Head &h : heads) {
        const int cells = h.grid_w*h.grid_h;
        const float stride_x = static_cast<float>(box.target_w)/h.grid_w;
        const float stride_y = static_cast<float>(box.target_h)/h.grid_h;
        for (int cell = 0; cell < cells; ++cell) {
            if (h.score_sum && (!std::isfinite(h.score_sum[cell]) || h.score_sum[cell] < confidence)) continue;
            float score = -1; int class_id = -1;
            for (int c = 0; c < h.class_count; ++c) {
                const float probability = h.classes[static_cast<std::size_t>(c)*cells+cell];
                // Rockchip's class heads already contain sigmoid probabilities.
                if (std::isfinite(probability) && probability >= 0 && probability <= 1.00001f && probability > score) {
                    score = std::min(1.0f, probability); class_id = c;
                }
            }
            if (class_id < 0 || score < confidence || (best.size() == max_candidates && score <= best.top().score)) continue;
            float distances[4]; bool valid = true;
            for (int side = 0; side < 4; ++side) {
                if (!dfl_expectation(h.boxes+static_cast<std::size_t>(side)*h.dfl_len*cells+cell,
                                     h.dfl_len,cells,distances[side])) { valid = false; break; }
            }
            if (!valid) continue;
            const float center_x = (cell%h.grid_w+0.5f)*stride_x;
            const float center_y = (cell/h.grid_w+0.5f)*stride_y;
            Detection d;
            d.left = clamp((center_x-distances[0]*stride_x-box.pad_x)/box.scale_x,0,box.source_w);
            d.top = clamp((center_y-distances[1]*stride_y-box.pad_y)/box.scale_y,0,box.source_h);
            d.right = clamp((center_x+distances[2]*stride_x-box.pad_x)/box.scale_x,0,box.source_w);
            d.bottom = clamp((center_y+distances[3]*stride_y-box.pad_y)/box.scale_y,0,box.source_h);
            d.score = score; d.class_id = class_id;
            if (!valid_detection(d)) continue;
            if (best.size() == max_candidates) best.pop();
            best.push(d);
        }
    }
    std::vector<Detection> candidates;
    candidates.reserve(best.size());
    while (!best.empty()) { candidates.push_back(best.top()); best.pop(); }
    class_aware_nms(candidates,nms,max_results,results);
    return true;
}

} // namespace detector_math
