#include "detector_math.h"
#include <algorithm>
#include <vector>

// Use the shipping decoder for both ONNX reference and actual RKNN output.
extern "C" int decode_reference(const float *const *outputs, float *result,
                                 int capacity, float confidence, float nms) {
    if (!outputs || !result || capacity < 128) return -1;
    std::vector<detector_math::Head> heads;
    const int sides[] = {80, 40, 20};
    for (int i=0; i<3; ++i)
        heads.push_back({outputs[3*i], outputs[3*i+1], outputs[3*i+2],
                         sides[i], sides[i], 16, 9});
    detector_math::Letterbox box{};
    std::string error;
    std::vector<Detection> detections;
    if (!detector_math::make_letterbox(640,640,640,640,box,error) ||
        !detector_math::decode_heads(heads,box,confidence,nms,detections,error)) return -2;
    for (std::size_t i=0; i<detections.size(); ++i) {
        const auto &d=detections[i];
        const float values[]={d.left,d.top,d.right,d.bottom,d.score,float(d.class_id)};
        std::copy(values,values+6,result+6*i);
    }
    return static_cast<int>(detections.size());
}
