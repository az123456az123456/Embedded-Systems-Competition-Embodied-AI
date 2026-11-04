#ifndef PERCEPTION_CAMERA_H
#define PERCEPTION_CAMERA_H
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
struct CameraLayout {
    unsigned width=0,height=0;
    uint32_t fourcc=0;
    size_t stride=0,chroma_stride=0;
    bool full_range=false,bt709=false;
};
bool camera_to_rgb(const CameraLayout &,const uint8_t *plane0,size_t bytes0,
                   const uint8_t *plane1,size_t bytes1,std::vector<uint8_t> &rgb,std::string &error);
// NV12/NV21 auto backend. PERCEPTION_CAMERA_SCALAR=1 selects the reference for A/B.
const char *camera_color_backend();
struct CameraTiming {
    double wait_ms=0, dequeue_ms=0, convert_ms=0, requeue_ms=0;
    // Trusted monotonic V4L2 frame time, or dequeue completion before conversion.
    uint64_t source_ms=0;
};
class Camera {
public:
    Camera(); ~Camera();
    bool open(const std::string &device,unsigned width,unsigned height,std::string &error);
    // 1=new frame, 0=timeout/interrupted, -1=error. Copies and requeues before inference.
    int read(std::vector<uint8_t> &rgb,std::string &error,CameraTiming *timing=nullptr);
    const CameraLayout &layout() const;
    std::string description() const;
private:
    struct Impl; Impl *p;
    Camera(const Camera &)=delete; Camera &operator=(const Camera &)=delete;
};
#endif
