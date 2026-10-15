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
class Camera {
public:
    Camera(); ~Camera();
    bool open(const std::string &device,unsigned width,unsigned height,std::string &error);
    // 1=new frame, 0=timeout/interrupted, -1=error. Copies and requeues before inference.
    int read(std::vector<uint8_t> &rgb,std::string &error);
    const CameraLayout &layout() const;
    std::string description() const;
private:
    struct Impl; Impl *p;
    Camera(const Camera &)=delete; Camera &operator=(const Camera &)=delete;
};
#endif
