#ifndef PERCEPTION_PREPROCESS_RGA_H
#define PERCEPTION_PREPROCESS_RGA_H

#include "preprocess_plan.h"
#include <climits>
#include <cstdio>
#include <memory>

#ifdef HAVE_RGA
#include "im2d.h"
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace detector_math {
// The camera currently owns packed RGB vectors, not shareable DMA buffers.
// This implementation deliberately stages each ROI in a reused DMA buffer and
// copies the completed tensor into existing RKNN input memory. It is NOT an
// end-to-end zero-copy path. No CPU cache alias is shared with RGA writes.
class RgaEngine {
public:
    virtual ~RgaEngine()=default;
    virtual bool process(const unsigned char *rgb,int stride,const Letterbox &box,
                         unsigned char *dst,size_t dst_stride,
                         PreprocessTiming &timing,std::string &reason)=0;
};

#ifdef HAVE_RGA
class NativeRgaEngine final:public RgaEngine {
    struct Allocation { uint64_t len;uint32_t fd,fd_flags;uint64_t heap_flags; };
    struct Buffer {
        int fd=-1;unsigned char *data=nullptr;size_t bytes=0;
        ~Buffer(){clear();}
        void clear(){if(data)munmap(data,bytes);if(fd>=0)::close(fd);fd=-1;data=nullptr;bytes=0;}
        bool reserve(size_t size,std::string &reason) {
            if(bytes==size && data)return true;
            clear();
            // Use uncached heaps only. A cached-VA fallback would require a
            // separate CPU/device ownership protocol; CPU preprocessing is the
            // safe compatibility fallback when these heaps are unavailable.
            for(const char *path:{"/dev/dma_heap/system-uncached-dma32",
                                  "/dev/dma_heap/system-dma32-uncached",
                                  "/dev/dma_heap/system-uncached"}) {
                int heap=::open(path,O_RDWR|O_CLOEXEC);if(heap<0)continue;
                Allocation allocation{};allocation.len=size;allocation.fd_flags=O_RDWR|O_CLOEXEC;
                const int rc=ioctl(heap,_IOWR('H',0,Allocation),&allocation);::close(heap);
                if(rc<0)continue;
                void *address=mmap(nullptr,size,PROT_READ|PROT_WRITE,MAP_SHARED,allocation.fd,0);
                if(address==MAP_FAILED){::close(allocation.fd);continue;}
                fd=static_cast<int>(allocation.fd);data=static_cast<unsigned char *>(address);bytes=size;return true;
            }
            reason="RGA uncached DMA heap unavailable; cached CPU path retained";return false;
        }
    } source_,output_;
    void *library_=nullptr;
    using Wrap=rga_buffer_t (*)(int,int,int,int,int,int);
    using Check=IM_STATUS (*)(rga_buffer_t,rga_buffer_t,rga_buffer_t,im_rect,im_rect,im_rect,int);
    using Process=IM_STATUS (*)(rga_buffer_t,rga_buffer_t,rga_buffer_t,im_rect,im_rect,im_rect,int);
    using HeaderCheck=IM_STATUS (*)(im_api_version_t);
    Wrap wrap_=nullptr;Check check_=nullptr;Process process_=nullptr;
    HeaderCheck header_check_=nullptr;bool abi_checked_=false;
    bool load_attempted_=false;
public:
    ~NativeRgaEngine() override {if(library_)dlclose(library_);}
    bool process(const unsigned char *rgb,int stride,const Letterbox &box,
                 unsigned char *dst,size_t dst_stride,PreprocessTiming &timing,
                 std::string &reason) override {
        auto start=StageClock::now();
        if(!load_attempted_) {
            load_attempted_=true;library_=dlopen("librga.so",RTLD_NOW|RTLD_LOCAL);
            if(!library_)library_=dlopen("librga.so.2",RTLD_NOW|RTLD_LOCAL);
            if(library_) {
                wrap_=reinterpret_cast<Wrap>(dlsym(library_,"wrapbuffer_fd_t"));
                check_=reinterpret_cast<Check>(dlsym(library_,"imcheck_t"));
                process_=reinterpret_cast<Process>(dlsym(library_,"improcess"));
                // Official API is a C++ scalar-only function. Validate the
                // header/library contract before any rga_buffer_t by-value
                // call; missing older APIs safely retain the CPU path.
                header_check_=reinterpret_cast<HeaderCheck>(dlsym(library_,"_Z13imcheckHeaderj"));
            }
        }
        if(!wrap_ || !check_ || !process_){reason="RGA IM2D C API unavailable; cached CPU path retained";return false;}
        if(!abi_checked_) {
            if(!header_check_ || header_check_(RGA_CURRENT_API_HEADER_VERSION)!=IM_STATUS_SUCCESS) {
                reason="RGA header/library version check unavailable or incompatible; cached CPU path retained";return false;
            }
            abi_checked_=true;
        }
        const int source_stride=(box.source_w+15)&~15;
        const int output_stride=(box.target_w+15)&~15;
        const size_t source_bytes=static_cast<size_t>(source_stride)*box.source_h*3;
        const size_t output_bytes=static_cast<size_t>(output_stride)*box.target_h*3;
        if(source_bytes>64u*1024u*1024u || output_bytes>64u*1024u*1024u) {
            reason="RGA staging extent exceeds 64 MiB; cached CPU path retained";return false;
        }
        if(!source_.reserve(source_bytes,reason) || !output_.reserve(output_bytes,reason))return false;
        timing.plan_ms+=stage_elapsed(start);start=StageClock::now();
        for(int y=0;y<box.source_h;++y)
            std::memcpy(source_.data+static_cast<size_t>(y)*source_stride*3,
                        rgb+static_cast<size_t>(y)*stride,box.source_w*3);
        timing.copy_ms+=stage_elapsed(start);start=StageClock::now();
        // Fill the whole output, including hardware stride. Only valid RGB
        // bytes are copied to the RKNN tensor after synchronous completion.
        std::memset(output_.data,114,output_bytes);
        timing.fill_ms+=stage_elapsed(start);start=StageClock::now();
        const auto src=wrap_(source_.fd,box.source_w,box.source_h,source_stride,box.source_h,RK_FORMAT_RGB_888);
        const auto out=wrap_(output_.fd,box.target_w,box.target_h,output_stride,box.target_h,RK_FORMAT_RGB_888);
        const rga_buffer_t pattern{};const im_rect empty{};
        const im_rect sr{0,0,box.source_w,box.source_h};
        const im_rect dr{box.pad_x,box.pad_y,box.resized_w,box.resized_h};
        const int usage=IM_SYNC;
        const IM_STATUS checked=check_(src,out,pattern,sr,dr,empty,usage);
        if(checked!=IM_STATUS_NOERROR) {
            timing.rga_ms+=stage_elapsed(start);
            reason="RGA rejected exact RGB letterbox geometry ("+std::to_string(checked)+"); cached CPU path retained";return false;
        }
        const IM_STATUS completed=process_(src,out,pattern,sr,dr,empty,usage);
        timing.rga_ms+=stage_elapsed(start);
        if(completed!=IM_STATUS_SUCCESS) {
            reason="RGA synchronous resize failed ("+std::to_string(completed)+"); cached CPU path retained";return false;
        }
        start=StageClock::now();
        for(int y=0;y<box.target_h;++y)
            std::memcpy(dst+static_cast<size_t>(y)*dst_stride,
                        output_.data+static_cast<size_t>(y)*output_stride*3,box.target_w*3);
        timing.copy_ms+=stage_elapsed(start);timing.backend=1;return true;
    }
};
#endif

class RgaPreprocessor {
    std::unique_ptr<RgaEngine> engine_;
    Letterbox validated_geometry_{};
    bool disabled_=false;
    std::string reason_="RGA not attempted";
public:
    RgaPreprocessor() {
#ifdef HAVE_RGA
        engine_.reset(new NativeRgaEngine);
#endif
    }
    explicit RgaPreprocessor(std::unique_ptr<RgaEngine> engine):engine_(std::move(engine)){}
    const std::string &reason()const{return reason_;}
    bool run(const unsigned char *rgb,int width,int height,int stride,int tw,int th,
             unsigned char *dst,size_t bytes,size_t dst_stride,Letterbox &box,
             PreprocessTiming &timing) {
        if(disabled_)return false;
        if(!engine_){disabled_=true;reason_="RGA support not compiled; cached CPU path retained";return false;}
        if(!make_letterbox(width,height,tw,th,box,reason_) || !rgb || !dst ||
           stride<width*3 || dst_stride<static_cast<size_t>(tw)*3 ||
           dst_stride>64u*1024u*1024u || bytes<dst_stride*static_cast<size_t>(th-1)+tw*3) {
            disabled_=true;reason_="RGA received invalid RGB geometry/buffer";return false;
        }
        // A deterministic textured probe validates each geometry before any
        // real frame. A flat first camera frame must not accidentally approve
        // different half-pixel sampling rules on another RGA generation.
        if(validated_geometry_.source_w!=width || validated_geometry_.source_h!=height ||
           validated_geometry_.target_w!=tw || validated_geometry_.target_h!=th) {
            if(static_cast<uint64_t>(width)*height*3>64u*1024u*1024u) {
                disabled_=true;reason_="RGA validation extent exceeds 64 MiB";return false;
            }
            auto start=StageClock::now();
            std::vector<unsigned char> probe(static_cast<size_t>(width)*height*3),reference;
            for(int y=0;y<height;++y)for(int x=0;x<width;++x) {
                const uint32_t pattern=(static_cast<uint32_t>(x)*2654435761u)^
                                       (static_cast<uint32_t>(y)*2246822519u);
                auto *pixel=probe.data()+(static_cast<size_t>(y)*width+x)*3;
                pixel[0]=static_cast<unsigned char>(pattern);
                pixel[1]=static_cast<unsigned char>(pattern>>11);
                pixel[2]=static_cast<unsigned char>(pattern>>21);
            }
            Letterbox checked{};std::string error;
            if(!preprocess_rgb(probe.data(),width,height,width*3,tw,th,reference,checked,error)) {
                disabled_=true;reason_="RGA CPU validation failed: "+error;return false;
            }
            timing.validate_ms+=stage_elapsed(start);
            if(!engine_->process(probe.data(),width*3,box,dst,dst_stride,timing,reason_)) {
                disabled_=true;return false;
            }
            start=StageClock::now();
            unsigned max_error=0;
            for(int y=0;y<th;++y)for(int x=0;x<tw*3;++x)
                max_error=std::max(max_error,static_cast<unsigned>(std::abs(
                    int(dst[static_cast<size_t>(y)*dst_stride+x])-int(reference[static_cast<size_t>(y)*tw*3+x]))));
            timing.validate_ms+=stage_elapsed(start);
            if(max_error>2) {
                disabled_=true;reason_="RGA/CPU interpolation difference exceeds 2 U8 levels; cached CPU path retained";return false;
            }
            validated_geometry_=box;
        }
        if(!engine_->process(rgb,stride,box,dst,dst_stride,timing,reason_)) {
            disabled_=true;return false;
        }
        reason_="RGB bilinear DMA staging; two explicit copies; normalization remains in RKNN";
        return true;
    }
};
}
#endif
