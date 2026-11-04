#include "preprocess_rga.h"
#include <cassert>
#include <cstdarg>
#include <map>
#include <random>
using namespace detector_math;
struct Memory {unsigned char *data=nullptr;size_t size=0;};
static std::map<int,Memory> memory;
static int next_fd=1000,allocations=0,unmaps=0,wraps=0,checks=0,processes=0,mode=0;
static size_t assertions=0;
#define CHECK(x) do{++assertions;assert(x);}while(0)
extern "C" {
static IM_STATUS fake_header(im_api_version_t version) {
    CHECK(version==RGA_CURRENT_API_HEADER_VERSION);
    return mode==6?IM_STATUS_ERROR_VERSION:IM_STATUS_SUCCESS;
}
static rga_buffer_t fake_wrap(int fd,int width,int height,int wstride,int hstride,int format) {
    ++wraps;CHECK(memory.count(fd));CHECK(format==RK_FORMAT_RGB_888);
    CHECK(wstride>=width && hstride>=height && !(wstride%16));
    rga_buffer_t b{};b.fd=fd;b.width=width;b.height=height;b.wstride=wstride;b.hstride=hstride;b.format=format;return b;
}
static IM_STATUS fake_check(rga_buffer_t s,rga_buffer_t d,rga_buffer_t,im_rect sr,im_rect dr,im_rect,int usage) {
    ++checks;CHECK(usage==IM_SYNC);CHECK(sr.x==0 && sr.y==0 && sr.width==s.width && sr.height==s.height);
    Letterbox b;std::string error;CHECK(make_letterbox(s.width,s.height,d.width,d.height,b,error));
    CHECK(dr.x==b.pad_x && dr.y==b.pad_y && dr.width==b.resized_w && dr.height==b.resized_h);
    CHECK(memory[s.fd].size==static_cast<size_t>(s.wstride)*s.hstride*3);
    CHECK(memory[d.fd].size==static_cast<size_t>(d.wstride)*d.hstride*3);
    return mode==3?IM_STATUS_NOT_SUPPORTED:IM_STATUS_NOERROR;
}
static IM_STATUS fake_process(rga_buffer_t s,rga_buffer_t d,rga_buffer_t,im_rect,im_rect dr,im_rect,int usage) {
    ++processes;CHECK(usage==IM_SYNC);
    if(mode==4)return IM_STATUS_FAILED;
    std::vector<unsigned char> out;Letterbox b;std::string error;
    CHECK(preprocess_rgb(memory[s.fd].data,s.width,s.height,s.wstride*3,d.width,d.height,out,b,error));
    for(int y=0;y<d.height;++y) {
        const auto *row=memory[d.fd].data+static_cast<size_t>(y)*d.wstride*3;
        for(int x=0;x<d.width*3;++x)CHECK(row[x]==114);
    }
    for(int y=0;y<dr.height;++y)
        std::memcpy(memory[d.fd].data+static_cast<size_t>(y+dr.y)*d.wstride*3+dr.x*3,
                    out.data()+static_cast<size_t>(y+dr.y)*d.width*3+dr.x*3,dr.width*3);
    if(mode==5)memory[d.fd].data[dr.y*d.wstride*3+dr.x*3]^=255;
    return IM_STATUS_SUCCESS;
}
void *__wrap_dlopen(const char *,int){return mode==1?nullptr:reinterpret_cast<void *>(1);}
int __wrap_dlclose(void *){return 0;}
void *__wrap_dlsym(void *,const char *name) {
    if(!std::strcmp(name,"wrapbuffer_fd_t"))return reinterpret_cast<void *>(&fake_wrap);
    if(!std::strcmp(name,"imcheck_t"))return reinterpret_cast<void *>(&fake_check);
    if(!std::strcmp(name,"improcess"))return reinterpret_cast<void *>(&fake_process);
    if(!std::strcmp(name,"_Z13imcheckHeaderj"))return reinterpret_cast<void *>(&fake_header);
    return nullptr;
}
int __wrap_open(const char *path,int,...) {CHECK(!std::strncmp(path,"/dev/dma_heap/",14));return mode==2?-1:900;}
int __wrap_close(int fd) {if(fd>=1000)CHECK(memory.erase(fd)==1);else CHECK(fd==900);return 0;}
int __wrap_ioctl(int fd,unsigned long request,...) {
    CHECK(fd==900);va_list args;va_start(args,request);auto *a=va_arg(args,void *);va_end(args);
    struct A{uint64_t len;uint32_t fd,fd_flags;uint64_t heap_flags;};auto *allocation=static_cast<A *>(a);
    CHECK(allocation->len>0 && allocation->heap_flags==0);
    allocation->fd=next_fd++;memory[allocation->fd].size=allocation->len;++allocations;return 0;
}
void *__wrap_mmap(void *,size_t length,int,int,int fd,off_t offset) {
    CHECK(memory.count(fd) && memory[fd].size==length && offset==0);
    memory[fd].data=static_cast<unsigned char *>(std::malloc(length));CHECK(memory[fd].data);
    return memory[fd].data;
}
int __wrap_munmap(void *data,size_t bytes) {
    bool found=false;for(const auto &m:memory)if(m.second.data==data){CHECK(m.second.size==bytes);found=true;break;}
    CHECK(found);std::free(data);++unmaps;return 0;
}
}
static void run(int fail_mode) {
    mode=fail_mode;const int old_allocations=allocations;
    std::mt19937 random(34);const int w=960,h=1080,tw=640,th=640,stride=1920*3;
    const size_t row=tw*3+31;
    std::vector<unsigned char> rgb(static_cast<size_t>(stride)*h),out(row*th+17,0xa5),reference;
    for(auto &x:rgb)x=random()%256;
    const auto *right=rgb.data()+w*3;Letterbox a,b;std::string error;PreprocessTiming timing;
    CHECK(preprocess_rgb(right,w,h,stride,tw,th,reference,a,error));
    {
        RgaPreprocessor rga;const bool ok=rga.run(right,w,h,stride,tw,th,out.data(),row*th,row,b,timing);
        CHECK(ok==(fail_mode==0));
        if(ok) {
            CHECK(timing.backend==1 && timing.copy_ms>0 && timing.rga_ms>0 && timing.validate_ms>0);
            CHECK(allocations-old_allocations==2);
            for(int y=0;y<th;++y) {
                CHECK(!std::memcmp(out.data()+row*y,reference.data()+tw*3*y,tw*3));
                for(size_t x=tw*3;x<row;++x)CHECK(out[row*y+x]==0xa5);
            }
            for(size_t x=row*th;x<out.size();++x)CHECK(out[x]==0xa5);
            // Same geometry reuses the two DMA allocations and skips the CPU
            // validation after its initial successful golden comparison.
            PreprocessTiming repeat;CHECK(rga.run(right,w,h,stride,tw,th,out.data(),row*th,row,b,repeat));
            CHECK(repeat.validate_ms==0 && allocations-old_allocations==2);
        } else {
            RgbPreprocessor cpu;CHECK(cpu.run(right,w,h,stride,tw,th,out.data(),row*th,row,b,error));
            for(int y=0;y<th;++y)CHECK(!std::memcmp(out.data()+row*y,reference.data()+tw*3*y,tw*3));
            const int old_processes=processes;
            CHECK(!rga.run(right,w,h,stride,tw,th,out.data(),row*th,row,b,timing));CHECK(processes==old_processes);
        }
    }
    CHECK(memory.empty());CHECK(allocations==unmaps);
}
int main() {
    for(int mode=0;mode<=6;++mode)run(mode);
    std::printf("PASS native RGA call path: %zu assertions; real C ABI argument/stride/ROI checks; DMA reuse/teardown; missing library/heap, geometry/process failure, corrupt output all fall back (mock driver, not hardware)\n",assertions);
}
