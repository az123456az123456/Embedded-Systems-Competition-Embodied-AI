#include "camera.h"
#include "camera_color_math.h"
#include "camera_timestamp.h"
#include "stage_clock.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

static int ctl(int fd,unsigned long request,void *value) {
    int r; do { r=ioctl(fd,request,value); } while(r<0 && errno==EINTR); return r;
}
static bool fail(std::string &error,const std::string &message) { error=message;return false; }
static bool rows_fit(size_t bytes,size_t stride,size_t rows,size_t visible) {
    return rows && stride>=visible && bytes>=visible && (rows-1)<=(bytes-visible)/stride;
}
static bool force_scalar() {
    const char *value=std::getenv("PERCEPTION_CAMERA_SCALAR");
    return value && std::strcmp(value,"1")==0;
}
const char *camera_color_backend() {
    if(force_scalar())return "scalar-reference";
#if defined(__aarch64__) && defined(__ARM_NEON)
    return "neon16-paired";
#else
    return "scalar-paired";
#endif
}
#if defined(__aarch64__) && defined(__ARM_NEON)
struct Chroma16 {int32x4_t r[4],g[4],b[4];};
static Chroma16 chroma16(const uint8_t *src,bool nv21,const camera_color::Matrix &m) {
    // Exactly sixteen bytes (eight UV pairs), never beyond the visible row.
    const auto uv=vld2_u8(src);
    const auto u=vreinterpretq_s16_u16(vsubl_u8(uv.val[nv21?1:0],vdup_n_u8(128)));
    const auto v=vreinterpretq_s16_u16(vsubl_u8(uv.val[nv21?0:1],vdup_n_u8(128)));
    const int32x4_t u0=vmovl_s16(vget_low_s16(u)),u1=vmovl_s16(vget_high_s16(u));
    const int32x4_t v0=vmovl_s16(vget_low_s16(v)),v1=vmovl_s16(vget_high_s16(v));
    const int32x4_t us[4]={vzip1q_s32(u0,u0),vzip2q_s32(u0,u0),vzip1q_s32(u1,u1),vzip2q_s32(u1,u1)};
    const int32x4_t vs[4]={vzip1q_s32(v0,v0),vzip2q_s32(v0,v0),vzip1q_s32(v1,v1),vzip2q_s32(v1,v1)};
    Chroma16 out;
    for(int i=0;i<4;i++) {
        out.r[i]=vmulq_n_s32(vs[i],m.crv);
        out.g[i]=vmlaq_n_s32(vmulq_n_s32(us[i],m.cgu),vs[i],m.cgv);
        out.b[i]=vmulq_n_s32(us[i],m.cbu);
    }
    return out;
}
static uint8x16_t channel16(const int32x4_t (&y)[4],const int32x4_t (&chroma)[4]) {
    const auto a=vcombine_u16(vqrshrun_n_s32(vaddq_s32(y[0],chroma[0]),16),
                             vqrshrun_n_s32(vaddq_s32(y[1],chroma[1]),16));
    const auto b=vcombine_u16(vqrshrun_n_s32(vaddq_s32(y[2],chroma[2]),16),
                             vqrshrun_n_s32(vaddq_s32(y[3],chroma[3]),16));
    return vcombine_u8(vqmovn_u16(a),vqmovn_u16(b));
}
static inline __attribute__((always_inline)) void row16(const uint8_t *src,uint8_t *dst,const Chroma16 &chroma,const camera_color::Matrix &m) {
    const auto bytes=vld1q_u8(src);
    const auto lo=vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(bytes),vdup_n_u8(m.y_offset)));
    const auto hi=vreinterpretq_s16_u16(vsubl_u8(vget_high_u8(bytes),vdup_n_u8(m.y_offset)));
    const int32x4_t y[4]={vmulq_n_s32(vmovl_s16(vget_low_s16(lo)),m.cy),
                         vmulq_n_s32(vmovl_s16(vget_high_s16(lo)),m.cy),
                         vmulq_n_s32(vmovl_s16(vget_low_s16(hi)),m.cy),
                         vmulq_n_s32(vmovl_s16(vget_high_s16(hi)),m.cy)};
    const uint8x16x3_t rgb={{channel16(y,chroma.r),channel16(y,chroma.g),channel16(y,chroma.b)}};
    vst3q_u8(dst,rgb);
}
#endif
static void nv_paired(const CameraLayout &f,const uint8_t *a,const uint8_t *b,size_t uv_stride,
                      bool nv21,const camera_color::Matrix &m,std::vector<uint8_t> &rgb) {
    for(unsigned y=0;y<f.height;y+=2) {
        const auto *row0=a+y*f.stride,*row1=row0+f.stride,*uv=b+(y/2)*uv_stride;
        auto *dst0=rgb.data()+static_cast<size_t>(y)*f.width*3,*dst1=dst0+f.width*3;
        unsigned x=0;
#if defined(__aarch64__) && defined(__ARM_NEON)
        for(;x+16<=f.width;x+=16) {
            const auto chroma=chroma16(uv+x,nv21,m);
            row16(row0+x,dst0+x*3,chroma,m);
            row16(row1+x,dst1+x*3,chroma,m);
        }
#endif
        for(;x<f.width;x+=2) {
            const int u=uv[x+(nv21?1:0)],v=uv[x+(nv21?0:1)];
            // Reuse one chroma contribution for the complete 2x2 YUV420 block.
            const int r=m.red_v[v],g=m.green_u[u]+m.green_v[v],bb=m.blue_u[u];
            const int ys[4]={m.luma[row0[x]],m.luma[row0[x+1]],m.luma[row1[x]],m.luma[row1[x+1]]};
            uint8_t *dst[4]={dst0+x*3,dst0+(x+1)*3,dst1+x*3,dst1+(x+1)*3};
            for(int i=0;i<4;i++) {
                dst[i][0]=camera_color::channel(ys[i]+r);
                dst[i][1]=camera_color::channel(ys[i]+g);
                dst[i][2]=camera_color::channel(ys[i]+bb);
            }
        }
    }
}
bool camera_to_rgb(const CameraLayout &f,const uint8_t *a,size_t na,const uint8_t *b,size_t nb,
                   std::vector<uint8_t> &rgb,std::string &error) {
    if(!a || !f.width || !f.height || f.width>4096 || f.height>2160)
        return fail(error,"Invalid camera dimensions");
    const bool nv12=f.fourcc==V4L2_PIX_FMT_NV12 || f.fourcc==V4L2_PIX_FMT_NV12M;
    const bool nv21=f.fourcc==V4L2_PIX_FMT_NV21 || f.fourcc==V4L2_PIX_FMT_NV21M;
    const bool yuyv=f.fourcc==V4L2_PIX_FMT_YUYV;
    const bool rgb24=f.fourcc==V4L2_PIX_FMT_RGB24,bgr=f.fourcc==V4L2_PIX_FMT_BGR24;
    size_t row=(rgb24||bgr)?f.width*3:yuyv?f.width*2:f.width;
    if(!(nv12||nv21||yuyv||rgb24||bgr))return fail(error,"Unsupported camera pixel format");
    if(!rows_fit(na,f.stride,f.height,row))return fail(error,"Truncated camera plane / invalid stride");
    size_t uv_stride=f.chroma_stride?f.chroma_stride:f.stride;
    if(nv12||nv21) {
        if((f.width&1)||(f.height&1)||uv_stride<f.width)return fail(error,"NV12/NV21 needs even dimensions and valid UV stride");
        if(!b) {
            if(f.stride>na/f.height)return fail(error,"Truncated NV12 luma plane");
            size_t offset=f.stride*f.height;
            if(na<offset)return fail(error,"Truncated NV12 luma plane");
            b=a+offset;nb=na-offset;
        }
        if(!rows_fit(nb,uv_stride,f.height/2,f.width))return fail(error,"Truncated camera chroma plane");
    }
    if(yuyv && (f.width&1))return fail(error,"YUYV needs even width");
    rgb.resize(static_cast<size_t>(f.width)*f.height*3);
    // V4L2 quantization + matrix metadata are applied to YUV input only.
    const auto &m=camera_color::matrix(f.bt709,f.full_range);
    if((nv12||nv21) && !force_scalar()) {
        nv_paired(f,a,b,uv_stride,nv21,m,rgb);return true;
    }
    for(unsigned y=0;y<f.height;y++) {
        const uint8_t *src=a+y*f.stride;uint8_t *dst=rgb.data()+static_cast<size_t>(y)*f.width*3;
        if(rgb24) { std::memcpy(dst,src,f.width*3);continue; }
        for(unsigned x=0;x<f.width;x++,dst+=3) {
            if(bgr) {dst[0]=src[x*3+2];dst[1]=src[x*3+1];dst[2]=src[x*3];continue;}
            int yy,u,v;
            if(yuyv) {size_t k=(x/2)*4;yy=src[k+(x%2)*2];u=src[k+1];v=src[k+3];}
            else {const uint8_t *uv=b+(y/2)*uv_stride+(x/2)*2;yy=src[x];u=uv[nv21?1:0];v=uv[nv21?0:1];}
            camera_color::reference_pixel(m,yy,u,v,dst);
        }
    }
    return true;
}
struct Camera::Impl {
    struct Plane {void *data=MAP_FAILED;size_t length=0;};
    struct Buffer {std::vector<Plane> planes;};
    int fd=-1;v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;unsigned planes=1;
    bool streaming=false,multi=false;CameraLayout layout;std::vector<Buffer> buffers;
    ~Impl() {
        if(fd>=0 && streaming)ctl(fd,VIDIOC_STREAMOFF,&type);
        for(auto &buffer:buffers)for(auto &plane:buffer.planes)
            if(plane.data!=MAP_FAILED)munmap(plane.data,plane.length);
        if(fd>=0)close(fd);
    }
    void prepare(v4l2_buffer &b,v4l2_plane (&pl)[VIDEO_MAX_PLANES]) {
        std::memset(&b,0,sizeof(b));std::memset(pl,0,sizeof(pl));
        b.type=type;b.memory=V4L2_MEMORY_MMAP;
        if(multi){b.length=planes;b.m.planes=pl;}
    }
};
Camera::Camera():p(new Impl){}
Camera::~Camera(){delete p;}
const CameraLayout &Camera::layout()const{return p->layout;}
std::string Camera::description()const {
    std::ostringstream s;uint32_t c=p->layout.fourcc;
    s<<p->layout.width<<"x"<<p->layout.height<<" "
      <<char(c)<<char(c>>8)<<char(c>>16)<<char(c>>24);
    return s.str();
}
bool Camera::open(const std::string &device,unsigned w,unsigned h,std::string &error) {
    error.clear();
    delete p;p=new Impl;
    p->fd=::open(device.c_str(),O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if(p->fd<0)return fail(error,"Camera open: "+std::string(std::strerror(errno)));
    v4l2_capability caps={};
    if(ctl(p->fd,VIDIOC_QUERYCAP,&caps)<0)return fail(error,"Not a V4L2 camera");
    uint32_t c=(caps.capabilities&V4L2_CAP_DEVICE_CAPS)?caps.device_caps:caps.capabilities;
    if(!(c&V4L2_CAP_STREAMING))return fail(error,"Camera needs V4L2 streaming");
    p->multi=!(c&V4L2_CAP_VIDEO_CAPTURE) && (c&V4L2_CAP_VIDEO_CAPTURE_MPLANE);
    if(!(c&(V4L2_CAP_VIDEO_CAPTURE|V4L2_CAP_VIDEO_CAPTURE_MPLANE)))return fail(error,"Select a capture video node");
    p->type=p->multi?V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint32_t candidates[]={V4L2_PIX_FMT_NV12,V4L2_PIX_FMT_NV12M,V4L2_PIX_FMT_NV21,V4L2_PIX_FMT_NV21M,V4L2_PIX_FMT_YUYV,V4L2_PIX_FMT_RGB24,V4L2_PIX_FMT_BGR24};
    bool found=false;v4l2_format fmt={};
    for(uint32_t candidate:candidates) {
        if(!p->multi && (candidate==V4L2_PIX_FMT_NV12M||candidate==V4L2_PIX_FMT_NV21M))continue;
        fmt={};fmt.type=p->type;
        if(p->multi){fmt.fmt.pix_mp.width=w;fmt.fmt.pix_mp.height=h;fmt.fmt.pix_mp.pixelformat=candidate;fmt.fmt.pix_mp.field=V4L2_FIELD_NONE;}
        else{fmt.fmt.pix.width=w;fmt.fmt.pix.height=h;fmt.fmt.pix.pixelformat=candidate;fmt.fmt.pix.field=V4L2_FIELD_NONE;}
        if(ctl(p->fd,VIDIOC_S_FMT,&fmt)<0)continue;
        uint32_t returned=p->multi?fmt.fmt.pix_mp.pixelformat:fmt.fmt.pix.pixelformat;
        if(std::find(std::begin(candidates),std::end(candidates),returned)!=std::end(candidates)){found=true;break;}
    }
    if(!found)return fail(error,"Camera needs linear NV12/NV21/YUYV/RGB24 (no tiled/MJPEG)");
    unsigned colorspace,encoding,quantization;
    if(p->multi) {
        const auto &f=fmt.fmt.pix_mp;p->planes=f.num_planes;
        p->layout.width=f.width;p->layout.height=f.height;p->layout.fourcc=f.pixelformat;
        p->layout.stride=f.plane_fmt[0].bytesperline;
        p->layout.chroma_stride=p->planes>1?f.plane_fmt[1].bytesperline:p->layout.stride;
        colorspace=f.colorspace;encoding=f.ycbcr_enc;quantization=f.quantization;
    }else {
        const auto &f=fmt.fmt.pix;p->layout.width=f.width;p->layout.height=f.height;
        p->layout.fourcc=f.pixelformat;p->layout.stride=f.bytesperline;p->layout.chroma_stride=f.bytesperline;
        colorspace=f.colorspace;encoding=f.ycbcr_enc;quantization=f.quantization;
    }
    if(p->planes<1||p->planes>2)return fail(error,"Unsupported camera plane count");
    if(!p->layout.width||!p->layout.height||p->layout.width>4096||p->layout.height>2160)return fail(error,"Camera dimensions outside supported range");
    bool rgb=p->layout.fourcc==V4L2_PIX_FMT_RGB24||p->layout.fourcc==V4L2_PIX_FMT_BGR24;
    if(colorspace==V4L2_COLORSPACE_DEFAULT)
        colorspace=V4L2_MAP_COLORSPACE_DEFAULT(p->layout.width<=720 && p->layout.height<=576,
                                              p->layout.width>=1280 || p->layout.height>=720);
    if(encoding==V4L2_YCBCR_ENC_DEFAULT)encoding=V4L2_MAP_YCBCR_ENC_DEFAULT(colorspace);
    if(!rgb && encoding!=V4L2_YCBCR_ENC_601 && encoding!=V4L2_YCBCR_ENC_709 && encoding!=V4L2_YCBCR_ENC_XV601 && encoding!=V4L2_YCBCR_ENC_XV709)
        return fail(error,"Unsupported camera YUV color matrix");
    p->layout.bt709=encoding==V4L2_YCBCR_ENC_709||encoding==V4L2_YCBCR_ENC_XV709;
    if(quantization==V4L2_QUANTIZATION_DEFAULT)quantization=V4L2_MAP_QUANTIZATION_DEFAULT(rgb,colorspace,encoding);
    p->layout.full_range=quantization==V4L2_QUANTIZATION_FULL_RANGE;
    v4l2_requestbuffers req={};req.count=4;req.type=p->type;req.memory=V4L2_MEMORY_MMAP;
    if(ctl(p->fd,VIDIOC_REQBUFS,&req)<0||req.count<2||req.count>16)return fail(error,"Camera buffer allocation failed");
    p->buffers.resize(req.count);
    for(unsigned i=0;i<req.count;i++) {
        v4l2_buffer b;v4l2_plane pl[VIDEO_MAX_PLANES];p->prepare(b,pl);b.index=i;
        if(ctl(p->fd,VIDIOC_QUERYBUF,&b)<0)return fail(error,"Camera QUERYBUF failed");
        p->buffers[i].planes.resize(p->planes);
        for(unsigned k=0;k<p->planes;k++) {
            auto &plane=p->buffers[i].planes[k];plane.length=p->multi?pl[k].length:b.length;
            auto offset=p->multi?pl[k].m.mem_offset:b.m.offset;
            plane.data=mmap(nullptr,plane.length,PROT_READ|PROT_WRITE,MAP_SHARED,p->fd,offset);
            if(plane.data==MAP_FAILED)return fail(error,"Camera mmap failed");
        }
        if(ctl(p->fd,VIDIOC_QBUF,&b)<0)return fail(error,"Camera QBUF failed");
    }
    if(ctl(p->fd,VIDIOC_STREAMON,&p->type)<0)return fail(error,"Camera STREAMON failed");
    p->streaming=true;return true;
}
int Camera::read(std::vector<uint8_t> &rgb,std::string &error,CameraTiming *timing) {
    error.clear();
    CameraTiming measured;if(!timing)timing=&measured;*timing={};
    auto tick=StageClock::now();
    pollfd fd={p->fd,POLLIN,0};int rc=poll(&fd,1,250);
    timing->wait_ms=stage_elapsed(tick);tick=StageClock::now();
    if(rc==0 || (rc<0 && errno==EINTR))return 0;
    if(rc<0 || (fd.revents&(POLLERR|POLLHUP|POLLNVAL))){error="Camera polling failed";return -1;}
    v4l2_buffer latest={};v4l2_plane latest_pl[VIDEO_MAX_PLANES]={};bool got=false;
    // Drain only the finite queued buffers. Inference always gets the newest available frame.
    for(size_t i=0;i<p->buffers.size();i++) {
        v4l2_buffer b;v4l2_plane pl[VIDEO_MAX_PLANES];p->prepare(b,pl);
        if(ctl(p->fd,VIDIOC_DQBUF,&b)<0) {
            if(errno==EAGAIN)break;
            if(got)ctl(p->fd,VIDIOC_QBUF,&latest);
            error="Camera DQBUF failed";return -1;
        }
        if(got && ctl(p->fd,VIDIOC_QBUF,&latest)<0){error="Camera requeue failed";return -1;}
        latest=b;std::memcpy(latest_pl,pl,sizeof(pl));
        if(p->multi)latest.m.planes=latest_pl;
        got=true;
    }
    if(!got)return 0;
    bool valid=latest.index<p->buffers.size() && !(latest.flags&V4L2_BUF_FLAG_ERROR);
    const uint8_t *data[2]={nullptr,nullptr};size_t bytes[2]={0,0};
    if(valid)for(unsigned k=0;k<p->planes;k++) {
        auto &plane=p->buffers[latest.index].planes[k];
        size_t used=p->multi?latest_pl[k].bytesused:latest.bytesused;
        size_t offset=p->multi?latest_pl[k].data_offset:0;
        if(used>plane.length||offset>used){valid=false;break;}
        data[k]=static_cast<const uint8_t *>(plane.data)+offset;bytes[k]=used-offset;
    }
    timing->dequeue_ms=stage_elapsed(tick);tick=StageClock::now();
    const auto dequeued_ms=std::chrono::duration_cast<std::chrono::milliseconds>(tick.time_since_epoch()).count();
    timing->source_ms=camera_source_timestamp(latest,static_cast<uint64_t>(dequeued_ms));
    if(valid)valid=camera_to_rgb(p->layout,data[0],bytes[0],data[1],bytes[1],rgb,error);
    timing->convert_ms=stage_elapsed(tick);tick=StageClock::now();
    int released=ctl(p->fd,VIDIOC_QBUF,&latest);
    timing->requeue_ms=stage_elapsed(tick);
    if(released<0){error="Camera frame release failed";return -1;}
    if(!valid){if(error.empty())error="Camera returned an invalid frame";return -1;}
    return 1;
}
