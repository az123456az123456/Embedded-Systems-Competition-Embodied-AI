#ifndef CAMERA_V4L2_OUTPUT_H
#define CAMERA_V4L2_OUTPUT_H
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include "pixels.h"
#include "camera_rate.h"
struct camera_output { int fd; unsigned w,h,x; uint8_t *pixels; size_t size; };
static int camera_output_disable_repeat(int fd,unsigned offset,const char *name) {
    /* These controls belong to the supplied v4l2loopback module. Refuse a
     * mismatched ID rather than accidentally changing an unrelated control. */
    struct v4l2_queryctrl query={.id=(V4L2_CID_USER_BASE|0xf000)+offset};
    if(ioctl(fd,VIDIOC_QUERYCTRL,&query) || strcmp((char*)query.name,name) ||
       (query.flags&V4L2_CTRL_FLAG_DISABLED)) {
        fprintf(stderr,"V4L2 missing loopback control %s\n",name);return -1;
    }
    struct v4l2_control control={.id=query.id,.value=0};
    if(ioctl(fd,VIDIOC_S_CTRL,&control) || ioctl(fd,VIDIOC_G_CTRL,&control) || control.value) {
        fprintf(stderr,"V4L2 cannot disable loopback %s\n",name);return -1;
    }
    return 0;
}
static int camera_output_open(struct camera_output *o,const char *path,unsigned w,unsigned h,unsigned x,unsigned fps) {
    /* Reject an invalid interval without opening or mutating a device. */
    if(!camera_target_fps_valid(fps)) {fputs("Invalid camera target FPS\n",stderr);return -1;}
    o->fd=open(path,O_WRONLY|O_CLOEXEC);
    if(o->fd<0){perror(path);return -1;}
    struct v4l2_capability cap={0};
    if(ioctl(o->fd,VIDIOC_QUERYCAP,&cap) || strncmp((char*)cap.card,"FPGA_",5)) {
        fprintf(stderr,"refusing non-FPGA output %s\n",path);return -1;
    }
    if(camera_output_disable_repeat(o->fd,1,"sustain_framerate") ||
       camera_output_disable_repeat(o->fd,2,"timeout"))return -1;
    struct v4l2_format f={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT};
    f.fmt.pix.width=w;f.fmt.pix.height=h;f.fmt.pix.pixelformat=V4L2_PIX_FMT_NV12;
    f.fmt.pix.field=V4L2_FIELD_NONE;f.fmt.pix.bytesperline=w;f.fmt.pix.sizeimage=w*h*3/2;
    f.fmt.pix.colorspace=V4L2_COLORSPACE_SMPTE170M;
    f.fmt.pix.ycbcr_enc=V4L2_YCBCR_ENC_601;f.fmt.pix.quantization=V4L2_QUANTIZATION_LIM_RANGE;
    if(ioctl(o->fd,VIDIOC_S_FMT,&f) || f.fmt.pix.width!=w || f.fmt.pix.height!=h ||
        f.fmt.pix.pixelformat!=V4L2_PIX_FMT_NV12 || f.fmt.pix.bytesperline!=w || f.fmt.pix.sizeimage!=w*h*3/2) {
        fprintf(stderr,"V4L2 output format mismatch %s\n",path);return -1;
    }
    struct v4l2_streamparm p={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT};
    p.parm.output.timeperframe.numerator=1;p.parm.output.timeperframe.denominator=fps;
    if(ioctl(o->fd,VIDIOC_S_PARM,&p)){perror("V4L2 output fps");return -1;}
    if(!p.parm.output.timeperframe.numerator || !p.parm.output.timeperframe.denominator) {
        fputs("V4L2 returned invalid output interval\n",stderr);return -1;
    }
    fprintf(stderr,"V4L2_INTERVAL requested_fps=%u negotiated_fps=%.3f nominal_only=1 sustain_framerate=0 timeout=0\n",
        fps,(double)p.parm.output.timeperframe.denominator/p.parm.output.timeperframe.numerator);
    o->w=w;o->h=h;o->x=x;o->size=w*h*3/2;o->pixels=malloc(o->size);
    if(!o->pixels)return -1;
    fprintf(stderr,"V4L2 %s NV12 %ux%u crop_x=%u\n",path,w,h,x);
    return 0;
}
static int camera_output_frame(struct camera_output *o,const uint8_t *nv12,unsigned stride) {
    const uint8_t *data=nv12;
    if(o->w!=stride) { nv12_crop(nv12,stride,o->h,o->x,o->w,o->pixels);data=o->pixels; }
    ssize_t n;do n=write(o->fd,data,o->size);while(n<0 && errno==EINTR);
    if(n!=(ssize_t)o->size){perror("V4L2 frame write");return -1;}
    return 0;
}
static void camera_output_close(struct camera_output *o) {
    if(o->fd>=0)close(o->fd);
    free(o->pixels);
}
#endif
