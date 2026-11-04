#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
static unsigned opens,closes,sets,controls,writes,interval,requested_interval;
static int foreign_card,negotiate;
static int mock_open(const char *path,int flags,...) {
    assert(!strcmp(path,"/dev/video60"));assert(flags==(O_WRONLY|O_CLOEXEC));opens++;return 77;
}
static int mock_close(int fd){assert(fd==77);closes++;return 0;}
static ssize_t mock_write(int fd,const void *data,size_t size) {
    assert(fd==77 && data && size==1920*1080*3/2);writes++;return (ssize_t)size;
}
static int mock_ioctl(int fd,unsigned long operation,...) {
    assert(fd==77);va_list args;va_start(args,operation);void *argument=va_arg(args,void*);va_end(args);
    switch(operation) {
    case VIDIOC_QUERYCAP: {
        struct v4l2_capability *cap=argument;
        strcpy((char*)cap->card,foreign_card?"unrelated_camera":"FPGA_Combined");break;
    }
    case VIDIOC_QUERYCTRL: {
        struct v4l2_queryctrl *query=argument;unsigned offset=query->id-(V4L2_CID_USER_BASE|0xf000);
        assert(offset==1 || offset==2);
        strcpy((char*)query->name,offset==1?"sustain_framerate":"timeout");break;
    }
    case VIDIOC_S_CTRL: {
        struct v4l2_control *control=argument;assert(control->value==0);controls++;break;
    }
    case VIDIOC_G_CTRL: ((struct v4l2_control*)argument)->value=0;break;
    case VIDIOC_S_FMT: {
        struct v4l2_format *format=argument;assert(format->type==V4L2_BUF_TYPE_VIDEO_OUTPUT);
        assert(format->fmt.pix.width==1920 && format->fmt.pix.height==1080);
        assert(format->fmt.pix.pixelformat==V4L2_PIX_FMT_NV12);
        assert(format->fmt.pix.bytesperline==1920 && format->fmt.pix.sizeimage==1920*1080*3/2);
        assert(format->fmt.pix.colorspace==V4L2_COLORSPACE_SMPTE170M);
        assert(format->fmt.pix.quantization==V4L2_QUANTIZATION_LIM_RANGE);sets++;break;
    }
    case VIDIOC_S_PARM: {
        struct v4l2_streamparm *parm=argument;assert(parm->type==V4L2_BUF_TYPE_VIDEO_OUTPUT);
        assert(parm->parm.output.timeperframe.numerator==1);
        requested_interval=parm->parm.output.timeperframe.denominator;
        if(negotiate)parm->parm.output.timeperframe.denominator=negotiate;
        interval=parm->parm.output.timeperframe.denominator;break;
    }
    default: assert(!"unexpected ioctl");return -1;
    }
    return 0;
}
#define open mock_open
#define close mock_close
#define write mock_write
#define ioctl mock_ioctl
#include "v4l2_output.h"
#undef open
#undef close
#undef write
#undef ioctl
int main(void) {
    unsigned fps=0;
    assert(!camera_target_fps_parse(NULL,&fps) && fps==CAMERA_DEFAULT_FPS);
    const char *legal[]={"15","20","30"};
    for(unsigned i=0;i<3;i++)assert(!camera_target_fps_parse(legal[i],&fps));
    const char *bad[]={"","0","19","21","60","020","20.0","+20","20 ","20\n","9999999999999999999999"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++)assert(camera_target_fps_parse(bad[i],&fps)<0);
    struct camera_output output={.fd=-1};
    assert(camera_output_open(&output,"/dev/video60",1920,1080,0,21)<0);
    assert(!opens && !sets && !controls && !writes);
    foreign_card=1;
    assert(camera_output_open(&output,"/dev/video60",1920,1080,0,20)<0);
    assert(opens==1 && !sets && !controls && !writes);camera_output_close(&output);foreign_card=0;
    unsigned rates[]={15,20,30};
    uint8_t *image=calloc(1,1920*1080*3/2);assert(image);
    for(unsigned i=0;i<3;i++) {
        output=(struct camera_output){.fd=-1};
        assert(!camera_output_open(&output,"/dev/video60",1920,1080,0,rates[i]));
        assert(interval==rates[i] && requested_interval==rates[i]);
        assert(!camera_output_frame(&output,image,1920));
        /* Exactly one write per actual ISP frame: setting nominal FPS does not
         * write, repeat, interpolate or sleep to manufacture frame counts. */
        assert(writes==i+1);camera_output_close(&output);
    }
    negotiate=15;output=(struct camera_output){.fd=-1};
    assert(!camera_output_open(&output,"/dev/video60",1920,1080,0,20));
    assert(requested_interval==20 && interval==15);camera_output_close(&output);
    assert(controls==8 && sets==4 && writes==3 && closes==5);free(image);
    puts("PASS default/environment FPS parsing, 15/20/30 S_PARM, negotiated rate, NV12 contract, repeat controls, invalid/foreign-device side-effect guards");
    return 0;
}
