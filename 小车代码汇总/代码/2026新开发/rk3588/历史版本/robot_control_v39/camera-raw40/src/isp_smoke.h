/* Synthetic RAW-only feasibility test. Never interpreted as OV5640 IQ tuning. */
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <rk_aiq_offline_raw.h>
#include "isp_udp.h"
#include "raw_receiver.h"
#include <stdatomic.h>
#include <signal.h>
#include "v4l2_output.h"
static unsigned smoke_width=640, smoke_height=480;
static unsigned smoke_target_fps=15;
static int smoke_swap_pairs=1;
static unsigned smoke_raw_fourcc=V4L2_PIX_FMT_SRGGB8;
static volatile sig_atomic_t smoke_stop_requested;
static void smoke_request_stop(int signum){(void)signum;smoke_stop_requested=1;}
static _Atomic(void*) smoke_completed;
static void smoke_raw_done(void *base){atomic_store(&smoke_completed,base);}

struct smoke_capture { int fd; void *mem[4]; size_t len[4]; unsigned count,quantization; };
static int smoke_ioctl(int fd, unsigned long op, void *arg) {
    int rc; do rc=ioctl(fd,op,arg); while(rc<0 && errno==EINTR);
    if(rc<0)fprintf(stderr,"ISP V4L2 ioctl=0x%lx: %s\n",op,strerror(errno));
    return rc;
}
static void smoke_close(struct smoke_capture *c) {
    if(c->fd<0)return;
    enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ioctl(c->fd,VIDIOC_STREAMOFF,&type);
    for(unsigned i=0;i<c->count;i++)if(c->mem[i]!=MAP_FAILED)munmap(c->mem[i],c->len[i]);
    close(c->fd);c->fd=-1;
}
static int smoke_open(struct smoke_capture *c) {
    c->fd=open("/dev/video22",O_RDWR|O_NONBLOCK);
    if(c->fd<0){perror("open mainpath");return -1;}
    struct v4l2_capability cap={0};
    if(smoke_ioctl(c->fd,VIDIOC_QUERYCAP,&cap)<0)return -1;
    if(!strstr((char*)cap.card,"rkisp_mainpath")){fprintf(stderr,"unexpected video22 card %s\n",cap.card);return -1;}
    /* S_FMT does not reset the mainpath crop left by a previous resolution. */
    struct v4l2_selection sel={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        .target=V4L2_SEL_TGT_CROP,.r={.left=0,.top=0,.width=smoke_width,.height=smoke_height}};
    if(smoke_ioctl(c->fd,VIDIOC_S_SELECTION,&sel)<0 || sel.r.left || sel.r.top ||
       sel.r.width!=smoke_width || sel.r.height!=smoke_height)return -1;
    struct v4l2_format f={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE};
    f.fmt.pix_mp.width=smoke_width;f.fmt.pix_mp.height=smoke_height;f.fmt.pix_mp.pixelformat=V4L2_PIX_FMT_NV12;
    f.fmt.pix_mp.field=V4L2_FIELD_NONE;
    if(smoke_ioctl(c->fd,VIDIOC_S_FMT,&f)<0)return -1;
    c->quantization=f.fmt.pix_mp.quantization;
    printf("capture %ux%u planes=%u stride=%u size=%u quantization=%u\n",f.fmt.pix_mp.width,f.fmt.pix_mp.height,
        f.fmt.pix_mp.num_planes,f.fmt.pix_mp.plane_fmt[0].bytesperline,f.fmt.pix_mp.plane_fmt[0].sizeimage,c->quantization);
    if(c->quantization!=V4L2_QUANTIZATION_FULL_RANGE && c->quantization!=V4L2_QUANTIZATION_LIM_RANGE)return -1;
    if(f.fmt.pix_mp.num_planes!=1 || f.fmt.pix_mp.width!=smoke_width || f.fmt.pix_mp.height!=smoke_height ||
       f.fmt.pix_mp.pixelformat!=V4L2_PIX_FMT_NV12 ||
       f.fmt.pix_mp.plane_fmt[0].bytesperline!=smoke_width ||
       f.fmt.pix_mp.plane_fmt[0].sizeimage!=smoke_width*smoke_height*3/2)return -1;
    struct v4l2_requestbuffers req={.count=4,.type=f.type,.memory=V4L2_MEMORY_MMAP};
    if(smoke_ioctl(c->fd,VIDIOC_REQBUFS,&req)<0 || !req.count || req.count>4)return -1;
    for(unsigned i=0;i<req.count;i++) {
        struct v4l2_plane p={0};
        struct v4l2_buffer b={.index=i,.type=f.type,.memory=V4L2_MEMORY_MMAP,.length=1,.m.planes=&p};
        if(smoke_ioctl(c->fd,VIDIOC_QUERYBUF,&b)<0)return -1;
        c->len[i]=p.length;c->mem[i]=mmap(NULL,p.length,PROT_READ|PROT_WRITE,MAP_SHARED,c->fd,p.m.mem_offset);
        c->count=i+1;
        if(c->mem[i]==MAP_FAILED || smoke_ioctl(c->fd,VIDIOC_QBUF,&b)<0)return -1;
    }
    enum v4l2_buf_type type=f.type;
    return smoke_ioctl(c->fd,VIDIOC_STREAMON,&type);
}
static int smoke_raw(rk_aiq_sys_ctx_t *ctx,struct smoke_capture *c,int camera,unsigned frame_count) {
    int rx=open("/dev/video27",O_RDWR|O_NONBLOCK);
    struct v4l2_format rf={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE};
    int valid=rx>=0 && smoke_ioctl(rx,VIDIOC_G_FMT,&rf)==0;
    if(rx>=0)close(rx);
    if(!valid || rf.fmt.pix_mp.width!=smoke_width || rf.fmt.pix_mp.height!=smoke_height ||
       rf.fmt.pix_mp.pixelformat!=smoke_raw_fourcc ||
       rf.fmt.pix_mp.plane_fmt[0].bytesperline<smoke_width ||
       rf.fmt.pix_mp.plane_fmt[0].bytesperline>4096) {
        fputs("unexpected RAW reader layout\n",stderr);
        smoke_close(c);rk_aiq_uapi2_sysctl_stop(ctx,false);return -1;
    }
    unsigned stride=rf.fmt.pix_mp.plane_fmt[0].bytesperline;
    size_t pixels=rf.fmt.pix_mp.plane_fmt[0].sizeimage;
    if(pixels<stride*smoke_height || pixels>stride*(smoke_height+32u)) {
        smoke_close(c);rk_aiq_uapi2_sysctl_stop(ctx,false);return -1;
    }
    /* MMAP lengths may include page padding; retain it inside the payload. */
    pixels=(pixels+4095u)&~4095u;
    printf("RAW_LAYOUT stride=%u payload=%zu\n",stride,pixels);
    int udp=camera?isp_udp_open():-1;
    if(camera && udp<0){smoke_close(c);rk_aiq_uapi2_sysctl_stop(ctx,false);return -1;}
    struct camera_output display={.fd=-1};
    const char *node=getenv("ISP_V4L2_OUTPUT");
    if(node && camera_output_open(&display,node,smoke_width,smoke_height,0,smoke_target_fps)<0){
        if(udp>=0)close(udp);
        camera_output_close(&display);
        smoke_close(c);rk_aiq_uapi2_sysctl_stop(ctx,false);return -1;
    }
    if(node) {
        struct v4l2_format f={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT};
        if(smoke_ioctl(display.fd,VIDIOC_G_FMT,&f)<0 ||
           f.fmt.pix.colorspace!=V4L2_COLORSPACE_SMPTE170M ||
           (f.fmt.pix.ycbcr_enc!=V4L2_YCBCR_ENC_DEFAULT && f.fmt.pix.ycbcr_enc!=V4L2_YCBCR_ENC_601) ||
           (f.fmt.pix.quantization!=V4L2_QUANTIZATION_DEFAULT && f.fmt.pix.quantization!=V4L2_QUANTIZATION_LIM_RANGE)) {
            fputs("loopback must report limited-range BT.601\n",stderr);
            if(udp>=0)close(udp);
            camera_output_close(&display);smoke_close(c);rk_aiq_uapi2_sysctl_stop(ctx,false);return -1;
        }
    }
    /* This loopback version ignores color-only S_FMT updates. Convert bytes
     * to its established limited-range BT.601 contract instead of relabeling. */
    unsigned char y_limited[256],uv_limited[256];
    for(unsigned i=0;i<256;i++) {
        y_limited[i]=16+(i*219+127)/255;
        uv_limited[i]=16+(i*224+127)/255;
    }
    if(node)printf("V4L2 limited-range output, source quantization=%u\n",c->quantization);
    if(rk_aiq_uapi2_sysctl_registRkRawCb(ctx,smoke_raw_done)) {
        if(udp>=0)close(udp);
        camera_output_close(&display);
        smoke_close(c);rk_aiq_uapi2_sysctl_stop(ctx,false);return -1;
    }
    struct raw_receiver receiver;
    if(camera && raw_receiver_open(&receiver,udp,stride,smoke_width,smoke_height,smoke_swap_pairs)<0) {
        raw_receiver_close(&receiver);close(udp);camera_output_close(&display);
        smoke_close(c);rk_aiq_uapi2_sysctl_stop(ctx,false);return -1;
    }
    int save_frames=getenv("ISP_SAVE_FRAMES") && strcmp(getenv("ISP_SAVE_FRAMES"),"0");
    size_t size=2+sizeof(struct _raw_format)+sizeof(rk_aiq_frame_info_t)+sizeof(struct _block_header)+pixels+2;
    unsigned char *packets[8]={0};int captured=0,failed=0;double peak=-1;
    struct timespec capture_start,capture_end;
    clock_gettime(CLOCK_MONOTONIC,&capture_start);
    unsigned char *raw=calloc(1,size);packets[0]=raw;
    if(!raw)failed=1;
    uint64_t report_start=raw_clock_ms(),wait_total=0,isp_total=0,output_total=0,last_age=0;
    unsigned reported=0;uint64_t rx_reported=0;
    for(unsigned frame=0;raw && frame<frame_count && !smoke_stop_requested;frame++) {
        unsigned char *p=raw;
        uint16_t tag=START_TAG;memcpy(p,&tag,2);p+=2;
        struct _raw_format fmt={.tag=FORMAT_TAG,.size=sizeof(fmt)-6,.vesrion=1,
            .frame_id=frame+1,.width=smoke_width,.height=smoke_height,.bit_width=8,
            .bayer_fmt=camera?RK_AIQ_BAYER_BGGR:RK_AIQ_BAYER_RGGB,
            .hdr_mode=1,.line_length=stride,.acive_line_length=smoke_width};
        memcpy(p,&fmt,sizeof(fmt));p+=sizeof(fmt);
        rk_aiq_frame_info_t info={.tag=STATS_TAG,.size=sizeof(info)-6,.vesrion=1,
            .frame_id=frame+1,.normal_exp=.01f,.normal_gain=1.f,.normal_exp_reg=100,.normal_gain_reg=16,.awg_rgain=1.f,.awg_bgain=1.f};
        memcpy(p,&info,sizeof(info));p+=sizeof(info);
        struct _block_header h={.block_id=NORMAL_RAW_TAG,.block_length=pixels};
        memcpy(p,&h,sizeof(h));p+=sizeof(h);
        const unsigned char *input_pixels=p;
        for(unsigned y=0;!camera && y<smoke_height;y++)for(unsigned x=0;x<smoke_width;x++) {
            unsigned bar=x/(smoke_width/8);
            unsigned color=((y&1)==0 && (x&1)==0)?4:((y&1) && (x&1))?1:2;
            p[y*stride+x]=(bar&color)?192:32;
        }
        uint64_t tick=raw_clock_ms();
        if(camera && raw_receiver_take(&receiver,p,&last_age)<0){failed=1;break;}
        wait_total+=raw_clock_ms()-tick;
        if(save_frames && camera && frame+1==frame_count) {
            FILE *input=fopen("/tmp/camera-capture.dat","wb");
            if(input){for(unsigned y=0;y<smoke_height;y++)fwrite(p+y*stride,1,smoke_width,input);fclose(input);}
        }
        p+=pixels;tag=END_TAG;memcpy(p,&tag,2);
        atomic_store(&smoke_completed,NULL);
        tick=raw_clock_ms();
        int rc=rk_aiq_uapi2_sysctl_enqueueRkRawBuf(ctx,raw,false);
        struct pollfd fds={.fd=c->fd,.events=POLLIN};
        if(rc || poll(&fds,1,1000)<=0){fprintf(stderr,"capture timeout/error frame=%u\n",frame+1);break;}
        struct v4l2_plane plane={0};
        struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,.memory=V4L2_MEMORY_MMAP,.length=1,.m.planes=&plane};
        if(smoke_ioctl(c->fd,VIDIOC_DQBUF,&b)<0)break;
        isp_total+=raw_clock_ms()-tick;tick=raw_clock_ms();
        if(!(b.flags&V4L2_BUF_FLAG_ERROR) && b.index<c->count &&
           plane.bytesused==smoke_width*smoke_height*3/2 && plane.bytesused<=c->len[b.index]) {
            if(save_frames) {
                FILE *f=fopen("/tmp/camera-frame.raw","wb");
                if(!f){failed=1;break;}
                size_t n=fwrite(c->mem[b.index],1,plane.bytesused,f);
                int closed=fclose(f);
                if(n!=plane.bytesused || closed){failed=1;break;}
            }
            const unsigned char *ydata=c->mem[b.index];unsigned long sum=0;
            double mean=0;
            if(getenv("ISP_FLASH_MEASURE")) {
                for(unsigned k=0;k<smoke_width*smoke_height;k+=16)sum+=ydata[k];
                mean=(double)sum/(smoke_width*smoke_height/16);
            }
            if(getenv("ISP_FLASH_MEASURE") && mean>peak) {
                FILE *pf=fopen("/tmp/camera-flash-peak.raw","wb");
                if(pf){fwrite(ydata,1,plane.bytesused,pf);fclose(pf);}
                FILE *pr=fopen("/tmp/camera-flash-input.raw","wb");
                if(pr){for(unsigned y=0;y<smoke_height;y++)fwrite(input_pixels+y*stride,1,smoke_width,pr);fclose(pr);}
                peak=mean;
            }
            if(node) {
                const unsigned char *out=c->mem[b.index];
                if(c->quantization==V4L2_QUANTIZATION_FULL_RANGE) {
                    unsigned ysize=smoke_width*smoke_height;
                    for(unsigned k=0;k<ysize;k++)display.pixels[k]=y_limited[out[k]];
                    for(unsigned k=ysize;k<ysize*3/2;k++)display.pixels[k]=uv_limited[out[k]];
                    out=display.pixels;
                }
                if(camera_output_frame(&display,out,smoke_width)<0){failed=1;break;}
            }
        } else {
            fprintf(stderr,"invalid ISP output index=%u flags=%x bytes=%u\n",b.index,b.flags,plane.bytesused);
            failed=1;break;
        }
        output_total+=raw_clock_ms()-tick;
        if(smoke_ioctl(c->fd,VIDIOC_QBUF,&b)<0)break;
        unsigned wait;
        for(wait=0;wait<1000 && atomic_load(&smoke_completed)!=raw;wait++)usleep(1000);
        if(atomic_load(&smoke_completed)!=raw){fputs("RAW completion callback timeout\n",stderr);break;}
        captured++;
        if(captured%30==0) {
            uint64_t now=raw_clock_ms();unsigned n=captured-reported;
            uint64_t received=camera?atomic_load(&receiver.complete):0;
            double dt=(now-report_start?now-report_start:1)/1000.0;
            printf("FASTPATH output_fps=%.2f rx_fps=%.2f rx_complete=%llu rx_superseded=%llu incomplete=%llu malformed=%llu wait_ms=%.2f isp_ms=%.2f output_ms=%.2f raw_age_ms=%llu\n",
                n/dt,(received-rx_reported)/dt,
                camera?(unsigned long long)atomic_load(&receiver.complete):0,
                camera?(unsigned long long)atomic_load(&receiver.skipped):0,
                camera?(unsigned long long)atomic_load(&receiver.incomplete):0,
                camera?(unsigned long long)atomic_load(&receiver.rejected):0,
                (double)wait_total/n,(double)isp_total/n,(double)output_total/n,(unsigned long long)last_age);
            reported=captured;rx_reported=received;report_start=now;wait_total=isp_total=output_total=0;
        }
    }
    clock_gettime(CLOCK_MONOTONIC,&capture_end);
    double elapsed=capture_end.tv_sec-capture_start.tv_sec+(capture_end.tv_nsec-capture_start.tv_nsec)/1e9;
    printf("ISP_RATE frames=%d seconds=%.3f fps=%.3f\n",captured,elapsed,elapsed>0?captured/elapsed:0);
    if(camera)raw_receiver_close(&receiver);
    /* Stop worker threads before freeing retained RAW inputs. */
    smoke_close(c);
    int stopped=rk_aiq_uapi2_sysctl_stop(ctx,false);
    if(stopped){fprintf(stderr,"AIQ stop failed rc=%d\n",stopped);failed=1;}
    /* On failed stop, retain input storage until process exit. */
    else for(unsigned i=0;i<8;i++)free(packets[i]);
    if(udp>=0)close(udp);
    camera_output_close(&display);
    printf("ISP_RESULT source=%s captured=%d expected=%u interrupted=%d\n",camera?"camera":"synthetic",captured,frame_count,(int)smoke_stop_requested);
    /* Interruption never converts a partial/error run into successful validation. */
    return !failed && !smoke_stop_requested && captured==(int)frame_count?0:-1;
}
