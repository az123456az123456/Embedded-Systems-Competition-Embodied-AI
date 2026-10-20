#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
/* Read-only RKAIQ discovery plus process-local offline pre-initialization.
 * A successful preInit alone does not prove RAW processing or calibrated IQ.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <rk_aiq_user_api2_sysctl.h>
#include <rk_aiq_user_api2_imgproc.h>
#include "isp_smoke.h"
int main(int argc, char **argv)
{
    /* Validate before RKAIQ discovery/init or any V4L2/network operation. */
    if(camera_target_fps_parse(getenv("CAMERA_TARGET_FPS"),&smoke_target_fps)<0) {
        fputs("CAMERA_TARGET_FPS must be 15, 20 or 30\n",stderr);return 2;
    }
    setvbuf(stdout, NULL, _IONBF, 0);
    struct sigaction action={0};
    action.sa_handler=smoke_request_stop;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,NULL);sigaction(SIGTERM,&action,NULL);
#ifndef CAMERA_ENABLE_UNSAFE_ISP_RAW
    if (argc>2 && (!strcmp(argv[2],"--raw") || !strcmp(argv[2],"--camera") || !strcmp(argv[2],"--dual-camera") || !strcmp(argv[2],"--camera-1080") || !strcmp(argv[2],"--dual-camera-1080") || !strcmp(argv[2],"--raw-1080"))) {
        fputs("RAW readback requires the isolated FakeCam library and prelinked media graph. Use the opt-in ISP_RAW_EXPERIMENT build and run_isp_camera.sh; see the ISP bring-up report.\n",stderr);
        return 7;
    }
#endif
    if((argc>2 && !strcmp(argv[2],"--dual-camera")) || getenv("ISP_WIDE_RAW"))smoke_width=1280;
    int camera=argc>2 && (!strcmp(argv[2],"--camera") || !strcmp(argv[2],"--dual-camera"));
    if(argc>2 && (!strcmp(argv[2],"--camera-1080") || !strcmp(argv[2],"--dual-camera-1080") || !strcmp(argv[2],"--raw-1080"))) {
        smoke_width=1920;smoke_height=1080;
        camera=strcmp(argv[2],"--raw-1080")!=0;
    }
    smoke_swap_pairs=!(argc>2 && (!strcmp(argv[2],"--dual-camera") || !strcmp(argv[2],"--dual-camera-1080")));
    printf("ISP_MODE %ux%u swap_pairs=%d camera=%d target_fps=%u measured_rate=FASTPATH\n",
        smoke_width,smoke_height,smoke_swap_pairs,camera,smoke_target_fps);
    if(camera)smoke_raw_fourcc=V4L2_PIX_FMT_SBGGR8;
    rk_aiq_static_info_t info = {0};
    int count = 0;
    for (int i=0; i<8; ++i) {
        int rc = rk_aiq_uapi2_sysctl_enumStaticMetas(i, &info);
        printf("enumStaticMetas index=%d rc=%d\n", i, rc);
        if (rc) break;
        ++count;
    }
    rk_aiq_frame_info_t frames[2] = {0};
    const char *sensor = rk_aiq_uapi2_sysctl_rawReproc_preInit(
        "rkisp0-vir0", "ov5640", frames);
    printf("ISP_PROBE sensors=%d offline_sensor=%s\n", count,
           sensor ? sensor : "NULL");
    if (!sensor) return 3;
    if (argc < 2) return 0;
    /* Explicit surrogate IQ for a synthetic test only, never sensor calibration. */
    setenv("USE_AS_FAKE_CAM", "1", 1);
    int rc = rk_aiq_uapi2_sysctl_preInit(sensor, RK_AIQ_WORKING_MODE_NORMAL, argv[1]);
    printf("preInit rc=%d surrogate_IQ=%s\n", rc, argv[1]); fflush(stdout);
    if (rc) return 4;
    rk_aiq_sys_ctx_t *ctx = rk_aiq_uapi2_sysctl_init(sensor, "/etc/iqfiles", NULL, NULL);
    printf("init ctx=%p\n", (void *)ctx); fflush(stdout);
    if (!ctx) return 5;
    rk_aiq_raw_prop_t prop = {.frame_width=smoke_width, .frame_height=smoke_height,
        .format=camera?RK_PIX_FMT_SBGGR8:RK_PIX_FMT_SRGGB8, .rawbuf_type=RK_AIQ_RAW_DATA};
    rc = rk_aiq_uapi2_sysctl_prepareRkRaw(ctx, prop);
    printf("prepareRkRaw rc=%d\n", rc); fflush(stdout);
    if (!rc) rc = rk_aiq_uapi2_sysctl_prepare(ctx, smoke_width, smoke_height, RK_AIQ_WORKING_MODE_NORMAL);
    printf("prepare rc=%d\n", rc); fflush(stdout);
    struct smoke_capture capture={.fd=-1};
    int rawtest=argc>2 && (!strcmp(argv[2],"--raw") || !strcmp(argv[2],"--raw-1080") || camera);
    int streamtest=argc>2 && !strcmp(argv[2],"--stream");
    unsigned frame_count=8;
    if(argc>3){char *end=NULL;unsigned long n=strtoul(argv[3],&end,10);
        if(!*argv[3] || *end || n<1 || n>100000){rk_aiq_uapi2_sysctl_deinit(ctx);return 2;}
        frame_count=(unsigned)n;}
    if (!rc) {
        rc = rk_aiq_uapi2_sysctl_start(ctx);
        printf("start rc=%d\n", rc); fflush(stdout);
        if (!rc) {
            if(getenv("ISP_MANUAL_WB")) {
                rk_aiq_wb_gain_t gain={1.f,1.f,1.f,1.f};
                int mode_rc=rk_aiq_uapi2_setWBMode(ctx,OP_MANUAL);
                int gain_rc=mode_rc?mode_rc:rk_aiq_uapi2_setMWBGain(ctx,&gain);
                printf("manualWB mode=%d gain=%d\n",mode_rc,gain_rc);
                if(mode_rc || gain_rc)rc=-1;
            }
            if(getenv("ISP_SHARP_OFF")){int r=rk_aiq_uapi2_setSharpness(ctx,0);printf("sharpness0 rc=%d\n",r);if(r)rc=-1;}
            if(getenv("ISP_DRC_UNITY")){int r=rk_aiq_uapi2_setDrcGain(ctx,1.f,0.f,16.f);printf("drcUnity rc=%d\n",r);if(r)rc=-1;}
            if(getenv("ISP_GAMMA_LINEAR")){int r=rk_aiq_uapi2_setGammaCoef(ctx,1.f,0.f);printf("gammaLinear rc=%d\n",r);if(r)rc=-1;}
            if(rc){rk_aiq_uapi2_sysctl_stop(ctx,false);rk_aiq_uapi2_sysctl_deinit(ctx);return 6;}
            if(streamtest) {
                puts("ISP_STAGE capture stream-on without RAW enqueue");
                rc=smoke_open(&capture);
                printf("ISP_STAGE capture stream-on rc=%d\n",rc);
                if(!rc)sleep(1);
                puts("ISP_STAGE capture stream-off");
                smoke_close(&capture);
                puts("ISP_STAGE aiq stop");
                rk_aiq_uapi2_sysctl_stop(ctx,false);
            }
            else if(rawtest) {
                rc=smoke_open(&capture);
                if(!rc)rc=smoke_raw(ctx,&capture,camera,frame_count);
                else rk_aiq_uapi2_sysctl_stop(ctx,false);
            }
            else rk_aiq_uapi2_sysctl_stop(ctx, false);
        }
    }
    smoke_close(&capture);
    rk_aiq_uapi2_sysctl_deinit(ctx);
    return rc ? 6 : 0;
}
