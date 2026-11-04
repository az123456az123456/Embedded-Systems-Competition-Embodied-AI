#!/bin/sh
set -eu
APP=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
ROOT=$APP
while [ "$ROOT" != / ] && [ ! -d "$ROOT/bsp_r4" ]; do ROOT=$(dirname -- "$ROOT"); done
SDK=${BSP_SDK:-$ROOT/bsp_r4/ACG138-Offline-BSP-20260923-r4/sdk/yy3588_extras}
CC=${ARM64_CC:-$SDK/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc}
INC=${RKAIQ_INCLUDE:-$SDK/external/camera_engine_rkaiq/rkaiq/include}
LIB=${RKAIQ_LIB:-$ROOT/rk3588_acg138/14_FPGA_摄像头1080p/project/runtime/lib}
CAMERA_DEFAULT_FPS=${CAMERA_DEFAULT_FPS-40}
case "$CAMERA_DEFAULT_FPS" in 15|20|30|35|40) ;; *) echo 'CAMERA_DEFAULT_FPS must be 15, 20, 30, 35 or 40' >&2; exit 2 ;; esac
mkdir -p "$APP/build"
"$CC" -O2 -std=gnu11 -Wall -Wextra -Werror -Wno-unused-function \
 -DCAMERA_ENABLE_UNSAFE_ISP_RAW -DCAMERA_DEFAULT_FPS="$CAMERA_DEFAULT_FPS" -I"$INC" -I"$INC/uAPI2" -I"$INC/common" -I"$INC/xcore" \
 -I"$INC/algos" -I"$INC/iq_parser" -I"$INC/iq_parser_v2" \
 "$APP/src/isp_probe.c" -L"$LIB" -Wl,--allow-shlib-undefined -lrkaiq -lm -pthread -o "$APP/build/isp-probe"
"${CC%gcc}strip" "$APP/build/isp-probe"
printf '%s\n' "Built build/isp-probe (default $CAMERA_DEFAULT_FPS FPS); pair with the existing isolated FakeCam librkaiq.so and IQ file."
