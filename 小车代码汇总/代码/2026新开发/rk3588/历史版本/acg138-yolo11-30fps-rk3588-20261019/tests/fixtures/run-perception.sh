#!/bin/sh
# Reuse ACG138's existing camera setup, model/runtime and desktop session.
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
YOLO_DIR=${YOLO_DIR:-/root/rknn_yolov8_demo}
FIND_VIDEO_SCRIPT=${FIND_VIDEO_SCRIPT:-/root/find-video.sh}
ISP_CAMERA_SCRIPT=${ISP_CAMERA_SCRIPT:-/opt/fpga-camera-isp-1080/run_isp_camera.sh}
ISP_LOG=${ISP_LOG:-/tmp/perception-isp.log}
PREPARE_ISP=${PERCEPTION_PREPARE_ISP:-auto}
WAIT_ATTEMPTS=${PERCEPTION_VIDEO_WAIT_ATTEMPTS:-30}

# Inspect arguments without changing quoting or order for the native program.
NO_CAPTURE=0
CLI_CAMERA=
NEXT=
for argument in "$@"; do
    if [ -n "$NEXT" ]; then
        if [ "$NEXT" = camera ]; then CLI_CAMERA=$argument; fi
        NEXT=
        continue
    fi
    case "$argument" in
        --demo|--help) NO_CAPTURE=1 ;;
        --preview) NO_CAPTURE=1; NEXT=value ;;
        --camera) NEXT=camera ;;
        --model|--labels|--view|--interface|--width|--height|--confidence|--nms|--seconds|--npu-workers) NEXT=value ;;
    esac
done
if [ -n "$NEXT" ]; then printf '%s\n' 'Missing command-line option value' >&2; exit 2; fi

if [ -z "${MODEL_PATH:-}" ]; then
    if [ -f "$YOLO_DIR/model/yolov8s.rknn" ]; then
        MODEL_PATH=$YOLO_DIR/model/yolov8s.rknn
    else MODEL_PATH=$APP_DIR/model/model.rknn; fi
fi
if [ -z "${LABELS_PATH:-}" ]; then
    if [ "$MODEL_PATH" = "$YOLO_DIR/model/yolov8s.rknn" ] && [ -f "$YOLO_DIR/model/coco_80_labels_list.txt" ]; then
        LABELS_PATH=$YOLO_DIR/model/coco_80_labels_list.txt
    else LABELS_PATH=$APP_DIR/model/coco_80_labels_list.txt; fi
fi
for directory in "$YOLO_DIR/lib" "$APP_DIR/lib"; do
    if [ -d "$directory" ]; then LD_LIBRARY_PATH=${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$directory; fi
done
if [ -n "${LD_LIBRARY_PATH:-}" ]; then export LD_LIBRARY_PATH; fi

if [ -z "${WAYLAND_DISPLAY:-}" ]; then
    for directory in "${XDG_RUNTIME_DIR:-/run}" /run /var/run /run/user/0 /run/user/1000 /tmp; do
        for socket in "$directory"/wayland-*; do
            if [ -S "$socket" ]; then
                XDG_RUNTIME_DIR=$directory
                WAYLAND_DISPLAY=${socket##*/}
                break 2
            fi
        done
    done
fi
if [ -z "${XDG_RUNTIME_DIR:-}" ]; then XDG_RUNTIME_DIR=/run; fi
if [ -z "${WAYLAND_DISPLAY:-}" ]; then WAYLAND_DISPLAY=wayland-0; fi
export XDG_RUNTIME_DIR
export WAYLAND_DISPLAY

if [ -z "${PERCEPTION_BINARY:-}" ]; then
    case $(uname -m) in
        aarch64|arm64) candidates='perception-dashboard.aarch64 build/perception-dashboard.aarch64 perception-dashboard build/perception-dashboard' ;;
        *) candidates='perception-dashboard build/perception-dashboard' ;;
    esac
    for name in $candidates; do
        if [ -x "$APP_DIR/$name" ]; then PERCEPTION_BINARY=$APP_DIR/$name; break; fi
    done
fi
if [ -z "${PERCEPTION_BINARY:-}" ] || [ ! -x "$PERCEPTION_BINARY" ]; then
    printf '%s\n' 'Program not found. Build with make (native) or ./scripts/build-arm64.sh (ARM64 SDK), or unpack the matching release package.' >&2
    exit 1
fi

if [ "$NO_CAPTURE" = 0 ]; then
    case "$WAIT_ATTEMPTS" in ''|*[!0-9]*|0) printf '%s\n' 'Invalid video wait attempts' >&2; exit 2 ;; esac
    if [ "$WAIT_ATTEMPTS" -gt 300 ]; then printf '%s\n' 'Video wait attempts exceeds 300' >&2; exit 2; fi
    case "$PREPARE_ISP" in
        auto) if [ -f "$ISP_CAMERA_SCRIPT" ]; then PREPARE_ISP=1; else PREPARE_ISP=0; fi ;;
        0|1) ;;
        *) printf '%s\n' 'PERCEPTION_PREPARE_ISP must be auto, 0 or 1' >&2; exit 2 ;;
    esac
    if [ "$PREPARE_ISP" = 1 ]; then
        if [ ! -f "$ISP_CAMERA_SCRIPT" ]; then
            printf 'ISP camera script not found: %s\n' "$ISP_CAMERA_SCRIPT" >&2; exit 1
        fi
        if ! command -v v4l2-ctl >/dev/null 2>&1; then
            printf '%s\n' 'v4l2-ctl is required to check FPGA camera readiness' >&2; exit 1
        fi
        if ! ps -ef | grep '[i]sp-probe' >/dev/null 2>&1; then
            printf '%s\n' 'Preparing FPGA dual-1080 camera...'
            (
                if [ -d "$YOLO_DIR" ]; then cd "$YOLO_DIR"; fi
                export ISP_DISPLAY=0
                if command -v setsid >/dev/null 2>&1; then
                    exec nohup setsid sh "$ISP_CAMERA_SCRIPT" dual-1080 100000
                else exec nohup sh "$ISP_CAMERA_SCRIPT" dual-1080 100000; fi
            ) >"$ISP_LOG" 2>&1 </dev/null &
        fi
    fi
    VIDEO=
    i=0
    while [ "$i" -lt "$WAIT_ATTEMPTS" ]; do
        # Discover after ISP setup; never assume a permanent /dev/video number.
        if [ -n "$CLI_CAMERA" ]; then VIDEO=$CLI_CAMERA
        elif [ -n "${CAMERA_DEVICE:-}" ]; then VIDEO=$CAMERA_DEVICE
        elif [ -r "$FIND_VIDEO_SCRIPT" ]; then
            if [ -d "$YOLO_DIR" ]; then VIDEO=$(cd "$YOLO_DIR" && sh "$FIND_VIDEO_SCRIPT" || true)
            else VIDEO=$(sh "$FIND_VIDEO_SCRIPT" || true); fi
        elif [ -c /dev/video41 ]; then VIDEO=/dev/video41
        elif [ -c /dev/video0 ]; then VIDEO=/dev/video0
        fi
        if [ -n "$VIDEO" ] && [ -c "$VIDEO" ]; then
            if [ "$PREPARE_ISP" = 0 ]; then break; fi
            format=$(v4l2-ctl -d "$VIDEO" --get-fmt-video 2>/dev/null || true)
            case "$format" in *NV12*) break ;; esac
        fi
        i=$((i+1))
        if [ "$i" -lt "$WAIT_ATTEMPTS" ]; then sleep 1; fi
    done
    if [ "$i" -ge "$WAIT_ATTEMPTS" ]; then
        printf 'FPGA camera not ready (last node: %s). Check %s or pass --camera /dev/videoN.\n' "${VIDEO:-none}" "$ISP_LOG" >&2
        exit 1
    fi
    printf 'Using camera: %s\nModel: %s\n' "$VIDEO" "$MODEL_PATH"
else
    VIDEO=${CLI_CAMERA:-${CAMERA_DEVICE:-/dev/video41}}
fi
exec "$PERCEPTION_BINARY" \
    --model "$MODEL_PATH" --labels "$LABELS_PATH" --camera "$VIDEO" "$@"
