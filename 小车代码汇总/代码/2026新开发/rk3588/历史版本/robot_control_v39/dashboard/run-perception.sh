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
CAMERA_SOURCE=${PERCEPTION_CAMERA_SOURCE:-auto}
case "$CAMERA_SOURCE" in auto|v4l2|fpga-rgb) ;; *) printf '%s\n' 'PERCEPTION_CAMERA_SOURCE must be auto, v4l2 or fpga-rgb' >&2; exit 2 ;; esac
# The matching FPGA image controls the real sensor rate. This value only
# describes the V4L2 publisher when this launcher starts a new ISP session.
CAMERA_TARGET_FPS=${CAMERA_TARGET_FPS:-40}
case "$CAMERA_TARGET_FPS" in 15|20|30|40) ;; *) printf '%s\n' 'CAMERA_TARGET_FPS must be 15, 20, 30 or 40' >&2; exit 2 ;; esac
export CAMERA_TARGET_FPS

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
        --demo|--help|--version) NO_CAPTURE=1 ;;
        --preview) NO_CAPTURE=1; NEXT=value ;;
        --camera) NEXT=camera ;;
        --model|--labels|--view|--interface|--imu-interface|--motor-interface|--width|--height|--confidence|--nms|--seconds|--npu-workers|--rotate|--imu-deadband|--phone-port|--phone-bind|--pair-token-file|--left-mask|--invert-mask|--phone-fps|--page) NEXT=value ;;
    esac
done
if [ -n "$NEXT" ]; then printf '%s\n' 'Missing command-line option value' >&2; exit 2; fi
case "$CLI_CAMERA" in
    fpga-rgb:*) CAMERA_SOURCE=fpga-rgb ;;
    '') ;;
    *)
        # A concrete legacy camera path is an explicit request to use V4L2.
        # Do not silently open raw Ethernet while printing /dev/videoN.
        CAMERA_SOURCE=v4l2 ;;
esac
if [ "$NO_CAPTURE" = 0 ] && [ "${PERCEPTION_CAMERA_PROBE:-1}" != 0 ]; then
    PROBE=${PERCEPTION_CAMERA_PROBE_BINARY:-$APP_DIR/camera-link-probe.aarch64}
    if [ -x "$PROBE" ]; then
        DETECTED=$("$PROBE" --interface "${PERCEPTION_RGB_INTERFACE:-eth0}" --timeout-ms 1800) || {
            printf '%s\n' 'No recognized FPGA camera packets. Check the matching v39 RGB or compatible RAW bitstream, eth0 link and camera power. No RAW/RGB fallback is performed.' >&2; exit 1;
        }
        if [ "$CAMERA_SOURCE" = auto ]; then CAMERA_SOURCE=$DETECTED
        elif [ "$CAMERA_SOURCE" != "$DETECTED" ]; then
            printf 'Camera protocol mismatch: requested=%s detected=%s. RAW requires V4L2/ISP; RGP1 requires fpga-rgb. Remove old PERCEPTION_CAMERA_SOURCE overrides.\n' "$CAMERA_SOURCE" "$DETECTED" >&2; exit 2
        fi
        printf 'CAMERA_LINK detected=%s protocol_verified=1\n' "$DETECTED"
    elif [ "$CAMERA_SOURCE" = auto ]; then
        CAMERA_SOURCE=v4l2
        printf '%s\n' 'CAMERA_LINK probe unavailable; using compatible RAW default. Install camera-link-probe.aarch64 to verify the wire protocol.' >&2
    fi
fi
[ "$CAMERA_SOURCE" != auto ] || CAMERA_SOURCE=v4l2
PERCEPTION_CAMERA_SOURCE=$CAMERA_SOURCE
export PERCEPTION_CAMERA_SOURCE

if [ -z "${MODEL_PATH:-}" ]; then
    MODEL_PATH=$APP_DIR/model/acg138-best-e7f26253-int8-rk3588.rknn
fi
if [ -z "${LABELS_PATH:-}" ]; then
    if [ "$MODEL_PATH" = "$YOLO_DIR/model/yolov8s.rknn" ] && [ -f "$YOLO_DIR/model/coco_80_labels_list.txt" ]; then
        LABELS_PATH=$YOLO_DIR/model/coco_80_labels_list.txt
    else LABELS_PATH=$APP_DIR/model/acg138-best-e7f26253-labels.txt; fi
fi
for directory in "$YOLO_DIR/lib" "$APP_DIR/lib"; do
    if [ -d "$directory" ]; then LD_LIBRARY_PATH=${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$directory; fi
done
if [ -n "${LD_LIBRARY_PATH:-}" ]; then export LD_LIBRARY_PATH; fi

discover_wayland() {
    if [ -n "${WAYLAND_DISPLAY:-}" ]; then
        case "$WAYLAND_DISPLAY" in
            /*) if [ -S "$WAYLAND_DISPLAY" ]; then XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run}; return 0; fi ;;
            *) if [ -S "${XDG_RUNTIME_DIR:-/run}/$WAYLAND_DISPLAY" ]; then
                XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run}; return 0
            fi ;;
        esac
    fi
    for directory in "${XDG_RUNTIME_DIR:-/run}" /run /var/run /run/user/0 /run/user/1000 "${PERCEPTION_WAYLAND_RUNTIME:-/run/robot-wayland}" /tmp; do
        for socket in "$directory"/wayland-*; do
            if [ -S "$socket" ]; then
                XDG_RUNTIME_DIR=$directory; WAYLAND_DISPLAY=${socket##*/}; return 0
            fi
        done
    done
    return 1
}
prepare_wayland() {
    if discover_wayland; then export XDG_RUNTIME_DIR WAYLAND_DISPLAY; return; fi
    if ! command -v weston >/dev/null 2>&1; then
        printf '%s\n' 'No Wayland display or system Weston found. See /tmp/robot-weston.log.' >&2; exit 1
    fi
    # Match the previously delivered no-mouse startup. Keep configuration inside
    # its private runtime directory and never rewrite system weston.ini.
    XDG_RUNTIME_DIR=${PERCEPTION_WAYLAND_RUNTIME:-/run/robot-wayland}
    if [ -L "$XDG_RUNTIME_DIR" ]; then printf '%s\n' 'Unsafe Weston runtime directory.' >&2; exit 1; fi
    (umask 077; mkdir -p "$XDG_RUNTIME_DIR")
    chmod 700 "$XDG_RUNTIME_DIR"
    WAYLAND_DISPLAY=wayland-0
    export XDG_RUNTIME_DIR WAYLAND_DISPLAY
    weston_config=$(mktemp "$XDG_RUNTIME_DIR/weston.XXXXXX")
    printf '%s\n' '[core]' 'require-input=false' 'idle-time=0' '[shell]' 'locking=false' > "$weston_config"
    printf '%s\n' 'Starting system Weston without requiring a mouse...'
    if command -v setsid >/dev/null 2>&1; then
        nohup setsid weston --config="$weston_config" --socket="$WAYLAND_DISPLAY" > /tmp/robot-weston.log 2>&1 </dev/null &
    else
        nohup weston --config="$weston_config" --socket="$WAYLAND_DISPLAY" > /tmp/robot-weston.log 2>&1 </dev/null &
    fi
    weston_pid=$!
    i=0
    while [ "$i" -lt 15 ]; do
        if [ -S "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" ]; then return; fi
        if ! kill -0 "$weston_pid" 2>/dev/null; then
            # A compositor can fork, and a mock may leave a real socket before
            # exiting. Check the published endpoint once more before failing.
            if [ -S "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" ]; then return; fi
            break
        fi
        i=$((i+1)); sleep 1
    done
    printf '%s\n' 'Weston did not open a display within 15 seconds. See /tmp/robot-weston.log.' >&2
    exit 1
}

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

if [ "$NO_CAPTURE" = 0 ]; then prepare_wayland; fi

if [ "$NO_CAPTURE" = 0 ] && [ "$CAMERA_SOURCE" = v4l2 ]; then
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
elif [ "$CAMERA_SOURCE" = fpga-rgb ]; then
    VIDEO=${CLI_CAMERA:-fpga-rgb:${PERCEPTION_RGB_INTERFACE:-eth0}}
    printf 'Using FPGA prepared stereo RGB: %s\nModel: %s\n' "$VIDEO" "$MODEL_PATH"
else
    VIDEO=${CLI_CAMERA:-${CAMERA_DEVICE:-/dev/video41}}
fi
exec "$PERCEPTION_BINARY" \
    --model "$MODEL_PATH" --labels "$LABELS_PATH" --camera "$VIDEO" \
    --pair-token-file "${PAIR_TOKEN_FILE:-$APP_DIR/pair-token.txt}" "$@"
