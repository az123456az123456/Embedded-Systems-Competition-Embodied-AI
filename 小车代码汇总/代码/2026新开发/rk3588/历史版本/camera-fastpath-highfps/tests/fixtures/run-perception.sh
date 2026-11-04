#!/bin/sh
# Unified camera + NPU + sensors + phone process. No separate phone server.
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
YOLO_DIR=${YOLO_DIR:-/root/rknn_yolov8_demo}
FIND_VIDEO_SCRIPT=${FIND_VIDEO_SCRIPT:-/root/find-video.sh}
ISP_CAMERA_SCRIPT=${ISP_CAMERA_SCRIPT:-/opt/fpga-camera-isp-1080/run_isp_camera.sh}
ISP_LOG=${ISP_LOG:-/tmp/perception-isp.log}
PREPARE_ISP=${PERCEPTION_PREPARE_ISP:-auto}
WAIT_ATTEMPTS=${PERCEPTION_VIDEO_WAIT_ATTEMPTS:-30}
CAMERA_TARGET_FPS=${CAMERA_TARGET_FPS:-30}
case "$CAMERA_TARGET_FPS" in 15|20|30) ;; *) printf '%s\n' 'CAMERA_TARGET_FPS must be 15, 20 or 30' >&2; exit 2 ;; esac
export CAMERA_TARGET_FPS

# Preserve native argument order: explicit CLI values follow the defaults.
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
        --model|--labels|--view|--interface|--imu-interface|--motor-interface|--sensor-interface|--width|--height|--confidence|--nms|--seconds|--npu-workers|--rotate|--imu-deadband|--phone-port|--phone-bind|--pair-token-file|--left-mask|--invert-mask|--phone-fps|--page) NEXT=value ;;
    esac
done
if [ -n "$NEXT" ]; then printf '%s\n' 'Missing command-line option value' >&2; exit 2; fi

MODEL_PATH=${MODEL_PATH:-$APP_DIR/model/yolo11s-int8-rk3588.rknn}
LABELS_PATH=${LABELS_PATH:-$APP_DIR/model/coco_80_labels_list.txt}
PAIR_TOKEN_FILE=${PAIR_TOKEN_FILE:-$APP_DIR/pair-token.txt}
# Prefer the bundled matching runtime; keep the original board libraries last.
LIB_PATH=
for directory in "$APP_DIR/lib" "$YOLO_DIR/lib"; do
    if [ -d "$directory" ]; then LIB_PATH=${LIB_PATH:+$LIB_PATH:}$directory; fi
done
LD_LIBRARY_PATH=${LIB_PATH}${LD_LIBRARY_PATH:+${LIB_PATH:+:}$LD_LIBRARY_PATH}
export LD_LIBRARY_PATH
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
    printf '%s\n' 'Program not found. Reinstall the matching RK3588 restore package.' >&2
    exit 1
fi

# Lock only real capture sessions. Never delete a live PID's lock, an unknown
# directory, or another installation's metadata. No rm -rf is used on /tmp.
LOCK_DIR=/tmp/robot-perception.lock
LOCKED=0
CHILD_PID=
cleanup() {
    if [ "$LOCKED" = 1 ] && [ ! -L "$LOCK_DIR" ] &&
       [ "$(cat "$LOCK_DIR/pid" 2>/dev/null || true)" = "$$" ] &&
       [ "$(cat "$LOCK_DIR/app" 2>/dev/null || true)" = "$APP_DIR" ]; then
        rm -f "$LOCK_DIR/pid" "$LOCK_DIR/app" "$LOCK_DIR/uid" "$LOCK_DIR/child_pid"
        rmdir "$LOCK_DIR" 2>/dev/null || true
    fi
}
stop_child() {
    code=$1
    if [ -n "$CHILD_PID" ]; then
        kill -TERM "$CHILD_PID" 2>/dev/null || true
        wait "$CHILD_PID" 2>/dev/null || true
    fi
    exit "$code"
}
acquire_lock() {
    uid=$(id -u)
    retries=0
    while ! (umask 077; mkdir "$LOCK_DIR") 2>/dev/null; do
        if [ -L "$LOCK_DIR" ] || [ ! -d "$LOCK_DIR" ]; then
            printf 'Unsafe or invalid lock path: %s\n' "$LOCK_DIR" >&2; exit 1
        fi
        old_pid=$(cat "$LOCK_DIR/pid" 2>/dev/null || true)
        old_app=$(cat "$LOCK_DIR/app" 2>/dev/null || true)
        old_uid=$(cat "$LOCK_DIR/uid" 2>/dev/null || true)
        # Another launcher may still be writing its three metadata files.
        if [ -z "$old_pid" ] || [ -z "$old_app" ] || [ -z "$old_uid" ]; then
            retries=$((retries+1))
            if [ "$retries" -lt 3 ]; then sleep 1; continue; fi
            printf 'Lock metadata is incomplete: %s. Inspect it before removing it.\n' "$LOCK_DIR" >&2; exit 1
        fi
        case "$old_pid" in ''|*[!0-9]*|0|1) printf '%s\n' 'Invalid lock PID; refusing to remove lock.' >&2; exit 1 ;; esac
        if kill -0 "$old_pid" 2>/dev/null; then
            printf 'Another unified dashboard is active (PID %s). Exit it first.\n' "$old_pid" >&2; exit 1
        fi
        old_child=$(cat "$LOCK_DIR/child_pid" 2>/dev/null || true)
        if [ -n "$old_child" ]; then
            case "$old_child" in *[!0-9]*|0|1) printf '%s\n' 'Invalid child PID; refusing to remove lock.' >&2; exit 1 ;; esac
            if kill -0 "$old_child" 2>/dev/null; then
                printf 'Dashboard child is still active (PID %s). Exit it before restarting.\n' "$old_child" >&2; exit 1
            fi
        fi
        if [ "$old_uid" != "$uid" ] || [ "$old_app" != "$APP_DIR" ]; then
            printf 'A stale lock belongs to another user or installation: %s\n' "$LOCK_DIR" >&2; exit 1
        fi
        # Claim recovery inside the old directory so two launchers cannot
        # rename/remove a newly acquired lock after observing the same dead PID.
        if mkdir "$LOCK_DIR/recover" 2>/dev/null; then
            if [ "$(cat "$LOCK_DIR/pid" 2>/dev/null || true)" = "$old_pid" ] &&
               [ "$(cat "$LOCK_DIR/app" 2>/dev/null || true)" = "$APP_DIR" ] &&
               [ "$(cat "$LOCK_DIR/uid" 2>/dev/null || true)" = "$uid" ] &&
               ! kill -0 "$old_pid" 2>/dev/null; then
                rm -f "$LOCK_DIR/pid" "$LOCK_DIR/app" "$LOCK_DIR/uid" "$LOCK_DIR/child_pid"
            fi
            rmdir "$LOCK_DIR/recover" 2>/dev/null || true
            rmdir "$LOCK_DIR" 2>/dev/null || true
        else
            sleep 1
        fi
        retries=$((retries+1))
        if [ "$retries" -ge 5 ]; then printf '%s\n' 'Could not acquire dashboard lock.' >&2; exit 1; fi
    done
    LOCKED=1
    trap cleanup 0
    trap 'stop_child 130' INT
    trap 'stop_child 143' TERM
    trap 'stop_child 129' HUP
    printf '%s\n' "$$" > "$LOCK_DIR/pid"
    printf '%s\n' "$APP_DIR" > "$LOCK_DIR/app"
    printf '%s\n' "$uid" > "$LOCK_DIR/uid"
}

discover_wayland() {
    if [ -n "${WAYLAND_DISPLAY:-}" ]; then
        case "$WAYLAND_DISPLAY" in
            /*) if [ -S "$WAYLAND_DISPLAY" ]; then return 0; fi ;;
            *) if [ -S "${XDG_RUNTIME_DIR:-/run}/$WAYLAND_DISPLAY" ]; then
                XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run}; return 0
            fi ;;
        esac
    fi
    for directory in "${XDG_RUNTIME_DIR:-/run}" /run /var/run /run/user/0 /run/user/1000 /run/robot-wayland /tmp; do
        for socket in "$directory"/wayland-*; do
            if [ -S "$socket" ]; then
                XDG_RUNTIME_DIR=$directory
                WAYLAND_DISPLAY=${socket##*/}
                return 0
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
    # Use a private temporary configuration. No output mode/rotation is forced,
    # and the system /etc/xdg/weston/weston.ini is never changed.
    XDG_RUNTIME_DIR=/run/robot-wayland
    if [ -L "$XDG_RUNTIME_DIR" ]; then printf '%s\n' 'Unsafe Weston runtime directory.' >&2; exit 1; fi
    (umask 077; mkdir -p "$XDG_RUNTIME_DIR")
    chmod 700 "$XDG_RUNTIME_DIR"
    WAYLAND_DISPLAY=wayland-0
    export XDG_RUNTIME_DIR WAYLAND_DISPLAY
    weston_config=$(mktemp /tmp/robot-weston.XXXXXX)
    printf '%s\n' '[core]' 'require-input=false' 'idle-time=0' '[shell]' 'locking=false' > "$weston_config"
    printf '%s\n' 'Starting system Weston without requiring a mouse...'
    # require-input is a weston.ini setting. Do not pass the non-portable
    # --continue-without-input flag to the board's compositor.
    if command -v setsid >/dev/null 2>&1; then
        nohup setsid weston --config="$weston_config" --socket="$WAYLAND_DISPLAY" > /tmp/robot-weston.log 2>&1 </dev/null &
    else
        nohup weston --config="$weston_config" --socket="$WAYLAND_DISPLAY" > /tmp/robot-weston.log 2>&1 </dev/null &
    fi
    weston_pid=$!
    i=0
    while [ "$i" -lt 15 ]; do
        if [ -S "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" ]; then return; fi
        if ! kill -0 "$weston_pid" 2>/dev/null; then break; fi
        i=$((i+1)); sleep 1
    done
    printf '%s\n' 'Weston did not open a display within 15 seconds. See /tmp/robot-weston.log.' >&2
    exit 1
}

if [ "$NO_CAPTURE" = 0 ]; then
    if [ "$(id -u)" != 0 ]; then printf '%s\n' 'Run as root for FPGA raw Ethernet and camera access.' >&2; exit 1; fi
    acquire_lock
    if [ ! -f "$APP_DIR/scripts/check-board.sh" ]; then printf '%s\n' 'Missing scripts/check-board.sh; reinstall the restore package.' >&2; exit 1; fi
    sh "$APP_DIR/scripts/check-board.sh" --run
    case "$WAIT_ATTEMPTS" in ''|*[!0-9]*|0) printf '%s\n' 'Invalid video wait attempts' >&2; exit 2 ;; esac
    if [ "$WAIT_ATTEMPTS" -gt 300 ]; then printf '%s\n' 'Video wait attempts exceeds 300' >&2; exit 2; fi
    case "$PREPARE_ISP" in
        auto) if [ -f "$ISP_CAMERA_SCRIPT" ]; then PREPARE_ISP=1; else PREPARE_ISP=0; fi ;;
        0|1) ;;
        *) printf '%s\n' 'PERCEPTION_PREPARE_ISP must be auto, 0 or 1' >&2; exit 2 ;;
    esac
    prepare_wayland
    if [ "$PREPARE_ISP" = 1 ]; then
        if [ ! -f "$ISP_CAMERA_SCRIPT" ]; then printf 'ISP camera script not found: %s\n' "$ISP_CAMERA_SCRIPT" >&2; exit 1; fi
        if ! command -v v4l2-ctl >/dev/null 2>&1; then printf '%s\n' 'v4l2-ctl is required to check FPGA camera readiness' >&2; exit 1; fi
        if ! ps -ef | grep '[i]sp-probe' >/dev/null 2>&1; then
            printf '%s\n' 'Preparing FPGA dual-1080 camera...'
            (
                export ISP_DISPLAY=0
                if command -v setsid >/dev/null 2>&1; then exec nohup setsid sh "$ISP_CAMERA_SCRIPT" dual-1080 100000
                else exec nohup sh "$ISP_CAMERA_SCRIPT" dual-1080 100000; fi
            ) > "$ISP_LOG" 2>&1 </dev/null &
        fi
    fi
    if [ ! -r "$FIND_VIDEO_SCRIPT" ] && [ -r "$APP_DIR/scripts/find-video.sh" ]; then FIND_VIDEO_SCRIPT=$APP_DIR/scripts/find-video.sh; fi
    VIDEO=
    i=0
    while [ "$i" -lt "$WAIT_ATTEMPTS" ]; do
        if [ -n "$CLI_CAMERA" ]; then VIDEO=$CLI_CAMERA
        elif [ -n "${CAMERA_DEVICE:-}" ]; then VIDEO=$CAMERA_DEVICE
        elif [ -r "$FIND_VIDEO_SCRIPT" ]; then VIDEO=$(sh "$FIND_VIDEO_SCRIPT" || true)
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
        printf 'FPGA camera not ready (last node: %s). Check %s or pass --camera /dev/videoN.\n' "${VIDEO:-none}" "$ISP_LOG" >&2; exit 1
    fi
    printf 'Using camera: %s\nModel: %s\n' "$VIDEO" "$MODEL_PATH"
else
    VIDEO=${CLI_CAMERA:-${CAMERA_DEVICE:-/dev/video41}}
fi

if [ "$NO_CAPTURE" = 1 ]; then
    exec "$PERCEPTION_BINARY" --model "$MODEL_PATH" --labels "$LABELS_PATH" --camera "$VIDEO" \
        --pair-token-file "$PAIR_TOKEN_FILE" --npu-workers "${PERCEPTION_NPU_WORKERS:-3}" \
        --left-mask 10 --invert-mask 15 "$@"
fi
"$PERCEPTION_BINARY" --model "$MODEL_PATH" --labels "$LABELS_PATH" --camera "$VIDEO" \
    --pair-token-file "$PAIR_TOKEN_FILE" --npu-workers "${PERCEPTION_NPU_WORKERS:-3}" \
    --left-mask 10 --invert-mask 15 "$@" &
CHILD_PID=$!
printf '%s\n' "$CHILD_PID" > "$LOCK_DIR/child_pid"
status=0
wait "$CHILD_PID" || status=$?
CHILD_PID=
exit "$status"
