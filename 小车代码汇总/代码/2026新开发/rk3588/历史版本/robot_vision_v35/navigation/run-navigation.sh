#!/bin/sh
# v23: start the camera/dashboard + map gateway, never an autonomous mission.
set -eu
exec 3<&0
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
APP_DIR=${NAV_PERCEPTION_DIR:-/root/robot-perception}
APP=${NAV_DASHBOARD_BINARY:-$APP_DIR/perception-dashboard.aarch64}
RAW=${NAV_PERCEPTION_LAUNCHER:-$APP_DIR/run-perception.sh}
TOKEN=${NAV_PAIR_TOKEN:-$APP_DIR/pair-token.txt}
CONFIG=${NAV_CONFIG:-$HERE/config.json}
LOCK=${NAV_LAUNCH_LOCK:-/tmp/robot-navigation-launcher-v23.lock}
DASH_LOG=${NAV_DASHBOARD_LOG:-/tmp/robot-nav-dashboard.log}
CHECK_LOG=${NAV_CONFIG_LOG:-/tmp/robot-nav-config-check.log}
rotation=0
mode=auto
standalone=0
custom_phone=0
interface=
case "${1:-}" in
    -0) rotation=0;shift ;;
    -90) rotation=90;shift ;;
    -[0-9]*) echo 'Select -0 or -90 as the first argument.' >&2;exit 2 ;;
esac
# Retain argument boundaries while removing only supervisor-specific options.
remaining=$#
while [ "$remaining" -gt 0 ]; do
    argument=$1;shift;remaining=$((remaining-1))
    case "$argument" in
        --drive) mode=drive ;;
        --observe) mode=observe ;;
        --no-navigation) standalone=1 ;;
        -h|--help)
            printf '%s\n' 'Usage: sh run-dashboard.sh [-0|-90] [camera/inference options]' \
                'Starts YOLO/IMU/sonar, map and phone navigation together.' \
                'Use phone Start navigation / End navigation; startup never starts driving.' \
                '--observe: map/manual control only; --drive: require valid motion configuration.' \
                '--no-phone or --no-navigation: run the original dashboard without navigation.'
            exit 0 ;;
        --config|--interface)
            [ "$remaining" -gt 0 ] || { echo "Missing value: $argument" >&2;exit 2; }
            value=$1;shift;remaining=$((remaining-1))
            if [ "$argument" = --config ]; then CONFIG=$value
            else interface=$value;set -- "$@" --interface "$value";fi ;;
        --phone-bind|--phone-port)
            [ "$remaining" -gt 0 ] || { echo "Missing value: $argument" >&2;exit 2; }
            value=$1;shift;remaining=$((remaining-1));custom_phone=1
            set -- "$@" "$argument" "$value" ;;
        --preview|--model|--labels|--view|--camera|--imu-interface|--motor-interface|--width|--height|--confidence|--nms|--seconds|--npu-workers|--rotate|--imu-deadband|--pair-token-file|--left-mask|--invert-mask|--phone-fps|--page)
            [ "$remaining" -gt 0 ] || { echo "Missing value: $argument" >&2;exit 2; }
            value=$1;shift;remaining=$((remaining-1))
            [ "$argument" != --preview ] || standalone=1
            set -- "$@" "$argument" "$value" ;;
        --no-phone|--demo|--version)
            standalone=1;set -- "$@" "$argument" ;;
        *) set -- "$@" "$argument" ;;
    esac
done
if [ "$standalone" != 1 ] && [ "$custom_phone" = 1 ];then
    echo 'The unified launcher manages port 8080. Use --no-navigation for custom dashboard ports.' >&2
    exit 2
fi
[ -f "$RAW" ] && [ -x "$APP" ] || { echo 'Camera launcher or dashboard executable is missing.' >&2;exit 1; }
PERCEPTION_BINARY=$APP
PAIR_TOKEN_FILE=$TOKEN
export PERCEPTION_BINARY PAIR_TOKEN_FILE
if [ "$standalone" = 1 ]; then
    exec sh "$RAW" --fullscreen --rotate "$rotation" "$@" 3<&-
fi
[ "$(id -u)" = 0 ] || { echo 'Run as root on the RK3588.' >&2;exit 1; }
[ -x "$HERE/robot-navigation.aarch64" ] && [ -f "$CONFIG" ] && [ -f "$HERE/dashboard.sha256" ] || {
    echo 'Navigation runtime/configuration is missing. Install the complete v23 update.' >&2;exit 1;
}
expected=$(cat "$HERE/dashboard.sha256")
case "$expected" in ''|*[!0-9a-f]*) echo 'Invalid dashboard pairing checksum.' >&2;exit 1 ;; esac
[ "${#expected}" = 64 ] || { echo 'Invalid dashboard pairing checksum length.' >&2;exit 1; }
actual=$(sha256sum "$APP");actual=${actual%% *}
[ "$actual" = "$expected" ] || { echo 'Dashboard does not match the installed navigation pair.' >&2;exit 1; }
umask 077
mkdir "$LOCK" 2>/dev/null || {
    echo "A unified launcher is already running, or an interrupted lock remains: $LOCK" >&2
    exit 1
}
printf '%s\n' "$$" > "$LOCK/pid"
dash_pid=
nav_pid=
monitor_pid=
log_pid=
cleanup() {
    trap - EXIT INT TERM HUP
    if [ -n "$nav_pid" ]; then kill -TERM "$nav_pid" 2>/dev/null || :;wait "$nav_pid" 2>/dev/null || :;fi
    if [ -n "$monitor_pid" ]; then kill -TERM "$monitor_pid" 2>/dev/null || :;wait "$monitor_pid" 2>/dev/null || :;fi
    if [ -n "$dash_pid" ]; then kill -TERM "$dash_pid" 2>/dev/null || :;wait "$dash_pid" 2>/dev/null || :;fi
    if [ -n "$log_pid" ]; then kill -TERM "$log_pid" 2>/dev/null || :;wait "$log_pid" 2>/dev/null || :;fi
    rm -f "$LOCK/pid";rmdir "$LOCK" 2>/dev/null || :
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
if "$HERE/robot-navigation.aarch64" --config "$CONFIG" --check-config > "$CHECK_LOG" 2>&1;then
    ready=1
else
    result=$?
    ready=0
    if [ "$mode" = drive ];then cat "$CHECK_LOG" >&2;exit "$result";fi
    if [ "$result" != 2 ];then
        echo "Navigation configuration is unreadable; starting camera/manual control only. See $CHECK_LOG" >&2
        # Release our supervisor lock before replacing this shell.
        rm -f "$LOCK/pid";rmdir "$LOCK";trap - EXIT INT TERM HUP
        exec sh "$RAW" --fullscreen --rotate "$rotation" "$@" 3<&-
    fi
fi
drive=
if [ "$mode" != observe ] && [ "$ready" = 1 ];then drive=--drive
else echo "Navigation motion unavailable until measured control calibration is valid. Map and manual control remain available. See $CHECK_LOG";fi
# Call the existing raw camera launcher directly: never recurse via run-dashboard.
sh "$RAW" --fullscreen --rotate "$rotation" "$@" --phone-bind 127.0.0.1 --phone-port 8082 > "$DASH_LOG" 2>&1 < /dev/null 3<&- &
dash_pid=$!
# Preserve the camera/PERF output users expect from the original entry point.
tail -n +1 -f "$DASH_LOG" 3<&- &
log_pid=$!
i=0
while [ ! -s "$TOKEN" ];do
    kill -0 "$dash_pid" 2>/dev/null || { echo "Dashboard startup failed. See $DASH_LOG" >&2;exit 1; }
    [ "$i" -lt 45 ] || { echo "Pairing token not ready. See $DASH_LOG" >&2;exit 1; }
    sleep 1;i=$((i+1))
done
set -- --config "$CONFIG" --token-file "$TOKEN" --backend-port 8082 --port 8080
[ -z "$drive" ] || set -- "$@" --drive
[ -z "$interface" ] || set -- "$@" --interface "$interface"
echo 'Unified v23 ready: phone address/code unchanged; start and end navigation on the phone.'
"$HERE/robot-navigation.aarch64" "$@" <&3 3<&- &
nav_pid=$!
exec 3<&-
(
    while kill -0 "$nav_pid" 2>/dev/null;do
        if ! kill -0 "$dash_pid" 2>/dev/null;then
            echo "Dashboard exited; stopping navigation. See $DASH_LOG" >&2
            kill -TERM "$nav_pid" 2>/dev/null || :
            exit 1
        fi
        sleep 1
    done
) &
monitor_pid=$!
set +e
wait "$nav_pid"
result=$?
set -e
nav_pid=
if ! kill -0 "$dash_pid" 2>/dev/null && [ "$result" = 0 ];then result=1;fi
exit "$result"
