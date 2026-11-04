#!/bin/sh
# One dashboard at a time; never arm, send motion commands, or kill an existing app.
set -eu
LC_ALL=C
export LC_ALL
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
APP=${PERCEPTION_APP_DIR:-/root/robot-perception}
LAUNCHER=${BENCHMARK_LAUNCHER:-/root/perception-dashboard/run-dashboard.sh}
OUT=${BENCHMARK_OUTPUT:-/root/perception-benchmark-v35-$(date +%Y%m%d-%H%M%S)-$$}
DURATION=60
WARMUP=10
STARTUP=120
RGA=0
HEADLESS=0
MODEL=${MODEL_PATH:-$APP/model/acg138-best-e7f26253-int8-rk3588.rknn}
LABELS=${LABELS_PATH:-$APP/model/acg138-best-e7f26253-labels.txt}
while [ "$#" -gt 0 ];do
    case "$1" in
        --include-rga) RGA=1;shift ;;
        --headless) HEADLESS=1;shift ;;
        --duration|--warmup|--launcher|--output|--model|--labels)
            key=$1;shift;[ "$#" -gt 0 ] || { echo "Missing value: $key" >&2;exit 2; }
            case "$key" in
                --duration) DURATION=$1;; --warmup) WARMUP=$1;; --launcher) LAUNCHER=$1;;
                --output) OUT=$1;; --model) MODEL=$1;; --labels) LABELS=$1;;
            esac;shift ;;
        -h|--help)
            printf '%s\n' 'Usage: sh benchmark-perception-v35.sh [--model FILE --labels FILE] [--include-rga]' \
                '  Default: FPGA RGB with detection, then without detection; same 640-square source.' \
                '  Each run: wait for first PERF, warm up 10 s, measure 60 s; stop only the child started here.' \
                '  Phone and navigation are disabled. Park the car before running.' \
                '  --headless isolates vision from the HDMI compositor, for all cases equally.' \
                '  --duration N --warmup N --launcher FILE --output DIRECTORY' \
                '  Output: full logs, measured PERF/STAGE logs, metadata and a short summary.'
            exit 0 ;;
        *) echo "Unknown option: $1" >&2;exit 2 ;;
    esac
done
for value in "$DURATION" "$WARMUP";do
    case "$value" in ''|*[!0-9]*) echo 'Duration/warmup must be integer seconds.' >&2;exit 2;; esac
done
[ "$DURATION" -ge 1 ] && [ "$DURATION" -le 600 ] && [ "$WARMUP" -le 120 ] || { echo 'Duration 1..600, warmup 0..120.' >&2;exit 2; }
[ -f "$LAUNCHER" ] || { echo "Launcher missing: $LAUNCHER" >&2;exit 2; }
if [ -n "$MODEL" ];then [ -f "$MODEL" ] || { echo "Model missing: $MODEL" >&2;exit 2; };fi
if [ -n "$LABELS" ];then [ -f "$LABELS" ] || { echo "Labels missing: $LABELS" >&2;exit 2; };fi
LOCK=/tmp/perception-benchmark-v35.lock
mkdir "$LOCK" 2>/dev/null || { echo "Another benchmark is active, or its lock remains: $LOCK" >&2;exit 1; }
child=
cleanup(){
    trap - EXIT INT TERM HUP
    if [ -n "$child" ];then kill -TERM "$child" 2>/dev/null || :;wait "$child" 2>/dev/null || :;fi
    rmdir "$LOCK" 2>/dev/null || :
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
assert_idle(){
    for entry in /proc/[0-9]*;do
        [ -r "$entry/comm" ] || continue
        IFS= read -r name < "$entry/comm" || continue
        case "$name" in perception-dash*|robot-navigatio*|rknn_yolov*|rknn_yolo11*)
            echo "A perception/navigation program is already active: pid=${entry##*/} name=$name. Exit it first; nothing was killed." >&2;exit 1;;
        esac
    done
}
assert_idle
umask 077
mkdir -p "$OUT"
echo 'Park the car. This benchmark starts no phone service or navigation and sends no drive commands.'
printf 'Logs: %s\n' "$OUT"
{
    printf 'format=perception-v35-ab-v1\nduration_s=%s\nwarmup_s=%s\nheadless=%s\nlauncher=%s\n' "$DURATION" "$WARMUP" "$HEADLESS" "$LAUNCHER"
    printf 'same_model_all_cases=yes\nsame_scheduler_all_cases=v35\nphone=off\nnavigation=off\n'
    printf 'kernel=';uname -r
    printf 'machine=';uname -m
    printf 'cpu_frequency_settings=unchanged\n'
    [ -z "$MODEL" ] || { printf 'model_sha256=';sha256sum "$MODEL"; }
    [ -z "$LABELS" ] || { printf 'labels_sha256=';sha256sum "$LABELS"; }
} > "$OUT/metadata.txt"
cases='fpga-rgb-detect fpga-rgb-no-detect'
[ "$RGA" = 0 ] || { echo "RGA A/B is unavailable for FPGA-preprocessed tensors." >&2; exit 2; }
expected_model=$(sha256sum "$MODEL");expected_model=${expected_model%% *}
for mode in $cases;do
    assert_idle
    prep=identity;input=auto
    printf 'Running %s...\n' "$mode"
    full=$OUT/$mode.full.log
    measured=$OUT/$mode.measured.log
    guard=$((STARTUP+WARMUP+DURATION+30))
    set -- -0 --no-phone --no-navigation --seconds "$guard"
    [ "$HEADLESS" = 0 ] || set -- "$@" --headless
    [ -z "$MODEL" ] || set -- "$@" --model "$MODEL"
    [ -z "$LABELS" ] || set -- "$@" --labels "$LABELS"
    [ "$mode" != fpga-rgb-no-detect ] || set -- "$@" --no-detect
    PERCEPTION_CAMERA_SOURCE=fpga-rgb PERCEPTION_PREPROCESS=cpu PERCEPTION_INPUT=auto sh "$LAUNCHER" "$@" > "$full" 2>&1 </dev/null &
    child=$!
    waited=0
    until grep '^PERF ' "$full" >/dev/null 2>&1;do
        kill -0 "$child" 2>/dev/null || { echo "$mode exited before its first PERF. See $full" >&2;exit 1; }
        [ "$waited" -lt "$STARTUP" ] || { echo "$mode produced no PERF within ${STARTUP}s. See $full" >&2;exit 1; }
        sleep 1;waited=$((waited+1))
    done
    [ "$WARMUP" = 0 ] || sleep "$WARMUP"
    kill -0 "$child" 2>/dev/null || { echo "$mode exited during warmup." >&2;exit 1; }
    start_line=$(wc -l < "$full")
    start_line=$((start_line+1))
    sleep "$DURATION"
    kill -0 "$child" 2>/dev/null || { echo "$mode exited before measurement completed." >&2;exit 1; }
    end_line=$(wc -l < "$full")
    # Stop the child we created, never a process found via a name match.
    kill -TERM "$child" 2>/dev/null || :
    wait "$child" 2>/dev/null || :;child=
    sed -n "${start_line},${end_line}p" "$full" > "$measured"
    grep '^PERF ' "$measured" >/dev/null 2>&1 || { echo "$mode has no measured PERF windows; use a longer duration." >&2;exit 1; }
    printf 'case=%s preprocess=%s input=%s lines=%s..%s\n' "$mode" "$prep" "$input" "$start_line" "$end_line" >> "$OUT/metadata.txt"
    grep -E '^CONFIG |^MODEL |^INPUT |^PREPROCESS ' "$full" >> "$OUT/metadata.txt" || :
    # Verify the actual loaded file, including launcher defaults, instead of
    # assuming a filename implies a model identity.
    actual_model=$(sed -n 's/^CONFIG model=\([^ ]*\) .*/\1/p' "$full" | head -n 1)
    [ -n "$actual_model" ] && [ -f "$actual_model" ] || { echo "$mode did not identify its loaded model in CONFIG." >&2;exit 1; }
    actual_sha=$(sha256sum "$actual_model");actual_sha=${actual_sha%% *}
    [ "$actual_sha" = "$expected_model" ] || { echo "$mode loaded a different model; A/B is invalid." >&2;exit 1; }
    printf 'case_model_sha256=%s %s %s\n' "$mode" "$actual_sha" "$actual_model" >> "$OUT/metadata.txt"
done
set -- "$OUT"/*.measured.log
awk '
    /^PERF / {
        n[FILENAME]++
        for(i=2;i<=NF;i++){split($i,a,"=");if(a[1] ~ /^(capture_fps|result_fps|eye_fps|inference_ms|frame_age_ms|box_age_ms)$/){sum[FILENAME,a[1]]+=a[2]}}
    }
    END {
        print "Means of logged PERF windows; complete stereo pair FPS is result_fps, not eye_fps."
        for(file in n){printf "%s windows=%d",file,n[file];split("capture_fps result_fps eye_fps inference_ms frame_age_ms box_age_ms",keys," ");for(k=1;k<=6;k++)printf " %s=%.2f",keys[k],sum[file,keys[k]]/n[file];print ""}
        print "Full worker/identity-copy/queue/skip details remain in logs. Both cases use FPGA RGB."
        print "No hardware speed is claimed until this script is run on the board."
    }
' "$@" > "$OUT/summary.txt"
cat "$OUT/summary.txt"
printf 'Copy this directory to the computer for detailed JSON: python3 summarize-perception-v35.py %s/*.measured.log\n' "$OUT"
