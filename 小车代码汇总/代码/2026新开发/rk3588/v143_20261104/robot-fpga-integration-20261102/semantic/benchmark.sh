#!/bin/sh
# Isolated inference only. No ROS goal, START, or motion command.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REQUESTS=${1:?JSON-lines request file required}
OUT=${2:?output directory required}
mkdir -p "$OUT"
"$BASE/run-worker.sh" < "$REQUESTS" > "$OUT/model.log" 2>&1 &
MODEL_PID=$!
cleanup() { kill -TERM "$MODEL_PID" 2>/dev/null || :; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
(
    ticks=0
    while kill -0 "$MODEL_PID" 2>/dev/null; do
        available=$(awk '/^MemAvailable:/{print $2}' /proc/meminfo)
        printf '%s %s\n' "$ticks" "$available" >> "$OUT/available-kb.tsv"
        if [ "$available" -lt 204800 ] || [ "$ticks" -ge 900 ]; then
            echo 'RESOURCE_OR_TIME_LIMIT' >> "$OUT/model.log"
            kill -TERM "$MODEL_PID" 2>/dev/null || :
            sleep 1
            kill -KILL "$MODEL_PID" 2>/dev/null || :
            break
        fi
        ticks=$((ticks+1))
        sleep 0.1
    done
) &
WATCH_PID=$!
result=0
wait "$MODEL_PID" || result=$?
kill "$WATCH_PID" 2>/dev/null || :
wait "$WATCH_PID" 2>/dev/null || :
trap - EXIT
exit "$result"
