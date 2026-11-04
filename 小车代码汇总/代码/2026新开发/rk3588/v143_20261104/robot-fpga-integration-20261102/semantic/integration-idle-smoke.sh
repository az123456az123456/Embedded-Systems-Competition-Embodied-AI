#!/bin/sh
# Run only when the existing stack is stopped. No goal or START is submitted.
set -eu
BASE=/root/robot-fpga-integration-20261102
OUT=${SEMANTIC_SMOKE_OUT:-$BASE/semantic/integration-idle-smoke}
mkdir -p "$OUT"
setsid sh "$BASE/semantic/run-semantic.sh" > "$OUT/launcher.log" 2>&1 &
SUPERVISOR=$!
stop() {
    trap - EXIT INT TERM HUP
    kill -TERM "$SUPERVISOR" 2>/dev/null || :
    wait "$SUPERVISOR" 2>/dev/null || :
}
trap stop EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
tick=0
limit=${SEMANTIC_SMOKE_TICKS:-240}
while kill -0 "$SUPERVISOR" 2>/dev/null && [ "$tick" -lt "$limit" ]; do
    available=$(awk '/^MemAvailable:/{print $2}' /proc/meminfo)
    printf '%s %s\n' "$tick" "$available" >> "$OUT/available-kb.tsv"
    if [ -r /sys/kernel/debug/rknpu/load ]; then
        printf '%s ' "$tick" >> "$OUT/npu-load.tsv"
        cat /sys/kernel/debug/rknpu/load >> "$OUT/npu-load.tsv"
    fi
    if [ "$available" -lt 262144 ]; then
        echo 'INSUFFICIENT_MEMORY' >> "$OUT/launcher.log"
        exit 1
    fi
    tick=$((tick+1))
    sleep .5
done
if kill -0 "$SUPERVISOR" 2>/dev/null && [ "$tick" -ge "$limit" ]; then
    echo 'SCHEDULED_TEST_STOP' >> "$OUT/launcher.log"
    stop
    exit 0
fi
wait "$SUPERVISOR"
