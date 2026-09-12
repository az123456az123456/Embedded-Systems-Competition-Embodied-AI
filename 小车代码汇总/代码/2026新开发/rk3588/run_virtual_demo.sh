#!/usr/bin/env bash
# 在没有 FPGA 板卡时，用虚拟 FPGA 跑完整任务 6 闭环。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../../../../.." && pwd)"

if [[ -n "${PYTHON:-}" ]]; then
    PY="$PYTHON"
elif [[ -x "$ROOT/开发/venv/bin/python" ]]; then
    PY="$ROOT/开发/venv/bin/python"
else
    PY="python3"
fi

if [[ -n "${MODEL:-}" ]]; then
    MODEL_PATH="$MODEL"
elif [[ -f "$HERE/yolov8n.pt" ]]; then
    MODEL_PATH="$HERE/yolov8n.pt"
elif [[ -f "$ROOT/开发/.ultralytics/yolov8n.pt" ]]; then
    MODEL_PATH="$ROOT/开发/.ultralytics/yolov8n.pt"
else
    MODEL_PATH="yolov8n.pt"
fi

TMPDIR_DEMO="$(mktemp -d -t task6-virtual-fpga.XXXXXX)"
TTY_FILE="$TMPDIR_DEMO/virtual_fpga.tty"
SIM_LOG="$TMPDIR_DEMO/virtual_fpga.log"
SIM_PID=""

cleanup() {
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        kill "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
    rm -rf "$TMPDIR_DEMO"
}
trap cleanup EXIT INT TERM

"$PY" "$HERE/virtual_fpga.py" \
    --pty-file "$TTY_FILE" \
    --distance-cm "${VIRTUAL_DISTANCE_CM:-200}" \
    --watchdog-ms "${WATCHDOG_MS:-500}" \
    --duration "${SIM_DURATION_SECONDS:-600}" \
    >"$SIM_LOG" 2>&1 &
SIM_PID=$!

for _ in $(seq 1 100); do
    [[ -s "$TTY_FILE" ]] && break
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "虚拟 FPGA 启动失败："
        cat "$SIM_LOG"
        exit 1
    fi
    sleep 0.05
done

if [[ ! -s "$TTY_FILE" ]]; then
    echo "虚拟 FPGA 未生成 PTY 路径"
    cat "$SIM_LOG"
    exit 1
fi

TTY_PATH="$(cat "$TTY_FILE")"
echo "[演示] 虚拟 FPGA 串口: $TTY_PATH"
echo "[演示] 模型: $MODEL_PATH"

if [[ $# -gt 0 ]]; then
    TASK_ARGS=("$@")
else
    TASK_ARGS=(--source 0 --target person --conf 0.25 --imgsz 320)
fi

"$PY" "$HERE/task6_mission.py" \
    --model "$MODEL_PATH" \
    --serial "$TTY_PATH" \
    "${TASK_ARGS[@]}"
