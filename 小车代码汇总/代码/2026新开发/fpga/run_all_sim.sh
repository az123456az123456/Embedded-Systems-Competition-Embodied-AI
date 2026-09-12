#!/bin/bash
# ============================================================
# 一键跑通全部 FPGA 模块仿真
# 用法: bash 开发/fpga/run_all_sim.sh
# ============================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$HERE/../../../../../.." && pwd)"

if [[ -n "${IVERILOG:-}" ]]; then
    IVERILOG_BIN="$IVERILOG"
elif [[ -x "$HERE/../工具/eda-env/bin/iverilog" ]]; then
    IVERILOG_BIN="$HERE/../工具/eda-env/bin/iverilog"
elif command -v iverilog >/dev/null 2>&1; then
    IVERILOG_BIN="$(command -v iverilog)"
elif [[ -x "$PROJECT_ROOT/开发/工具/eda-env/bin/iverilog" ]]; then
    IVERILOG_BIN="$PROJECT_ROOT/开发/工具/eda-env/bin/iverilog"
else
    echo "找不到 iverilog，请设置 IVERILOG 或安装仿真工具" >&2
    exit 1
fi

if [[ -n "${VVP:-}" ]]; then
    VVP_BIN="$VVP"
elif [[ -x "$(dirname "$IVERILOG_BIN")/vvp" ]]; then
    VVP_BIN="$(dirname "$IVERILOG_BIN")/vvp"
elif command -v vvp >/dev/null 2>&1; then
    VVP_BIN="$(command -v vvp)"
else
    echo "找不到 vvp，请设置 VVP 或安装仿真工具" >&2
    exit 1
fi

echo "================================================"
echo " FPGA 模块仿真验证 (iverilog)"
echo "================================================"

run_one() {
    local name="$1"; shift
    local files="$*"
    local output
    echo ""
    echo "-------- [$name] 编译+仿真 --------"
    "$IVERILOG_BIN" -g2001 -o "$HERE/$name/sim" $files
    output="$(cd "$HERE/$name" && timeout 60 "$VVP_BIN" sim)"
    echo "$output" | grep -E "PASS|FAIL|=====" | head -20 || true
    if ! grep -q "PASS" <<<"$output"; then
        echo "[$name] 未发现 PASS 输出" >&2
        return 1
    fi
    if grep -q "FAIL" <<<"$output"; then
        echo "[$name] 存在 FAIL" >&2
        return 1
    fi
}

run_extra() {
    local name="$1"
    local workdir="$2"
    local output
    shift 2
    echo ""
    echo "-------- [$name] 编译+仿真 --------"
    "$IVERILOG_BIN" -g2001 -o "$workdir/sim" "$@"
    output="$(cd "$workdir" && timeout 120 "$VVP_BIN" sim)"
    echo "$output" | grep -E "PASS|FAIL|=====" | head -30 || true
    if ! grep -q "PASS" <<<"$output"; then
        echo "[$name] 未发现 PASS 输出" >&2
        return 1
    fi
    if grep -q "FAIL" <<<"$output"; then
        echo "[$name] 存在 FAIL" >&2
        return 1
    fi
}

run_one "uart"       "$HERE/uart/uart_tx.v" "$HERE/uart/uart_rx.v" "$HERE/uart/tb_uart.v"
run_one "encoder"    "$HERE/encoder/quadrature_decoder.v" "$HERE/encoder/tb_quadrature.v"
run_one "motor"      "$HERE/motor/motor_pwm.v" "$HERE/motor/encoder_velocity.v" "$HERE/motor/motor_pid.v" "$HERE/motor/tb_motor.v"
run_one "ultrasonic" "$HERE/ultrasonic/ultrasonic.v" "$HERE/ultrasonic/tb_ultrasonic.v"
run_one "imu"        "$HERE/imu/i2c_master.v" "$HERE/imu/imu_mpu6050.v" "$HERE/imu/tb_imu.v"
run_extra "protocol_v2" "$HERE/protocol" \
    "$HERE/protocol/crc16_byte.v" \
    "$HERE/protocol/protocol_rx.v" \
    "$HERE/protocol/comm_watchdog.v" \
    "$HERE/protocol/twist_to_wheel.v" \
    "$HERE/protocol/tb_protocol_v2.v"
run_one "top"        "$HERE/uart/uart_tx.v" "$HERE/uart/uart_rx.v" \
                     "$HERE/encoder/quadrature_decoder.v" \
                     "$HERE/motor/motor_pwm.v" "$HERE/motor/encoder_velocity.v" "$HERE/motor/motor_pid.v" \
                     "$HERE/ultrasonic/ultrasonic.v" \
                     "$HERE/imu/i2c_master.v" "$HERE/imu/imu_mpu6050.v" \
                     "$HERE/top/top_robot.v" "$HERE/top/tb_top.v"
run_extra "top_v2" "$HERE/top" \
    "$HERE/protocol/crc16_byte.v" \
    "$HERE/protocol/protocol_rx.v" \
    "$HERE/protocol/protocol_tx.v" \
    "$HERE/protocol/comm_watchdog.v" \
    "$HERE/protocol/twist_to_wheel.v" \
    "$HERE/uart/uart_tx.v" "$HERE/uart/uart_rx.v" \
    "$HERE/encoder/quadrature_decoder.v" \
    "$HERE/motor/motor_pwm.v" "$HERE/motor/encoder_velocity.v" "$HERE/motor/motor_pid.v" \
    "$HERE/ultrasonic/ultrasonic.v" \
    "$HERE/imu/i2c_master.v" "$HERE/imu/imu_mpu6050.v" \
    "$HERE/top/top_robot_v2.v" "$HERE/top/tb_top_v2.v"

echo ""
echo "================================================"
echo " 全部完成! 用 GTKWave 看波形:"
echo "   gtkwave 开发/fpga/uart/uart_wave.vcd  (等)"
echo "================================================"
