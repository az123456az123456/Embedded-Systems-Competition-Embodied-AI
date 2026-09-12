#!/bin/bash
# ============================================================
# 一键跑通全部 FPGA 模块仿真
# 用法: bash 开发/fpga/run_all_sim.sh
# ============================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
IVERILOG="$HERE/../工具/eda-env/bin/iverilog"
VVP="$HERE/../工具/eda-env/bin/vvp"

echo "================================================"
echo " FPGA 模块仿真验证 (iverilog)"
echo "================================================"

run_one() {
    local name="$1"; shift
    local files="$*"
    echo ""
    echo "-------- [$name] 编译+仿真 --------"
    "$IVERILOG" -o "$HERE/$name/sim" $files
    (cd "$HERE/$name" && timeout 60 "$VVP" sim | grep -E "PASS|FAIL|=====" | head -20)
}

run_one "uart"       "$HERE/uart/uart_tx.v" "$HERE/uart/uart_rx.v" "$HERE/uart/tb_uart.v"
run_one "encoder"    "$HERE/encoder/quadrature_decoder.v" "$HERE/encoder/tb_quadrature.v"
run_one "motor"      "$HERE/motor/motor_pwm.v" "$HERE/motor/encoder_velocity.v" "$HERE/motor/motor_pid.v" "$HERE/motor/tb_motor.v"
run_one "ultrasonic" "$HERE/ultrasonic/ultrasonic.v" "$HERE/ultrasonic/tb_ultrasonic.v"
run_one "imu"        "$HERE/imu/i2c_master.v" "$HERE/imu/imu_mpu6050.v" "$HERE/imu/tb_imu.v"

echo ""
echo "================================================"
echo " 全部完成! 用 GTKWave 看波形:"
echo "   gtkwave 开发/fpga/uart/uart_wave.vcd  (等)"
echo "================================================"
