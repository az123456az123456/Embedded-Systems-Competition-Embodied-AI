#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build reports
CCACHE_DISABLE=1 CCACHE_TEMPDIR=/tmp verilator --binary --timing -Wno-fatal \
    -GSENSOR_PERIOD_CYCLES=50000 --top-module tb_scheduler_planner_stress --Mdir build/sonar_stress_verilator \
    -CFLAGS '-std=c++20' camera/UDP_Send.v camera/CRC32_D8.v imu/imu_ethernet_tx.v \
    rtl/motor_status_tx.v sensors/sensor_status_tx.v rtl/nav_message_tx.v \
    tests/tb_scheduler_planner_stress.v > build/sonar_stress_compile.log 2>&1
build/sonar_stress_verilator/Vtb_scheduler_planner_stress > reports/sonar_scheduler_v18.log 2>&1
cat reports/sonar_scheduler_v18.log
