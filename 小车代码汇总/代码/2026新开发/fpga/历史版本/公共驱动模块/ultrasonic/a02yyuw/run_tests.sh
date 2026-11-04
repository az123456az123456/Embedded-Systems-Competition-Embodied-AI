#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
sources=(top.v a02_uart_rx.v a02_uart_tx.v ultrasonic_sensor.v ultrasonic_driver.v ultrasonic_3ch.v a02yyuw_obstacle_if.v)
for bench in tb_ultrasonic tb_ultrasonic_3ch; do
    "${IVERILOG:-iverilog}" -g2012 -Wall -s "$bench" -o "$build_dir/$bench" "${sources[@]}" "$bench.v"
    "${VVP:-vvp}" "$build_dir/$bench"
done
"${IVERILOG:-iverilog}" -g2012 -Wall -s top -o "$build_dir/top" "${sources[@]}"
if command -v verilator >/dev/null 2>&1; then
    verilator --lint-only --top-module top "${sources[@]}"
    verilator --lint-only --top-module a02yyuw_obstacle_if "${sources[@]}"
else
    echo "SKIP: verilator not installed; Icarus simulations completed"
fi
