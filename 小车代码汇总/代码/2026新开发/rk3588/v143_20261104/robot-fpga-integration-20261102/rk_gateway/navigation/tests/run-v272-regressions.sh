#!/bin/sh
# Focused current-contract regressions. No hardware or external network access.
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
CXX=${CXX:-g++}
CC=${CC:-gcc}
FLAGS='-O2 -std=c++17 -Wall -Wextra -pthread'
mkdir -p build reports
make all build/wrong-log.o build/astar-host.o
$CXX $FLAGS -c navcore.cc -o build/navcore-test.o
for name in navcore encoder_window encoder_lifecycle localization_diagnostics straight_assist reactive_avoidance; do
    astar=
    case "$name" in straight_assist|reactive_avoidance) astar=build/astar-host.o ;; esac
    $CXX $FLAGS build/navcore-test.o "tests/test_$name.cc" $astar build/wrong-log.o -o "build/test-$name"
    "./build/test-$name" > "../reports/test-$name.log" 2>&1
    tail -n 1 "../reports/test-$name.log"
done
# These fixtures include the actual Shared runtime, not an imitation gateway.
for name in runtime_profile_state straight_runtime; do
    $CXX $FLAGS build/navcore-test.o imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc "tests/test_$name.cc" build/wrong-log.o -o "build/test-$name"
    "./build/test-$name" > "../reports/test-$name.log" 2>&1
    tail -n 1 "../reports/test-$name.log"
done
python3 tests/test_runtime.py > ../reports/runtime-cli.log 2>&1
cat ../reports/runtime-cli.log
printf '%s\n' 'PASS v27.2 focused production-core and runtime regressions'
