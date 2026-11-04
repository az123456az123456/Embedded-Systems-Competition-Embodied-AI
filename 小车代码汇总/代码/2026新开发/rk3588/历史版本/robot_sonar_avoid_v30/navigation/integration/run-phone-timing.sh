#!/bin/sh
# Workspace integration test, never executed by the board installer.
# Android source is the unchanged APK 1.2.11 tree; only framework/JSON stubs are host adapters.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
NAV=$(CDPATH= cd -- "$HERE/.." && pwd)
MODE=${1:-new}
case "$MODE" in old) SOURCE_NAV=$(CDPATH= cd -- "$NAV/../../robot_phone_v26_1/navigation" && pwd);; new) SOURCE_NAV=$NAV;; *) echo 'Usage: run-phone-timing.sh old|new [APK-1.2.11-source-root]' >&2; exit 2;; esac
ANDROID=${2:-"$NAV/../../robot_phone_v26/navigation/android"}
ANDROID=$(CDPATH= cd -- "$ANDROID" && pwd)
BUILD=$(mktemp -d /tmp/acg138-phone-timing.XXXXXX)
cleanup() { if [ -n "${FIXTURE_PID:-}" ]; then kill "$FIXTURE_PID" 2>/dev/null || true; fi; rm -rf "$BUILD"; }
trap cleanup EXIT HUP INT TERM
mkdir -p "$BUILD/classes"
g++ -std=c++17 -O2 -pthread "$SOURCE_NAV/gateway.cc" "$SOURCE_NAV/vendor/http_server.cc" "$SOURCE_NAV/vendor/json_flat.cc" "$HERE/gateway_phone_timing_fixture.cc" "$NAV/../common/wrong_log.cc" -o "$BUILD/gateway-fixture"
find "$ANDROID/tests/host-stubs" -name '*.java' ! -path '*/cn/acg138/robotphone/RobotClient.java' ! -path '*/android/graphics/BitmapFactory.java' > "$BUILD/java-sources.txt"
find "$ANDROID/tests/network-stubs" -name '*.java' >> "$BUILD/java-sources.txt"
for name in ArmHandshake FollowPresentation ControlPolicy ConnectionPolicy SelectionGate LanAddress MultipartReader RobotClient NavigationMap NavigationTarget NavigationMapClient; do
    printf '%s\n' "$ANDROID/src/cn/acg138/robotphone/$name.java" >> "$BUILD/java-sources.txt"
done
printf '%s\n' "$HERE/ProductionPhoneTiming.java" >> "$BUILD/java-sources.txt"
javac -encoding UTF-8 --release 8 -d "$BUILD/classes" @"$BUILD/java-sources.txt"
"$BUILD/gateway-fixture" "$BUILD" > "$BUILD/native.log" 2>&1 &
FIXTURE_PID=$!
i=0
while [ ! -s "$BUILD/port" ] && [ "$i" -lt 500 ]; do sleep .01; i=$((i+1)); done
if [ ! -s "$BUILD/port" ]; then cat "$BUILD/native.log"; echo 'native fixture did not start' >&2; exit 1; fi
if ! timeout 10 java -Djava.awt.headless=true -cp "$BUILD/classes" ProductionPhoneTiming "$BUILD" "$MODE"; then cat "$BUILD/native.log"; exit 1; fi
wait "$FIXTURE_PID"
FIXTURE_PID=
cat "$BUILD/native.log"
