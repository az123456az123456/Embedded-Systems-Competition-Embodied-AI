#!/bin/sh
# Production RobotClient thread and HTTP flow through a host-only in-memory transport.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BUILD="$HERE/build/client-host"
mkdir -p "$BUILD/classes"
find "$HERE/tests/host-stubs" -name '*.java' ! -path '*/cn/acg138/robotphone/RobotClient.java' > "$BUILD/sources.txt"
for name in ConnectionPolicy ArmHandshake FollowPresentation ControlPolicy SelectionGate LanAddress MultipartReader RobotClient MappingPresentation SemanticPresentation MapDisplayFilter NavigationMap NavigationTarget NavigationMapClient; do
    printf '%s\n' "$HERE/src/cn/acg138/robotphone/$name.java" >> "$BUILD/sources.txt"
done
printf '%s\n' "$HERE/tests/ClientHandshakeTests.java" >> "$BUILD/sources.txt"
javac -encoding UTF-8 --release 8 -d "$BUILD/classes" @"$BUILD/sources.txt"
java -Djava.awt.headless=true -cp "$BUILD/classes" ClientHandshakeTests
