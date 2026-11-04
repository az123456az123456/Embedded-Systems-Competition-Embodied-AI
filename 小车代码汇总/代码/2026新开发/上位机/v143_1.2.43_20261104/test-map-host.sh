#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BUILD="$HERE/build/map-host"
mkdir -p "$BUILD/classes"
find "$HERE/tests/host-stubs" -name '*.java' > "$BUILD/sources.txt"
for name in LanAddress MapViewport MapDisplayFilter NavigationMap NavigationTarget FollowPresentation NavigationMapClient; do
 printf '%s\n' "$HERE/src/cn/acg138/robotphone/$name.java" >> "$BUILD/sources.txt"
done
printf '%s\n' "$HERE/tests/NavigationMapTests.java" "$HERE/tests/MapDisplayTests.java" >> "$BUILD/sources.txt"
javac -encoding UTF-8 --release 8 -d "$BUILD/classes" @"$BUILD/sources.txt"
java -Djava.awt.headless=true -cp "$BUILD/classes" NavigationMapTests "$@"

java -Djava.awt.headless=true -cp "$BUILD/classes" MapDisplayTests
