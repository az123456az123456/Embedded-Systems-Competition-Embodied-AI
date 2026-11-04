#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
mkdir -p "$HERE/build/video-host"
javac --release 8 -encoding UTF-8 -d "$HERE/build/video-host" "$HERE/src/cn/acg138/robotphone/MultipartReader.java" "$HERE/tests/VideoFreshnessTests.java"
java -cp "$HERE/build/video-host" cn.acg138.robotphone.VideoFreshnessTests
BUILD="$HERE/build/video-client-host"
mkdir -p "$BUILD/classes"
find "$HERE/tests/host-stubs" -name '*.java' ! -path '*/cn/acg138/robotphone/RobotClient.java' ! -path '*/android/graphics/BitmapFactory.java' > "$BUILD/sources.txt"
find "$HERE/tests/network-stubs" -name '*.java' >> "$BUILD/sources.txt"
for name in ArmHandshake FollowPresentation ControlPolicy ConnectionPolicy SelectionGate LanAddress MultipartReader RobotClient NavigationMap NavigationTarget NavigationMapClient; do
    printf '%s\n' "$HERE/src/cn/acg138/robotphone/$name.java" >> "$BUILD/sources.txt"
done
printf '%s\n' "$HERE/tests/ClientNetworkTests.java" "$HERE/tests/ClientVideoLatestTests.java" >> "$BUILD/sources.txt"
javac -encoding UTF-8 --release 8 -d "$BUILD/classes" @"$BUILD/sources.txt"
java -Djava.awt.headless=true -cp "$BUILD/classes" ClientVideoLatestTests
