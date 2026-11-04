#!/bin/sh
# Executes production DashboardView with bounded Android/JSON adapters on the host.
# Images are Java2D previews, not screenshots from an Android device.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BUILD="$HERE/build/ui-host"
mkdir -p "$BUILD/classes" "$HERE/dist/ui-preview"
find "$HERE/tests/host-stubs" -name '*.java' > "$BUILD/sources.txt"
printf '%s\n' "$HERE/src/cn/acg138/robotphone/FollowPresentation.java" "$HERE/src/cn/acg138/robotphone/ControlPolicy.java" "$HERE/src/cn/acg138/robotphone/SelectionGate.java" "$HERE/src/cn/acg138/robotphone/DashboardView.java" "$HERE/src/cn/acg138/robotphone/MappingPresentation.java" "$HERE/src/cn/acg138/robotphone/ButtonFeedback.java" "$HERE/src/cn/acg138/robotphone/PageIndicatorMotion.java" "$HERE/src/cn/acg138/robotphone/SonarPresentation.java" "$HERE/src/cn/acg138/robotphone/NavigationMap.java" "$HERE/src/cn/acg138/robotphone/NavigationTarget.java" "$HERE/tests/GuiHostTests.java" "$HERE/tests/ButtonFeedbackHostTests.java" "$HERE/tests/PageIndicatorHostTests.java" >> "$BUILD/sources.txt"
javac -encoding UTF-8 --release 8 -d "$BUILD/classes" @"$BUILD/sources.txt"
${CXX:-g++} -std=c++17 -O2 -Wall -Wextra "$HERE/tests/sonar_fixture.cc" -o "$BUILD/native-sonar-fixture"
"$BUILD/native-sonar-fixture" > "$BUILD/native-sonar-fixtures.jsonl"
java -Djava.awt.headless=true -cp "$BUILD/classes" GuiHostTests "$HERE/dist/ui-preview" "$BUILD/native-sonar-fixtures.jsonl"

java -Djava.awt.headless=true -cp "$BUILD/classes" ButtonFeedbackHostTests "$HERE/dist/ui-preview/feedback"

java -Djava.awt.headless=true -cp "$BUILD/classes" PageIndicatorHostTests "$HERE/dist/ui-preview/paging"
