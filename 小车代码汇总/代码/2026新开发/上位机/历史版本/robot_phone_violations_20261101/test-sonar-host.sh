#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BUILD="$HERE/build/sonar-host"
mkdir -p "$BUILD/classes"
${CXX:-g++} -std=c++17 -O2 -Wall -Wextra "$HERE/tests/sonar_fixture.cc" -o "$BUILD/native-sonar-fixture"
"$BUILD/native-sonar-fixture" > "$BUILD/native-sonar-fixtures.jsonl"
javac -encoding UTF-8 --release 8 -d "$BUILD/classes" "$HERE/tests/host-stubs/org/json/JSONObject.java" "$HERE/tests/host-stubs/org/json/JSONArray.java" "$HERE/src/cn/acg138/robotphone/SonarPresentation.java" "$HERE/tests/SonarPresentationTests.java"
java -cp "$BUILD/classes" SonarPresentationTests "$BUILD/native-sonar-fixtures.jsonl"
