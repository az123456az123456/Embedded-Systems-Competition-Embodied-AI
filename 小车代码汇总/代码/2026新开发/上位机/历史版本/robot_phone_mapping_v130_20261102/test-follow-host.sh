#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BUILD="$HERE/build/follow-host"
mkdir -p "$BUILD"
javac -encoding UTF-8 --release 8 -d "$BUILD" "$HERE/tests/host-stubs/org/json/JSONObject.java" "$HERE/tests/host-stubs/org/json/JSONArray.java" "$HERE/src/cn/acg138/robotphone/FollowPresentation.java" "$HERE/tests/FollowPresentationTests.java"
java -cp "$BUILD" FollowPresentationTests
