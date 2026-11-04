#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BUILD="$HERE/build/capture-host"
mkdir -p "$BUILD/classes"
javac -encoding UTF-8 --release 8 -d "$BUILD/classes" "$HERE/tests/host-stubs/org/json/JSONArray.java" "$HERE/tests/host-stubs/org/json/JSONObject.java" \
 "$HERE/src/cn/acg138/robotphone/NoVestCapturePolicy.java" "$HERE/src/cn/acg138/robotphone/CaptureArchive.java" "$HERE/tests/CaptureHostTests.java"
java -cp "$BUILD/classes" CaptureHostTests
find "$HERE/tests/storage-stubs" -name '*.java' > "$BUILD/storage-sources.txt"
printf '%s\n' "$HERE/src/cn/acg138/robotphone/SafCaptureStore.java" "$HERE/tests/SafStoreHostTests.java" >> "$BUILD/storage-sources.txt"
javac -encoding UTF-8 --release 8 -classpath "$BUILD/classes" -d "$BUILD/classes" @"$BUILD/storage-sources.txt"
java -cp "$BUILD/classes" SafStoreHostTests
