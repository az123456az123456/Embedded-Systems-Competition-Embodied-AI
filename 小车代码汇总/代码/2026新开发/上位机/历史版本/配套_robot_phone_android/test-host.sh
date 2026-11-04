#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
mkdir -p "$HERE/build/host-tests"
javac -encoding UTF-8 --release 8 -d "$HERE/build/host-tests" "$HERE/src/cn/acg138/robotphone/MultipartReader.java" "$HERE/src/cn/acg138/robotphone/LanAddress.java" "$HERE/src/cn/acg138/robotphone/SelectionGate.java" "$HERE/tests/HostTests.java"
java -cp "$HERE/build/host-tests" HostTests
