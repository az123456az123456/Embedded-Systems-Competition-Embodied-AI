#!/bin/sh
# Host-only regression; opens loopback HTTP, never physical robot devices.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
NAV=$(CDPATH= cd -- "$HERE/.." && pwd)
BUILD="$NAV/build"
mkdir -p "$BUILD"
"${CXX:-g++}" -std=c++17 -O2 -pthread -Wall -Wextra \
    "$NAV/gateway.cc" "$NAV/vendor/http_server.cc" "$NAV/vendor/json_flat.cc" \
    "$NAV/../dashboard/phone/controller.cc" "$NAV/../dashboard/phone/follow_geometry.cc" \
    "$NAV/../common/wrong_log.cc" "$HERE/test_mobile_backend_lease_v33.cc" \
    -o "$BUILD/test-mobile-backend-lease-v33"
exec "$BUILD/test-mobile-backend-lease-v33"
