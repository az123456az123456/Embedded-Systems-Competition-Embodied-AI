#!/bin/sh
# Host-side production client tests. No Android device or RF behavior is claimed.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
sh "$HERE/test-host.sh"
sh "$HERE/test-client-host.sh"
sh "$HERE/test-network-host.sh"
sh "$HERE/test-ui-host.sh"
sh "$HERE/test-map-host.sh"
sh "$HERE/test-capture-host.sh"
