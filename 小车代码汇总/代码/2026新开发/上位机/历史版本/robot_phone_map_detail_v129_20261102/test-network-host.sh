#!/bin/sh
# v36 replaces the historical 1.5s telemetry-disarm policy with 30s accepted-command authority.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec sh "$HERE/test-control-v36.sh" "$@"
