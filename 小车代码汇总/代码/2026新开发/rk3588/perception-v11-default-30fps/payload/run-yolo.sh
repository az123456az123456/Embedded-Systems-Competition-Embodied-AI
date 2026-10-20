#!/bin/sh
# The installer binds DEFAULT_DASH to the validated installation directory.
set -eu
DEFAULT_DASH='/root/perception-dashboard'
DASHBOARD_DIR=${DASHBOARD_DIR:-$DEFAULT_DASH}
exec sh "$DASHBOARD_DIR/run-dashboard.sh" "$@"
