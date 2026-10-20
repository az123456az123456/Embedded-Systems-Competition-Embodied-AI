#!/bin/sh
# One entry point; reuse the board's working camera/display launcher.
set -eu
APP=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ANGLE=0
case "${1:-}" in
 -0) ANGLE=0; shift ;;
 -90) ANGLE=90; shift ;;
 -h|--help)
  printf '%s\n' 'Usage: ./run-dashboard.sh [-0|-90] [dashboard options]' \
    '  -0   Normal fullscreen (default)' \
    '  -90  Clockwise 90-degree fullscreen' \
    'Example: ./run-dashboard.sh -90 --npu-workers 2'
  exit 0 ;;
 -[0-9]*) echo 'Select -0 or -90 as the first argument.' >&2;exit 2 ;;
esac
[ -f "$APP/run-perception.sh" ] && [ -x "$APP/perception-dashboard.aarch64" ] || {
 echo 'Install into the existing perception-dashboard directory first.' >&2;exit 2;
}
export PERCEPTION_BINARY="$APP/perception-dashboard.aarch64"
exec sh "$APP/run-perception.sh" --fullscreen --rotate "$ANGLE" "$@"
