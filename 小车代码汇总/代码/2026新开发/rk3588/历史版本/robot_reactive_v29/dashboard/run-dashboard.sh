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
if [ -z "${PERCEPTION_BINARY:-}" ]; then
 case $(uname -m) in
  aarch64|arm64) NAMES='perception-dashboard.aarch64 build/perception-dashboard.aarch64' ;;
  *) NAMES='build/perception-dashboard perception-dashboard' ;;
 esac
 for NAME in $NAMES; do
  if [ -x "$APP/$NAME" ]; then PERCEPTION_BINARY=$APP/$NAME;break;fi
 done
fi
[ -f "$APP/run-perception.sh" ] && [ -n "${PERCEPTION_BINARY:-}" ] && [ -x "$PERCEPTION_BINARY" ] || {
 echo 'Program not found. Unpack the RK3588 release or build with make.' >&2;exit 2;
}
export PERCEPTION_BINARY
exec sh "$APP/run-perception.sh" --fullscreen --rotate "$ANGLE" "$@"
