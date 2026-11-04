#!/bin/sh
# Stage the project in an ASCII path for this Gowin Linux installation.
set -eu
PROJECT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
GOWIN_SH=${GOWIN_SH:-gw_sh}
# Optional local host compatibility, never changes RTL, timing or license rules.
# Some distributions cannot load Gowin's bundled old libstdc++.
if [ -n "${GOWIN_COMPAT_CXX:-}" ]; then
    export LD_PRELOAD="$GOWIN_COMPAT_CXX${LD_PRELOAD:+:$LD_PRELOAD}"
fi
command -v "$GOWIN_SH" >/dev/null 2>&1 || { echo "Set GOWIN_SH to your IDE/bin/gw_sh" >&2; exit 1; }
BUILD_DIR=$(mktemp -d /tmp/acg138-navigation-v18.XXXXXX)
printf 'Build directory: %s\n' "$BUILD_DIR"
tar -C "$PROJECT" --exclude=./impl --exclude=./build --exclude=./reports -cf "$BUILD_DIR/source.tar" .
tar -C "$BUILD_DIR" -xf "$BUILD_DIR/source.tar"
rm "$BUILD_DIR/source.tar"
(
    cd "$BUILD_DIR"
    QT_QPA_PLATFORM=minimal QT_XCB_GL_INTEGRATION=none QT_OPENGL=software \
        LIBGL_ALWAYS_SOFTWARE=1 "$GOWIN_SH" build.tcl > gowin.log 2>&1
) || { tail -n 40 "$BUILD_DIR/gowin.log"; exit 1; }
python3 "$PROJECT/tools/audit_release.py" --build-dir "$BUILD_DIR"
printf 'Validated output: %s/impl/pnr/robot_navigation_v18.fs\n' "$PROJECT"
printf 'Full build retained: %s\n' "$BUILD_DIR"
