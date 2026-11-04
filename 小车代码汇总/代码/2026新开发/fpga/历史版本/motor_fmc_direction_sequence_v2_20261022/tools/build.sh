#!/bin/sh
set -eu
PROJECT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
GOWIN_SH=${GOWIN_SH:-gw_sh}
command -v "$GOWIN_SH" >/dev/null 2>&1 || { echo 'Set GOWIN_SH to IDE/bin/gw_sh'; exit 1; }
BUILD_DIR=$(mktemp -d /tmp/acg138-direction-diag.XXXXXX)
printf 'Build directory: %s\n' "$BUILD_DIR"
tar -C "$PROJECT" --exclude=./build --exclude=./reports --exclude=./impl --exclude=./__pycache__ -cf "$BUILD_DIR/source.tar" .
tar -C "$BUILD_DIR" -xf "$BUILD_DIR/source.tar"
rm "$BUILD_DIR/source.tar"
(
 cd "$BUILD_DIR"
 QT_QPA_PLATFORM=minimal QT_XCB_GL_INTEGRATION=none QT_OPENGL=software LIBGL_ALWAYS_SOFTWARE=1 \
 "$GOWIN_SH" build.tcl > gowin.log 2>&1
) || { tail -n 35 "$BUILD_DIR/gowin.log"; exit 1; }
python3 "$PROJECT/tools/audit_release.py" --build-dir "$BUILD_DIR"
printf 'Validated FS: %s/impl/pnr/fmc_direction_sequence_v2.fs\n' "$PROJECT"
