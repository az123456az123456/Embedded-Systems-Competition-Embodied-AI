#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
if [ -n "${NAVSIM_PYTHON:-}" ]; then
    exec "$NAVSIM_PYTHON" run_sim.py "$@"
fi
exec python3 run_sim.py "$@"
