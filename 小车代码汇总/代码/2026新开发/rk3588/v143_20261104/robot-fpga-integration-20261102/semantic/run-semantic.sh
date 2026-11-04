#!/bin/sh
# Uses the existing integration supervisor and existing FPGA authorization.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec "$BASE/deploy/run-semantic-stack.sh" "$@"
