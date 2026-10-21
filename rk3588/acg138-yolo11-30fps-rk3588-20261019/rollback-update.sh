#!/bin/sh
set -eu
PACKAGE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
exec sh "$PACKAGE/scripts/rollback-update.sh" "$@"
