#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
CC=${CC:-aarch64-none-linux-gnu-gcc}
mkdir -p "$BASE/bin"
"$CC" -std=c11 -O2 -Wall -Wextra -Werror "$BASE/mountns-exec.c" -o "$BASE/bin/mountns-exec.aarch64"
sha256sum "$BASE/bin/mountns-exec.aarch64"
