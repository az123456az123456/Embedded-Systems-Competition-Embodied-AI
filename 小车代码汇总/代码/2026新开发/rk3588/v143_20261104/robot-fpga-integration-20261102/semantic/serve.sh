#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
IPC=/run/acg138-ros-bridge/semantic
umask 077
mkdir -p "$IPC"
chmod 700 "$IPC"
"$BASE/run-worker.sh" --socket "$IPC/model.sock" &
CHILD=$!
cleanup() {
    trap - EXIT INT TERM HUP
    kill -TERM "$CHILD" 2>/dev/null || :
    wait "$CHILD" 2>/dev/null || :
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
while kill -0 "$CHILD" 2>/dev/null; do
    available=$(awk '/^MemAvailable:/{print $2}' /proc/meminfo)
    if [ "$available" -lt 262144 ]; then
        echo 'SEMANTIC_RESOURCE_LIMIT: available RAM below 256 MiB; stop owned semantic session.' >&2
        exit 1
    fi
    sleep 1
done
wait "$CHILD"
