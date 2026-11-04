#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
IMAGE=${ROS_IMAGE:-acg138-ros2:jazzy-v37}
ROS_BASE=${ROS_BASE:-ros:jazzy-ros-base-noble}
command -v docker >/dev/null 2>&1 || { echo 'Run this on the development PC with Docker/buildx installed.' >&2; exit 1; }
docker buildx build --platform linux/arm64 --load --progress plain \
    --build-arg "ROS_BASE=$ROS_BASE" -f "$BASE/deploy/Dockerfile" -t "$IMAGE" "$BASE"
mkdir -p "$BASE/reports/image"
docker image inspect "$IMAGE" --format '{{.Id}} {{.Architecture}} {{json .RepoDigests}}' > "$BASE/reports/image/image-identity.txt"
docker run --rm --platform linux/arm64 --entrypoint cat "$IMAGE" \
    /opt/acg138_ros/debian-packages.tsv > "$BASE/reports/image/debian-packages.tsv"
printf '%s\n' "Built $IMAGE. Record/pin the official base digest and archive this exact image for deployment."
