set -eu
BASE=/root/model-validation-last-20261102
cd "$BASE"
tar -xzf board-validation-int8.tar.gz
cd board-validation
sha256sum -c SHA256SUMS
chmod 755 npu-model-check.aarch64
export LD_LIBRARY_PATH=/root/robot-perception/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
# File-only test executable has no ROS, network, camera or motor entry points.
timeout 120 ./npu-model-check.aarch64 model-int8.rknn inputs.txt int8-single.jsonl 1 9 > int8-single-summary.json 2> int8-single.log
timeout 120 ./npu-model-check.aarch64 model-int8.rknn inputs.txt int8-three.jsonl 3 9 > int8-three-summary.json 2> int8-three.log
cat int8-single-summary.json int8-three-summary.json
wc -l int8-single.jsonl int8-three.jsonl
sha256sum int8-single.jsonl int8-three.jsonl
printf '\nCURRENT_LAUNCHER\n'
cat /root/robot-perception/run-perception.sh
printf '\nCURRENT_LABELS_AND_MODEL\n'
ls -l /root/robot-perception/model
sha256sum /root/robot-perception/run-perception.sh
