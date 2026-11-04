set -eu
cp /root/robot-perception/run-perception.sh /root/model-validation-last-20261102/run-perception.original.sh
sha256sum /root/model-validation-last-20261102/run-perception.original.sh
cat /root/robot-perception/run-yolo.sh
cat /root/run-yolo.sh 2>/dev/null || true
