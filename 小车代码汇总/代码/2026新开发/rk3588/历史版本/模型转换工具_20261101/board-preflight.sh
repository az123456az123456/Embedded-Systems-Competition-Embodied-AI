set -eu
uname -m
df -k /root
ps -o pid,comm
ls -l /root/robot-perception
ls -l /root/robot-perception/lib /usr/lib/librknnrt.so /lib/librknnrt.so 2>/dev/null || true
command -v base64
command -v timeout
command -v sha256sum
command -v tar
stty -a
