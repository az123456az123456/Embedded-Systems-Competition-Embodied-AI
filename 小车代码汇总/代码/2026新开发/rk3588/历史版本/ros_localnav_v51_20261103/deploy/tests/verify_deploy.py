#!/usr/bin/env python3
"""Host tests, no actual device/mount/container/network/board changes."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

BASE = Path(__file__).resolve().parents[1]
checks = []

def check(name, predicate):
    if not predicate:
        raise AssertionError(name)
    checks.append(name)

def run(args, **kw):
    return subprocess.run(args, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=12, **kw)

for script in list(BASE.glob('*.sh')) + [BASE/'S80acg138-ros2'] + list((BASE/'native').glob('*.sh')):
    if script.name == 'ros-entrypoint.sh':
        check('bash syntax '+script.name, run(['bash','-n',str(script)]).returncode == 0)
    else:
        check('POSIX syntax '+script.name, run(['sh','-n',str(script)]).returncode == 0)
        if shutil.which('busybox'):
            check('BusyBox syntax '+script.name, run(['busybox','sh','-n',str(script)]).returncode == 0)

with tempfile.TemporaryDirectory(prefix='v37-deploy-test-') as tmp_s:
    tmp=Path(tmp_s); fake=tmp/'bin'; fake.mkdir()
    def executable(name, text):
        p=fake/name; p.write_text('#!/bin/sh\n'+text); p.chmod(0o755)
    executable('uname', 'echo aarch64\n')
    executable('docker', 'printf "%s\\n" "$@" > "$DOCKER_ARGS"\n')
    executable('udevadm', 'printf "%s\\n" "ID_VENDOR_ID=10c4" "ID_MODEL_ID=ea60" "ID_SERIAL_SHORT=C1_unique_123"\n')
    env=dict(os.environ, PATH=str(fake)+':'+os.environ['PATH'], DATA_DIR=str(tmp/'data'),
             SERIAL_DEVICE='/dev/null', DOCKER_ARGS=str(tmp/'docker-args'))
    r=run(['sh',str(BASE/'run-container.sh')],env=env)
    check('container command generated',r.returncode==0)
    args=(tmp/'docker-args').read_text().splitlines()
    check('only NET_RAW capability',['--cap-drop','ALL','--cap-add','NET_RAW']==args[args.index('--cap-drop'):args.index('--cap-drop')+4])
    check('no privileged/full-device mount', '--privileged' not in args and not any('src=/dev,' in x for x in args))
    check('host network avoids veth',args[args.index('--network')+1]=='host')
    check('default launch matches deliverable',args[-4:]==['ros2','launch','acg138_bringup','mapping.launch.py'])
    check('data excludes rootfs tree',not any(x==f'type=bind,src={tmp}/data,dst=/data' for x in args))
    check('read-only host process view for planner exclusion','type=bind,src=/proc,dst=/host_proc,readonly' in args and 'ACG138_HOST_PROC_ROOT=/host_proc' in args)
    check('shared raw planner lock','type=bind,src=/run/lock,dst=/run/lock' in args)
    r=run(['sh',str(BASE/'generate-udev-rule.sh'),'/dev/null'],env=env)
    check('udev precise identity',r.returncode==0 and 'ATTRS{serial}=="C1_unique_123"' in r.stdout and 'MODE="0660"' in r.stdout)
    executable('udevadm','printf "%s\\n" "ID_VENDOR_ID=10c4" "ID_MODEL_ID=ea60"\n')
    r=run(['sh',str(BASE/'generate-udev-rule.sh'),'/dev/null'],env=env)
    check('udev rejects nonunique adapter',r.returncode!=0 and 'serial ID' in r.stderr)
    r=run(['sh',str(BASE/'native/cross-build-overlay.sh')],env=dict(env,
          ACG138_TOOLCHAIN_BIN=str(tmp),ACG138_SYSROOT=str(tmp),ACG138_ROS_UNDERLAY=str(tmp)))
    check('native build refuses missing ROS underlay',r.returncode!=0 and 'Missing Buildroot-native ROS' in r.stderr)
    # Failure injection: no real mount/chroot. The third synthetic mount fails.
    executable('id','echo 0\n')
    executable('mount','''n=0; [ ! -f "$MOUNT_COUNT" ] || n=$(cat "$MOUNT_COUNT")
n=$((n+1)); echo "$n" > "$MOUNT_COUNT"
[ "$n" -ne 3 ] || exit 17
exit 0
''')
    executable('umount','printf "%s\\n" "$1" >> "$UNMOUNT_LOG"\n')
    rootfs=tmp/'fake-rootfs'; (rootfs/'etc').mkdir(parents=True)
    (rootfs/'etc/acg138-ros2-rootfs').write_text('jazzy-v37\n')
    lock=tmp/'chroot-lock'
    mount_env=dict(env, ROOTFS_DIR=str(rootfs),CHROOT_LOCK_DIR=str(lock),
                   ACG138_ROS2_MOUNTNS='1',MOUNT_COUNT=str(tmp/'mount-count'),
                   UNMOUNT_LOG=str(tmp/'unmounts'))
    r=run(['sh',str(BASE/'run-chroot.sh')],env=mount_env)
    check('mount failure stops before chroot',r.returncode==17)
    check('mount failure cleanup reversed',(tmp/'unmounts').read_text().splitlines()==[str(rootfs/'dev/shm'),str(rootfs/'proc')])
    check('successful cleanup releases exclusive lock',not lock.exists())
    # Real supervisor loop using fake runtime and fake sleep. No chroot/mount.
    service=tmp/'service'; service.mkdir()
    shutil.copy2(BASE/'supervise.sh',service/'supervise.sh')
    runtime=service/'run-chroot.sh'
    runtime.write_text('''#!/bin/sh
n=0; [ ! -f "$CALLS" ] || n=$(cat "$CALLS")
n=$((n+1)); echo "$n" > "$CALLS"
if [ "$n" -ge 3 ]; then kill -TERM "$PPID"; fi
exit 7
'''); runtime.chmod(0o755)
    executable('sleep','exit 0\n')
    env.update(CALLS=str(tmp/'calls'),ACG138_SERVICE_CONFIG=str(tmp/'absent-config'))
    r=run(['sh',str(service/'supervise.sh')],env=env)
    check('supervisor exits on termination',r.returncode==0)
    check('supervisor retried failures',(tmp/'calls').read_text().strip()=='3')
    log=(tmp/'data/logs/service.log').read_text()
    check('supervisor logs exponential backoff','retry_in=2s' in log and 'retry_in=4s' in log)
    check('supervisor never enables motors','motor_enable=never' in log and 'no automatic motor enable' in log)

helper=BASE/'bin/mountns-exec.aarch64'
check('ARM64 namespace helper included',helper.exists() and helper.read_bytes()[:4]==b'\x7fELF')
text=(BASE/'run-chroot.sh').read_text()
check('private namespace required','exec "$HELPER" sh "$0" "$@"' in text)
check('capability drop before ROS','--bounding-set=-all,+net_raw' in text)
check('bounded retry/log storage','5242880' in (BASE/'supervise.sh').read_text())
check('no global process kill','pkill' not in (BASE/'S80acg138-ros2').read_text())
report={'passed':len(checks),'checks':checks,'scope':'Host syntax + fake runtime failure tests; no physical board, actual chroot, OCI image build, full ROS build or SLAM runtime tested',
        'helper_sha256':hashlib.sha256(helper.read_bytes()).hexdigest()}
report_path=BASE.parent/'reports/deploy-verification.json'
report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
