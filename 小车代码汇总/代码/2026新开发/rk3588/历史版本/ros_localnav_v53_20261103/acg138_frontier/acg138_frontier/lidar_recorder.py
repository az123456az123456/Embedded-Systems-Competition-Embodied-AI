"""Opt-in, read-only rosbag supervisor. No services or motion publishers.

Each invocation creates a new directory. Existing recordings are never removed.
Quota stops recording before reserve is exhausted; it never stops navigation.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid

MIB = 1024 * 1024
TOPICS = ('/scan', '/scan_mapping', '/scan_clearing', '/tf', '/tf_static', '/odom', '/imu/data',
          '/map', '/slam/map', '/mapping/state', '/mapping/dynamic_map_state',
          '/exploration/state', '/fpga/state')


def directory_bytes(path):
    # Refuse symlinks to keep quota accounting within the private log directory.
    total = 0
    for base, dirs, files in os.walk(path, followlinks=False):
        dirs[:] = [d for d in dirs if not (Path(base) / d).is_symlink()]
        for name in files:
            file = Path(base) / name
            try:
                if not file.is_symlink():
                    total += file.stat().st_size
            except FileNotFoundError:
                pass
    return total


def quota_reason(session_bytes, all_bytes, free_bytes, session_limit, total_limit,
                 minimum_free, reserve=16 * MIB):
    # Leave room for both rosbag cache buffers, a large map and clean finalization.
    if free_bytes <= minimum_free + reserve:
        return 'low_disk_space'
    if session_bytes >= session_limit - reserve:
        return 'session_size_limit'
    if all_bytes >= total_limit - reserve:
        return 'total_size_limit'
    return None


def command(bag, qos_file, split_bytes):
    return ['ros2', 'bag', 'record', '--storage', 'sqlite3', '--output', str(bag),
            '--max-bag-size', str(split_bytes), '--max-cache-size', str(2 * MIB),
            '--disable-keyboard-controls', '--node-name', 'acg138_lidar_recorder',
            '--qos-profile-overrides-path', str(qos_file), '--topics', *TOPICS]


def stop_child(child):
    if child is None or child.poll() is not None:
        return None
    for sig, delay in ((signal.SIGINT, 8), (signal.SIGTERM, 3)):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            return child.poll()
        try:
            return child.wait(delay)
        except subprocess.TimeoutExpired:
            pass
    os.killpg(child.pid, signal.SIGKILL)
    return child.wait(2)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', default='/data/bags/lidar')
    p.add_argument('--qos-file', required=True)
    p.add_argument('--session-mib', type=int, default=256)
    p.add_argument('--total-mib', type=int, default=512)
    p.add_argument('--minimum-free-mib', type=int, default=512)
    p.add_argument('--split-mib', type=int, default=32)
    a = p.parse_args(argv)
    if min(a.session_mib, a.total_mib) < 32 or a.minimum_free_mib < 32 or not 1 <= a.split_mib <= 32:
        p.error('quota minimum is 32 MiB; split must be 1..32 MiB')
    root = Path(a.directory)
    if root.is_symlink() or not Path(a.qos_file).is_file():
        raise ValueError('invalid logging directory or QoS file')
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    lock = open(root / '.recorder.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('LIDAR_LOG disabled: another recorder owns this directory', flush=True)
        return 1
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda signum, frame: stop.append(signum))
    reason = quota_reason(0, directory_bytes(root), shutil.disk_usage(root).free,
                          a.session_mib*MIB, a.total_mib*MIB, a.minimum_free_mib*MIB)
    if reason:
        print('LIDAR_LOG disabled: ' + reason + '; existing data retained at ' + str(root), flush=True)
        return 1
    name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
    session = root / name
    session.mkdir(mode=0o700)
    manifest = {'schema': 1, 'session': name, 'topics': list(TOPICS),
                'started_unix_ns': time.time_ns(), 'started_monotonic_ns': time.monotonic_ns(),
                'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                'quotas_mib': vars(a), 'source_timestamps_preserved': True,
                'recording_window': 'whole --log program session, including idle and exploration',
                'motion_commands_sent': 0, 'state': 'STARTING'}
    def save():
        tmp = session / 'session.json.tmp'
        tmp.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        tmp.replace(session / 'session.json')
    child = None
    save()
    print('LIDAR_LOG directory=' + str(session) + ' split=32MiB session<=256MiB total<=512MiB (defaults)', flush=True)
    try:
        with open(session / 'recorder.log', 'ab', buffering=0) as output:
            child = subprocess.Popen(command(session / 'bag', a.qos_file, a.split_mib*MIB),
                                     stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            manifest['state'] = 'RECORDING'
            save()
            while not stop and child.poll() is None:
                reason = quota_reason(directory_bytes(session), directory_bytes(root),
                                      shutil.disk_usage(root).free, a.session_mib*MIB,
                                      a.total_mib*MIB, a.minimum_free_mib*MIB)
                if reason:
                    break
                time.sleep(.5)
            reason = reason or ('signal_' + str(stop[0]) if stop else 'recorder_exited')
    except Exception as ex:
        reason = 'error:' + str(ex)
    finally:
        code = stop_child(child)
        if child is not None and code is None:
            code = child.poll()
        manifest.update(state='STOPPED', reason=reason, recorder_returncode=code,
                        finished_unix_ns=time.time_ns(), bytes=directory_bytes(session),
                        metadata_present=(session / 'bag' / 'metadata.yaml').is_file())
        save()
        print('LIDAR_LOG stopped=' + str(reason) + ' saved=' + str(session), flush=True)
        lock.close()  # Keep lock inode; never unlink it.
    return 0 if manifest['metadata_present'] and code == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
