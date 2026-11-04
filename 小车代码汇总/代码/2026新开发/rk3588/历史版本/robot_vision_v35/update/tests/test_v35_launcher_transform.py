#!/usr/bin/env python3
"""Execute transformed installed launchers with isolated native/desktop fixtures."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--report', type=Path, required=True)
ap.add_argument('--busybox', action='store_true')
args = ap.parse_args()
package = Path(__file__).resolve().parents[1]
ws = package.parents[2]
awk = package / 'transform-launcher.awk'
shell = ['busybox', 'sh'] if args.busybox else ['sh']
checks = 0
cases = []
def ck(value, why):
    global checks
    assert value, why
    checks += 1

def transform(text):
    p = subprocess.run(['awk', '-f', str(awk)], input=text, capture_output=True, text=True)
    ck(p.returncode == 0, p.stderr)
    return p.stdout

originals = {
 'restore': ws/'rk3588/acg138-restore-20261023/launchers/run-perception.sh',
 'v33-source': ws/'rk3588/robot_sonar_avoid_v33/dashboard/run-perception.sh',
}
with tempfile.TemporaryDirectory(prefix='v35-transform-launcher-') as temp:
    temp = Path(temp)
    for name, path in originals.items():
        before = path.read_text()
        after = transform(before)
        ck(transform(after) == after, name+' transform not idempotent')
        for line in before.splitlines():
            if any(token in line for token in ('CAMERA_TARGET_FPS=', 'case "$CAMERA_TARGET_FPS"', '--left-mask', '--invert-mask', 'trap ', 'wait "$CHILD_PID"', 'kill -TERM "$CHILD_PID"')):
                expected = line.replace('CAMERA_TARGET_FPS:-30', 'CAMERA_TARGET_FPS:-40').replace('15|20|30)', '15|20|30|40)').replace('15, 20 or 30', '15, 20, 30 or 40')
                ck(expected in after, name+' preserved flags/lock/signal/fps changed: '+line)
        for function in ('acquire_lock', 'cleanup', 'stop_child', 'discover_wayland', 'prepare_wayland'):
            pattern = rf'^{function}\(\) \{{\n.*?^\}}'
            found = re.search(pattern, before, re.M|re.S)
            if found: ck(found.group(0) in after, name+' '+function+' body changed')
        root = temp / name
        root.mkdir()
        app = root / 'app with spaces'
        app.mkdir()
        (app/'scripts').mkdir()
        marker = root/'markers'
        record = root/'record.json'
        lock = root/'lock'
        # Isolate the existing lock path. Desktop discovery is kept byte-exact
        # above, then instrumented to prove which branch calls it, no Weston/NIC.
        runnable = after.replace('LOCK_DIR=/tmp/robot-perception.lock', 'LOCK_DIR='+str(lock))
        runnable = runnable.replace('prepare_wayland() {\n', 'prepare_wayland() {\n    echo wayland >> "$V35_MARKERS"\n    export WAYLAND_DISPLAY=fixture-wayland XDG_RUNTIME_DIR=/fixture-runtime\n    return 0\n')
        launcher = app/'run-perception.sh'
        launcher.write_text(runnable)
        syntax = subprocess.run([*shell,'-n',str(launcher)],capture_output=True,text=True)
        ck(syntax.returncode == 0, syntax.stderr)
        tools = root/'tools'; tools.mkdir()
        idtool = tools/'id'; idtool.write_text('#!/bin/sh\n[ "$1" != -u ] || { echo 0; exit; }; exec /usr/bin/id "$@"\n'); idtool.chmod(0o755)
        finder = root/'find.sh'; finder.write_text('#!/bin/sh\necho find >> "$V35_MARKERS"\necho /dev/null\n')
        check = app/'scripts/check-board.sh'; check.write_text('#!/bin/sh\necho check-board >> "$V35_MARKERS"\n')
        ctl = tools/'v4l2-ctl'; ctl.write_text('#!/bin/sh\necho v4l2-ctl >> "$V35_MARKERS"\necho NV12\n');ctl.chmod(0o755)
        isp = root/'forbidden-isp.sh';isp.write_text('#!/bin/sh\necho ISP >> "$V35_MARKERS"\nexit 99\n')
        binary = root/'native.py';binary.write_text('''#!/usr/bin/python3
import json,os,signal,sys,time
if os.environ.get('V35_HOLD'):
 def end(a,b):
  open(os.environ['V35_MARKERS'],'a').write('native-terminated\\n')
  sys.exit(0)
 signal.signal(signal.SIGTERM,end)
open(os.environ['V35_RECORD'],'w').write(json.dumps({'args':sys.argv[1:],'source':os.environ.get('PERCEPTION_CAMERA_SOURCE'),'wayland':os.environ.get('WAYLAND_DISPLAY'),'runtime':os.environ.get('XDG_RUNTIME_DIR'),'camera_target_fps':os.environ.get('CAMERA_TARGET_FPS')}))
if os.environ.get('V35_HOLD'):
 while True: time.sleep(.05)
''');binary.chmod(0o755)
        env = dict(os.environ)
        for key in ('PERCEPTION_CAMERA_SOURCE','CAMERA_DEVICE','MODEL_PATH','LABELS_PATH','PERCEPTION_RGB_INTERFACE','V35_HOLD','CAMERA_TARGET_FPS'):
            env.pop(key,None)
        env.update(PATH=str(tools)+':'+env['PATH'], PERCEPTION_BINARY=str(binary), V35_RECORD=str(record), V35_MARKERS=str(marker), FIND_VIDEO_SCRIPT=str(finder), ISP_CAMERA_SCRIPT=str(isp), PERCEPTION_PREPARE_ISP='0', PERCEPTION_VIDEO_WAIT_ATTEMPTS='1', XDG_RUNTIME_DIR='/fixture-runtime', WAYLAND_DISPLAY='fixture-wayland')
        def command(cli=()):
            if args.busybox:
                # BusyBox standalone applets bypass PATH; keep only the UID shim.
                return [*shell, '-c', 'id() { "$V35_ID_TOOL" "$@"; }; . "$0"', str(launcher), *cli]
            return [*shell, str(launcher), *cli]
        env['V35_ID_TOOL'] = str(idtool)
        def invoke(extra=None, cli=(), success=True):
            record.unlink(missing_ok=True);marker.unlink(missing_ok=True)
            p = subprocess.run(command(cli), env=env|(extra or {}),capture_output=True,text=True,timeout=4)
            ck((p.returncode==0)==success,(name,extra,cli,p.returncode,p.stdout,p.stderr))
            ck(not lock.exists(),name+' lock not cleaned')
            return (json.loads(record.read_text()) if success else p), marker.read_text().splitlines() if marker.exists() else []
        def value(data, option):return data['args'][data['args'].index(option)+1]
        data, marks = invoke({'PERCEPTION_PREPARE_ISP':'invalid-unused','PERCEPTION_VIDEO_WAIT_ATTEMPTS':'invalid-unused'}, ['--fullscreen','--rotate','90','--no-phone'])
        ck(data['source']=='fpga-rgb' and value(data,'--camera')=='fpga-rgb:eth0',name+' default not raw')
        ck(data['camera_target_fps']=='40',name+' publisher description did not default40')
        ck(not any(m in marks for m in ('find','ISP','check-board','v4l2-ctl')),name+' raw entered camera preparation')
        ck(data['args'][-4:]==['--fullscreen','--rotate','90','--no-phone'],name+' native flags changed')
        ck(value(data,'--model').endswith('/model/acg138-best-e7f26253-int8-rk3588.rknn') and value(data,'--labels').endswith('/model/acg138-best-e7f26253-labels.txt'),name+' model/label did not switch together')
        ck(data['wayland']=='fixture-wayland',name+' Wayland export lost')
        if name=='restore':ck(marks==['wayland'],name+' raw did not prepare Wayland independently')
        data, marks = invoke({'CAMERA_TARGET_FPS':'40'})
        ck(data['source']=='fpga-rgb' and data['camera_target_fps']=='40' and not any(m in marks for m in ('find','ISP','check-board','v4l2-ctl')),name+' explicit40 raw failed/prepared ISP')
        data, marks = invoke({'PERCEPTION_CAMERA_SOURCE':'v4l2','CAMERA_TARGET_FPS':'40'})
        ck(data['source']=='v4l2' and data['camera_target_fps']=='40' and value(data,'--camera')=='/dev/null' and 'find' in marks,name+' V4L2 fallback lost')
        if name=='restore':ck(marks==['check-board','wayland','find'],name+' restore V4L2 prep ordering changed')
        data, marks=invoke({'PERCEPTION_CAMERA_SOURCE':'v4l2'}, ['--camera','fpga-rgb:eth7'])
        ck(data['source']=='fpga-rgb' and value(data,'--camera')=='fpga-rgb:eth7' and 'find' not in marks,name+' explicit raw URI not honored')
        _, marks=invoke({'PERCEPTION_CAMERA_SOURCE':'unknown'}, success=False)
        ck(not marks,name+' invalid source performed side effects')
        data,marks=invoke(cli=['--demo'])
        ck(data['source']=='fpga-rgb' and not marks,name+' demo started desktop/camera')
        # Verify the original lock + signal forwarding executes and removes lock.
        if name=='restore':
            marker.unlink(missing_ok=True);record.unlink(missing_ok=True)
            child=subprocess.Popen(command(),env=env|{'V35_HOLD':'1'},stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,start_new_session=True)
            deadline=time.monotonic()+3
            while not record.exists() and child.poll() is None and time.monotonic()<deadline:time.sleep(.01)
            ck(record.exists() and (lock/'child_pid').exists(),'native child/lock not started')
            child.send_signal(signal.SIGTERM)
            _,err=child.communicate(timeout=3)
            ck(child.returncode==143 and not lock.exists() and 'native-terminated' in marker.read_text(),'signal forwarding/cleanup failed '+str(err))
        cases.append({'launcher':name,'source_sha256':hashlib.sha256(before.encode()).hexdigest(),'transformed_sha256':hashlib.sha256(after.encode()).hexdigest(),'status':'PASS'})
report={'status':'PASS','checks':checks,'cases':cases,'awk_sha256':hashlib.sha256(awk.read_bytes()).hexdigest(),'shell':'busybox sh' if args.busybox else 'host sh','scope':'Actual transformed scripts executed with native binary/check-board/desktop fixtures; original lock and signal forwarding used in temporary paths; no camera, NIC or Weston hardware accessed'}
args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
