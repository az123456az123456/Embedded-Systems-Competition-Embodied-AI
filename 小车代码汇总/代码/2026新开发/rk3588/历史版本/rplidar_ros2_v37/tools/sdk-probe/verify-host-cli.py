#!/usr/bin/env python3
"""Test argument handling without opening any real device or replacing the SDK."""
from pathlib import Path
import hashlib
import json
import subprocess

root=Path(__file__).resolve().parents[2]
package=root/'ros2_ws/src/acg138_rplidar'
archive=root/'build/driver-host/libsl_lidar_sdk.a'
if not archive.is_file():
    subprocess.run(['python3',str(package/'tests/run-host-tests.py')],check=True)
build=root/'build/sdk-probe-host-cli'
build.mkdir(parents=True,exist_ok=True)
binary=build/'lidar-scan-probe.host'
source=root/'tools/sdk-probe/lidar_scan_probe.cpp'
command=['g++','-std=c++17','-O2','-Wall','-Wextra','-Wpedantic','-Werror','-pthread',
         '-isystem',str(package/'vendor/sl_lidar_sdk/include'),'-I'+str(package/'include'),
         str(source),str(archive),'-o',str(binary)]
subprocess.run(command,check=True)
cases=[(['--help'],0),(['--version'],0),(['--port','relative'],1),(['--port'],1),
       (['--seconds'],1),(['--seconds','0'],1),(['--seconds','-1'],1),
       (['--seconds','3601'],1),(['--seconds','nan'],1),(['--seconds','inf'],1),
       (['--seconds','abc'],1),(['--seconds','10x'],1),(['--baudrate','115200'],1),
       (['--baudrate','256000'],1),(['--unknown'],1)]
results=[]
for arguments,expected in cases:
    result=subprocess.run([str(binary)]+arguments,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=5)
    assert result.returncode==expected,(arguments,result.returncode,result.stdout)
    assert 'PROBE version=' not in result.stdout,'Must not open a sensor during argument-only tests'
    results.append({'args':arguments,'expected_exit':expected,'actual_exit':result.returncode,'output':result.stdout})
assert 'SDK=2.1.0 baud=460800 architecture=host-test' in results[1]['output']
report={'status':'PASS','hardware_tested':False,'cases':results,'compile_command':command,
        'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
(build/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print('PASS native host CLI tests:',len(results),'cases; no serial device was opened')
