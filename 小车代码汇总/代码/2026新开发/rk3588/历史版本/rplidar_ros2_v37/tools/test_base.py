#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1];P=ROOT/'ros2_ws/src/acg138_base_telemetry'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
files=[*P.glob('include/**/*.hpp'),*P.glob('vendor/*.[ch]'),*P.glob('tests/*.cpp'),Path(__file__)]
inputs={str(p.relative_to(ROOT)):sha(p) for p in files}
with tempfile.TemporaryDirectory(prefix='v37-base-') as tmp:
 b=Path(tmp);objects=[]
 for name in ['sensor_data','imu_data']:
  obj=b/(name+'.o');subprocess.run(['gcc','-std=c11','-O2','-Wall','-Wextra','-Werror','-I'+str(P/'vendor'),'-c',str(P/'vendor'/(name+'.c')),'-o',str(obj)],check=True);objects.append(str(obj))
 binary=b/'test-base';subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(P/'include'),'-I'+str(P/'vendor'),str(P/'tests/test_odometry.cpp'),*objects,'-lm','-o',str(binary)],check=True)
 out=subprocess.check_output([str(binary)],text=True);print(out,end='')
 kernel=b/'test-kernel';subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-pthread','-I'+str(P/'include'),str(P/'tests/test_kernel_timestamp.cpp'),'-o',str(kernel)],check=True)
 actual=subprocess.check_output([str(kernel)],text=True);print(actual,end='');out+=actual
assert inputs=={n:sha(ROOT/n) for n in inputs}
(ROOT/'reports/base-test.json').write_text(json.dumps({'result':'PASS','hardware_tested':False,'ROS_node_compiled':False,'output':out,'source_sha256':inputs},indent=2)+'\n')
