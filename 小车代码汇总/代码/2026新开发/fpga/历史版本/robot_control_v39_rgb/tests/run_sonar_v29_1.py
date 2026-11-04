#!/usr/bin/env python3
"""Execute the changed sonar boundaries, unchanged production C KF, and RTL regressions."""
from pathlib import Path
import hashlib,json,os,subprocess,time
ROOT=Path(__file__).resolve().parents[1];R=ROOT/'reports';R.mkdir(exist_ok=True);(ROOT/'build').mkdir(exist_ok=True)
h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
paths=[]
for folder in ['sensors','rtl','imu','softcore','tests']:
 for p in (ROOT/folder).rglob('*'):
  if p.is_file() and p.suffix in ('.v','.sv','.vh','.c','.h','.cpp','.py') and not any(n in ('build','reports','__pycache__') for n in p.relative_to(ROOT).parts):paths.append(p)
paths += [ROOT/'softcore/firmware/firmware.hex',ROOT/'Makefile']
source={str(p.relative_to(ROOT)):h(p) for p in sorted(set(paths))};commands=[]
def run(cmd,name,required=None):
 t=time.monotonic()
 with (R/name).open('w') as f:subprocess.run(cmd,cwd=ROOT,check=True,stdout=f,stderr=subprocess.STDOUT,env={**os.environ,'CCACHE_TEMPDIR':'/tmp','CCACHE_DIR':'/tmp/acg138-v29-1-ccache'})
 if required:assert required in (R/name).read_text(),name
 commands.append({'command':cmd,'log':'reports/'+name,'wall_s':round(time.monotonic()-t,3)})
run(['iverilog','-g2012','-s','tb_sonar_zero_uart_v29_1','-o','build/sonar-zero-uart','sensors/a02_uart_rx.v','sensors/ultrasonic_sensor.v','tests/tb_sonar_zero_uart_v29_1.sv'],'v29_1-sonar-uart-compile.log')
run(['vvp','build/sonar-zero-uart'],'v29_1-sonar-uart.log','PASS v29.1 actual UART10bit bytes')
sensors=[str(p.relative_to(ROOT)) for p in sorted((ROOT/'sensors').glob('*.v'))]
run(['verilator','--binary','--timing','-Wno-fatal','--top-module','tb_sonar_zero_capture_v29_1','--Mdir','build/sonar_zero_capture','-CFLAGS','-std=c++20',*sensors,'tests/tb_sonar_zero_capture_v29_1.sv','tests/v29_1_kf_dpi.cpp'],'v29_1-sonar-capture-compile.log')
run(['build/sonar_zero_capture/Vtb_sonar_zero_capture_v29_1'],'v29_1-sonar-capture.log','PASS v29.1 actualthreeUART->SNS1')
run(['make','test-sonar-v26','test-sensor-mmio','test-layered-v28'],'v29_1-base-regression.log','PASS v28 actualSoC coherent')
assert source=={n:h(ROOT/n) for n in source},'Source changed while test executing'
evidence={c['log']:h(ROOT/c['log']) for c in commands}
for n in ['sonar-ma-v26.log','sonar-capture-v26.log','sonar-uart.log','encoder-pipeline-v28.log','layered-motor-v28.log','layered-network-v28.log','status-ack4-v28.log','layered-capture-v28.log','layered-mmio-v28.log']:
 assert 'PASS' in (R/n).read_text(),n;evidence['reports/'+n]=h(R/n)
record={'version':'v29.1-sonar-functional','result':'PASS','hardware_tested':False,'fresh_execution':True,'firmware_hex_sha256':h(ROOT/'softcore/firmware/firmware.hex'),'source_sha256':source,'evidence_sha256':evidence,'commands':commands,'scope':'Actual UART bit timing -> RAW/MA1/2/4 decoders, actual3UART/SNS1 capture with native execution of unchanged production C Kalman fed back through publication, plus independent MA model/UART/encoder/MOT4/MMIO regressions','limitations':['No physical sensor or board','C KF integration uses native DPI execution with CPU ready heldtrue to stress matched publication; not actual RV32 scheduling or I2C','Protocol FCS/CRC16 in Ethernet tests is separate from A02 8bit additive checksum']}
(R/'v29_1-sonar-functional.json').write_text(json.dumps(record,indent=2)+'\n');print('PASS new sonar function/RTL regression',h(R/'v29_1-sonar-functional.json'))
