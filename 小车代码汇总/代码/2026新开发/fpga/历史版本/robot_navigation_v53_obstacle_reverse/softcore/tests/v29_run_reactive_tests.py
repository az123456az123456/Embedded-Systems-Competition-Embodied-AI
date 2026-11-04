#!/usr/bin/env python3
"""Reproduce v29 reactive gate tests in host C, production RTL and actual RV32."""
import hashlib,json,os,re,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
REPORTS=ROOT/'reports';REPORTS.mkdir(exist_ok=True)
ENV={**os.environ,'ASAN_OPTIONS':'detect_leaks=0','CCACHE_TEMPDIR':'/tmp','CCACHE_DIR':'/tmp/acg138-v29-ccache'}
SOURCES=['rtl/motor_remote_control.v','rtl/motor_encoder_sync.v','softcore/robot_nav_soc.v','softcore/vendor/picorv32.v','softcore/firmware/layered_control.c','softcore/firmware/layered_control.h','softcore/firmware/layered_mmio.c','softcore/firmware/layered_mmio.h','softcore/firmware/firmware.hex','softcore/tests/v29_reactive_gate_test.c','softcore/tests/v29_tb_reactive_rv32.sv','tests/tb_reactive_motor_v29.sv','tests/tb_layered_motor_v28.sv']
RTL=['softcore/vendor/picorv32.v','softcore/robot_nav_soc.v','imu/i2c_reg_master.v','imu/mpu9250_reader.v','imu/imu_sample_filter.v','imu/imu_gmii_source.v','imu/imu_ethernet_tx.v']
SOURCES+=RTL+[str(Path(__file__).relative_to(ROOT))]
h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before={n:h(ROOT/n) for n in sorted(set(SOURCES))};commands=[]
def run(command,name,pass_text=None):
 start=time.monotonic()
 with (REPORTS/name).open('w') as f:subprocess.run(command,cwd=ROOT,env=ENV,stdout=f,stderr=subprocess.STDOUT,check=True)
 if pass_text:assert pass_text in (REPORTS/name).read_text(),name
 commands.append({'command':command,'cwd':'FPGA project','log':name,'wall_s':round(time.monotonic()-start,3)})
run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic','-fsanitize=address,undefined','-fno-omit-frame-pointer','softcore/tests/v29_reactive_gate_test.c','softcore/firmware/layered_control.c','-o','build/v29-reactive-gate-host'],'v29-reactive-gate-host-compile.log')
run(['build/v29-reactive-gate-host'],'v29-reactive-gate-host.log','PASS v29 production C reactive gates')
run(['iverilog','-g2012','-s','tb_reactive_motor_v29','-o','build/v29-reactive-motor','rtl/motor_remote_control.v','rtl/motor_encoder_sync.v','tests/tb_reactive_motor_v29.sv'],'v29-reactive-motor-compile.log')
run(['vvp','build/v29-reactive-motor'],'v29-reactive-motor.log','PASS v29 raw200mm floor')
run(['make','test-sensor-mmio','test-layered-v28'],'v29-layered-rtl-regression.log','PASS v28 actualSoC coherent')
run(['verilator','--binary','--timing','-Wno-fatal','--top-module','v29_tb_reactive_rv32','--Mdir','build/v29_reactive_rv32','-CFLAGS','-std=c++20',*RTL,'softcore/tests/v29_tb_reactive_rv32.sv'],'v29-reactive-rv32-compile.log')
run(['build/v29_reactive_rv32/Vv29_tb_reactive_rv32'],'v29-reactive-rv32.log','PASS v29 actual source/filter/IMU9->MMIO->RV32 reactive control')
after={n:h(ROOT/n) for n in before};assert before==after,'Production/test input changed during run'
log=(REPORTS/'v29-reactive-rv32.log').read_text();m=re.search(r'cycles=(\d+) frames=(\d+) commits=(\d+) max_commit_cycles=(\d+)',log);assert m
cycles,frames,commits,gap=map(int,m.groups());assert gap<1200000
r={'version':'v29-reactive-gate-tests','result':'PASS','hardware_tested':False,'fresh_execution':True,'firmware_hex_sha256':h(ROOT/'softcore/firmware/firmware.hex'),'source_sha256':before,'evidence_sha256':{c['log']:h(REPORTS/c['log']) for c in commands},'commands':commands,'actual_rv32':{'cycles':cycles,'frames':frames,'commits':commits,'max_commit_cycles':gap,'max_commit_ms':gap/50000,'clock_hz':50000000},'covered':['C ASAN/UBSAN dynamic 200mm+speed/age/braking gate','RTL raw200 floor and corresponding side; all3 online; independent CPU/external lease','low20mmps versus300mmps at250mm; stationary turn forward-component cancellation; measured forward coasting','negativeZ real producer/filter/IMU9/MMIO/actual RV32 calibration; sonar WAIT leaves cal/ready; real timestamps expire','opposite side recovery and genuine encoder impossible jump hard stop','MOT1..4 CRC/FCS/CDC/sharedsequence/watchdog, actual UART seq/KF sourceage, atomic MMIO commit and CPU60ms'], 'limitations':['No physical board or actual I2C bus: raw reader values forced, remaining producer chain production','Sonar/encoder environment in RV32 reactive fixture directly drives raw SoC inputs; independent RTL UART test covers parser/source chain','Complete synthesized graph firmware A* selftest is not claimed here']}
(REPORTS/'v29-reactive-tests.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ('source_sha256','evidence_sha256','commands')},indent=2))
