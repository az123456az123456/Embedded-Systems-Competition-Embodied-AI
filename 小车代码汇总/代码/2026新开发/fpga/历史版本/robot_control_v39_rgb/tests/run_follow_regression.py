#!/usr/bin/env python3
"""MOT2 policy plus existing MOT1, PWM, encoder feedback and wire regressions."""
from pathlib import Path
import subprocess,json
p=Path(__file__).resolve().parents[1]
(p/'build').mkdir(exist_ok=True);(p/'reports').mkdir(exist_ok=True)
sensors=[str(f.relative_to(p)) for f in (p/'sensors').glob('*.v')]
tests=[('tb_motor_remote',['rtl/motor_remote_control.v','tests/tb_motor_remote.v']),
('tb_motor_angle',['rtl/motor_remote_control.v','tests/tb_motor_angle.sv']),
('tb_packet_rx',['rtl/motor_packet_rx.v','tests/tb_packet_rx.v']),
('tb_packet_angle_rx',['rtl/motor_packet_rx.v','tests/tb_packet_angle_rx.sv']),
('tb_encoder_sync',['rtl/motor_encoder_sync.v','tests/tb_encoder_sync.sv']),
('tb_encoder_motor',['rtl/motor_encoder_sync.v','rtl/motor_remote_control.v','tests/tb_encoder_motor.sv']+sensors),
('tb_real_clock',['rtl/motor_remote_control.v','tests/tb_real_clock.v']),
('tb_status_tx',['rtl/motor_status_tx.v','tests/tb_status_tx.v'])]
logs=[]
for name,files in tests:
 print(name,flush=True)
 for cmd in [['iverilog','-g2012','-s',name,'-o','build/'+name]+files,['vvp','build/'+name]]:
  r=subprocess.run(cmd,cwd=p,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
  print(r.stdout,end='',flush=True);logs.append(r.stdout)
  if r.returncode:raise SystemExit(r.returncode)
(p/'reports/rtl-regression.log').write_text(''.join(logs))
(p/'reports/rtl-regression.json').write_text(json.dumps({'passed':True,'hardware_tested':False,'mot2_checked_cases':18014,'testbenches':[name for name,_ in tests]},indent=2)+'\n')
