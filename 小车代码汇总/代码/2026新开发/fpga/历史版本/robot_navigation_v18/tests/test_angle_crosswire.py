#!/usr/bin/env python3
from pathlib import Path
import subprocess,hashlib,json
p=Path(__file__).resolve().parents[1]
rk=p.parent.parent/'rk3588/robot_navigation_v18/dashboard/phone'
ide=p.parent.parent/'Gowin_V1.9.12.03/IDE'
b=p/'build/angle_crosswire';b.mkdir(parents=True,exist_ok=True)
commands=[['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',str(rk),'tests/cross_angle_vectors.cc',str(rk/'motor_link.cc'),'-o',str(b/'vectors')],
[str(b/'vectors'),'generate',str(b/'mot2.hex')],
['iverilog','-g2012','-s','tb_angle_crosswire','-o',str(b/'test'),'rtl/motor_packet_rx.v','rtl/motor_remote_control.v','rtl/motor_status_tx.v','imu/imu_ethernet_tx.v','tests/tb_angle_crosswire.sv',str(ide/'simlib/gw5a/prim_sim.v')],
['vvp',str(b/'test')],[str(b/'vectors'),'verify',str(b/'ack2.hex')]]
logs=[]
for cmd in commands:
 r=subprocess.run(cmd,cwd=p,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
 print(r.stdout,end='',flush=True);logs.append(r.stdout)
 if r.returncode:raise SystemExit(r.returncode)
(p/'reports/angle-crosswire.log').write_text(''.join(logs))
(p/'reports/angle-crosswire.json').write_text(json.dumps({'passed':True,'hardware_tested':False,'command_fixtures':19,'left_mask':10,'invert_mask':15,'rk_wire_source':str(rk/'motor_link.cc'),'rk_source_sha256':hashlib.sha256((rk/'motor_link.cc').read_bytes()).hexdigest(),'scope':'Actual RK encoder -> vendor IDDR -> RX parser/FCS/CRC16 -> CDC -> motor controller -> ACK/NAV CDC -> actual RK decoder with ACK Ethernet FCS checked'},indent=2)+'\n')
