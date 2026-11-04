#!/usr/bin/env python3
"""Generate fixed UART ROM and a human-readable test/record table."""
from pathlib import Path
import json
root=Path(__file__).resolve().parents[1]
names=[('M1_ONLY','M1 单轮：右前'),('M2_ONLY','M2 单轮：左后'),
       ('M3_ONLY','M3 单轮：右后'),('M4_ONLY','M4 单轮：左前'),
       ('FORWARD','前进'),('BACKWARD','后退'),('SPIN_LEFT_CCW','原地左转／逆时针'),
       ('SPIN_RIGHT_CW','原地右转／顺时针'),('FRONT_LEFT_ARC','向左前方弧线行驶'),
       ('FRONT_RIGHT_ARC','向右前方弧线行驶'),('BACK_LEFT_ARC','倒向左后方'),('BACK_RIGHT_ARC','倒向右后方')]
commands=[[40,0,0,0],[0,40,0,0],[0,0,40,0],[0,0,0,40],[40]*4,[-40]*4,
          [40,-40,40,-40],[-40,40,-40,40],[40,20,40,20],[20,40,20,40],
          [-40,-20,-40,-20],[-20,-40,-20,-40]]
rows=[]
messages=['BOOT DIRECTION DIAG: RF=M1 LR=M2 RR=M3 LF=M4; LEFT=0xA INV=0x9; WAIT 8s; ONE SHOT']
for index,((tag,label),duties) in enumerate(zip(names,commands),1):
    in1=sum((1<<m) for m,v in enumerate(duties) if v and not ((v<0)^bool(9&(1<<m))))
    in2=sum((1<<m) for m,v in enumerate(duties) if v and ((v<0)^bool(9&(1<<m))))
    messages.append(f'STEP {index:02d} {tag}: M1={duties[0]:+d} M2={duties[1]:+d} M3={duties[2]:+d} M4={duties[3]:+d} IN1={in1:X} IN2={in2:X}')
    rows.append({'step':index,'name':tag,'expected_action':label,'duties_percent_M1_to_M4':duties,
                 'led_D3_D2_D1_D0':format(index,'04b'),'in1_hex':format(in1,'X'),'in2_hex':format(in2,'X'),
                 'observed_wheel_or_motion':'','observed_correct':None})
messages.append('DONE: ALL PWM=0; STOPPED; NOT REPEATING. Reload SRAM bitstream to run again.')
rom=[]
for msg in messages+['']*(16-len(messages)):
    data=msg.encode('ascii');assert len(data)<=126
    rom.extend(data.ljust(126,b' ')+b'\r\n')
(root/'rom/messages.hex').write_text(''.join(f'{v:02x}\n' for v in rom))
(root/'test_sequence.json').write_text(json.dumps({'wheel_map':{'M1':'right_front','M2':'left_rear','M3':'right_rear','M4':'left_front'},'left_mask':10,'invert_mask':9,'physical_directions_verified':False,'steps':rows},ensure_ascii=False,indent=2)+'\n')
table='步骤,预期动作,M1占空比%,M2占空比%,M3占空比%,M4占空比%,D3D2D1D0,实际车轮或动作,方向正确与否\n'
for row in rows:table+=','.join(map(str,[row['step'],row['expected_action'],*row['duties_percent_M1_to_M4'],row['led_D3_D2_D1_D0'],'','']))+'\n'
(root/'测试顺序与记录.csv').write_text('\ufeff'+table)
