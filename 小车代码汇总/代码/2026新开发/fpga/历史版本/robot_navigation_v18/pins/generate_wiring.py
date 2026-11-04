#!/usr/bin/env python3
"""Regenerate the P3 manifest, CSV and matching SVG/PNG wiring diagram.

Uses the already-audited adapter -> FMC -> FPGA join, never the old xlsx balls.
PNG rendering uses Pillow; SVG stays code-native and editable.
"""
from pathlib import Path
from xml.sax.saxutils import escape
import csv, hashlib, io, json
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REFERENCE = ROOT.parent/'motor/fmc_four_test/pins/adapter_acg138_mapping.json'
if not REFERENCE.is_file():
    REFERENCE=HERE/'adapter_acg138_mapping_reference.json'
reference_bytes=REFERENCE.read_bytes()
adapter=json.loads(reference_bytes)
(HERE/'adapter_acg138_mapping_reference.json').write_bytes(reference_bytes)
roles={
 1:('motor','M1 PWM','motor_pwm[0]'),2:('motor','M1 IN1','motor_in1[0]'),5:('motor','M1 IN2','motor_in2[0]'),
 6:('motor','M2 PWM','motor_pwm[1]'),7:('motor','M2 IN1','motor_in1[1]'),9:('motor','M2 IN2','motor_in2[1]'),
 10:('motor','M3 PWM','motor_pwm[2]'),28:('motor','M3 IN1','motor_in1[2]'),31:('motor','M3 IN2','motor_in2[2]'),
 16:('motor','M4 PWM','motor_pwm[3]'),19:('motor','M4 IN1','motor_in1[3]'),39:('motor','M4 IN2','motor_in2[3]'),
 3:('encoder','E1A / M1 A','encoder_a[0]'),4:('encoder','E1B / M1 B','encoder_b[0]'),
 13:('encoder','E2A / M2 A','encoder_a[1]'),14:('encoder','E2B / M2 B','encoder_b[1]'),
 17:('encoder','E3A / M3 A','encoder_a[2]'),18:('encoder','E3B / M3 B','encoder_b[2]'),
 21:('encoder','E4A / M4 A','encoder_a[3]'),22:('encoder','E4B / M4 B','encoder_b[3]'),
 23:('reserved','SSPI 相关 / 本版保留',''),24:('reserved','SSPI 相关 / 本版保留',''),
}
for motor, position in [(1,'右前'),(2,'左后'),(3,'右后'),(4,'左前')]:
 for pin,(role,label,port)in list(roles.items()):
  if role=='motor'and label.startswith(f'M{motor} '):roles[pin]=(role,label.replace(f'M{motor} ',f'M{motor} {position} '),port)
  if role=='encoder'and label.startswith(f'E{motor}'):
   roles[pin]=(role,label.replace(f'/ M{motor} ',f'/ M{motor} {position} '),port)
source={r['pin']:r for r in adapter['pins']if r['interface']=='GPIO0/P3'}
power={11:('power5','5V 电源'),12:('gnd','GND 共地'),29:('power3','3.3V 电源'),30:('gnd','GND 共地')}
rows=[]
for pin in range(1,41):
 if pin in power:
  role,label=power[pin]
  rows.append(dict(pin=pin,role=role,label=label,rtl_port=None,gpio=None,fpga_ball=None,fmc_contact=None,bank=None,fpga_direction=None,logic_v=None,existing=True))
 else:
  r=source[pin];role,label,port=roles.get(pin,('free','未分配',''))
  rows.append(dict(pin=pin,role=role,label=label,rtl_port=port or None,gpio=r['gpio'],fpga_ball=r['ball'],fmc_contact=r['fmc'],bank=r['bank'],fpga_direction='output'if role=='motor'else 'input'if role in ('encoder','sonar')else None,logic_v=3.3 if role in ('motor','encoder','sonar')else None,existing=role in ('motor','free','reserved')))
manifest={
 'version':'v15','date':'2026-10-22','interface':'FMC adapter GPIO0/P3, 2x20',
 'physical_view':'Pin-number relationship only; locate Pin1 using the actual adapter silkscreen. Not a component-side/top-view claim.',
 'sources':{'adapter_reference':'adapter_acg138_mapping_reference.json','adapter_reference_sha256':hashlib.sha256(reference_bytes).hexdigest(),'adapter_pdf':'FMCtoGPIO-241118.pdf pages1-2','device_package_reference':'p3_device_pin_functions.json','constraints':'../robot_fmc_remote.cst','protected_baseline':'../tools/baseline_v14/robot_fmc_remote.cst'},
 'wheel_positions':{'status':'user_confirmed_standalone_v2','M1':'right_front','M2':'left_rear','M3':'right_rear','M4':'left_front','left_mask':10,'motor_invert_mask':15,'motor_numbers':'E1/E2/E3/E4 correspond to M1/M2/M3/M4; electrical pin numbers unchanged'},
 'encoder':{'confirmed_signal_voltage_v':3.3,'channels':4,'phases':['A','B'],'counts_per_ab_cycle':4,'sample_window_ms':50,'positive_ab_sequence':['00','10','11','01','00'],'invert_parameter':'ENCODER_INVERT_MASK','default_invert_mask':0,'physical_forward_sign':'must verify on actual wheels; independent of motor invert mask','wheel_diameter_ppr_gear_ratio_known':False},
 'sonar':{'model':'A02YYUW','channels':['front','left','right'],'supply_v':3.3,'rx_mode_pin':'leave floating and insulate:500ms processed-value mode, matching previous standalone A02 wiring','wiring_file':'a02yyuw_wiring.json','uart_inputs':'GPIO1/P2-1 Y18/front, P2-3 N20/left, P2-5 U20/right','baud':9600,'uart':'8N1','frame':'FF H L SUM; SUM=(FF+H+L)&FF','distance_mm_range':[30,4500],'distance_semantics':'echo range along each physical sensor, not an automatically associated selected YOLO object'},
 'power':{'pin11':'5V','pin12':'GND','pin29':'3.3V','pin30':'GND','vccadj_requirement_v':3.3,'capacity':'not measured; use a stable sufficient 3.3V supply and common ground; do not parallel independent regulator outputs'},
 'pins':rows,
}
(HERE/'p3_sensor_mapping.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
with (HERE/'p3_wiring.csv').open('w',encoding='utf-8-sig',newline='')as f:
 w=csv.writer(f);w.writerow(['P3针号','类别','接线信号','RTL端口','FPGA球位','FMC触点','Bank','FPGA方向','信号电平V','说明'])
 for r in rows:
  w.writerow([r['pin'],r['role'],r['label'],r['rtl_port']or '',r['fpga_ball']or '',r['fmc_contact']or '',r['bank']or '',r['fpga_direction']or '',r['logic_v']or '', '原有电机接线'if r['role']=='motor'else '新增3.3V输入'if r['role']in('encoder','sonar')else '未约束，暂不使用'if r['role']=='free'else 'SSPI相关脚，本版本保留未接；组复用设置见接线说明'if r['role']=='reserved'else '硬件电源/地，无HDL端口'])

# Emit SVG and PNG from identical native layout primitives.
WIDTH,HEIGHT=1600,1710
im=Image.new('RGB',(WIDTH,HEIGHT),'#f5f6f8');draw=ImageDraw.Draw(im)
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">','<style>text{font-family:"Noto Sans CJK SC","Noto Sans CJK JP",sans-serif}</style>']
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BOLD='/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc'
fonts={}
def font(size,bold=False):
 key=(size,bold)
 if key not in fonts:fonts[key]=ImageFont.truetype(BOLD if bold else FONT,size)
 return fonts[key]
def rect(x,y,w,h,fill,stroke=None,r=0):
 svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}"'+(f' stroke="{stroke}"'if stroke else '')+'/>')
 draw.rounded_rectangle([x,y,x+w,y+h],radius=r,fill=fill,outline=stroke,width=1)
def text(value,x,y,size=24,color='#20252c',anchor='left',bold=False):
 svga={'left':'start','center':'middle','right':'end'}[anchor]
 svg.append(f'<text x="{x}" y="{y+size}" font-size="{size}" fill="{color}" text-anchor="{svga}" font-weight="{600 if bold else 400}">{escape(value)}</text>')
 f=font(size,bold);bbox=draw.textbbox((0,0),value,font=f);width=bbox[2]-bbox[0]
 tx=x if anchor=='left'else x-width/2 if anchor=='center'else x-width
 draw.text((tx,y-bbox[1]+3),value,font=f,fill=color)
def line(x1,y1,x2,y2,color='#c7ccd2',width=2):
 svg.append(f'<path d="M{x1} {y1}L{x2} {y2}" stroke="{color}" stroke-width="{width}" fill="none"/>')
 draw.line((x1,y1,x2,y2),fill=color,width=width)
rect(0,0,WIDTH,HEIGHT,'#f5f6f8')
text('P3 接线图 · 电机 / 编码器',50,32,43,bold=True)
text('GPIO0 · 2×20 排针 · ACG138 / FMCtoGPIO · v15',52,96,25,'#5c6572')
text('先找实物 Pin1 丝印，再按针号接线；图中左右不代表实物观察面。',52,141,24,'#5c6572')
colors={'motor':('#276bba','#edf4ff'),'encoder':('#7251af','#f3eefb'),'sonar':('#16866b','#eaf7f2'),'power5':('#b33b39','#fdeeee'),'power3':('#a16d0b','#fff4d8'),'gnd':('#3c4650','#eef0f3'),'free':('#828b96','#f6f7f9'),'reserved':('#9d5c28','#fff1e5')}
for x,role,label in [(55,'motor','电机输出'),(380,'encoder','编码器输入'),(740,'gnd','供电 / 共地'),(1090,'free','未分配'),(1350,'reserved','保留未接')]:
 fg,bg=colors[role];rect(x,195,24,24,fg,r=6);text(label,x+37,188,24)
rect(35,246,1530,1120,'#ffffff','#e4e7eb',18)
text('奇数列',360,260,22,'#68717d',anchor='center')
text('针号',800,260,22,'#68717d',anchor='center')
text('偶数列',1240,260,22,'#68717d',anchor='center')
for row in range(20):
 y=310+row*51
 for side in (0,1):
  r=rows[row*2+side];fg,bg=colors[r['role']]
  x=55 if side==0 else 916;pinx=738 if side==0 else 862
  rect(x,y,629,42,bg,r=8)
  if r['fpga_ball']:
   label=f"{r['label']}   |   {r['fpga_ball']}   |   FMC {r['fmc_contact']}"
  else:label=r['label']+'  ·  电源网络'
  text(label,x+14,y+5,21,fg,bold=r['role']!='free')
  rect(pinx-23,y+1,46,40,fg,r=6)
  text(str(r['pin']),pinx,y+4,24,'#ffffff',anchor='center',bold=True)
  if side==0:line(684,y+21,pinx-24,y+21,fg)
  else:line(pinx+24,y+21,916,y+21,fg)
text('图例中的球位是 FPGA 焊球；FMC 是连接器触点，两者不是同一列。',55,1388,24,'#515c69')
text('P3-11 = 5V；P3-29 = 3.3V；P3-12 / 30 = GND。',55,1434,25,'#3b4654',bold=True)
text('A02 TX 沿用 P2-1 / 3 / 5；P3-25 / 26 / 27 在本版不再接超声波。',55,1480,24,'#515c69')
text('E1～E4 对应 M1～M4；M1右前 / M2左后 / M3右后 / M4左前。',55,1526,24,'#515c69')
text('3.3V 信号输入；先确认 VCCADJ 约 3.3V。不要把不同稳压输出直接并联。',55,1572,24,'#515c69')
text('P3 有 20 路工作信号；23 / 24 本版保留未接，另有 14 路暂未分配。',55,1618,24,'#515c69')
svg.append('</svg>')
(HERE/'p3_wiring.svg').write_text('\n'.join(svg)+'\n')
im.save(HERE/'p3_wiring.png')
print('Generated p3_sensor_mapping.json, p3_wiring.csv, p3_wiring.svg, p3_wiring.png')


# Preserve the user's complete original A02 wiring, including power and RX.
a02=[]
p2map={r['pin']:r for r in adapter['pins']if r['interface']=='GPIO1/P2'}
for channel,zh,pin,vcc_header,vcc_pin,gnd_header,gnd_pin in [
 ('front','前方',1,'P3',29,'P2',12),('left','左侧',3,'P10',24,'P10',23),('right','右侧',5,'P2',29,'P2',30)]:
 r=p2map[pin]
 a02.append(dict(channel=channel,direction=zh,model='A02YYUW',tx_header='P2',tx_pin=pin,fpga_ball=r['ball'],fmc_contact=r['fmc'],bank=r['bank'],gpio=r['gpio'],rtl_port=f"sonar_rx[{len(a02)}]",vcc_header=vcc_header,vcc_pin=vcc_pin,vcc_v=3.3,gnd_header=gnd_header,gnd_pin=gnd_pin,rx='floating_insulated',response_mode='processed',response_ms=500))
a02_manifest={'version':'v15','date':'2026-10-22','connection_policy':'Preserve the existing standalone a02yyuw wiring in full. Sensor RX pins are left floating and insulated.','channels':a02,'uart':'9600 8N1','frame':'FF H L SUM','checksum':'(FF+H+L)&FF','units':'distance from each physical transducer to reflecting obstacle, millimetres; not selected YOLO-object depth','range_mm':[30,4500],'supply_v':3.3,'ground':'all supplies and FPGA share a common reference ground; do not parallel independent regulators','configuration_note':'N20 is SSPI_WPN/SSI2. This v15 bitstream explicitly enables SSPI-port reuse as GPIO after configuration, while MSPI Flash and JTAG modes are preserved.','installation':'Download the new v15 bitstream. Existing matching RK application and Android APK continue to work.'}
(HERE/'a02yyuw_wiring.json').write_text(json.dumps(a02_manifest,ensure_ascii=False,indent=2)+'\n')
with (HERE/'a02yyuw_wiring.csv').open('w',encoding='utf-8-sig',newline='')as f:
 w=csv.writer(f);w.writerow(['传感器方向','传感器引线','接到接口','针号','FPGA球位','FMC触点','Bank','说明'])
 for r in a02:
  w.writerow([r['direction'],'VCC',r['vcc_header'],r['vcc_pin'],'','','','3.3V，沿用原供电'])
  w.writerow([r['direction'],'GND',r['gnd_header'],r['gnd_pin'],'','','','共地，沿用原接地'])
  w.writerow([r['direction'],'RX','悬空并绝缘','','','','','约500ms处理值模式，不接FPGA'])
  w.writerow([r['direction'],'TX','P2',r['tx_pin'],r['fpga_ball'],r['fmc_contact'],r['bank'],r['rtl_port']+'，3.3V UART输出到FPGA输入'])
WIDTH,HEIGHT=1600,1120
im=Image.new('RGB',(WIDTH,HEIGHT),'#f5f6f8');draw=ImageDraw.Draw(im)
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">','<style>text{font-family:"Noto Sans CJK SC","Noto Sans CJK JP",sans-serif}</style>']
rect(0,0,WIDTH,HEIGHT,'#f5f6f8')
text('三路 A02YYUW · 沿用原接法 · v15',50,32,43,bold=True)
text('前 / 左 / 右独立接收；三根 RX 默认悬空绝缘，约 500ms 处理值模式。',52,100,26,'#5c6572')
text('需要下载 v15 新位流；RK 程序和手机 APK 继续使用当前配套版本。',52,150,25,'#5c6572')
for i,r in enumerate(a02):
 x=40+i*520
 rect(x,220,480,660,'#ffffff','#e4e7eb',18)
 text(r['direction']+'传感器',x+26,248,36,bold=True)
 text('A02YYUW · UART 9600 / 8N1',x+26,302,22,'#5c6572')
 wires=[('VCC',f"{r['vcc_header']}-{r['vcc_pin']} · 3.3V",'沿用原供电','#a16d0b','#fff4d8'),('GND',f"{r['gnd_header']}-{r['gnd_pin']} · GND",'所有设备共地','#3c4650','#eef0f3'),('RX','悬空并绝缘','不占 FPGA 针脚','#828b96','#f2f4f6'),('TX',f"P2-{r['tx_pin']} · {r['fpga_ball']}",f"FMC {r['fmc_contact']} · Bank{r['bank']}",'#16866b','#eaf7f2')]
 for k,(label,destination,note,fg,bg)in enumerate(wires):
  y=358+k*112
  rect(x+23,y+8,78,43,fg,r=8);text(label,x+62,y+13,23,'#ffffff',anchor='center',bold=True)
  line(x+101,y+29,x+141,y+29,fg,3)
  rect(x+141,y,314,83,bg,r=9)
  text(destination,x+156,y+10,24,fg,bold=True);text(note,x+156,y+47,20,'#637080')
 text('看接口功能标识，不按线色猜测。',x+26,832,22,'#5c6572')
text('TX 是传感器输出、FPGA 输入；三只 TX 分别接，不能并接。',50,920,25,'#3b4654',bold=True)
text('供电为 3.3V、所有设备共地；FMC Bank 的实际 VCCADJ 须匹配 3.3V。',50,969,24,'#515c69')
text('超声波显示各方向反射障碍物距离，不自动绑定某个 YOLO 目标。',50,1018,24,'#515c69')
text('Pin1 方向以实物丝印为准；示意图仅表达电气连接关系。',50,1067,22,'#68717d')
svg.append('</svg>')
(HERE/'a02yyuw_wiring.svg').write_text('\n'.join(svg)+'\n')
im.save(HERE/'a02yyuw_wiring.png')
print('Generated original A02 full wiring JSON/CSV/SVG/PNG')
