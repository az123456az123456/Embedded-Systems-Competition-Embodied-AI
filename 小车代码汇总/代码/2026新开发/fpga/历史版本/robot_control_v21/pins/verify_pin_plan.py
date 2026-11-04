#!/usr/bin/env python3
"""Independent v15 wiring audit: original adapter/package, old CST, final placement."""
from pathlib import Path
from collections import Counter
from html.parser import HTMLParser
import argparse,hashlib,json,re
parser=argparse.ArgumentParser();parser.add_argument('--pin-report',type=Path)
args=parser.parse_args()
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((HERE/'p3_sensor_mapping.json').read_text());rows=manifest['pins']
a02=json.loads((HERE/'a02yyuw_wiring.json').read_text());sonar=a02['channels']
assert manifest['version']==a02['version']=='v15'
assert [r['pin']for r in rows]==list(range(1,41))
counts=Counter(r['role']for r in rows)
assert counts==dict(motor=12,encoder=8,reserved=2,free=14,power5=1,power3=1,gnd=2),counts
adapter=json.loads((HERE/'adapter_acg138_mapping_reference.json').read_text())
join={r['pin']:r for r in adapter['pins']if r['interface']=='GPIO0/P3'}
p2join={r['pin']:r for r in adapter['pins']if r['interface']=='GPIO1/P2'}
pkg=json.loads((HERE/'p3_device_pin_functions.json').read_text())
facts={r['fpga_ball']:r for r in pkg['pins']+pkg['a02_p2_pins']+pkg['constraint_pins']}
for r in rows:
 if r['pin']in(11,12,29,30):
  assert r['fpga_ball']is None and r['rtl_port']is None;continue
 a=join[r['pin']];f=facts[r['fpga_ball']]
 assert (r['fpga_ball'],r['fmc_contact'],r['bank'],r['gpio'])==(a['ball'],a['fmc'],a['bank'],a['gpio']),r
 assert (r['bank'],f['type'])==(f['bank'],'I/O'),r
 if r['role']=='encoder':
  assert r['bank']in(4,5)and r['logic_v']==3.3 and r['fpga_direction']=='input'
  assert not any(x in f['configuration_function']for x in ('SSI0','SSPI','MSPI')),r
 if r['role']=='reserved':assert r['pin']in(23,24)and r['rtl_port']is None
for column in ('fpga_ball','fmc_contact','gpio'):
 v=[r[column]for r in rows if r[column]is not None];assert len(v)==len(set(v))==36
expected_sonar=[('front',1,'Y18','G2',8,'P3',29,'P2',12),('left',3,'N20','C26',5,'P10',24,'P10',23),('right',5,'U20','C22',8,'P2',29,'P2',30)]
for i,(r,expected)in enumerate(zip(sonar,expected_sonar)):
 actual=tuple(r[k]for k in ('channel','tx_pin','fpga_ball','fmc_contact','bank','vcc_header','vcc_pin','gnd_header','gnd_pin'))
 assert actual==expected,(actual,expected)
 a=p2join[r['tx_pin']];f=facts[r['fpga_ball']]
 assert (r['fpga_ball'],r['fmc_contact'],r['bank'],r['gpio'])==(a['ball'],a['fmc'],a['bank'],a['gpio'])
 assert r['tx_header']=='P2'and r['rtl_port']==f'sonar_rx[{i}]'and r['vcc_v']==3.3
 assert r['rx']=='floating_insulated'and r['response_mode']=='processed'and r['response_ms']==500
 assert f['type']=='I/O'and f['bank']==r['bank']
assert len(sonar)==3
assert 'SSPI_WPN'in facts['N20']['configuration_function']
assert manifest['wheel_positions']['left_mask']==10 and manifest['wheel_positions']['motor_invert_mask']==15

def cst(path):
 text=path.read_text();loc=re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+);',text)
 ports=re.findall(r'IO_PORT\s+"([^"]+)"\s+([^;]+);',text)
 assert len(loc)==len(dict(loc))==len(set(b for _,b in loc)),f'duplicate port/ball in {path}'
 assert len(ports)==len(dict(ports)),f'duplicate IO_PORT in {path}'
 return dict(loc),{p:' '.join(a.split())for p,a in ports}
current=ROOT/'robot_fmc_remote.cst'
baseline=ROOT/'tools/baseline_v14/robot_fmc_remote.cst'
base12=ROOT/'tools/baseline_v12/robot_fmc_remote.cst'
loc,elec=cst(current);old,old_elec=cst(baseline);old12,elec12=cst(base12)
assert len(old)==len(loc)==75 and len(old12)==64
assert set(loc)==set(elec)==set(old)==set(old_elec)
expected_bytes=baseline.read_bytes()
for i,oldball,newball in [(0,'K16','Y18'),(1,'M15','N20'),(2,'L16','U20')]:
 source=f'IO_LOC "sonar_rx[{i}]" {oldball};'.encode();dest=f'IO_LOC "sonar_rx[{i}]" {newball};'.encode()
 assert expected_bytes.count(source)==1
 expected_bytes=expected_bytes.replace(source,dest)
assert current.read_bytes()==expected_bytes,'CST must be byte-identical to v14 except3 sonar IO_LOC replacements'
changed={p:{'old':old[p],'new':loc[p]}for p in loc if loc[p]!=old[p]}
assert set(changed)=={'sonar_rx[0]','sonar_rx[1]','sonar_rx[2]'}
assert elec==old_elec
assert all(loc[p]==b and elec[p]==elec12[p]for p,b in old12.items()),'original64 pin declarations changed'
p3_active=[r for r in rows if r['role']in('motor','encoder')]
occupied=p3_active+sonar
for r in occupied:
 assert loc[r['rtl_port']]==r['fpga_ball'],r
 assert 'IO_TYPE=LVCMOS33'in elec[r['rtl_port']]and 'BANK_VCCIO=3.3'in elec[r['rtl_port']],r
for p in [23,24,25,26,27]:assert rows[p-1]['fpga_ball']not in loc.values()
assert len({r['fpga_ball']for r in occupied})==23
# SSPI reuse is deliberate and group-wide; MSPI/JTAG settings are preserved.
newbuild=(ROOT/'build.tcl').read_text();oldbuild=(ROOT/'tools/baseline_v14/build.tcl').read_text()
def option(text,name):return re.findall(r'^\s*set_option\s+-'+re.escape(name)+r'\s+(\S+)',text,re.M)
assert option(newbuild,'use_sspi_as_gpio')==['1']
assert option(oldbuild,'use_sspi_as_gpio')in([],['0'])
for key in ('use_mspi_as_gpio','use_jtag_as_gpio'):
 assert option(newbuild,key)==option(oldbuild,key)and option(newbuild,key)in([],['0'])
project=(ROOT/'robot_fmc_encoder_sync_v15.gprj').read_text()
assert 'GW5AT-60B'in project and 'GW5AT-LV60PG484AC1/I0'in project
header=(ROOT/'rtl/robot_fmc_remote_top.v').read_text().split(');',1)[0]
directions={}
for line in header.splitlines():
 m=re.search(r'\b(input|output|inout)\s+(?:wire\s+|reg\s+)?(?:\[(\d+):(\d+)\]\s+)?(.+)',line)
 if not m:continue
 for name in m[4].rstrip(' ,').split(','):
  name=name.strip();assert re.fullmatch(r'\w+',name),name
  if m[2]is None:directions[name]=m[1]
  else:
   for n in range(min(int(m[2]),int(m[3])),max(int(m[2]),int(m[3]))+1):directions[f'{name}[{n}]']=m[1]
assert set(directions)==set(loc),'top declarations and75 constrained pins differ'
capture=(ROOT/'sensors/robot_sensor_capture.v').read_text()
assert re.search(r'ENCODER_INVERT_MASK\s*=\s*4\x27b0000',capture)
placed=None
if args.pin_report:
 class Tables(HTMLParser):
  def __init__(self):super().__init__();self.rows=[];self.row=None;self.cell=None
  def handle_starttag(self,tag,attrs):
   if tag=='tr':self.row=[]
   if tag in ('td','th'):self.cell=''
  def handle_data(self,data):
   if self.cell is not None:self.cell+=data
  def handle_endtag(self,tag):
   if tag in ('td','th')and self.cell is not None:
    if self.row is not None:self.row.append(' '.join(self.cell.split()))
    self.cell=None
   if tag=='tr'and self.row is not None:self.rows.append(self.row);self.row=None
 html=Tables();html.feed(args.pin_report.read_text())
 for port,ball in loc.items():
  f=facts[ball];bank=f['bank']
  matching=[r for r in html.rows if len(r)>5 and r[0]==f'{ball}/{bank}'and r[1]==port]
  assert len(matching)==1,(port,ball)
  expected_direction={'input':'in','output':'out','inout':'io'}[directions[port]]
  assert matching[0][2]==expected_direction,(port,matching[0][2],expected_direction)
  io_type=re.search(r'IO_TYPE=(\S+)',elec[port])[1]
  bank_v=re.search(r'BANK_VCCIO=(\S+)',elec[port])[1]
  assert io_type in matching[0]and bank_v in matching[0],(port,io_type,bank_v)
 for ball in ('N22','M22','L16','K16','M15'):
  bank=facts[ball]['bank'];matching=[r for r in html.rows if len(r)>5 and r[0]==f'{ball}/{bank}']
  assert len(matching)==1 and matching[0][1]=='-',ball
 placed={'status':'passed','file':args.pin_report.name,'sha256':sha(args.pin_report),'all_constrained_ports_checked':75,'p3_active_pins_checked':20,'p2_sonar_pins_checked':3,'deliberately_unassigned_balls':['N22','M22','L16','K16','M15'],'N20':'actually placed as3.3V input with SSPI port reuse enabled'}
report={
 'status':'passed_final_placement'if placed else 'passed_cst_package_only','version':'v15','date':'2026-10-22','hardware_tested':False,
 'counts':dict(counts),'p3_active_signal_pins':20,'p3_unallocated_signals':14,'p3_related_sspi_pins_intentionally_unassigned':[23,24],'p2_sonar_input_pins':[1,3,5],
 'total_constraints':75,'original_v14_ports_unchanged':72,'original_v14_sonar_loc_changes':changed,'cst_byte_identity':'exact oldv14 bytes with only3 IO_LOC replacements','all75_io_port_declarations_unchanged':True,'original_v12_pins_preserved':64,
 'configuration_reuse':{'use_sspi_as_gpio':1,'use_mspi_as_gpio':'unchanged default0','use_jtag_as_gpio':'unchanged default0','phase':'configuration pins remain configuration during loading; user-mode SSPI port is reused as GPIO','N20_cfg':facts['N20']['configuration_function'],'evidence':'sspi_mspi_evidence.json'},
 'placed_pin_report':placed,
 'checks':['complete40pinP3 manifest and adapter joins','all36P3 package banks match original reference','threeP2 UART pins match old adapter and package facts','all12 A02 power/GND/RX/TX wire assignments match standalone table','encoder/motor pins unchanged','all75CST balls and ports unique','only3sonar LOC lines differ bytewise from oldv14','all75IO_PORT declarations remain unchanged','SSPIgroupGPIO1 is deliberate; MSPI/JTAG settings unchanged','every constrained port exists in actualtop','left_mask10 motor_invert15 distinct from encoder invert0'],
 'sonar_wiring':sonar,'package_reference':pkg['source'],'package_reference_sha256':pkg['source_sha256'],
 'source_sha256':{str(p.relative_to(ROOT)):sha(p)for p in [current,baseline,base12,ROOT/'build.tcl',HERE/'p3_sensor_mapping.json',HERE/'p3_wiring.csv',HERE/'p3_wiring.svg',HERE/'p3_wiring.png',HERE/'a02yyuw_wiring.json',HERE/'a02yyuw_wiring.csv',HERE/'a02yyuw_wiring.svg',HERE/'a02yyuw_wiring.png',HERE/'p3_device_pin_functions.json',HERE/'adapter_acg138_mapping_reference.json',HERE/'sspi_mspi_evidence.json',ROOT/'README.md']},
 'limits':['No actual board voltage, continuity, encoder direction or coldboot test','unallocated pins are not automatically approved for future use','without --pin-report this does not claim final placement verification']
}
(HERE/'p3_pin_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('PASS v15 independent pin audit:75ports; only3sonarLOC changed; all75IO_PORT identical; P3 motor12+encoder8 preserved; A02 old P2/power/GND/RX wiring verified'+('; all75 finalplaced ports/Bank/voltage/direction verified'if placed else '; placement not checked'))
