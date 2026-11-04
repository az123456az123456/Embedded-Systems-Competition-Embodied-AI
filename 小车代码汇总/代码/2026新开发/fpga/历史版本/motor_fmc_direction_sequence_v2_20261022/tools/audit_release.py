#!/usr/bin/env python3
import argparse,hashlib,html,json,re,shutil,xml.etree.ElementTree as ET
from pathlib import Path
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(s):
    return [[html.unescape(re.sub('<[^>]+>','',c)).strip()for c in re.findall(r'<t[dh][^>]*>(.*?)</t[dh]>',r,re.S)]for r in re.findall(r'<tr[^>]*>(.*?)</tr>',s,re.S)]
def section(s,a,b):return s.split('name="'+a+'"',1)[1].split('name="'+b+'"',1)[0]
args=argparse.ArgumentParser();args.add_argument('--build-dir',required=True,type=Path);args=args.parse_args()
project=Path(__file__).resolve().parents[1];build=args.build_dir;name='fmc_direction_sequence_v2';pnr=build/'impl/pnr'
log=(build/'gowin.log').read_text()
assert 'Bitstream generation completed' in log and '[100%] Power analysis completed' in log
assert not re.search(r'^ERROR',log,re.M)
inputs=re.findall(r'^add_file (\S+)$',(project/'build.tcl').read_text(),re.M)
assert set(inputs)=={n.attrib['path']for n in ET.parse(project/(name+'.gprj')).findall('.//File')}
inputs+=['build.tcl','project_settings.tcl','rom/messages.hex']
hashes={f:sha(project/f)for f in inputs};assert hashes=={f:sha(build/f)for f in inputs}
locations=dict(re.findall(r'IO_LOC "([^"]+)" (\w+);',(project/'fmc_direction_sequence.cst').read_text()))
pin_markup=(pnr/(name+'.pin.html')).read_text()
pins={r[0]:r for r in rows(pin_markup.split('name="Pinout_by_Port_Name"',1)[1].split('</table>',1)[0])[1:]}
assert len(pins)==18 and locations.keys()==pins.keys()
for port,ball in locations.items():
    assert pins[port][2].split('/')[0]==ball and pins[port][3]=='Y'
    assert pins[port][7]=='LVCMOS33' and pins[port][17]=='3.3'
attributes={'IO_TYPE':7,'DRIVE':8,'PULL_MODE':9,'BANK_VCCIO':17}
for port,attrs in re.findall(r'IO_PORT "([^"]+)" ([^;]+);',(project/'fmc_direction_sequence.cst').read_text()):
    for key,value in re.findall(r'(\w+)=([\w.]+)',attrs):assert pins[port][attributes[key]]==value,(port,key)
markup=(pnr/(name+'_tr_content.html')).read_text()
summary=dict(r[:2]for r in rows(section(markup,'STA_Tool_Run_Summary','Clock_Report'))if len(r)==2)
timing={'setup_violated_endpoints':int(summary['Numbers of Setup Violated Endpoints']),'hold_violated_endpoints':int(summary['Numbers of Hold Violated Endpoints'])}
assert timing['setup_violated_endpoints']==timing['hold_violated_endpoints']==0
for kind,end in [('Setup','Hold_Slack_Table'),('Hold','Recovery_Slack_Table')]:
    slacks=[float(r[1])for r in rows(section(markup,kind+'_Slack_Table',end))if len(r)>1 and r[0].isdigit()]
    assert slacks and min(slacks)>=0;timing[kind.lower()+'_minimum_reported_slack_ns']=min(slacks)
resources={}
report=(pnr/(name+'.rpt.txt')).read_text()
for key in ['Logic','Register','BSRAM']:
    m=re.search(r'^  '+key+r' +\| (\d+)/(\d+)',report,re.M);assert m,key
    resources[key]={'used':int(m[1]),'total':int(m[2])}
out=project/'reports';out.mkdir(exist_ok=True);(project/'impl/pnr').mkdir(parents=True,exist_ok=True)
fs=pnr/(name+'.fs');shutil.copy2(fs,project/'impl/pnr'/fs.name)
for p in pnr.glob(name+'*'):
    if p.is_file() and p.suffix in {'.html','.txt','.log','.timing_paths'}:shutil.copy2(p,out/p.name)
shutil.copy2(build/'gowin.log',out/'gowin.log')
sequence=json.loads((project/'test_sequence.json').read_text())
result={'date':'2026-10-22','project':name,'device':'GW5AT-LV60PG484AC1/I0','tool':'Gowin V1.9.12.03',
 'bitstream':'impl/pnr/'+fs.name,'bitstream_sha256':sha(fs),'bitstream_bytes':fs.stat().st_size,
 'synthesis_inputs':hashes,'pins_verified':len(pins),'timing':timing,'resources':resources,'sequence':sequence,
 'physical_hardware_tested':False,'cpu_used':False,'rk_or_phone_required':False,'automatic_repeat':False,
 'warnings':[l for l in log.splitlines()if l.startswith('WARN')],
 'limits':['Direction labels are hypotheses until individual wheel observations.','PWM percentage is not measured wheel speed.','No encoder feedback or autonomous navigation.','External board-level electrical and acoustic validation not performed.']}
(out/'release_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
(out/'source_and_fs_SHA256SUMS.txt').write_text(''.join(h+'  '+f+'\n'for f,h in {**hashes,'impl/pnr/'+fs.name:sha(fs)}.items()))
print(json.dumps({k:result[k]for k in ['bitstream_sha256','pins_verified','timing','resources']},indent=2))
