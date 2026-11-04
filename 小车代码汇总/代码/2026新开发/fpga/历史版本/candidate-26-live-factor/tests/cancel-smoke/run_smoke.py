#!/usr/bin/env python3
"""Frozen candidate26: related cancellation + actual status-consumer smoke only."""
from pathlib import Path
import argparse,hashlib,json,subprocess,sys,time,statistics,zlib,gzip
sys.dont_write_bytecode=True
D=Path(__file__).resolve().parent;O=D.parent.parent;P=O.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
sys.path.insert(0,str(P/'measurement/supplement-20261101'))
import collect_supplement as c
ap=argparse.ArgumentParser();ap.add_argument('--smoke',action='store_true');args=ap.parse_args()
old=json.loads((P/'physical-candidate-04-pipeline/measurement/design-manifest.json').read_text())
sources=[str(O/'source'/Path(p).relative_to(P/'physical-candidate-04-pipeline/source')) if '/physical-candidate-04-pipeline/source/' in p else p for p in old]
sources += [str(O/'source'/p) for p in ['rtl/motor_status_align.v','rtl/motor_status_tx.v','imu/imu_ethernet_tx.v']]
manifest={p:sha(p) for p in sources};(D/'design-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
all_before={str(p.relative_to(O/'source')):sha(p) for p in (O/'source').rglob('*') if p.is_file() and 'impl' not in p.relative_to(O/'source').parts}
(D/'source-hashes-before.json').write_text(json.dumps(all_before,indent=2)+'\n')
mf={'baseline_manifest':str(D/'design-manifest.json'),'baseline_manifest_sha256':sha(D/'design-manifest.json'),'testbench_sha256':sha(D/'tb_cancel_status.sv'),'setup_tasks_sha256':sha(D/'owner_setup_supplement.svh')}
cmd=['iverilog','-g2012','-s','tb_pipeline_measurement','-I',str(D),'-o',str(D/'test.vvp'),str(D/'tb_cancel_status.sv'),*sources]
r=subprocess.run(cmd,capture_output=True,text=True);(D/'compile.log').write_text(r.stdout+r.stderr)
(D/'compile-result.json').write_text(json.dumps({'command':cmd,'returncode':r.returncode,'manifest':mf},indent=2)+'\n')
if r.returncode:sys.exit(r.returncode)

def wire_frames(path):
 out=[];active=None
 for line in path.read_text().splitlines():
  ts,v,b=line.split(',');ts=int(ts)
  if v=='1':
   if active is None:active={'start_ps':ts,'data':[]}
   active['data'].append(int(b,16));active['end_ps']=ts
  elif active is not None:out.append(active);active=None
 if active is not None:raise ValueError('incomplete telemetry frame')
 return out

def check_telemetry(d):
 ref=wire_frames(d/'reference-status-stream.csv');dut=wire_frames(d/'candidate-status-stream.csv')
 if len(ref)!=len(dut) or not dut:raise ValueError('telemetry frame counts differ or empty')
 packets=[]
 inputs=[json.loads(x) for x in (d/'telemetry-input.jsonl').read_text().splitlines()]
 for i,(a,b)in enumerate(zip(ref,dut)):
  x,y=bytes(a['data']),bytes(b['data']);nav=y[20:22]==b'\x88\xb7';end=48 if nav else 38
  z={'frame_index':i,'kind':'NAV1' if nav else 'ACK1','reference_start_ps':a['start_ps'],'candidate_start_ps':b['start_ps'],'start_delay_ns':(b['start_ps']-a['start_ps'])/1000,'equal_whole_frame':x==y,'crc16_ok':c.crc16(y[22:end])==int.from_bytes(y[end:end+2],'big'),'fcs_ok':zlib.crc32(y[8:-4])==int.from_bytes(y[-4:],'little'),'frame_hex':y.hex(),'sha256':hashlib.sha256(y).hexdigest()}
  z['payload_magic_ok']=y[22:26]==(b'NAV1'if nav else b'ACK1')
  z['payload_matches_same_source_valid_sample']=any(y[22:end]==bytes.fromhex(v['nav' if nav else 'status']) for v in inputs)
  if not all(z[k] for k in ['equal_whole_frame','crc16_ok','fcs_ok','payload_matches_same_source_valid_sample','payload_magic_ok']):raise ValueError('telemetry bundle/CRC mismatch')
  packets.append(z)
 if not {'NAV1','ACK1'} <= {z['kind']for z in packets}:raise ValueError('both ACK and NAV1 not exercised')
 (d/'telemetry-packet-comparison.json').write_text(json.dumps(packets,indent=2)+'\n')
 return {'status':'PASS','packets':len(packets),'same_valid_bundle_and_crc_ok':True,'NAV1_source':'explicit synthetic changing marker; not RV32','real_motor_ACK_source':True}

cases=[]
for fam,phases in [('nav3',[0,13]),('nva1',[1,16])]:
 for ph in phases:cases.append((fam+'_phase'+str(ph),{'MODE':3 if fam=='nva1' else 0,'PHASE':ph},'off'))
for mode in [0,3]:cases.append(('duplicate_'+str(mode),{'KEY_CASE':1,'MODE':mode},'duplicate'))
cases += [('bad_fcs',{'KEY_CASE':2},'reject'),('wrong_session',{'KEY_CASE':3},'reject'),('busy_two_new_tasks',{'BUSY':1,'SETTLE':1},'busy')]
if args.smoke:cases=cases[:1]
root=D/('compile-smoke' if args.smoke else 'results');root.mkdir(exist_ok=False)
results=[];allrecords=[]
for name,opts,expect in cases:
 d=root/name;d.mkdir(exist_ok=False);start=time.monotonic()
 run=subprocess.run(['vvp',str(D/'test.vvp'),*[f'+{k}={v}' for k,v in opts.items()]],cwd=d,capture_output=True,text=True,timeout=100)
 (d/'run.log').write_text(run.stdout+run.stderr)
 rec={'case':name,'expect':expect,'options':opts,'returncode':run.returncode,'wall_s':time.monotonic()-start,'failures':[],'waveform_path':str(d/'trace.vcd')}
 if run.returncode:rec['failures'].append('simulation failed')
 else:
  try:
   events=[json.loads(x) for x in (d/'events.jsonl').read_text().splitlines()];hard=[e for e in events if e['event']=='hard_event'];wave=c.Wave(d/'trace.vcd');measure_end=next(e['time_ps']for e in reversed(events)if e['event']=='observation_end')
   rec['boundary_checks']=[json.loads(x) for x in (d/'checks.jsonl').read_text().splitlines()]
   if not all(x['pass'] for x in rec['boundary_checks']):rec['failures'].append('boundary check failed')
   trial={'case_id':name,'path':str(d),'options':opts,'phase_ns':opts.get('PHASE',0),'phase_adjust_ps':0,'pwm_sign':1,'mode':opts.get('MODE',0)}
   rr=c.process(trial,mf)
   for z in rr:
    z['evidence']['notes']=[n for n in z['evidence']['notes'] if 'no real RV32, MOT' not in n]
    z['evidence']['notes']+=['Frozen candidate26: official IDDR/receiver/parser/mailbox/owner/real motor; owner-to-motor direct plus final registered pin bundle; real motor_status_align and motor_status_gmii plus reference consumer. No real RV32; source read only.','Cancellation observation ends before independent real unarmed MOT1 telemetry stimulus. Synthetic NAV1 changing marker validates bundle capture, not CPU state semantics.']
    z['evidence']['rtl_modified']=False
    if z['result']['functional_status']!='PASS':rec['failures'].append('same-frame functional record failed')
    if z['association']['frame_kind']==1:
     quals=c.summarize.qualification(z)
     if quals:rec['failures'].append('not qualified shutdown: '+repr(quals))
     lat=z['result']['latency_ns']['parser_first_idle__to__all_outputs_off_candidate']
     if lat is None or lat>=1000:rec['failures'].append('temporary 1000ns limit failed')
   allrecords+=rr
   required=0 if expect=='reject' else 2 if expect=='busy' else 1
   if len(hard)!=required:rec['failures'].append('hard_event count mismatch')
   if wave.integer('owner_cancel_count',measure_end)!=required:rec['failures'].append('revoke counter mismatch')
   if expect=='reject' and wave.integer('nav_live',measure_end)!=1:rec['failures'].append('invalid packet stopped live navigation')
   rec.update(records=len(rr),hard_event_count=len(hard),qualified=sum(not c.summarize.qualification(z) for z in rr),latencies_ns=[z['result']['latency_ns']['parser_first_idle__to__all_outputs_off_candidate'] for z in rr if z['association']['frame_kind']==1],telemetry=check_telemetry(d))
   if expect=='busy':
    wire=json.loads((d/'supplement-wire-checks.json').read_text());gaps=wire['gaps'][1:]
    legal=[z for z in rr if z['association']['frame_kind']==1];identities={(z['transaction']['run_generation'],z['transaction']['goal_id'])for z in legal}
    rec['busy_ifg']={'sampled_adjacent_intervals':len(gaps),'strict_12_count':sum(g['strict_12_byte_ifg']for g in gaps),'two_distinct_legal_tasks':len(identities)==2,'legal_preceding_ifg':[z['association']['preceding_ifg']['idle_byte_slots']for z in legal]}
    if not gaps or not all(g['strict_12_byte_ifg']for g in gaps) or len(identities)!=2:rec['failures'].append('strict busy IFG or two task association failed')
  except Exception as ex:rec['failures'].append(type(ex).__name__+': '+str(ex))
 # Storage only: parse each raw waveform first, then losslessly compress it.
 # Keep acquisition JSON immutable and record a reversible path/hash map.
 storage=[]
 for raw in d.glob('*.vcd'):
  original_hash=sha(raw);packed=raw.with_suffix('.vcd.gz')
  with raw.open('rb') as f,gzip.open(packed,'wb',compresslevel=2) as g:
   while block:=f.read(1048576):g.write(block)
  verified=hashlib.sha256()
  with gzip.open(packed,'rb') as f:
   while block:=f.read(1048576):verified.update(block)
  assert verified.hexdigest()==original_hash
  storage.append({'original_path':str(raw),'gzip_path':str(packed),'uncompressed_sha256':original_hash,'verified_exact_decompression':True})
  raw.unlink()
 (d/'waveform-storage-manifest.json').write_text(json.dumps(storage,indent=2)+'\n')
 rec['status']='FAIL' if rec['failures'] else 'PASS';results.append(rec);print(name,rec['status'],rec.get('latencies_ns'),rec['failures'],flush=True)
 (root/'results.json').write_text(json.dumps({'trials':results,'scope':'targeted functional simulation only; no SDF/PVT/physical latency guarantee','temporary_limit_ns':1000,'source_manifest':mf},indent=2)+'\n')
 (root/'timeline.jsonl').write_text(''.join(json.dumps(z)+'\n' for z in allrecords))
 if rec['failures']:break
all_after={str(p.relative_to(O/'source')):sha(p)for p in (O/'source').rglob('*')if p.is_file()and 'impl'not in p.relative_to(O/'source').parts}
(D/'source-hashes-after.json').write_text(json.dumps(all_after,indent=2)+'\n')
qualified=[z for z in allrecords if not c.summarize.qualification(z)]
origins=['parser_first_idle','gmii_eof','rgmii_first_idle_high','rgmii_last_fcs_high','rgmii_sfd_high'];distribution={};worst={}
for origin in origins:
 key=origin+'__to__all_outputs_off_candidate';rr=[z for z in qualified if z['result']['latency_ns'][key]is not None]
 if rr:
  vals=[z['result']['latency_ns'][key]for z in rr];w=max(rr,key=lambda z:z['result']['latency_ns'][key]);distribution[origin]={'n':len(vals),'min_ns':min(vals),'median_ns':statistics.median(vals),'max_ns':max(vals),'max_case':w['case_id'],'max_frame':w['frame_id']};worst[origin]={'case_id':w['case_id'],'frame_id':w['frame_id'],'cdc_generation':w['association']['cdc_generation'],'transaction':w['transaction'],'timeline_ns':w['timeline_ns'],'waveform':w['evidence']['waveform_path']}
summary={'status':'PASS'if len(results)==len(cases)and all(r['status']=='PASS'for r in results)and all_before==all_after else'FAIL','case_count':len(results),'cancel_records':len(allrecords),'qualified_shutdown_samples':len(qualified),'temporary_parser_idle_limit_ns':1000,'distribution':distribution,'source_unchanged':all_before==all_after,'source_file_count':len(all_before),'official_IDDR_model':True,'real_status_consumers':True,'real_RV32':False,'coverage_not_run':['full 810-record regression','full L7','SDF/PVT','physical hardware','full SDF back-annotated simulation'],'prior_snapshot_evidence':str(O.parent/'candidate-13-status-snapshot/tests/results/result.json'),'physical_timing_note':'This functional smoke does not supersede PNR reports or establish a physical bound.'}
(root/'summary-measured.json').write_text(json.dumps(summary,indent=2)+'\n');(root/'worst-samples.json').write_text(json.dumps(worst,indent=2)+'\n')
print(json.dumps(summary,indent=2));sys.exit(0 if summary['status']=='PASS'else 1)
