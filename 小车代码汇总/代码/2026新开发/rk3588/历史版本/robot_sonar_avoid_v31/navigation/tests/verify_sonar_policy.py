#!/usr/bin/env python3
"""Portable source-only comparison, no board, network or private log required."""
import hashlib,json,pathlib,re,subprocess,time
NAV=pathlib.Path(__file__).resolve().parents[1]
ROOT=NAV.parent
REPORTS=ROOT/'reports'
BUILD=NAV/'build'/'v31-sonar-policy'
BASE=NAV/'tests/fixtures/baseline-v29.2'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 REPORTS.mkdir(exist_ok=True);BUILD.mkdir(parents=True,exist_ok=True)
 manifest=json.loads((BASE/'origin.json').read_text())
 for name,digest in manifest['files_sha256'].items():assert sha(BASE/name)==digest,(name,'baseline mutated')
 flags=['g++','-O2','-std=c++17','-Wall','-Wextra','-pthread']
 logs=[]
 def run(name,args):
  p=REPORTS/('v31-sonar-'+name+'.log')
  with p.open('w') as f:
   f.write('command='+json.dumps(args)+'\n');f.flush()
   subprocess.run(args,cwd=NAV,stdout=f,stderr=subprocess.STDOUT,check=True)
  logs.append(p)
 tests={}
 source='tests/test_sonar_avoidance_policy.cc'
 for name,extra in [('old',['-DV30_EXPECT_OLD','-DNAVCORE_HEADER="fixtures/baseline-v29.2/navcore.h"','tests/fixtures/baseline-v29.2/navcore.cc']),('current',['navcore.cc'])]:
  out='build/v31-sonar-policy/'+name
  run(name+'-build',flags+extra+[source,'-o',out])
  run(name,[out,'tests/fixtures/sonar-policy-blocker-20210101.json'])
  text=logs[-1].read_text();assert 'PASS ' in text
  tests[name]=int(re.search(r'\b(\d+) checks\b',text)[1])
 run('three-beam-build',flags+['navcore.cc','tests/test_three_sonar_policy.cc','-o','build/v31-sonar-policy/three-beam'])
 run('three-beam',['build/v31-sonar-policy/three-beam'])
 text=logs[-1].read_text();assert 'PASS ' in text
 tests['three_beam']=int(re.search(r'\b(\d+) checks\b',text)[1])
 sources=[NAV/n for n in ['navcore.cc','navcore.h','navcore_reactive.inc','vehicle_geometry.h',source,'tests/fixtures/sonar-policy-blocker-20210101.json','tests/verify_sonar_policy.py','tests/test_three_sonar_policy.cc','tests/fixtures/three-sonar-real-20261028.json']]
 sources +=[BASE/n for n in manifest['files_sha256']]+[BASE/'origin.json']
 report={'result':'PASS','version':'v31-three-sonar-local-policy','hardware_tested':False,'network_accessed':False,'created_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'tests':tests,'checks_total':sum(tests.values()),'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in sources},'evidence_sha256':{str(p.relative_to(ROOT)):sha(p) for p in logs},'recorded_fixture_scope':'Sparse old occupied blocker plus recorded v30 range-only samples; synthetic healthy odometry is explicitly used for the latter. No full raw sensor replay is claimed.','coverage':['v29.2 occupied-cell veto reproduced','any of three beams .30m stop-first; left-close turns right and right-close turns left','front-only/both-close select larger opening, tie clockwise','new single close flank stops before reversing turn','both flanks release above .32m with front dynamic clearance','turn-to-forward handoff has no inserted zero-command frame','unknown/occupied local maps not queried or erased','normal map-planning unknown-space guard retained','real near-range/invalid-source/5s timeout/STOP/IMU/encoder protection retained'],'limits':['host source execution only','three beams cannot certify unseen space; physical body-sweep and direction floors retained','raw source ages and actual board IO are not fabricated']}
 path=REPORTS/'v31-sonar-policy.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(path)
if __name__=='__main__':main()
