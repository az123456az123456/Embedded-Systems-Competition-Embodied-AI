#!/usr/bin/env python3
"""Package only the verified v29-reactive-clearance RK incremental runtime; the paired FPGA is required."""
from pathlib import Path
import argparse,hashlib,importlib.util,json,subprocess,tarfile,tempfile
SRC=Path(__file__).resolve().parent
NAME='robot-reactive-v29-rk3588-20261027'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,default=SRC/'dist');args=p.parse_args()
 proof=json.loads((SRC/'reports/rk-verification.json').read_text())
 assert proof['result']=='PASS' and proof['new_fpga_bitstream_required'] and not proof['apk_update_required']
 for name,value in proof['source_sha256'].items():assert sha(SRC/name)==value,name
 for name,value in proof['reports_sha256'].items():assert sha(SRC/name)==value,name
 spec=importlib.util.spec_from_file_location('seal',SRC/'update/seal_package.py');seal=importlib.util.module_from_spec(spec);spec.loader.exec_module(seal)
 subprocess.run(['sha256sum','-c','SHA256SUMS'],cwd=SRC/'update',check=True,stdout=subprocess.DEVNULL)
 for name in ('installer-tests.json','installer-busybox.json'):
  test=json.loads((SRC/'update/reports'/name).read_text());assert test['status']=='passed'
  assert not test['hardware_accessed'] and not test['arm_programs_executed']
  for member,value in test['tested_runtime_files_sha256'].items():assert sha(SRC/'update'/member)==value,member
  assert test['test_script_sha256']==sha(SRC/'update/tests/test_installer.py')
 args.output.mkdir(parents=True,exist_ok=True);archive=args.output/(NAME+'.tar.gz')
 assert not archive.exists(),f'Existing release is never overwritten: {archive}'
 members=(*seal.RUNTIME_FILES,'SHA256SUMS')
 with tarfile.open(archive,'w:gz') as tar:
  for member in members:tar.add(SRC/'update'/member,arcname=NAME+'/'+member,recursive=False)
 with tempfile.TemporaryDirectory(prefix='v29-reactive-clearance-rk-release-') as tmp:
  with tarfile.open(archive) as tar:
   assert all(not Path(m.name).is_absolute() and '..' not in Path(m.name).parts and m.isfile() for m in tar.getmembers())
   tar.extractall(tmp,filter='data')
  root=Path(tmp)/NAME
  for member in members:assert sha(root/member)==sha(SRC/'update'/member),member
  subprocess.run(['sha256sum','-c','SHA256SUMS'],cwd=root,check=True,stdout=subprocess.DEVNULL)
 report={'result':'PASS','runtime':str(archive),'sha256':sha(archive),'new_fpga_bitstream_required':True,'apk_update_required':False,'hardware_tested':False,'files':len(members)}
 (archive.with_suffix('').with_suffix('.verification.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
