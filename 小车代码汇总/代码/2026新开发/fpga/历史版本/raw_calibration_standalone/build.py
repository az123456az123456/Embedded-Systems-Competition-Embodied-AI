#!/usr/bin/env python3
"""PC-only frozen-source build of the independent calibration bitstream."""
from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,sys,tempfile,time
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def sources():
    paths=[BASE/'build.tcl',BASE/'raw_calibration.gprj',BASE/'raw_calibration.cst',BASE/'raw_calibration.sdc']
    paths+=sorted((BASE/'camera').glob('*.v'))+sorted((BASE/'rtl').glob('*.v'))
    return {str(p.relative_to(BASE)):sha(p) for p in paths}
initial=sources();stage=Path(tempfile.mkdtemp(prefix='acg138-rawcal-'))
shutil.copytree(BASE,stage,dirs_exist_ok=True,ignore=shutil.ignore_patterns('reports','build','impl','bitstream','__pycache__'))
(BASE/'reports/build-progress.json').write_text(json.dumps({'stage':str(stage),'source_sha256':initial},indent=2)+'\n')
env=dict(os.environ,QT_QPA_PLATFORM='minimal',QT_XCB_GL_INTEGRATION='none',QT_OPENGL='software',LIBGL_ALWAYS_SOFTWARE='1')
compat=Path('/usr/lib/x86_64-linux-gnu/libstdc++.so.6')
if compat.exists():env['LD_PRELOAD']=str(compat)
tool=Path(os.environ.get('GOWIN_IDE',str(ROOT/'Gowin_V1.9.12.03/IDE')))/'bin/gw_sh'
started=time.monotonic()
with (stage/'gowin.log').open('w') as f:p=subprocess.run([str(tool),'build.tcl'],cwd=stage,env=env,stdout=f,stderr=subprocess.STDOUT)
assert initial==sources(),'Source changed during implementation'
report={'result':'FAIL' if p.returncode else 'IMPLEMENTED_PENDING_STA','stage':str(stage),'source_sha256':initial,'compiler_returncode':p.returncode,'elapsed_s':round(time.monotonic()-started,2),'hardware_tested':False}
dest=BASE/'reports/implementation';dest.mkdir(exist_ok=True)
shutil.copy2(stage/'gowin.log',dest/'gowin.log')
for f in (stage/'impl').rglob('*'):
    if f.is_file() and f.suffix in ('.txt','.html'):shutil.copy2(f,dest/f.name)
if p.returncode==0:
    sys.path.insert(0,str(BASE))
    from build_support import timing_report
    report['timing']=timing_report(stage)
    text=(stage/'impl/pnr/raw_calibration.rpt.txt').read_text();resources={}
    for name in ('Logic','Register','CLS','BSRAM','DSP'):
        m=re.search(r'^\s*'+name+r'\s*\|\s*([\d.]+)/([\d.]+)',text,re.M)
        if m:resources[name]=dict(zip(('used','capacity'),map(float,m.groups())))
        else:
            assert name=='DSP',name
            vg=(stage/'impl/gwsynthesis/raw_calibration.vg').read_text()
            assert not re.search(r'^\s*(MULT\w*|ALU54D)\s+',vg,re.M)
            resources[name]={'used':0,'capacity':118}
    report['resources']=resources;fs=stage/'impl/pnr/raw_calibration.fs';report['bitstream_sha256']=sha(fs)
    if report['timing']['result']=='PASS':
        report['result']='PASS_INTERNAL_STA';shutil.copy2(fs,BASE/'bitstream/raw_calibration.fs')
    else:report['result']='FAIL_INTERNAL_STA'
(BASE/'reports/build.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if report['result']!='PASS_INTERNAL_STA':raise SystemExit(1)
