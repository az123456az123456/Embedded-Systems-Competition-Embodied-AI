#!/usr/bin/env python3
"""Actual POSIX supervisor tests with fixture children, never hardware."""
from pathlib import Path
import argparse,atexit,hashlib,json,os,shutil,signal,subprocess,tempfile,time
ACTIVE=[]
def cleanup_children():
    for proc in ACTIVE:
        if proc.poll() is None:
            proc.terminate()
            try:proc.communicate(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
atexit.register(cleanup_children)
ROOT=Path(__file__).resolve().parents[1]
HELPER=r'''#!/usr/bin/python3
import json,os,select,signal,sys,time
from pathlib import Path
role=sys.argv[1];args=sys.argv[2:];events=Path(os.environ['EVENTS'])
if role=='nav' and '--check-config' in args:
 print('fixture measured configuration');sys.exit(int(os.environ.get('CHECK_RC','0')))
(events/(role+'-args')).write_text(json.dumps(args))
(events/(role+'-env')).write_text(json.dumps({k:os.environ.get(k) for k in ('PERCEPTION_BINARY','PAIR_TOKEN_FILE')}))
(events/(role+'-pid')).write_text(str(os.getpid()))
def stop(*_):
 (events/(role+'-stopped')).write_text('yes');sys.exit(0)
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
if role=='raw':print('PERF fixture camera output',flush=True)
deadline=time.monotonic()+float(os.environ.get(role.upper()+'_EXIT_S','60'))
readable=role=='nav'
while time.monotonic()<deadline:
 if readable:
  ready,_,_=select.select([sys.stdin],[],[],.03)
  if ready:
   line=sys.stdin.readline()
   if not line:readable=False
   elif line.strip()=='quit':stop()
 else:time.sleep(.03)
sys.exit(int(os.environ.get(role.upper()+'_EXIT_RC','0')))
'''
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--busybox',action='store_true');args=parser.parse_args()
    shell=['busybox','sh','-c','. \"$TEST_PRELUDE\"; . \"$0\"'] if args.busybox else ['sh'];checks=0;cases=[]
    def ck(value,label):
        nonlocal checks
        assert value,label;checks+=1
    with tempfile.TemporaryDirectory(prefix='unified-v23-launch-') as td:
        base=Path(td);tools=base/'tools';tools.mkdir();(tools/'id').write_text('#!/bin/sh\nif [ \"$1\" = -u ];then echo 0;else exec /usr/bin/id \"$@\";fi\n');(tools/'id').chmod(0o755);helper=base/'helper.py';helper.write_text(HELPER);helper.chmod(0o755)
        prelude=base/'busybox-prelude.sh';shell_wrapper=tools/'fixture-sh'
        prelude.write_text('id() { "$TEST_TOOLS/id" "$@"; };\n' + "alias exec='command exec '\n" + "alias sh='"+str(shell_wrapper)+"'\n")
        shell_wrapper.write_text('#!/bin/sh\nexec /usr/bin/busybox sh -c \' . "$TEST_PRELUDE"; . "$0" \' "$@"\n');shell_wrapper.chmod(0o755)
        def fixture(label,**extra):
            directory=base/label;nav=directory/'navigation path';app=directory/'perception path';events=directory/'events'
            nav.mkdir(parents=True);app.mkdir();events.mkdir()
            shutil.copy2(ROOT/'run-navigation.sh',nav/'run-navigation.sh');shutil.copy2(ROOT/'run-dashboard.sh',app/'run-dashboard.sh')
            binary=app/'perception-dashboard.aarch64';binary.write_text('untouched dashboard binary fixture');binary.chmod(0o755)
            (nav/'dashboard.sha256').write_text(hashlib.sha256(binary.read_bytes()).hexdigest()+'\n')
            (nav/'config.json').write_text('unchanged configuration')
            token=app/'pair-token.txt';token.write_text('012345\n')
            for path,role in ((nav/'robot-navigation.aarch64','nav'),(app/'run-perception.sh','raw')):
                path.write_text('#!/bin/sh\nexec /usr/bin/python3 '+"'"+str(helper)+"' "+role+' "$@"\n');path.chmod(0o755)
            alias=directory/'perception-dashboard';alias.symlink_to(app,target_is_directory=True)
            env=dict(os.environ,PATH=str(tools)+':'+os.environ['PATH'],TEST_TOOLS=str(tools),TEST_PRELUDE=str(prelude),NAVIGATION_HOME=str(nav),EVENTS=str(events),NAV_LAUNCH_LOCK=str(directory/'lock'),NAV_DASHBOARD_LOG=str(directory/'dashboard.log'),NAV_CONFIG_LOG=str(directory/'config.log'),**extra)
            return directory,nav,app,events,env,[*shell,str(alias/'run-dashboard.sh')]
        def wait_file(path,proc):
            deadline=time.monotonic()+5
            while not path.exists() and proc.poll() is None and time.monotonic()<deadline:time.sleep(.02)
            
            if not path.exists():
                if proc.poll() is None:proc.terminate()
                output,_=proc.communicate(timeout=5)
                raise AssertionError('child failed: '+str(path)+' '+output)
            ck(True,'fixture child started')
        def finish(proc,action='quit'):
            if action=='quit':out,_=proc.communicate('quit\n',timeout=6)
            else:
                proc.terminate();out,_=proc.communicate(timeout=6)
            return out
        def start(label,options=(),**extra):
            fixture_data=fixture(label,**extra);d,n,a,e,env,cmd=fixture_data
            proc=subprocess.Popen(cmd+list(options),env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            ACTIVE.append(proc)
            return fixture_data,proc
        f,p=start('default',('-0','--model','/path with spaces/model.rknn','--npu-workers','3'))
        d,n,a,e,env,cmd=f;wait_file(e/'nav-args',p);wait_file(e/'raw-args',p)
        navargs=json.loads((e/'nav-args').read_text());rawargs=json.loads((e/'raw-args').read_text())
        ck('--drive' in navargs,'valid config permits explicitly requested future mission')
        ck('start' not in navargs and 'explore' not in navargs,'startup never starts a mission')
        ck(rawargs==['--fullscreen','--rotate','0','--model','/path with spaces/model.rknn','--npu-workers','3','--phone-bind','127.0.0.1','--phone-port','8082'],'quoted camera flags preserved and private backend forced')
        ck(json.loads((e/'raw-env').read_text())['PERCEPTION_BINARY']==str(a/'perception-dashboard.aarch64'),'same paired binary')
        ck(json.loads((e/'raw-env').read_text())['PAIR_TOKEN_FILE']==str(a/'pair-token.txt'),'same pairing token')
        second=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=3)
        ck(second.returncode!=0 and 'already running' in second.stderr,'duplicate supervisor rejected')
        output=finish(p);ck(p.returncode==0,'console quit propagated');ck(not (d/'lock').exists(),'lock released')
        ck((e/'raw-stopped').exists(),'camera child stopped on quit')
        ck((n/'config.json').read_text()=='unchanged configuration' and (a/'pair-token.txt').read_text()=='012345\n','config/token unchanged')
        cases.append('one command, preserved args/rotation/pair/model, no mission, duplicate guard, quit cleanup')
        for label,options,extra,drive in [('rotation',('-90',),{},True),('uncalibrated',(),{'CHECK_RC':'2'},False),('observe',('--observe',),{},False),('interface',('--interface','eth0'),{},True)]:
            f,p=start(label,options,**extra);d,n,a,e,env,cmd=f;wait_file(e/'nav-args',p);wait_file(e/'raw-args',p)
            na=json.loads((e/'nav-args').read_text());ra=json.loads((e/'raw-args').read_text())
            ck(('--drive' in na)==drive,label+' drive authorization')
            if label=='rotation':ck(ra[1:3]==['--rotate','90'],label)
            if label=='interface':ck(na[-2:]==['--interface','eth0'] and '--interface' in ra,'internal FPGA interface forwarded')
            finish(p);ck(p.returncode==0 and not (d/'lock').exists(),label+' cleanup');cases.append(label)
        f,p=start('explicit-drive',('--drive',),CHECK_RC='2');d,n,a,e,env,cmd=f;p.communicate(timeout=5)
        ck(p.returncode==2 and not (e/'nav-args').exists() and not (e/'raw-args').exists(),'explicit drive refuses invalid motion calibration before startup');ck(not (d/'lock').exists(),'failed preflight lock cleaned');cases.append('explicit drive preflight')
        for label,options,check_rc in [('no-phone',('--no-phone',),'0'),('custom-port',('--phone-port','8090','--no-navigation'),'0'),('version',('--version',),'0'),('malformed',(),'1')]:
            f,p=start(label,options,CHECK_RC=check_rc,RAW_EXIT_S='.04');d,n,a,e,env,cmd=f;p.communicate(timeout=5)
            ck(p.returncode==0 and (e/'raw-args').exists() and not (e/'nav-args').exists(),label+' standalone behavior')
            ck(not (d/'lock').exists(),label+' no leftover lock');cases.append(label)
        f,p=start('null-stdin');d,n,a,e,env,cmd=f;wait_file(e/'nav-args',p)
        p.stdin.close();p.stdin=None;time.sleep(.2);ck(p.poll() is None,'closed terminal input keeps services running');finish(p,'terminate')
        ck((e/'raw-stopped').exists() and (e/'nav-stopped').exists() and not (d/'lock').exists(),'signal stops both services');cases.append('no console input and signal cleanup')
        f,p=start('dashboard-dies',(),RAW_EXIT_S='.15');d,n,a,e,env,cmd=f;p.communicate(timeout=5)
        ck(p.returncode!=0 and (e/'nav-stopped').exists() and not (d/'lock').exists(),'dashboard failure stops navigation');cases.append('dashboard death')
        f,p=start('nav-dies',(),NAV_EXIT_S='.1',NAV_EXIT_RC='7');d,n,a,e,env,cmd=f;p.communicate(timeout=5)
        ck(p.returncode==7 and (e/'raw-stopped').exists() and not (d/'lock').exists(),'navigation failure stops dashboard and propagates error');cases.append('navigation death')
        for label,opts in [('reserved-port',('--phone-port','8090')),('bad-angle',('-45',)),('missing-value',('--model',))]:
            d,n,a,e,env,cmd=fixture(label);run=subprocess.run(cmd+list(opts),env=env,capture_output=True,text=True,timeout=3)
            ck(run.returncode==2 and not (e/'raw-args').exists() and not (e/'nav-args').exists(),label+' rejected');cases.append(label)
        d,n,a,e,env,cmd=fixture('wrong-pair');(n/'dashboard.sha256').write_text('a'*64+'\n');run=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=3)
        ck(run.returncode!=0 and not (e/'raw-args').exists(),'wrong dashboard pair rejected');cases.append('pair guard')
    result={'status':'PASS','shell':'busybox sh' if args.busybox else 'host sh','checks':checks,'cases':cases,'hardware_tested':False,'children':'fixture processes; actual supervisor scripts; BusyBox identity emulated by test-only shell functions and aliases; no privilege changes','source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ('run-dashboard.sh','run-navigation.sh')}}
    report=ROOT/'reports'/('launcher-busybox.json' if args.busybox else 'launcher.json');report.parent.mkdir(exist_ok=True);report.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
