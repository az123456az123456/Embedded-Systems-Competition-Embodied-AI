#!/usr/bin/env python3
"""Run the real reconnect shell with fake radio/DHCP commands, no host networking."""
import json, os, pathlib, signal, subprocess, tempfile, time

ROOT = pathlib.Path(__file__).resolve().parents[1]
FAKE = r'''#!/usr/bin/env python3
import json,os,pathlib,sys,time
p=pathlib.Path(os.environ['WIFI_FAKE']);name=pathlib.Path(sys.argv[0]).name;a=sys.argv[1:]
with (p/'trace').open('a') as f:f.write(json.dumps([name,a])+'\n')
mode=os.environ['WIFI_SCENARIO']
if name=='sleep':time.sleep(.015);sys.exit(0)
if name=='pidof':sys.exit(0 if mode=='external' else 1)
if name=='wpa_supplicant':
 while True:time.sleep(.02)
if name=='wpa_cli':
 a=a[4:];cmd=a[0]
 if cmd=='ping':print('PONG')
 elif cmd=='select_network':
  (p/'selected').write_text(a[1]);(p/'drop').unlink(missing_ok=True);print('OK')
 elif cmd=='status':
  n=(p/'selected').read_text();ok=not (p/'drop').exists() and mode!='offline' and not(mode=='fallback' and n=='0')
  print('wpa_state='+('COMPLETED' if ok else 'DISCONNECTED'))
  print('id='+n);print('ssid='+('Ciallo' if n=='0' else 'REDMI K90'))
 sys.exit(0)
if name=='udhcpc':
 if mode=='dhcp_fail' and (p/'selected').read_text()=='0':sys.exit(1)
 (p/'lease').write_text('192.168.10.2/24')
 while True:time.sleep(.02)
if name=='ip':
 assert 'eth0' not in a
 if a[:3]==['-4','addr','flush']:(p/'lease').unlink(missing_ok=True)
 if a[:4]==['-o','-4','addr','show'] and (p/'lease').exists():print('2: wlan0 inet '+(p/'lease').read_text()+' scope global wlan0')
 sys.exit(0)
'''

def wait_for(pred, timeout=12):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if pred():return
        time.sleep(.025)
    raise AssertionError('deadline')

def trial(mode):
    with tempfile.TemporaryDirectory(prefix='wifi-unit-') as td:
        p=pathlib.Path(td);b=p/'bin';b.mkdir();(p/'trace').touch()
        for n in ('wpa_cli','wpa_supplicant','udhcpc','ip','pidof','sleep'):
            x=b/n;x.write_text(FAKE);x.chmod(0o755)
        conf=p/'credentials';conf.write_text('network={\nssid="test-only"\n}\n');conf.chmod(0o600)
        log=p/'wifi.log'
        env=dict(os.environ,PATH=str(b)+':'+os.environ['PATH'],WIFI_FAKE=td,WIFI_SCENARIO=mode,
                 ROBOT_WIFI_CONFIG=str(conf),ROBOT_WIFI_RUN_DIR=str(p/'run'),ROBOT_WIFI_LOG=str(log))
        def text():return log.read_text() if log.exists() else ''
        def trace():return [json.loads(s) for s in (p/'trace').read_text().splitlines()]
        def selected():return [a[-1] for n,a in trace() if n=='wpa_cli' and 'select_network' in a]
        proc=subprocess.Popen(['sh',str(ROOT/'wifi-reconnect.sh')],env=env,start_new_session=True,
                              stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        try:
            if mode=='external':
                wait_for(lambda:'external_network_manager_present_waiting' in text())
                assert not any(n=='wpa_supplicant' for n,a in trace())
            elif mode=='offline':
                wait_for(lambda:len(selected())>=4)
                assert selected()[:4]==['0','1','0','1']
            else:
                expected='REDMI K90' if mode in ('fallback','dhcp_fail') else 'Ciallo'
                wait_for(lambda:'connected ssid='+expected+' ipv4=' in text())
                if mode in ('fallback','dhcp_fail'):assert selected()[:2]==['0','1']
                else:assert selected()==['0']
                if mode=='drop':
                    (p/'drop').touch();wait_for(lambda:len(selected())>=2)
                    assert selected()[:2]==['0','0']
                else:
                    old=selected();time.sleep(.25);assert selected()==old
                if mode=='first':
                    second=subprocess.run(['sh',str(ROOT/'wifi-reconnect.sh')],env=env,timeout=2,capture_output=True)
                    assert second.returncode==0
                    assert sum(n=='wpa_supplicant' for n,a in trace())==1
            assert not any('eth0' in a for n,a in trace())
        finally:
            proc.send_signal(signal.SIGTERM)
            try:out,err=proc.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);out,err=proc.communicate();raise
        assert proc.returncode==0,(proc.returncode,err.decode())
        assert 'stopped' in text()
        assert all('killall'!=n for n,a in trace())
        return {'case':mode,'status':'PASS','selected_network_ids':selected(),
                'stopped_cleanly':True,'FPGA_interface_touched':False}

if __name__=='__main__':
    r={'scope':'production shell with fake radio/DHCP only; real process lifecycle and flock',
       'cases':[trial(m) for m in ('first','fallback','offline','dhcp_fail','drop','external')]}
    print(json.dumps(r,indent=2))
