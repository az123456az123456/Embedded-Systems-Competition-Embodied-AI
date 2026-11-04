#!/usr/bin/env python3
"""Production v31 CLI checks; no hardware, sockets or motor access."""
import json
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
def main():
    binary=ROOT/'build/robot-navigation';checks=0
    def check(value):
        nonlocal checks
        assert value;checks+=1
    def run(*args):return subprocess.run([str(binary),*map(str,args)],capture_output=True,text=True,timeout=5)
    check(run('--version').stdout.strip() == 'robot-navigation-v31-three-sonar-smooth')
    check('observation' in run('--help').stdout)
    check(run('--nonexistent','value').returncode!=0)
    check(run('--config').returncode!=0)
    check(run('--config',ROOT/'config.example.json','--check-config').returncode==0)
    with tempfile.TemporaryDirectory(prefix='nav-v23-cli-') as tmp:
        p=Path(tmp)/'settings.json';c=json.loads((ROOT/'config.example.json').read_text())
        c.update(calibration_confirmed=True,drive_mps_per_full_pwm=.6)
        p.write_text(json.dumps(c));check(run('--config',p,'--check-config').returncode==0)
        c.update(encoder_ticks_per_turn=1.5,encoder_signs=[1,0],track_m=.01)
        p.write_text(json.dumps(c));profile=json.loads(run('--config',p,'--dump-profile').stdout)
        check(profile['encoder_ticks_per_turn']==60527.7)
        check(profile['encoder_signs']==[-1,1,-1,1])
        check(profile['track_m']==.180635)
        p.write_text('{}');check(run('--config',p,'--check-config').returncode==1)
    print(f'PASS {checks} production v31 CLI/config checks; no board/network')
if __name__=='__main__':main()
