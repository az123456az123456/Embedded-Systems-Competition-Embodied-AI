#!/usr/bin/env python3
"""Run existing final-version tests in temporary copies; never access hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(argv, cwd):
    print('+ ' + ' '.join(map(str, argv)), flush=True)
    subprocess.run(list(map(str, argv)), cwd=cwd, check=True)


def main():
    for name in ['gcc', 'g++', 'make', 'javac', 'java', 'iverilog', 'vvp']:
        if not shutil.which(name):
            raise SystemExit('Missing test dependency: ' + name)
    with tempfile.TemporaryDirectory(prefix='acg138-source-tests-') as tmp:
        work = Path(tmp)
        native_src = ROOT / 'rk3588/v143_20261104/robot-fpga-integration-20261102'
        native = work / 'native'
        shutil.copytree(native_src, native)
        run(['make', '-C', native / 'perception', 'JPEG=stb', 'test-front-sonar-follow',
             'test-angle-follow', 'test-velocity', 'test-box-stability'], work)
        app = work / 'android'
        shutil.copytree(ROOT / '上位机/v143_1.2.43_20261104', app)
        run(['sh', app / 'test-follow-host.sh'], app)
        run(['sh', app / 'test-client-host.sh'], app)
        fpga = ROOT / 'fpga/v54_150_70_20261104'
        rtl = work / 'encoder-test'
        run(['iverilog', '-g2012', '-s', 'tb_encoder_prearm_relocation', '-o', rtl,
             fpga / 'tests/tb_encoder_prearm_relocation.sv', fpga / 'sensors/encoder_4ch.v'], work)
        run(['vvp', rtl], work)
    print('PASS current RK follow/velocity/box, Android follow/client and FPGA encoder smoke tests')


if __name__ == '__main__':
    main()
