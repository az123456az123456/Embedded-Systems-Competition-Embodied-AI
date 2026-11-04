#!/usr/bin/env python3
"""Bind final RV32 firmware, source, ABI and real-core evidence without hardware."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]

def main():
    size=subprocess.check_output(['riscv64-unknown-elf-size',str(HERE/'firmware.elf')],text=True).splitlines()[1].split()
    text,data,bss=map(int,size[:3])
    assert text+data+bss<=65536-4096
    undefined=subprocess.check_output(['riscv64-unknown-elf-nm','-u',str(HERE/'firmware.elf')],text=True)
    assert not undefined.strip(),undefined
    header=subprocess.check_output(['riscv64-unknown-elf-readelf','-h',str(HERE/'firmware.elf')],text=True)
    assert 'ELF32' in header and 'RISC-V' in header and 'little endian' in header
    assembly=subprocess.check_output(['riscv64-unknown-elf-objdump','-d',str(HERE/'firmware.elf')],text=True)
    assert not re.search(r'\b(fadd|fmul|fdiv|flw|fld|fsw|fsd)\b',assembly)
    elfhash=hashlib.sha256((HERE/'firmware.hex').read_bytes()).hexdigest()
    evidence=json.loads((ROOT/'reports/v28_1_firmware_tests.json').read_text())
    assert evidence['passed']
    assert evidence['firmware_hex_sha256']==elfhash
    for rel,digest in evidence['source_sha256'].items():
        assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==digest,rel
    for rel,digest in evidence['evidence_sha256'].items():
        assert hashlib.sha256((ROOT/'reports'/rel).read_bytes()).hexdigest()==digest,rel
    rv=evidence['actual_rv32'];assert rv['passed']
    assert rv['astar_expanded']==2048
    assert rv['max_commit_gap_ms']<24 and rv['max_service_gap_cycles']<500000
    files=[p for p in HERE.iterdir() if p.name in ('firmware.elf','firmware.bin','firmware.hex','firmware_symbols.vh') or p.name.startswith('firmware_lane')]
    rec={'version':'v28.1-mount-readiness-dff-registerfile','cpu':'RV32IM ILP32 soft-float','clock_hz':50000000,
        'ram_bytes':65536,'text_bytes':text,'data_bytes':data,'bss_bytes':bss,'stack_reserved_bytes':4096,
        'remaining_bytes_below_stack':65536-4096-text-data-bss,'unresolved_symbols':[],
        'nominal_control_period_ms':20,'input_poll_period_ms':2,'hardware_tested':False,
        'profile':{'cpr':60527.7,'encoder_signs':[-1,1,-1,1],'circumference_um':205070,'track_um':180635,
          'maximum_wheel_mmps':300,'maximum_pwm_permille':600},
        'actual_rv32_simulation':rv,
        'production_imu_producer_rv32':evidence['production_imu_producer_rv32'],
        'host_tests':['ASan/UBSan fixed-point control','649 BFS A* comparisons','NAV2 protocol regression',
                      'ASan/UBSan positive-Z, negative-Z, sideways and real tilted mounting',
                      'Distinct packet, format, initialization, age, motion, gravity, bias, stability readiness reasons'],
        'sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)},
        'source_sha256':evidence['source_sha256'],
        'evidence_sha256':evidence['evidence_sha256'],
        'limitations':['No physical hardware test',
          'Planar relative yaw via fixed calibrated gravity projection; near-level vehicle ground, no full quaternion or magnetic-north fusion',
          'PI/PD and KF default gains require physical tuning','Historical quadrature illegal_seen is sticky; not a permanent motor inhibit',
          'Large single-window encoder jumps or >40ms control interval zero PWM and reset I; calibration reference retained',
          'Nominal 20ms control cadence has the measured commit jitter reported above',
          'Actual producer simulation forces raw reader outputs and does not exercise physical I2C',
          'Synthesized register-file equivalence and final PNR are separate required RTL evidence; C/behavior RTL tests alone cannot prove the silicon startup fix']}
    (HERE/'verification.json').write_text(json.dumps(rec,indent=2)+'\n')
    print(json.dumps({k:v for k,v in rec.items() if k not in ('sha256','source_sha256')},indent=2))

if __name__=='__main__':main()
