#!/usr/bin/env python3
"""PC-only: startup fault simulation + full dual-DVP/FIFO/UDP wire proof."""
from pathlib import Path
import binascii, json, os, struct, subprocess
BASE=Path(__file__).resolve().parents[1]
os.chdir(BASE)
build=BASE/'build';build.mkdir(exist_ok=True)
def run(args,log):
    env=dict(os.environ,CCACHE_DISABLE='1',CCACHE_TEMPDIR='/tmp')
    with (build/log).open('w') as f: subprocess.run(args,env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
run(['iverilog','-g2012','-s','tb_setup','-o',str(build/'setup-rtl'),'rtl/raw_camera_setup.v','camera/ov5640_init_table_raw.v','tests/tb_setup.sv'],'setup-rtl-compile.log')
run(['vvp',str(build/'setup-rtl')],'setup-rtl-simulation.log')
assert 'PASS all startup/lock/readback/fault cases' in (build/'setup-rtl-simulation.log').read_text()
run(['iverilog','-g2012','-s','tb_i2c','-o',str(build/'i2c-bit'),'camera/i2c_control.v','camera/i2c_bit_shift.v','tests/tb_i2c.sv'],'i2c-bit-compile.log')
run(['vvp',str(build/'i2c-bit')],'i2c-bit-simulation.log')
assert 'PASS actual SCCB' in (build/'i2c-bit-simulation.log').read_text()
regs=bytearray(64)
regs[0:2]=bytes.fromhex('5640');regs[2]=3;regs[4]=0x21;regs[5]=0xa0;regs[7]=0x10;regs[8]=1
regs[9]=regs[11]=regs[13]=4;regs[28:32]=bytes.fromhex('03c00438');regs[15]=0xa7;regs[16]=0x83;regs[42]=0x46
profile=b'CAL1'+bytes([1,7])+bytes.fromhex('03c00438000000004101')+bytes([10,0,10,0])+bytes(4)+regs+regs
assert len(profile)==152
(build/'profile.hex').write_text(''.join(f'{x:02x}\n' for x in profile))
sources=['rtl/raw_calibration_top.v']+[f'camera/{n}.v' for n in ['Camera_ETH_Formator','controller','controller_top','cache','camera_async_fifo','eth_fifo_timing','UDP_Send','CRC32_D8']]+['tests/tb_wire.sv']
run(['verilator','--binary','--timing','-Wno-fatal','-CFLAGS','-std=c++20','--top-module','tb_wire','--Mdir',str(build/'wire-verilator')]+sources,'wire-compile.log')
run([str(build/'wire-verilator/Vtb_wire')],'wire-simulation.log')
wire=(build/'wire.bin').read_bytes();packets=[];nrows=0;nmeta=0
for line in (build/'wire-index.txt').read_text().splitlines():
    offset,size,gap=map(int,line.split());assert gap>=12
    b=wire[offset:offset+size];assert b[:8]==b'\x55'*7+b'\xd5'
    frame=b[8:-4];assert int.from_bytes(b[-4:],'little')==binascii.crc32(frame)
    ip=frame[14:34];assert sum(struct.unpack('!10H',ip))%65535==0
    p=frame[42:];assert len(p)==1922
    y=int.from_bytes(p[:2],'little')
    if y==65535:
        assert p[2:154]==profile and p[154:]==bytes(1922-154);nmeta+=1
    else:
        f=nrows//1080;assert y==nrows%1080
        assert p[2:962]==bytes((x+3*y+7*f)&255 for x in range(960))
        assert p[962:]==bytes((2*x+5*y+11*f+40)&255 for x in range(960));nrows+=1
    packets.append(frame)
assert nrows==2160 and nmeta>0
(build/'wire.rawcap').write_bytes(b''.join(struct.pack('<I',len(p))+p for p in packets))
report={'result':'PASS','pixel_bytes':nrows*1920,'raw_frames_per_eye':2,'metadata_packets':nmeta,'Ethernet_CRC':'PASS','IP_checksum':'PASS','minimum_IFG':'at least 12 byte clocks','startup_fault_modes':6,'SCCB_bit_level':'PASS read/write/repeated START/NACK','hardware_tested':False}
(BASE/'reports/wire-verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
