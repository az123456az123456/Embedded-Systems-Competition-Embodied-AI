"""Decode the real filtered-source Ethernet/DDR simulation independently."""
from pathlib import Path
import struct
import zlib

frames=[]
buf=bytearray()
for line in Path('build/filter_top_frames.hex').read_text().splitlines():
    if line=='END':
        wire=bytes(buf);buf.clear()
        assert wire[:8]==b'\x55'*7+b'\xd5' and len(wire)==90
        assert wire[8:22]==bytes.fromhex('ffffffffffff02ac1380000188b5')
        assert int.from_bytes(wire[-4:],'little')==zlib.crc32(wire[8:-4])
        frames.append(wire[22:-4])
    else:buf.append(int(line,16))
assert not buf and len(frames)==4
assert [int.from_bytes(p[6:8],'big') for p in frames]==[13,13,15,12]
assert [int.from_bytes(p[8:12],'big') for p in frames]==[1,2,2,2]
assert [int.from_bytes(p[60:64],'big') for p in frames]==[0,0,1,1]
assert struct.unpack('>hhhhhhh',frames[0][16:30])==(16384,-16384,0,0,131,-131,0)
expected=(14336,-14336,4096,333,262,0,-131)
assert all(struct.unpack('>hhhhhhh',p[16:30])==expected for p in frames[1:])
assert all(p[:6]==b'IMU9\x01\x92' and p[39]==0x71 and p[41]==0x69 for p in frames)
assert all(int.from_bytes(p[58:60],'big')==100 for p in frames)
assert int.from_bytes(frames[-1][56:58],'big')>=500
print('PASS actual filtered second sample, signed units, temperature alignment, sequences, flags, heartbeat, Ethernet CRC and DDR order')
