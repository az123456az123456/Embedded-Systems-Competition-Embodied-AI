"""Independent Ethernet CRC/protocol check plus native receiver decoding."""
from pathlib import Path
import struct
import subprocess
import zlib

frames = []
buf = bytearray()
for line in Path('build/frames.hex').read_text().splitlines():
    if line == 'END':
        frames.append(bytes(buf))
        buf.clear()
    else:
        buf.append(int(line, 16))
assert len(frames) == 3 and not buf
for index, wire in enumerate(frames):
    assert wire[:8] == b'\x55' * 7 + b'\xd5'
    frame = wire[8:]
    assert len(frame) == 82
    assert frame[:14] == bytes.fromhex('ffffffffffff02ac1380000188b5')
    assert struct.unpack('<I', frame[-4:])[0] == zlib.crc32(frame[:-4])
    p = frame[14:-4]
    assert p[:8] == b'IMU9\x01\x92\x00\x0f'
    assert struct.unpack('>I', p[60:])[0] == (7 ^ index)
    if index:
        assert p[:60] == frames[0][22:-8]
    path = Path(f'build/frame{index}.bin')
    path.write_bytes(frame)
    result = subprocess.run(['build/imu-monitor', '--decode', str(path)], check=True, capture_output=True, text=True)
    assert 'AG: VALID' in result.stdout and 'MAG: VALID' in result.stdout
    assert 'X=  +1.00000 Y=  -1.00000' in result.stdout
    assert 'X= +15.00000 Y= -16.87500 Z= +26.25000' in result.stdout
    assert 'temperature [C] = 21.00' in result.stdout

def check_bad(frame):
    path = Path('build/invalid.bin')
    path.write_bytes(frame)
    assert subprocess.run(['build/imu-monitor', '--decode', str(path)], capture_output=True).returncode != 0

frame = frames[0][8:]
check_bad(frame[:-1])
check_bad(frame[:-1] + bytes([frame[-1] ^ 1]))
for offset in (6, 12, 14, 18, 19, 20, 55, 72):
    bad = bytearray(frame[:-4])
    bad[offset] ^= 0x80
    check_bad(bad)
Path('build/no_fcs.bin').write_bytes(frame[:-4])
subprocess.run(['build/imu-monitor', '--decode', 'build/no_fcs.bin'], check=True, stdout=subprocess.DEVNULL)
# Valid heartbeat with stale ages must not print old measurements as current.
stale = bytearray(frame[:-4])
stale[14+42:14+44] = b'\xff\xff'
stale[14+56:14+58] = b'\xff\xff'
Path('build/stale.bin').write_bytes(stale)
r = subprocess.run(['build/imu-monitor', '--decode', 'build/stale.bin'], check=True, capture_output=True, text=True)
assert 'Accel [g]' not in r.stdout and 'Mag   [uT]' not in r.stdout
print('PASS frame CRC32, endian, CDC payload coherence, receiver units, malformed frames and stale data')

top = []
buf = bytearray()
for line in Path('build/top_frames.hex').read_text().splitlines():
    if line == 'END':
        wire = bytes(buf)
        assert wire[:8] == b'\x55' * 7 + b'\xd5'
        assert len(wire) == 90
        assert struct.unpack('<I', wire[-4:])[0] == zlib.crc32(wire[8:-4])
        top.append(wire[22:-4])
        buf.clear()
    else:
        buf.append(int(line, 16))
assert len(top) == 3
assert [struct.unpack('>H', p[6:8])[0] for p in top] == [13, 15, 12]
assert all(struct.unpack('>I', p[8:12])[0] == 1 for p in top)
assert [struct.unpack('>I', p[60:64])[0] for p in top] == [0, 1, 1]
assert all(p[16:42] == frames[0][22+16:22+41]+b'\x69' for p in top)
assert struct.unpack('>H', top[2][56:58])[0] >= 500
print('PASS actual top packet layout, status, sequences, ages, heartbeat and RGMII nibble order')
