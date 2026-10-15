from pathlib import Path
import struct,zlib
frames=[];buf=bytearray()
for line in Path('build/shared_frames.hex').read_text().splitlines():
    if line=='END': frames.append(bytes(buf));buf.clear()
    else: buf.append(int(line,16))
assert len(frames)==24
camera=imu=ack=0
for wire in frames:
    assert wire[:8]==b'\x55'*7+b'\xd5'
    frame=wire[8:]
    assert struct.unpack('<I',frame[-4:])[0]==zlib.crc32(frame[:-4]),'FCS mismatch'
    if frame[12:14]==b'\x08\x00':
        ip=frame[14:34]
        assert sum(struct.unpack('>10H',ip))%65535==0
        assert frame[34:38]==struct.pack('>HH',5000,6102)
        payload=frame[42:-4]
        assert len(payload)==1922
        assert payload[:2]==bytes([camera,0])
        assert payload[2:]==bytes((i+camera)%251 for i in range(2,1922))
        camera+=1
    elif frame[12:14]==b'\x88\xb6':
        payload=frame[14:32]
        expected=struct.pack('>4sBBIhhBB',b'ACK1',1,5,ack,100+ack,200+ack,3,12)
        assert payload[:16]==expected
        crc=0xffff
        for byte in expected:
            crc^=byte<<8
            for _ in range(8):crc=((crc<<1)^0x1021)&0xffff if crc&0x8000 else (crc<<1)&0xffff
        assert payload[16:]==struct.pack('>H',crc)
        assert frame[32:-4]==bytes(28)
        ack+=1
    else:
        assert frame[12:18]==b'\x88\xb5IMU9'
        assert frame[14:-4]==b'IMU9'+bytes(59)+bytes([imu])
        imu+=1
assert camera==imu==ack==8
print('PASS independent CRC32, IPv4 checksum, UDP ports, all camera row bytes, IMU payloads and ACK payload/CRC16')
