#!/usr/bin/env python3
"""Reconstruct actual synthesized 64KiB CPU block-RAM image and verify wiring."""
from pathlib import Path
import hashlib,json,re,struct,sys

def verify(project,build):
    project=Path(project);build=Path(build)
    net=build/'impl/gwsynthesis/robot_navigation_v20.vg'
    text=net.read_text(); firmware=project/'softcore/firmware/firmware.hex'
    expected=[int(w,16) for w in firmware.read_text().split()]
    assert len(expected)==16384
    actual=[0]*16384; planes=[]
    for name,ports in re.findall(r'SDPB (ram_\w+) \((.*?)\n\);',text,re.S):
        match=re.search(r'\.DO\(\{[^{}]*,ram_q\[(\d+)\]\}\)',ports)
        if not match:continue
        bit=int(match[1]);assert bit<32
        for addr in ['ADA','ADB']:assert f'.{addr}(mem_addr_Z[15:2])' in ports
        assert re.search(r'\.DI\(\{(?:GND,){31}mem_wdata_Z\['+str(bit)+r'\]\}\)',ports)
        assert '.CLKA(clk50m_d)' in ports and '.CLKB(clk50m_d)' in ports
        for width in ['BIT_WIDTH_0','BIT_WIDTH_1']:assert f'defparam {name}.{width}=1;' in text
        assert f"defparam {name}.READ_MODE=1'b0;" in text
        chunks={int(i,16):int(v,16) for i,v in re.findall(r'defparam '+re.escape(name)+r"\.INIT_RAM_([0-9A-Fa-f]{2})=256'h([0-9A-Fa-f]{64});",text)}
        assert len(chunks)<=64 and all(i<64 for i in chunks)
        for chunk,value in chunks.items():
            for i in range(256): actual[chunk*256+i]|=((value>>i)&1)<<bit
        planes.append({'bit':bit,'instance':name,'explicit_init_chunks':len(chunks)})
    assert sorted(p['bit'] for p in planes)==list(range(32))
    mismatch=sum(a!=b for a,b in zip(actual,expected));assert mismatch==0,mismatch
    image=b''.join(struct.pack('<I',v) for v in actual)
    report={'verified':True,'hardware_tested':False,'ram_bytes':len(image),'words_compared':16384,
      'bits_compared':524288,'mismatched_words':mismatch,'cpu_bsram_blocks':32,
      'scope':'Actual synthesized CPU RAM init and address/data wiring; no board test',
      'vendor_omitted_init_chunks_default_to_zero':True,'no_separate_external_flash_firmware_load':True,
      'firmware_sha256':hashlib.sha256(firmware.read_bytes()).hexdigest(),
      'initialized_image_sha256':hashlib.sha256(image).hexdigest(),
      'netlist_sha256':hashlib.sha256(net.read_bytes()).hexdigest(),'planes':sorted(planes,key=lambda p:p['bit'])}
    (project/'reports/ram_init_verification.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
if __name__=='__main__':
    r=verify(Path(__file__).resolve().parents[1],sys.argv[1]);print('PASS synthesized CPU RAM: 16384 words, 524288 bits, all32 wiring planes, no mismatches')
