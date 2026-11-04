#!/usr/bin/env python3
"""PC-only: clone v40 and append actual per-camera manual exposure/gain/WB.
Never edits the original project, compiles it or programs the board.
"""
from pathlib import Path
import argparse,hashlib,json,shutil
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--profile',type=Path,required=True)
ap.add_argument('--source',type=Path,required=True)
ap.add_argument('--output',type=Path,required=True)
a=ap.parse_args();doc=json.loads(a.profile.read_text());b=bytes.fromhex(doc['profile_hex'])
assert doc['schema']=='ACG138-RAW-CAL1-v1' and len(b)==152 and b[:6]==b'CAL1\x01\x07'
assert b[6:10]==bytes.fromhex('03c00438') and b[12:16]==bytes.fromhex('00004101')
source=a.source.resolve();out=a.output.resolve();assert source!=out and not out.exists(),'Output must be a NEW directory'
assert out not in source.parents and source not in out.parents,'Keep new clone outside source tree'
rom=source/'camera/ov5640_init_table_raw.v'
assert hashlib.sha256(rom.read_bytes()).hexdigest()=='5004f1c0832d2fcb7134be513dfef7f9c90987294c079414a085fe0391ff9b21','Requires unmodified v40 sensor table'
assert (source/'robot_image_v40.gprj').is_file(),'Requires v40 project'
settings=[]
for eye in (0,1):
 r=b[24+64*eye:88+64*eye];assert r[:2]==b'\x56\x40' and r[2]&3==3 and r[8]&1
 assert r[28:32]==bytes.fromhex('03c00438')
 settings.append([(0x3503,r[2]),(0x3406,r[8]),(0x3500,r[3]),(0x3501,r[4]),(0x3502,r[5]),
                  (0x350a,r[6]),(0x350b,r[7])]+[(0x3400+i,r[9+i]) for i in range(6)])
shutil.copytree(source,out,ignore=shutil.ignore_patterns('build','impl','reports','bitstream','__pycache__','.git'))
p=out/'camera/ov5640_init_table_raw.v';s=p.read_text();assert 'parameter CAL_CAMERA_INDEX' not in s
s=s.replace('module ov5640_init_table_rgb #(\n','module ov5640_init_table_rgb #(\n  parameter CAL_CAMERA_INDEX = 0,\n',1)
lines=[]
for i,((reg,v0),(reg1,v1)) in enumerate(zip(*settings)):
 assert reg==reg1
 lines.append(f"    rom[{259+i}] = CAL_CAMERA_INDEX==0 ? 24'h{reg:04x}_{v0:02x} : 24'h{reg:04x}_{v1:02x};")
marker='  end\n\n  always @';assert s.count(marker)==1
s=s.replace(marker,'\n    // Actual per-camera flat profile; AEC/AGC/AWB frozen.\n'+'\n'.join(lines)+'\n'+marker,1);p.write_text(s)
p=out/'camera/camera_init.v';s=p.read_text();s=s.replace('module camera_init #(\n','module camera_init #(\n  parameter CAL_CAMERA_INDEX = 0,\n',1)
assert s.count("9'd259")==1;s=s.replace("9'd259","9'd272",1)
marker='ov5640_init_table_rgb #(\n';assert s.count(marker)==1
s=s.replace(marker,marker+'        .CAL_CAMERA_INDEX(CAL_CAMERA_INDEX),\n',1);p.write_text(s)
p=out/'camera/ov5640x2_udp_rgmii.v';s=p.read_text();marker='    // Model-input transport replaces';assert s.count(marker)==1
s=s.replace(marker,'    defparam camera_init1.CAL_CAMERA_INDEX=0;\n    defparam camera_init2.CAL_CAMERA_INDEX=1;\n\n'+marker,1);p.write_text(s)
(out/'calibration-source-profile.json').write_text(json.dumps(doc,indent=2)+'\n')
(out/'LOCKED-RUNTIME-NOTE.txt').write_text('此副本追加了两眼真实曝光、增益、白平衡设置，须重新完整编译、检查时序、上板确认后使用。原v40目录未改。自动对焦、黑电平、Gamma等仍应按实际成像验证，不代表所有光照均适用。\n')
print('Created separate locked-runtime source:',out)
print('Open the cloned robot_image_v40.gprj and run a FULL implementation before using FPN.')
