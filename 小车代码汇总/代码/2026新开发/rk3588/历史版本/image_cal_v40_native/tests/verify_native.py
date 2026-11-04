#!/usr/bin/env python3
"""Independent existing Python wire oracle vs host and emulated static ARM64."""
from pathlib import Path
import hashlib, importlib.util, json, os, subprocess, tempfile

BASE=Path(__file__).resolve().parents[1]
ROOT=BASE.parents[1]
spec=importlib.util.spec_from_file_location('fpc1_reference',ROOT/'fpga/robot_image_v40/tools/fpc1.py')
ref=importlib.util.module_from_spec(spec);spec.loader.exec_module(ref)
tool=next(ROOT.glob('bsp_r4/*/sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/*/bin/aarch64-none-linux-gnu-g++'))
qemu=next(ROOT.glob('bsp_r4/*/sdk/yy3588_sdk-bsp/device/rockchip/common/tools/x86_64/qemu-aarch64-static'))
checks=[]
def run(args, **kw):
    return subprocess.run(list(map(str,args)),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=25,**kw)
def check(name,ok):
    assert ok,name
    checks.append(name)

with tempfile.TemporaryDirectory(prefix='fpc1-native-verify-') as folder:
    work=Path(folder); probes=[]
    for label,cxx,flags in [('host','g++',[]),('arm64',tool,['-static'])]:
        out=work/('probe-'+label)
        p=run([cxx,'-std=c++17','-O2','-Wall','-Wextra','-Wpedantic','-Werror',*flags,'-I',BASE/'vendor',BASE/'tests/codec_probe.cpp','-o',out]);check('build codec '+label,p.returncode==0)
        probes.append((label,[out] if label=='host' else [qemu,out]))
    native=BASE/'bin/fpc1.aarch64';p=run([qemu,native,'--self-test']);check('ARM64 production self-test without Python/runtime libraries',p.returncode==0 and 'PASS' in p.stdout)
    wire_cases=[]
    for eye in (0,1):
        for op in (1,3,4,6):wire_cases.append((op,eye,0xffffffff,0,0,0,0))
        for value in (0,1):wire_cases.append((5,eye,123,0,0,0,value))
        for col,d,g in ((0,0,1),(959,255,1023),(120,7,256),(511,128,400)):
            wire_cases.append((2,eye,0x12345678,col,d,g,0))
    for label,probe in probes:
        for fields in wire_cases:
            op,eye,rid,col,d,g,val=fields
            expected=ref.encode_ethernet(ref.make_request(op,eye,rid,column=col,dark=d,gain=g,value=val),'02:ac:13:80:00:02').hex()
            p=run(probe+['encode',*map(str,fields),'0']);check(label+' exact wire '+str(fields),p.returncode==0 and p.stdout.strip()==expected)
        ack=bytearray(ref.make_request('info',1,777));ack[7]=0x80;ack[19]=1;ack[22]=1;ack[20:22]=(960).to_bytes(2,'big')
        for i,v in ((24,42),(28,1234),(32,12),(36,1922),(40,0xffffffff)):ack[i:i+4]=v.to_bytes(4,'big')
        wire=ref.encode_ethernet(ack,ref.FPGA_MAC,'02:ac:13:80:00:02');expected=ref.decode_ack(wire,destination='02:ac:13:80:00:02')
        p=run(probe+['decode'],input=wire.hex());check(label+' full ACK metadata',p.returncode==0 and json.loads(p.stdout)==expected)
        for index in (0,6,12,14,18,19,20,21,22,23,32,58,78,79):
            bad=bytearray(wire);bad[index]^=1
            p=run(probe+['decode'],input=bad.hex());check(label+' corrupt frame rejected '+str(index),p.returncode!=0)
        for index in (4,5,6,7,23,44,63):
            bad=bytearray(ack);bad[index]=255
            frame=ref.encode_ethernet(bad,ref.FPGA_MAC,'02:ac:13:80:00:02')
            p=run(probe+['decode'],input=frame.hex());check(label+' semantic ACK rejected '+str(index),p.returncode!=0)
        p=run(probe+['decode'],input=wire[:-1].hex());check(label+' truncated ACK rejected',p.returncode!=0)
        for fields in ((2,0,1,960,0,256,0),(2,0,1,0,256,256,0),(2,0,1,0,0,0,0),(2,0,1,0,0,1024,0),(5,0,1,0,0,0,2),(6,2,1,0,0,0,0),(6,0,0,0,0,0,0)):
            p=run(probe+['encode',*map(str,fields),'0']);check(label+' invalid request rejected '+str(fields),p.returncode!=0)
    valid={'schema':'FPN-COLUMN-RAW8-Q2.8-v1','width':960,'eye':'left','raw_bits':8,'gain_fraction_bits':8,'dark':[i%256 for i in range(960)],'gain':[1+i%1023 for i in range(960)]}
    cases=[('valid native schema',valid,True)]
    other={'schema':'fpc1-column-q8-v1','width':960,'eye':0,'columns':[{'dark':4,'gain_q8':256} for _ in range(960)]}
    cases.append(('valid columns schema',other,True))
    for name,edit in [('width',lambda d:d.update(width=959)),('eye',lambda d:d.update(eye='right')),('raw bits',lambda d:d.update(raw_bits=12)),('missing column',lambda d:d['dark'].pop()),('gain high',lambda d:d['gain'].__setitem__(959,1024)),('gain zero',lambda d:d['gain'].__setitem__(959,0)),('dark boolean',lambda d:d['dark'].__setitem__(959,True)),('fraction',lambda d:d['gain'].__setitem__(959,256.5))]:
        d=json.loads(json.dumps(valid));edit(d);cases.append((name,d,False))
    for name,data,ok in cases:
        path=work/'coefficients.json';path.write_text(json.dumps(data));p=run([qemu,native,'--check',path,'--eye','0']);check('ARM64 offline coefficients '+name,(p.returncode==0)==ok)
    path=work/'bad.json';path.write_text('{bad json')
    p=run([qemu,native,'--interface','no_device','upload',path]);check('invalid JSON fails before socket/network access',p.returncode!=0 and 'JSON' in p.stderr and 'interface' not in p.stderr)
    for args in (['--timeout','nan','info'],['--retries','0','info'],['--eye','2','info'],['gamma','invalid']):
        p=run([qemu,native,*args]);check('ARM64 invalid CLI before socket '+str(args),p.returncode!=0 and 'socket' not in p.stderr)
report={'result':'PASS','checks':len(checks),'cases':checks,'machine':'AArch64','static_link':True,'executed_under_qemu':True,'executed_on_rk3588':False,'fpga_network_tested':False,
        'binary_sha256':hashlib.sha256(native.read_bytes()).hexdigest(),
        'source_sha256':{str(p.relative_to(BASE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [BASE/'src/fpc1.cpp',BASE/'vendor/picojson.h',Path(__file__),BASE/'tests/codec_probe.cpp']}}
(BASE/'reports/verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='cases'},indent=2))
