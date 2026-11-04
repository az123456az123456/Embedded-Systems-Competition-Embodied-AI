#!/usr/bin/env python3
"""RTL Ethernet packet validation against Python's independent zlib CRC32."""
from pathlib import Path
import argparse,hashlib,json,os,re,subprocess,tempfile,time,zlib
PROJECT=Path(__file__).resolve().parents[1]
RTL=PROJECT/'preprocess/rgb_packet_tx.v';TB=PROJECT/'tests/tb_rgb_packet_tx.sv'

def pixel(eye,seq,n):return ((n+seq*3+eye*11)&255)<<16|((n//256+eye*47)&255)<<8|((n*7+eye*53+seq)&255)
def frame(eye,seq,height,stamp=None):
    yield (1<<32)|seq;yield (2<<32)|(stamp if stamp is not None else seq*33+eye*2)
    for n in range(640*height):yield pixel(eye,seq,n)
    yield 3<<32

def decode(raw,height):
    assert len(raw)%1018==0,('invalid capture size',len(raw))
    packets=[]
    for n in range(0,len(raw),1018):
        wire=raw[n:n+1018];assert wire[:8]==b'\x55'*7+b'\xd5'
        eth=wire[8:-4];assert len(eth)==1006
        assert eth[:6]==b'\xff'*6 and eth[6:12]==bytes.fromhex('02ac13800001') and eth[12:14]==bytes.fromhex('88ba')
        assert zlib.crc32(eth)==int.from_bytes(wire[-4:],'little'),('FCS mismatch',n)
        h=eth[14:46];payload=eth[46:]
        assert h[:4]==b'RGP1'and h[4]==1 and h[5]in(0,1)and h[6]==1
        eye=h[5];seq=int.from_bytes(h[8:12],'big');stamp=int.from_bytes(h[12:16],'big')
        row=int.from_bytes(h[16:18],'big');part=h[18];epoch=int.from_bytes(h[26:28],'big')
        assert h[7]==int(seq==1)and part in(0,1)and h[19]==2
        assert int.from_bytes(h[20:22],'big')==640 and int.from_bytes(h[22:24],'big')==height
        assert len(payload)==960 and int.from_bytes(h[24:26],'big')==960 and epoch!=0
        assert zlib.crc32(h[:28]+payload)==int.from_bytes(h[28:32],'big'),('app CRC mismatch',n)
        for p in range(320):
            want=pixel(eye,seq,row*640+part*320+p)
            assert payload[p*3:p*3+3]==want.to_bytes(3,'big'),('payload mismatch',eye,seq,row,part,p,payload[p*3:p*3+3].hex(),want)
        packets.append({'eye':eye,'seq':seq,'stamp':stamp,'row':row,'part':part,'epoch':epoch})
    return packets

def run_case(folder,name,height,words0,words1,complete,full=False,epoch_change=0,initial_epoch=1,expected_stamps=None,source_fps=30):
    input0=folder/(name+'-left.hex');input1=folder/(name+'-right.hex');capture=folder/(name+'.raw')
    for path,words in ((input0,words0),(input1,words1)):
        with path.open('w')as f:
            for word in words:f.write(f'{word:09x}\n')
    executable=folder/('sim-'+name);depth=max(len(words0),len(words1))+8
    frames=max(len(words0),len(words1))//(640*height+3)
    frame_cycles=round(125000000/source_fps)
    pixel_gap=7 if source_fps==40 else 8
    max_cycles=frames*frame_cycles+1000000 if full else 1000000
    start=time.monotonic()
    parameters={'HEIGHT':height,'WORDS0':len(words0),'WORDS1':len(words1),'MAX_WORDS':depth,
                'CHANGE_EPOCH':epoch_change,'MAX_CYCLES':max_cycles,'INITIAL_EPOCH':initial_epoch}
    if full:parameters.update(FRAME_CYCLES=frame_cycles,PIXEL_GAP=pixel_gap)
    args=['iverilog','-g2012','-s','tb_rgb_packet_tx',f'-Ptb_rgb_packet_tx.HEIGHT={height}',
          f'-Ptb_rgb_packet_tx.WORDS0={len(words0)}',f'-Ptb_rgb_packet_tx.WORDS1={len(words1)}',
          f'-Ptb_rgb_packet_tx.MAX_WORDS={depth}',f'-Ptb_rgb_packet_tx.CHANGE_EPOCH={epoch_change}',
          f'-Ptb_rgb_packet_tx.MAX_CYCLES={max_cycles}',f'-Ptb_rgb_packet_tx.INITIAL_EPOCH={initial_epoch}']
    if full:args+=['-Ptb_rgb_packet_tx.FRAME_CYCLES='+str(frame_cycles),'-Ptb_rgb_packet_tx.PIXEL_GAP='+str(pixel_gap)]
    if full:
        obj=PROJECT/('build/rgb-packet-verilated-full'+str(source_fps));obj.parent.mkdir(parents=True,exist_ok=True)
        log_path=folder/'verilator-build.log'
        env=os.environ.copy();env.update(CCACHE_DISABLE='1',CCACHE_TEMPDIR='/tmp')
        with log_path.open('w')as log:
            subprocess.run(['verilator','--binary','--timing','-Wno-fatal','--top-module','tb_rgb_packet_tx',
                            '--Mdir',str(obj),'-j','2','-CFLAGS','-std=c++20']+['-G'+k+'='+str(v)for k,v in parameters.items()]+[str(RTL),str(TB)],
                           check=True,stdout=log,stderr=subprocess.STDOUT,env=env)
        command=[str(obj/'Vtb_rgb_packet_tx')]
    else:
        subprocess.run(args+['-o',str(executable),str(RTL),str(TB)],check=True)
        command=['vvp',str(executable)]
    run=subprocess.run(command+['+fifo0='+str(input0),'+fifo1='+str(input1),'+capture='+str(capture)],check=True,text=True,stdout=subprocess.PIPE)
    packets=decode(capture.read_bytes(),height);groups={}
    for p in packets:
        key=(p['epoch'],p['seq'],p['eye']);groups.setdefault(key,[]).append(p)
    complete_keys=[]
    for key,group in groups.items():
        coords={(p['row'],p['part'])for p in group};assert len(coords)==len(group),('duplicate packet',key)
        if len(coords)==height*2:
            assert coords=={(r,s)for r in range(height)for s in (0,1)}
            complete_keys.append(key)
        assert len({p['stamp']for p in group})==1,('inconsistent stamp',key)
    assert sorted(complete_keys)==sorted(complete),(name,'complete frames',complete_keys,complete)
    for epoch,seq,eye in complete:
        other=groups.get((epoch,seq,1-eye),[])
        if other:assert groups[(epoch,seq,eye)][0]['stamp']==other[0]['stamp']
        if expected_stamps is not None:assert groups[(epoch,seq,eye)][0]['stamp']==expected_stamps[seq],(name,'wrong modular stamp',seq,groups[(epoch,seq,eye)][0]['stamp'])
    result=re.search(r'RESULT (.*)',run.stdout);assert result,run.stdout
    stats={k:int(v)for k,v in re.findall(r'(\w+)=(\d+)',result.group(1))}
    assert stats['packets']==len(packets)and stats['ifg_min']>=12
    if full:assert stats['cycles']<frames*frame_cycles,(name,'cannot sustain source deadline',stats['cycles'],frames*frame_cycles)
    if name in ('sequence-resync','timestamp-reject','abort-recovery','missing-eof','timeout-no-EOF'):
        assert stats['faults']>0,(name,stats)
    if epoch_change:
        epochs=[p['epoch']for p in packets];assert epochs==sorted(epochs),('old epoch after new',epochs)
    result={'name':name,'height':height,'packets_checked':len(packets),'complete_frames':complete_keys,
            'stats':stats,'elapsed_s':round(time.monotonic()-start,3),'capture_sha256':hashlib.sha256(capture.read_bytes()).hexdigest()}
    if full:result['source']={'clock_hz':125000000,'fps':source_fps,'frame_period_cycles':frame_cycles,'pixel_gap_cycles':pixel_gap,
                             'arbiter_blocked_per_5000_cycles':160,'simulated_elapsed_ms':stats['cycles']/125000}
    return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--full',action='store_true');parser.add_argument('--fps',type=int,choices=(30,40),default=30);parser.add_argument('--reports',type=Path,default=PROJECT/'reports/rgb-packet-v35');args=parser.parse_args()
    args.reports.mkdir(parents=True,exist_ok=True);results=[];h=4
    with tempfile.TemporaryDirectory(prefix='rgp1-v35-')as tmp:
        t=Path(tmp)
        words=lambda e,s,st=None:list(frame(e,s,h,st))
        results.append(run_case(t,'normal',h,words(0,1)+words(0,2),words(1,1)+words(1,2),[(1,s,e)for s in(1,2)for e in(0,1)]))
        results.append(run_case(t,'sequence-resync',h,words(0,1)+words(0,2),words(1,2),[(1,2,e)for e in(0,1)]))
        results.append(run_case(t,'timestamp-reject',h,words(0,1,100)+words(0,2),words(1,1,120)+words(1,2),[(1,2,e)for e in(0,1)]))
        bad=words(0,1);bad[1500]=15<<32
        results.append(run_case(t,'abort-recovery',h,bad+words(0,2),words(1,1)+words(1,2),[(1,2,e)for e in(0,1)]))
        results.append(run_case(t,'missing-eof',h,words(0,1)[:-1]+words(0,2),words(1,1)+words(1,2),[(1,2,e)for e in(0,1)]))
        results.append(run_case(t,'epoch-mid-frame',h,words(0,1),words(1,1),[(2,1,e)for e in(0,1)],epoch_change=1500))
        results.append(run_case(t,'high-BE-header',h,words(0,0x01020304,0x05060708),words(1,0x01020304,0x0506070a),
                                [(0xbeef,0x01020304,e)for e in(0,1)],initial_epoch=0xbeef))
        results.append(run_case(t,'sequence-zero-wrap',h,words(0,0xffffffff,100)+words(0,0,133),
                                words(1,0xffffffff,102)+words(1,0,135),
                                [(1,s,e)for s in(0xffffffff,0)for e in(0,1)],expected_stamps={0xffffffff:100,0:133}))
        results.append(run_case(t,'uptime-wrap',h,words(0,7,0xfffffffe),words(1,7,2),
                                [(1,7,e)for e in(0,1)],expected_stamps={7:0xfffffffe}))
        results.append(run_case(t,'uptime-wrap-reverse',h,words(0,8,2),words(1,8,0xfffffffe),
                                [(1,8,e)for e in(0,1)],expected_stamps={8:0xfffffffe}))
        results.append(run_case(t,'timeout-no-EOF',h,words(0,1)[:-1],words(1,1),[]))
        # Keep a production-size capture for the independently implemented RK
        # codec to decode; small geometry cases intentionally vary height.
        if args.full:
            h=640
            words=lambda e:sum((list(frame(e,s,h,(s*25+e*2) if args.fps==40 else None)) for s in(1,2,3)),[])
            full_name='full'+str(args.fps)+'fps'
            expected_stamps={s:s*(25 if args.fps==40 else 33) for s in(1,2,3)}
            results.append(run_case(t,full_name,h,words(0),words(1),[(1,s,e)for s in(1,2,3)for e in(0,1)],full=True,source_fps=args.fps,expected_stamps=expected_stamps))
            (args.reports/(full_name+'.raw')).write_bytes((t/(full_name+'.raw')).read_bytes())
            raw=(t/(full_name+'.raw')).read_bytes()
            (args.reports/'verilator-build.log').write_bytes((t/'verilator-build.log').read_bytes())
            with (args.reports/(full_name+'.records')).open('wb')as f:
                for i in range(0,len(raw),1018):
                    frame_bytes=raw[i+8:i+1018];f.write(len(frame_bytes).to_bytes(4,'big')+frame_bytes)
    report={'result':'PASS','hardware_tested':False,'protocol':'RGP1 Ethernet0x88ba RGB888, CRC32 IEEE app+FCS, BE fields','cases':results,
            'source_sha256':{str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in(RTL,TB,Path(__file__))}}
    (args.reports/'verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
