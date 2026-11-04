#!/usr/bin/env python3
"""Independent full-frame RGB golden comparison, not camera IQ validation."""
from pathlib import Path
import argparse,hashlib,json,math,random,re,subprocess,tempfile,time
import numpy as np

PROJECT=Path(__file__).resolve().parents[1]
RTL=PROJECT/'preprocess/bayer_demosaic.v'
TB=PROJECT/'tests/tb_bayer_demosaic.sv'

def golden(raw,phase):
    h,w=raw.shape;p=np.pad(raw.astype(np.uint16),1,mode='reflect')
    centre=p[1:h+1,1:w+1]
    horizontal=(p[1:h+1,:w]+p[1:h+1,2:w+2]+1)//2
    vertical=(p[:h,1:w+1]+p[2:h+2,1:w+1]+1)//2
    cross=(p[:h,1:w+1]+p[2:h+2,1:w+1]+p[1:h+1,:w]+p[1:h+1,2:w+2]+2)//4
    diagonal=(p[:h,:w]+p[:h,2:w+2]+p[2:h+2,:w]+p[2:h+2,2:w+2]+2)//4
    yy,xx=np.indices((h,w));site=((yy&1)^((phase>>1)&1))*2+((xx&1)^(phase&1))
    out=np.zeros((h,w,3),np.uint8)
    for s,values in enumerate(((diagonal,cross,centre),(vertical,centre,horizontal),
                               (horizontal,centre,vertical),(centre,cross,diagonal))):
        for c,v in enumerate(values):out[:,:,c][site==s]=v[site==s]
    return out

def stream(raw,bubbles=0,seed=0):
    r=random.Random(seed);h,w=raw.shape;rows=[]
    for y in range(h):
        for x in range(w):
            if bubbles:rows.extend([(0,0,0,0,0)]*r.randrange(bubbles+1))
            rows.append((1,int(x==0 and y==0),int(x==w-1),0,int(raw[y,x])))
    return rows

def run_case(folder,name,w,h,phase,stimulus,expected,minimum_aborts=0,truth=None):
    executable=folder/f'sim-{w}-{h}-{phase}'
    if not executable.exists():
        subprocess.run(['iverilog','-g2012','-s','tb_bayer_demosaic',
            f'-Ptb_bayer_demosaic.WIDTH={w}',f'-Ptb_bayer_demosaic.HEIGHT={h}',
            f'-Ptb_bayer_demosaic.BAYER={phase}','-o',str(executable),str(RTL),str(TB)],check=True)
    data=folder/(name+'.stim');trace=folder/(name+'.trace')
    with data.open('w')as f:
        for row in stimulus:f.write('0 '+' '.join(map(str,row))+'\n')
    start=time.monotonic()
    with trace.open('w')as f:subprocess.run(['vvp',str(executable),'+stimulus='+str(data)],check=True,stdout=f)
    frames=[];current=[];aborts=0;timestamps=[];seen_start=False
    with trace.open()as f:
        for line in f:
            if line.startswith('ABORT '):aborts+=1;current=[];timestamps=[];seen_start=False
            if not line.startswith('PIXEL '):continue
            _,cycle,x,y,sof,eol,eof,value=line.split()
            cycle,x,y,sof,eol,eof=map(int,(cycle,x,y,sof,eol,eof));value=int(value,16)
            if sof:
                assert not current,(name,'unexpected duplicate SOF')
                seen_start=True
            assert seen_start,(name,'pixel before SOF',x,y)
            n=len(current);assert (x,y)==(n%w,n//w),(name,n,x,y)
            assert eol==int(x==w-1),(name,'bad EOL',x,y,eol)
            assert sof==int(x==0 and y==0),(name,'bad SOF',x,y,sof)
            assert eof==int(x==w-1 and y==h-1),(name,'bad EOF',x,y,eof)
            current.append((value>>16,(value>>8)&255,value&255));timestamps.append(cycle)
            if eof:
                frames.append((np.array(current,np.uint8).reshape(h,w,3),timestamps))
                current=[];timestamps=[];seen_start=False
    assert not current,(name,'incomplete trailing frame')
    assert len(frames)==len(expected),(name,'frame count',len(frames),len(expected))
    assert aborts>=minimum_aborts,(name,'missing abort',aborts,minimum_aborts)
    for (actual,_),want in zip(frames,expected):
        assert np.array_equal(actual,want),(name,'pixel mismatch',np.argwhere(actual!=want)[:10].tolist())
    result={'name':name,'width':w,'height':h,'phase':phase,'complete_frames':len(frames),
            'checked_pixels':w*h*len(frames),'aborts':aborts,'hardware_software_max_error':0,
            'one_pixel_per_clock_interior':all(all(b-a==1 for a,b in zip(t,t[1:])) for _,t in frames) if ifnotbubbles(stimulus) else None,
            'simulation_s':round(time.monotonic()-start,3)}
    if truth is not None:
        error=frames[0][0].astype(np.float64)-truth.astype(np.float64)
        mse=float(np.mean(error*error));result['synthetic_RGB_groundtruth_PSNR_dB']=None if mse==0 else 10*math.log10(255*255/mse)
        result['synthetic_RGB_groundtruth_MSE']=mse
        result['quality_scope']='Synthetic colour sample only; not camera image-quality validation or old RK ISP equivalence'
    return result

def ifnotbubbles(stimulus):
    # Interior bubbles are deliberately allowed; this flag is informational.
    return all(row[0] for row in stimulus[:min(len(stimulus),100)])

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--full',action='store_true')
    parser.add_argument('--reports',type=Path,default=PROJECT/'reports/bayer-v35')
    args=parser.parse_args();args.reports.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(35);results=[]
    with tempfile.TemporaryDirectory(prefix='bayer-v35-')as t:
        folder=Path(t)
        for phase in range(4):
            for w,h in ((2,2),(7,5),(8,6),(31,18)):
                a=rng.integers(0,256,(h,w),dtype=np.uint8)
                b=rng.integers(0,256,(h,w),dtype=np.uint8)
                results.append(run_case(folder,f'normal-{w}-{h}-{phase}',w,h,phase,
                    stream(a)+[(0,0,0,0,0)]*(w+8)+stream(b,2,seed=35)+[(0,0,0,0,0)]*(w+8),
                    [golden(a,phase),golden(b,phase)]))
        w,h,phase=17,12,0;a=rng.integers(0,256,(h,w),dtype=np.uint8);want=golden(a,phase)
        bad=stream(a)[:w*5+3];bad[-1]=(1,0,1,0,77)
        results.append(run_case(folder,'short-row-recovery',w,h,phase,bad+[(0,0,0,0,0)]*7+stream(a),[want],1))
        bad=stream(a);v,s,e,a0,p=bad[w-1];bad[w-1]=(v,s,0,a0,p)
        results.append(run_case(folder,'missing-eol-recovery',w,h,phase,bad[:w]+stream(a),[want],1))
        results.append(run_case(folder,'explicit-abort-recovery',w,h,phase,
            stream(a)[:w*5+4]+[(0,0,0,1,0)]+stream(a),[want],1))
        results.append(run_case(folder,'sof-preempts-active',w,h,phase,
            stream(a)[:w*5+4]+stream(a),[want],1))
        results.append(run_case(folder,'sof-preempts-bottom-flush',w,h,phase,
            stream(a)+[(0,0,0,0,0)]*3+stream(a),[want],1))
        # Constant colour sampling detects all four Bayer phase/channel maps.
        for phase in range(4):
            y,x=np.indices((12,16));site=((y&1)^((phase>>1)&1))*2+((x&1)^(phase&1))
            raw=np.choose(site,[11,73,73,191]).astype(np.uint8)
            expected=np.zeros((12,16,3),np.uint8);expected[:]=[191,73,11]
            results.append(run_case(folder,f'constant-RGB-{phase}',16,12,phase,stream(raw),[expected]))
        for sharp in (False,True):
            h,w=96,128;y,x=np.indices((h,w));truth=np.stack((
                30+160*x/(w-1),40+150*y/(h-1),50+100*(x+y)/(w+h-2)),axis=2).astype(np.uint8)
            if sharp:truth[20:70,30:90]=[220,31,42];truth[40:90,70:120]=[20,201,171]
            phase=0;site=((y&1)*2+(x&1));raw=np.empty((h,w),np.uint8)
            for s,c in enumerate((2,1,1,0)):raw[site==s]=truth[:,:,c][site==s]
            results.append(run_case(folder,'synthetic-RGB-'+('edges'if sharp else'smooth'),w,h,phase,
                                     stream(raw),[golden(raw,phase)],truth=truth))
        if args.full:
            w,h=960,1080;raw=rng.integers(0,256,(h,w),dtype=np.uint8)
            results.append(run_case(folder,'full-960x1080-BGGR',w,h,0,stream(raw),[golden(raw,0)]))
    report={'result':'PASS','algorithm':'3x3 bilinear, RAW8 -> RGB888, reflect101 borders; NOT MHC',
        'camera_IQ_tested':False,'hardware_tested':False,'cases':results,
        'checked_pixels':sum(r['checked_pixels']for r in results),'phase_values':{'BGGR':0,'GBRG':1,'GRBG':2,'RGGB':3},
        'source_sha256':{str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in (RTL,TB,Path(__file__))}}
    (args.reports/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'result':'PASS','cases':len(results),'checked_pixels':report['checked_pixels']},indent=2))
if __name__=='__main__':main()
