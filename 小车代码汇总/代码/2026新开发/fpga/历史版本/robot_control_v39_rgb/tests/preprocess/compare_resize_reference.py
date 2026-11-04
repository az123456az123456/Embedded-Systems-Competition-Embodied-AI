#!/usr/bin/env python3
"""Compare fixed Q11 arithmetic with CPU half-pixel/OpenCV; no hardware timing."""
from pathlib import Path
import json
import math
import sys
import numpy as np

BASE=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(BASE/'preprocess'))
from generate_resize_coeffs import axis


def fixed(image):
    x=np.array(axis(960,569),dtype=np.int64)
    y=np.array(axis(1080,640),dtype=np.int64)
    a=image[y[:,0,None],x[None,:,0]].astype(np.uint64)
    b=image[y[:,0,None],x[None,:,1]].astype(np.uint64)
    c=image[y[:,1,None],x[None,:,0]].astype(np.uint64)
    d=image[y[:,1,None],x[None,:,1]].astype(np.uint64)
    wx=x[:,2].astype(np.uint64)[None,:,None];wy=y[:,2].astype(np.uint64)[:,None,None]
    valid=(((a*(2048-wx)+b*wx)*(2048-wy)+(c*(2048-wx)+d*wx)*wy+(1<<21))>>22).astype(np.uint8)
    out=np.full((640,640,3),114,dtype=np.uint8);out[:,35:604]=valid
    return out


def main():
    import cv2
    random=np.random.default_rng(3501)
    y,x=np.mgrid[:1080,:960]
    images=[np.full((1080,960,3),127,dtype=np.uint8),
            np.repeat((((x+y)&1)*255).astype(np.uint8)[:,:,None],3,axis=2),
            random.integers(0,256,(1080,960,3),dtype=np.uint8)]
    rows=[]
    for name,image in zip(('flat','checkerboard','random_rgb'),images):
        output=fixed(image)
        ref=np.full((640,640,3),114,dtype=np.uint8)
        ref[:,35:604]=cv2.resize(image,(569,640),interpolation=cv2.INTER_LINEAR)
        difference=output.astype(np.float64)-ref.astype(np.float64)
        mse=float(np.mean(difference*difference));psnr=None if not mse else 10*math.log10(255*255/mse)
        maximum=int(np.max(np.abs(difference)))
        assert maximum<=2 and (psnr is None or psnr>=50)
        assert np.all(output[:,:35]==114) and np.all(output[:,604:]==114)
        rows.append({'pattern':name,'max_u8_error':maximum,'psnr_db':psnr,'exact':mse==0})
    report={'status':'PASS','reference':'OpenCV INTER_LINEAR, 569x640 active image, L35/R36 padding=114',
            'q_fractional_bits':11,'minimum_required_psnr_db':50,'cases':rows,
            'hardware_speed_measured':False}
    (BASE/'build/resize-reference.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
