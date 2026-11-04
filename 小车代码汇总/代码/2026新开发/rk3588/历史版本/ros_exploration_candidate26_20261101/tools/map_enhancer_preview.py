#!/usr/bin/env python3
"""Run the pinned Wizard's real filter method on a recorded ROS map, offline.

No ROS/hardware connections. Raw Wizard threshold output is never installed.
Export a separate review map preserving every original occupied/unknown cell.
Live UI instead uses only the matching 3x3 Gaussian stage, on display pixels.
"""
import argparse, gzip, hashlib, json, sys
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
import yaml

p=argparse.ArgumentParser();p.add_argument('snapshot');p.add_argument('output');a=p.parse_args()
root=Path(__file__).resolve().parent/'map-enhancer-wizard';sys.path.insert(0,str(root/'code'))
from classes.map_enhancer_wizard import MapEnhancerWizard
source=Path(a.snapshot);raw=source.read_bytes();doc=json.loads(gzip.decompress(raw) if source.suffix=='.gz' else raw)
m=doc['render_map'];w,h=m['width'],m['height'];cells=np.array(list(m['cells'])).reshape(h,w)[::-1]
image=np.where(cells=='#',0,np.where(cells=='.',254,205)).astype(np.uint8)
out=Path(a.output);out.mkdir(parents=True,exist_ok=True);cv2.setNumThreads(1)
vars={'median':0,'blur':3,'opening':0,'closing':0,'dilation':0,'erosion':0,'threshold':.5}
dummy=SimpleNamespace(original_map=image,filter_input_map=image,use_adaptive=SimpleNamespace(get=lambda:False))
for k,v in vars.items():setattr(dummy,k+'_var',SimpleNamespace(get=lambda v=v:v))
filtered=MapEnhancerWizard.apply_filters(dummy)
review=filtered.copy();review[cells=='?']=205;review[cells=='#']=0
meta={'image':'original.pgm','resolution':m['resolution_m'],'origin':m['origin'],'negate':0,'occupied_thresh':.65,'free_thresh':.196}
cv2.imwrite(str(out/'original.pgm'),image);(out/'original.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
cv2.imwrite(str(out/'enhanced-review.pgm'),review);meta['image']='enhanced-review.pgm'
(out/'enhanced-review.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
cv2.imwrite(str(out/'wizard-unreviewed.png'),filtered)
gray=np.where(cells=='#',255,np.where(cells=='.',55,0)).astype(np.uint8)
smooth=cv2.GaussianBlur(gray,(3,3),0);smooth=np.maximum(smooth,55)
smooth[cells=='#']=255;smooth[cells=='?']=0
comparison=np.concatenate((gray,np.zeros((h,8),np.uint8),smooth),axis=1)
cv2.imwrite(str(out/'display-before-after.png'),cv2.resize(comparison,(2*w+8,h),interpolation=cv2.INTER_NEAREST))
report={'upstream':'https://github.com/ali-pahlevani/Map_Enhancer_Wizard',
        'commit':'29aec185dbab5ef35fbc81efaa613275ac2289f0','snapshot_sha256':hashlib.sha256(raw).hexdigest(),
        'source_measurement':'prior saved snapshot, not a new live scan','resolution_m':m['resolution_m'],
        'actual_upstream_apply_filters_executed':True,'gaussian_kernel':3,'threshold':.5,
        'raw_wizard_unknown_to_free':int(np.count_nonzero((cells=='?')&(filtered>250))),
        'raw_wizard_occupied_to_free':int(np.count_nonzero((cells=='#')&(filtered>250))),
        'review_unknown_preserved':bool(np.all(review[cells=='?']==205)),
        'review_obstacles_preserved':bool(np.all(review[cells=='#']==0)),
        'review_new_blocked_cells':int(np.count_nonzero((cells=='.')&(review<64))),
        'installed_into_navigation':False,'live_ui':'Gaussian stage only; original cells/pose/paths unchanged'}
(out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
