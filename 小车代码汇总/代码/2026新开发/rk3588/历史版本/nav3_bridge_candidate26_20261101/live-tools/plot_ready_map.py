from pathlib import Path
import json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
r=Path(__file__).resolve().parents[1]
p=r/'board-evidence/stage3-ready-evidence/acg138-ros2/logs'
a=json.loads((p/'nav3-stage3-ready-map-audit.json').read_text());o=json.loads((p/'nav3-stage3-ready-observer-result.json').read_text());m=a['map'];s=a['scan'];c=o['local_map']
fig,ax=plt.subplots(1,3,figsize=(16,5),layout='constrained')
g=np.array(m['data']).reshape(m['height'],m['width']);x,y=m['origin'];h=m['resolution']
ax[0].imshow(g,origin='lower',extent=[x,x+h*m['width'],y,y+h*m['height']],cmap=ListedColormap(['#999999','#fafafa','#111111']),vmin=-1,vmax=100,interpolation='none')
# Explicit mapping: -1 unknown grey; 0 known free white; 100 occupied black.
ax[0].images[0].set_data(np.where(g<0,0,np.where(g==0,1,2)));ax[0].images[0].set_clim(0,2)
ax[0].set_title('Actual /map: grey unknown, white free, black occupied')
pts=[]
for i,d in enumerate(s['ranges']):
 if d is not None:
  t=s['angle_min']+i*s['angle_increment'];pts.append((d*math.cos(t),d*math.sin(t)))
xx,yy=zip(*pts);ax[1].scatter(xx,yy,s=8);ax[1].set_title('Actual /scan, all finite ranges')
cg=np.array(c['data']).reshape(64,64);x,y=c['origin'];h=c['resolution'];ax[2].imshow(cg,origin='lower',extent=[x,x+64*h,y,y+64*h],cmap=ListedColormap(['white','#333333']),vmin=0,vmax=100,interpolation='none');ax[2].set_title('FPGA local map: white traversable, dark blocked')
for axy in ax:
 axy.set_xlim(-1.2,1.2);axy.set_ylim(-1.2,1.2);axy.set_aspect('equal');axy.plot(0,0,'r+');axy.add_patch(plt.Circle((0,0),.2663,fill=False,color='red',lw=1));axy.set_xlabel('x / m (forward)');axy.set_ylabel('y / m (left)');axy.grid(alpha=.2)
fig.savefig(r/'board-evidence/stage3-ready-map-detail.png',dpi=160)
# Direct source-cell values near the robot; no map modifications.
near=[]
for iy in range(m['height']):
 for ix in range(m['width']):
  cx=m['origin'][0]+(ix+.5)*m['resolution'];cy=m['origin'][1]+(iy+.5)*m['resolution']
  if abs(cx)<=.4 and abs(cy)<=.4:near.append({'x':cx,'y':cy,'value':int(g[iy,ix])})
(r/'board-evidence/stage3-ready-near-source-cells.json').write_text(json.dumps(near,indent=2)+'\n')
print({'local_free':[i for i,v in enumerate(c['data']) if v==0],'source_near_counts':{str(v):sum(t['value']==v for t in near) for v in [-1,0,100]},'figure':str(r/'board-evidence/stage3-ready-map-detail.png')})
