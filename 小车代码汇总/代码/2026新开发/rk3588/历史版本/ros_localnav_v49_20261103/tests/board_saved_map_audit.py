"""Read actual saved SLAM map; no publishers, edits or hardware transmission."""
import json,math,pathlib,collections
record=json.loads(pathlib.Path('/data/maps/current-map.json').read_text());g=json.loads((pathlib.Path(record['directory'])/'map.json').read_text())
x,y,_=record['pose'];ox,oy,yaw=g['origin'];c,s=math.cos(yaw),math.sin(yaw);res=g['resolution'];bad=[];near=collections.Counter()
for index,value in enumerate(g['cells']):
    gx,gy=index%g['width'],index//g['width'];lx,ly=(gx+.5)*res,(gy+.5)*res
    wx,wy=ox+c*lx-s*ly,oy+s*lx+c*ly;d=math.hypot(wx-x,wy-y)
    kind='unknown' if value<0 else 'free' if value<=25 else 'blocked'
    if d<.30:near[kind]+=1
    if kind!='free':bad.append((d,wx,wy,value))
bad.sort()
print(json.dumps({'real_saved_map':record,'width':g['width'],'height':g['height'],'resolution':res,'origin':g['origin'],'occupancy_counts':dict(collections.Counter(g['cells'])),'within_300mm':dict(near),'nearest_nonfree_cells':bad[:12],'read_only':True}),flush=True)
