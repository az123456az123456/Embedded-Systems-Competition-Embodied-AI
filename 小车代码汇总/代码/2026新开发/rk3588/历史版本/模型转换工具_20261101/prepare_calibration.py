from pathlib import Path
from PIL import Image,ImageOps
from collections import Counter,defaultdict
import argparse,json,hashlib,math
p=argparse.ArgumentParser();p.add_argument('--source',default='/home/tan/Downloads/valid');a=p.parse_args()
r=Path(__file__).resolve().parent;source=Path(a.source);images=source/'images';labels=source/'labels';assert images.is_dir(),images
items=[];counts=Counter();families=defaultdict(list)
for f in sorted(images.iterdir()):
 if f.suffix.lower() not in ('.jpg','.jpeg','.png','.bmp'):continue
 classes=set();lf=labels/(f.stem+'.txt')
 if lf.exists():
  for row in lf.read_text().splitlines():
   v=[float(x) for x in row.split()]
   if not v:continue
   assert len(v)==5 and v[0].is_integer() and 0<=v[0]<9 and all(math.isfinite(x) for x in v),lf
   classes.add(int(v[0]))
 sha=hashlib.sha256(f.read_bytes()).hexdigest();family=f.stem.split('.rf.')[0]
 item={'path':f,'classes':classes,'sha':sha,'family':family};items.append(item);families[family].append(item);counts.update(classes)
keys=sorted(families,key=lambda x:hashlib.sha256(('split-20261102:'+x).encode()).hexdigest());test_families=set(keys[::5]);pool=[x for x in items if x['family'] not in test_families];holdout=[x for x in items if x['family'] in test_families]
# Guard exact-content duplicates even when their filenames differ.
test_shas={x['sha'] for x in holdout};pool=[x for x in pool if x['sha'] not in test_shas]
selected=[];coverage=Counter();remaining=sorted(pool,key=lambda x:x['sha'])
while remaining and len(selected)<min(320,len(pool)):
 best=max(range(len(remaining)),key=lambda i:sum(max(0,25-coverage[c])/max(1,counts[c]) for c in remaining[i]['classes']))
 item=remaining.pop(best);selected.append(item);coverage.update(item['classes'])
out=r/'calibration';out.mkdir(exist_ok=True);manifest=[]
for i,item in enumerate(selected):
 f=item['path']
 with Image.open(f) as image:
  image=ImageOps.exif_transpose(image).convert('RGB');w,h=image.size;scale=min(640/w,640/h);new=(round(w*scale),round(h*scale));resized=image.resize(new,Image.Resampling.BILINEAR);canvas=Image.new('RGB',(640,640),(114,114,114));canvas.paste(resized,((640-new[0])//2,(640-new[1])//2));target=out/f'{i:03}.png';canvas.save(target)
 manifest.append({'source':str(f),'source_sha256':item['sha'],'source_family':item['family'],'classes':sorted(item['classes']),'prepared':str(target)})
(r/'dataset.txt').write_text(''.join(x['prepared']+'\n' for x in manifest))
validation=[{'source':str(x['path']),'source_sha256':x['sha'],'source_family':x['family'],'classes':sorted(x['classes'])} for x in holdout]
assert not {x['source_sha256'] for x in manifest}.intersection(x['source_sha256'] for x in validation)
(r/'reports/calibration.json').write_text(json.dumps({'count':len(manifest),'scope':'user supplied valid set; subset reused for INT8 calibration','source':str(source),'class_names_from':'checkpoint','dataset_yaml':'not supplied','all_image_count':len(items),'all_image_class_coverage':dict(counts),'calibration_class_coverage':dict(coverage),'holdout_count':len(validation),'split':'source filename family and exact SHA256 disjoint; adjacent video frames may remain correlated','images':manifest},ensure_ascii=False,indent=2)+'\n')
(r/'reports/holdout.json').write_text(json.dumps(validation,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'all':len(items),'calibration':len(manifest),'class_coverage':dict(coverage),'held_out':len(validation)}))
