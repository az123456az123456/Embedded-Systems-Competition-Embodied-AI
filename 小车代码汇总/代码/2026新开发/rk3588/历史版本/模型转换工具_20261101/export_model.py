from pathlib import Path
import copy,hashlib,json,types
import numpy as np
import torch,onnx
from ultralytics import YOLO
ROOT=Path(__file__).resolve().parent
p=Path('/home/tan/Downloads/last.pt')
torch.set_num_threads(6)
model=YOLO(str(p)).model.float().eval()
original=copy.deepcopy(model)
head=model.model[-1]
assert head.nc==9 and head.reg_max==16 and not getattr(head,'end2end',False)
def forward(self,x):
 out=[]
 for i in range(self.nl):
  reg=self.cv2[i](x[i]);cls=self.cv3[i](x[i]).sigmoid()
  out.extend((reg,cls,cls.sum(1,keepdim=True)))
 return tuple(out)
head.forward=types.MethodType(forward,head)
x=torch.rand(1,3,640,640)
with torch.inference_mode():
 ref=original(x);out=model(x)
 raw=ref[1]
 if isinstance(raw,dict) and 'one2many' in raw:raw=raw['one2many']
 if isinstance(raw,dict):
  parts=[];offset=0
  for side in (80,40,20):
   count=side*side;parts.append(torch.cat((raw['boxes'][:,:,offset:offset+count],raw['scores'][:,:,offset:offset+count]),1).reshape(1,73,side,side));offset+=count
  raw=parts
 errors=[]
 for i in range(3):
  errors.extend([float((out[3*i]-raw[i][:,:64]).abs().max()),float((out[3*i+1]-raw[i][:,64:].sigmoid()).abs().max())])
assert max(errors)<1e-6,errors
names=[v for _,v in sorted(YOLO(str(p)).names.items())]
onnx_file=ROOT/'acg138-last-e0e54560.onnx'
torch.onnx.export(model,x,str(onnx_file),input_names=['images'],output_names=[f'{name}_{side}' for side in (80,40,20) for name in ('dfl','classes','score_sum')],opset_version=12,dynamo=False,do_constant_folding=True)
m=onnx.load(str(onnx_file));onnx.checker.check_model(m)
shapes={o.name:[d.dim_value for d in o.type.tensor_type.shape.dim] for o in m.graph.output}
expected=[[1,c,s,s] for s in (80,40,20) for c in (64,9,1)]
assert list(shapes.values())==expected,shapes
(ROOT/'labels.txt').write_text('\n'.join(names)+'\n')
(ROOT/'reports/export.json').write_text(json.dumps({'checkpoint_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'onnx_sha256':hashlib.sha256(onnx_file.read_bytes()).hexdigest(),'input':[1,3,640,640],'outputs':shapes,'labels':names,'native_head_max_abs_errors':errors,'native_head_equivalent':True},indent=2)+'\n')
print(json.dumps({'onnx':str(onnx_file),'head_equivalent':True,'outputs':shapes}))
