"""Make bit-identical RGB640 inputs and native ONNX references for RK testing."""
from pathlib import Path
import ctypes, hashlib, json
import numpy as np
import onnxruntime as ort
from PIL import Image

root=Path(__file__).resolve().parent
out=root/'board-validation'
(out/'images').mkdir(parents=True,exist_ok=True)
cases=json.loads((root/'reports/holdout.json').read_text())
options=ort.SessionOptions(); options.intra_op_num_threads=6; options.inter_op_num_threads=1
session=ort.InferenceSession(str(root/'acg138-last-e0e54560.onnx'),options,providers=['CPUExecutionProvider'])
lib=ctypes.CDLL(str(root/'build/libdecode_reference.so'))
ptr=ctypes.POINTER(ctypes.c_float)
lib.decode_reference.argtypes=[ctypes.POINTER(ptr),ptr,ctypes.c_int,ctypes.c_float,ctypes.c_float]
lib.decode_reference.restype=ctypes.c_int
manifest=[]; references=[]
for i,item in enumerate(cases):
    source=Path(item['source'])
    assert hashlib.sha256(source.read_bytes()).hexdigest()==item['source_sha256']
    with Image.open(source) as image:
        assert image.size==(640,640) and image.getexif().get(274,1)==1
        rgb=np.asarray(image.convert('RGB')).copy()
    name=f'h{i:03d}.png'; target=out/'images'/name
    Image.fromarray(rgb).save(target)
    values=session.run(None,{'images':rgb.transpose(2,0,1)[None].astype(np.float32)/255})
    values=[np.ascontiguousarray(x,dtype=np.float32) for x in values]
    pointers=(ptr*9)(*(x.ctypes.data_as(ptr) for x in values))
    result=np.empty((128,6),np.float32)
    count=lib.decode_reference(pointers,result.ctypes.data_as(ptr),128,.25,.45)
    assert count>=0
    gt=[]
    for line in (source.parent.parent/'labels'/(source.stem+'.txt')).read_text().splitlines():
        if not line.strip():continue
        c,x,y,w,h=map(float,line.split())
        gt.append([(x-w/2)*640,(y-h/2)*640,(x+w/2)*640,(y+h/2)*640,int(c)])
    manifest.append({**item,'id':f'h{i:03d}','input':f'images/{name}',
                     'input_sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
                     'rgb_sha256':hashlib.sha256(rgb.tobytes()).hexdigest(),'ground_truth':gt})
    references.append({'id':f'h{i:03d}','source':'native_onnx_float32',
                       'confidence':.25,'nms':.45,'detections':result[:count].tolist()})
    print(f'reference {i+1}/{len(cases)} detections={count}',flush=True)
(out/'inputs.txt').write_text(''.join(x['input']+'\n' for x in manifest))
(out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
(root/'reports/native-holdout.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in references))
