from pathlib import Path
from rknn.api import RKNN
import argparse,hashlib,json
p=argparse.ArgumentParser();p.add_argument('--int8',action='store_true');p.add_argument('--algorithm',choices=('normal','mmse'),default='normal');a=p.parse_args();r=Path(__file__).resolve().parent
name=('int8' if a.algorithm=='normal' else 'int8-'+a.algorithm) if a.int8 else 'fp16';model=r/f'acg138-last-e0e54560-{name}-rk3588.rknn'
n=RKNN(verbose=False)
try:
 assert n.config(mean_values=[[0,0,0]],std_values=[[255,255,255]],target_platform='rk3588',quantized_algorithm=a.algorithm)==0
 assert n.load_onnx(model=str(r/'acg138-last-e0e54560.onnx'))==0
 assert n.build(do_quantization=a.int8,dataset=str(r/'dataset.txt') if a.int8 else None)==0
 assert n.export_rknn(str(model))==0
 (r/f'reports/rknn-{name}.json').write_text(json.dumps({'model':str(model),'bytes':model.stat().st_size,'sha256':hashlib.sha256(model.read_bytes()).hexdigest(),'int8':a.int8,'quantized_algorithm':a.algorithm if a.int8 else None,'target':'rk3588','toolkit':'2.0.0b0+9bab5682','input':'RGB888 640x640; normalization in model','hardware_tested':False},indent=2)+'\n')
finally:n.release()
