"""Report actual RK outputs against native ONNX and held-out annotations.

These are confidence=.25 operating-point metrics, not a full-dataset mAP claim.
"""
from pathlib import Path
from collections import Counter, defaultdict
import argparse, json, math, statistics

p=argparse.ArgumentParser();p.add_argument('--variant',default='int8');a=p.parse_args()
r=Path(__file__).resolve().parent
def load(path):return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
native={v['id']:v for v in load(r/'reports/native-holdout.jsonl')}
actual=load(r/f'reports/board-{a.variant}-single.jsonl')
manifest={v['id']:v for v in json.loads((r/'board-validation/manifest.json').read_text())}
assert len(actual)==len(native)==len(manifest)==23
assert len({v['id'] for v in actual})==23 and {v['id'] for v in actual}==set(native)
names=(r/'labels.txt').read_text().splitlines()

def iou(x,y):
    area=max(0,min(x[2],y[2])-max(x[0],y[0]))*max(0,min(x[3],y[3])-max(x[1],y[1]))
    union=(x[2]-x[0])*(x[3]-x[1])+(y[2]-y[0])*(y[3]-y[1])-area
    return area/union if union>0 else 0.

def matches(predictions,targets,target_class_index):
    used=set();pairs=[]
    for pi in sorted(range(len(predictions)),key=lambda i:-predictions[i][4]):
        p=predictions[pi]
        eligible=[(iou(p,t),j) for j,t in enumerate(targets)
                  if j not in used and int(p[5])==int(t[target_class_index])]
        overlap,j=max(eligible,default=(0.,-1))
        if overlap>=.5:used.add(j);pairs.append((pi,j,overlap))
    return pairs

def summarize(records):
    total=defaultdict(Counter)
    for v in records:
        pred=v['detections'];gt=manifest[v['id']]['ground_truth']
        for d in pred:
            assert len(d)==6 and all(math.isfinite(n) for n in d)
            assert 0<=d[5]<9 and d[5]==int(d[5]) and .24999<=d[4]<=1.00001
            total[int(d[5])]['predictions']+=1
        for g in gt:total[int(g[4])]['ground_truth']+=1
        for i,j,_ in matches(pred,gt,4):total[int(pred[i][5])]['true_positive']+=1
    def metrics(counts):
        tp=counts.get('true_positive',0);pred=counts.get('predictions',0);gt=counts.get('ground_truth',0)
        precision=tp/pred if pred else None;recall=tp/gt if gt else None
        f1=2*tp/(pred+gt) if pred+gt else None
        return {**counts,'precision':precision,'recall':recall,'f1':f1}
    all_counts=sum(total.values(),Counter())
    return {'overall':metrics(all_counts),'per_class':{names[i]:metrics(total[i]) for i in range(9)}}

agreement=[];ious=[];confidence_deltas=[];coordinate_deltas=[];matched=0;ref_count=0;actual_count=0
for v in actual:
    p=v['detections'];ref=native[v['id']]['detections'];pairs=matches(p,ref,5)
    matched+=len(pairs);ref_count+=len(ref);actual_count+=len(p)
    agreement.append({'id':v['id'],'native_count':len(ref),'rknn_count':len(p),'matched_same_class_iou_0_5':len(pairs)})
    for i,j,overlap in pairs:
        ious.append(overlap);confidence_deltas.append(abs(p[i][4]-ref[j][4]))
        coordinate_deltas.extend(abs(p[i][k]-ref[j][k]) for k in range(4))

def stats(values):
    return {'min':min(values),'median':statistics.median(values),'mean':statistics.mean(values),'max':max(values)} if values else None

parallel=load(r/f'reports/board-{a.variant}-three.jsonl')
assert len(parallel)==69 and {v['core'] for v in parallel}=={0,1,2}
assert {v['job'] for v in parallel}==set(range(69))
single_by_id={v['id']:v['detections'] for v in actual}
repeat_differences=[]
for v in parallel:
    baseline=single_by_id[v['id']]
    pairs=matches(v['detections'],baseline,5)
    if len(pairs)!=len(baseline) or len(pairs)!=len(v['detections']):
        repeat_differences.append({'job':v['job'],'core':v['core'],'id':v['id'],'single':len(baseline),'parallel':len(v['detections'])})

native_metrics=summarize(native.values());quant_metrics=summarize(actual)
report={'variant':a.variant,'status':'MEASURED','scope':'23 held-out 640x640 images; operating point confidence .25, NMS .45, GT IoU .5',
        'not_full_dataset_map':True,'label_order_from':'checkpoint; dataset YAML not supplied',
        'calibration_disjoint_sha256':True,'native':native_metrics,'rknn':quant_metrics,
        'f1_delta_vs_native':quant_metrics['overall']['f1']-native_metrics['overall']['f1'],
        'agreement':{'native_detections':ref_count,'rknn_detections':actual_count,'matched':matched,
                     'native_recovered_fraction':matched/ref_count if ref_count else None,
                     'iou':stats(ious),'absolute_confidence_delta':stats(confidence_deltas),
                     'absolute_coordinate_delta_pixels':stats(coordinate_deltas)},
        'single_worker_timings_ms':{k:stats([v[k] for v in actual]) for k in ('preprocess_ms','input_ms','run_ms','output_ms','post_ms','total_ms')},
        'three_worker_timings_ms':{k:stats([v[k] for v in parallel]) for k in ('preprocess_ms','input_ms','run_ms','output_ms','post_ms','total_ms')},
        'three_worker_per_core':{str(c):{'jobs':sum(v['core']==c for v in parallel),'run_ms':stats([v['run_ms'] for v in parallel if v['core']==c])} for c in range(3)},
        'single_vs_three_detection_set_changes':repeat_differences,'per_image_agreement':agreement,
        'limitations':['functional NPU hardware check with files, not live stereo pipeline FPS',
                        'small held-out set, temporal correlation not ruled out',
                        'metrics depend on annotation class order matching checkpoint',
                        'no motor or FPGA commands sent']}
(r/f'reports/quality-{a.variant}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ('variant','status','f1_delta_vs_native','agreement','single_vs_three_detection_set_changes')},ensure_ascii=False,indent=2))
