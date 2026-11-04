#!/usr/bin/env python3
"""Prove the deterministic test rejects the old unsynchronized ARM transition.
Only temporary host Java is mutated; the production source/APK is never edited.
"""
import hashlib,json,pathlib,subprocess,tempfile
HERE=pathlib.Path(__file__).resolve().parent
src=HERE/'src/cn/acg138/robotphone/RobotClient.java'
s=src.read_text()
arm='''                    synchronized (navigationLock) {
                        if (foreground && !closed && generation == safetyGeneration.get()) {
                            armedAt = SystemClock.elapsedRealtime(); armedSequence = completedSequence.get(); armed = true;
                            listener.message("电机已使能，推动摇杆运动，松手即停");
                        }
                    }'''
arm_old='''                    if (foreground && !closed && generation == safetyGeneration.get()) {
                        armedAt = SystemClock.elapsedRealtime(); armedSequence = completedSequence.get(); armed = true;
                        listener.message("电机已使能，推动摇杆运动，松手即停");
                    }'''
opening='''            synchronized (navigationLock) {
            // Read ARM epoch'''
closing='''                listener.message("电机已停用：" + FollowPresentation.reason(reason));
            }
            }
            if (state.has("selected_id"))'''
if s.count(arm)!=1 or s.count(opening)!=1 or s.count(closing)!=1:raise RuntimeError('Production atomicity section changed; review negative mutation')
mutated=s.replace(arm,arm_old).replace(opening,'            // Read ARM epoch').replace(closing,closing.replace('\n            }\n            if','\n            if'))
with tempfile.TemporaryDirectory(prefix='acg138-v36-arm-negative-') as name:
    root=pathlib.Path(name);bad=root/'RobotClient.java';bad.write_text(mutated)
    source_list=(HERE/'build/control-v36-host/sources.txt').read_text().splitlines()
    source_list=[str(bad) if pathlib.Path(p).resolve()==src.resolve() else p for p in source_list]
    (root/'sources.txt').write_text('\n'.join(source_list)+'\n');(root/'classes').mkdir()
    subprocess.run(['javac','-encoding','UTF-8','--release','8','-d',str(root/'classes'),'@'+str(root/'sources.txt')],check=True)
    result=subprocess.run(['java','-Djava.awt.headless=true','-cp',str(root/'classes'),'ClientV36Tests','arm-race'],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=20)
    expected='ARM write waits for atomic false-observation section'
    if result.returncode==0 or expected not in result.stdout:raise RuntimeError('Negative control did not fail at intended ARM race assertion: '+result.stdout)
    report={'status':'PASS','test':'Negative control must fail: remove only the shared ARM/false-observation lock in a temporary host source','negative_exit_code':result.returncode,'negative_expected_assertion':expected,'negative_output':result.stdout.strip(),'production_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'temporary_mutated_sha256':hashlib.sha256(mutated.encode()).hexdigest(),'production_source_modified':False,'apk_generated_from_mutation':False}
    dest=HERE/'dist/android-arm-race-negative-control.json';dest.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('PASS deterministic ARM negative control: old unsynchronized transition fails exactly at intended assertion')
