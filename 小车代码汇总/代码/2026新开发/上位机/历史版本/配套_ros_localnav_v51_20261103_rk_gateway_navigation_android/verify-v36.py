#!/usr/bin/env python3
"""Source-bound host verification and locally signed APK build; never contacts a robot."""
import hashlib, json, os, pathlib, re, shutil, subprocess, sys, time
HERE=pathlib.Path(__file__).resolve().parent
OUT=HERE/'dist'
OUT.mkdir(exist_ok=True)
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def sources():
    paths=[HERE/'AndroidManifest.xml',*HERE.glob('*.sh'),*HERE.glob('*.py')]
    for name in ('src','res','tests'):
        paths += [p for p in (HERE/name).rglob('*') if p.is_file()]
    return {str(p.relative_to(HERE)):digest(p) for p in sorted(set(paths))}
before=sources()
report={'status':'RUNNING','scope':'Production Java client and View with fault-injected HTTP/Android host adapters; no physical Android/WLAN/RK test','started_unix_s':time.time(),'tests':[],'source_sha256':before}
report_path=OUT/'android-v36-verification.json'
def save():report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
def run(args,label,timeout=180):
    start=time.monotonic()
    p=subprocess.run(args,cwd=HERE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout,env=os.environ.copy())
    item={'name':label,'exit_code':p.returncode,'elapsed_s':round(time.monotonic()-start,3),'output':p.stdout.strip()}
    report['tests'].append(item);save()
    print(label+': '+p.stdout.strip(),flush=True)
    if p.returncode:raise RuntimeError(label+' failed')
    return p.stdout
save()
try:
    for script in ('test-host.sh','test-client-host.sh','test-control-v36.sh','test-ui-host.sh','test-capture-host.sh','test-map-host.sh','test-follow-host.sh','test-sonar-host.sh'):
        run(['sh',str(HERE/script)],script)
        if script=='test-control-v36.sh':run([sys.executable,str(HERE/'test-arm-race-negative.py')],'deterministic ARM race negative control')
    run(['sh',str(HERE/'build-apk.sh')],'signed APK build')
    tools=pathlib.Path(os.environ['BUILD_TOOLS'])
    old=HERE.parents[3]/'robot_sonar_avoid_v33'/'navigation'/'android'/'dist'/'ACG138-Robot-Control-debug.apk'
    # HERE = rk3588 / robot_control_v36 / navigation / android.
    old=HERE.parents[2]/'robot_sonar_avoid_v33'/'navigation'/'android'/'dist'/'ACG138-Robot-Control-debug.apk'
    apk=OUT/'ACG138-Robot-Control-debug.apk'
    new_cert=run([str(tools/'apksigner'),'verify','--verbose','--print-certs',str(apk)],'new APK signature')
    old_cert=run([str(tools/'apksigner'),'verify','--print-certs',str(old)],'previous APK signature')
    pattern=r'certificate SHA-256 digest: ([0-9a-f]+)'
    new_digest=re.search(pattern,new_cert).group(1);old_digest=re.search(pattern,old_cert).group(1)
    if new_digest!=old_digest:raise RuntimeError('APK signing certificate differs from previous app')
    manifest=run([str(tools/'aapt2'),'dump','badging',str(apk)],'new APK manifest')
    old_manifest=run([str(tools/'aapt2'),'dump','badging',str(old)],'previous APK manifest')
    version=int(re.search(r"versionCode='(\d+)'",manifest).group(1))
    previous=int(re.search(r"versionCode='(\d+)'",old_manifest).group(1))
    if version<=previous:raise RuntimeError('APK versionCode does not increase')
    if sources()!=before:raise RuntimeError('Source changed during verification; repeat from current snapshot')
    release=OUT/'ACG138-Robot-Control-v36.apk';shutil.copyfile(apk,release)
    report.update(status='PASS',apk={'filename':release.name,'sha256':digest(release),'bytes':release.stat().st_size,'versionCode':version,'previousVersionCode':previous,'certificate_sha256':new_digest,'same_previous_certificate':True},private_key_in_delivery=False,hardware_validation='Not performed; Android device touch/RF/storage provider and real RK timeout behavior require board verification')
except Exception as ex:
    report.update(status='FAIL',error=str(ex));save();raise
report['elapsed_s']=round(time.time()-report['started_unix_s'],3)
save();print('PASS source-bound Android v36 verification: '+str(report_path),flush=True)
