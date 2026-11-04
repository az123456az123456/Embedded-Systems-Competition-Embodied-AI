from pathlib import Path
import shutil,hashlib,json,tarfile
r=Path(__file__).resolve().parents[1];s=r/'runtime-payload'
manifest=dict(feature='PPE semantic target exploration',phone_version='1.2.35',upstream='robo-friends/m-explore-ros2',commit='326cf8a0b487c34246bb8f3326afbcd69576dc60',
    FPGA_modified=False,new_NPU_contexts=0,voice_modified=False,source_overlay='/opt/acg138_semantic_20261103',
    native_build='5 packages PASS',native_test='isolated synthetic camera/map/TF/odometry; no hardware bridge',
    gateway_sha256=hashlib.sha256((r/'bin/robot-fpga-gateway.aarch64').read_bytes()).hexdigest())
(s/'install-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
for f in ('run-stack.sh','exec-chroot.sh','ros-entrypoint.sh'):shutil.copy2(r/'deploy'/f,s/'deploy'/f)
shutil.copy2(r/'bin/robot-fpga-gateway.aarch64',s/'bin/robot-fpga-gateway.aarch64')
files=sorted(p for p in s.rglob('*') if p.is_file() and p.name!='SHA256SUMS')
(s/'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(s))+'\n' for p in files))
with tarfile.open(r/'reports/semantic-runtime.tar.gz','w:gz') as t:
 for p in s.iterdir():t.add(p,arcname=p.name)
print('runtime', (r/'reports/semantic-runtime.tar.gz').stat().st_size)
