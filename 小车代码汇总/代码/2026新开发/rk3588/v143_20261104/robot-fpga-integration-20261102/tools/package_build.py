from pathlib import Path
import tarfile,json,hashlib
r=Path(__file__).resolve().parents[1]; out=r/'reports/semantic-build-input.tar.gz'
with tarfile.open(out,'w:gz') as t:
 for name in ('acg138_frontier','acg138_ros_bridge','acg138_semantic','common','vendor'):
  t.add(r/name,arcname='src/'+name,filter=lambda x:None if ('__pycache__' in x.name or x.name.endswith('.pyc')) else x)
 for deb in (r/'dependencies').glob('*.deb'):t.add(deb,arcname='deps/'+deb.name)
print(out,out.stat().st_size,hashlib.sha256(out.read_bytes()).hexdigest())
