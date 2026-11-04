from pathlib import Path
import json,urllib.request,hashlib
r=Path(__file__).resolve().parents[1]/'dependencies'
for p in json.loads((r/'needed.json').read_text()):
 out=r/Path(p['Filename']).name
 url='https://mirrors.ustc.edu.cn/ros2/ubuntu/'+p['Filename']
 if not out.exists():
  with urllib.request.urlopen(url,timeout=40) as response:out.write_bytes(response.read())
 assert hashlib.sha256(out.read_bytes()).hexdigest()==p['SHA256'],out
 print(out.name,len(out.read_bytes()),'SHA256_OK',flush=True)
