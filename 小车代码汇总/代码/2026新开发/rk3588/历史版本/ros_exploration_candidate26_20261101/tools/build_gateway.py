#!/usr/bin/env python3
"""Build the existing HTTP Gateway plus FPGA lifecycle adapter; no ROS linkage."""
import argparse,json,subprocess,hashlib,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--compiler',default='g++');p.add_argument('--static-cxx',action='store_true');p.add_argument('--output',required=True);a=p.parse_args()
r=Path(__file__).resolve().parents[1];out=Path(a.output).resolve();out.parent.mkdir(parents=True,exist_ok=True)
sources=['gateway/runtime.cc','gateway/http_client.cc','rk_gateway/navigation/gateway.cc','rk_gateway/navigation/vendor/http_server.cc','rk_gateway/navigation/vendor/json_flat.cc','common/wrong_log.cc','common/ros_bridge_codec.cc','common/ros_bridge_session.cc','common/ros_bridge_transport.cc']
command=[a.compiler,'-std=c++17','-O2','-Wall','-Wextra','-Werror','-pthread','-I'+str(r/'common'),*[str(r/x) for x in sources],'-o',str(out)]
if a.static_cxx:command[1:1]=['-static-libstdc++','-static-libgcc']
t=time.monotonic();c=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
Path(str(out)+'.build.log').write_bytes(c.stdout)
Path(str(out)+'.build.json').write_text(json.dumps({'status':'PASS' if c.returncode==0 else 'FAIL','seconds':time.monotonic()-t,'command':command,'sources':{x:hashlib.sha256((r/x).read_bytes()).hexdigest() for x in sources},'FPGA_modified':False,'ROS_libraries_linked':False},indent=2)+'\n')
if c.returncode:print(c.stdout.decode(errors='replace'))
raise SystemExit(c.returncode)
