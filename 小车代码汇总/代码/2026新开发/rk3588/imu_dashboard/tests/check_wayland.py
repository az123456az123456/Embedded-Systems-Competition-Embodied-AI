from pathlib import Path
import os, subprocess, tempfile
with tempfile.TemporaryDirectory(prefix='imu-wayland-') as runtime:
    env=dict(os.environ,XDG_RUNTIME_DIR=runtime,WAYLAND_DISPLAY='imu-test')
    server=subprocess.Popen(['build/test-wayland'],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        first=server.stdout.readline().strip()
        assert first=='READY',(first,server.stderr.read())
        client=subprocess.run(['build/imu-dashboard','--demo','--seconds','5'],env=env,capture_output=True,text=True,timeout=10)
        out,err=server.communicate(timeout=5)
        assert client.returncode==0,(client.stdout,client.stderr)
        assert server.returncode==0,(out,err)
        print(out.strip())
    finally:
        if server.poll() is None: server.kill(); server.wait()
