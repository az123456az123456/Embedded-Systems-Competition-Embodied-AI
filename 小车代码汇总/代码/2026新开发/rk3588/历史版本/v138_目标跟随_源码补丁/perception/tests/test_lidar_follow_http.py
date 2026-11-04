"""Real HTTP/controller bridge with synthetic camera, IMU and scan observations."""
import base64
import http.client
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    with tempfile.TemporaryDirectory() as tmp:
        path=Path(tmp);token=path/'pair';token.write_text('012345\n')
        eye=dict(calibrated=True,fx_px=600,cx_px=480,yaw_right_deg=0,lateral_right_m=0,min_u_px=0,max_u_px=959,max_error_deg=.5)
        geometry=path/'geometry.json';geometry.write_text(json.dumps(dict(schema=1,source_eye_width_px=960,enabled=False,lookahead_m=1,eyes=[eye,eye])))
        env=dict(os.environ,FOLLOW_GEOMETRY_FILE=str(geometry))
        with (path/'stderr').open('w') as log:
            process=subprocess.Popen([str(ROOT/'build/phone-publish-probe'),str(port),str(token)],env=env,stdout=log,stderr=log)
            def request(method,url,value=None,authorized=True):
                connection=http.client.HTTPConnection('127.0.0.1',port,timeout=1)
                headers={'Authorization':'Bearer 012345'} if authorized else {}
                connection.request(method,url,json.dumps(value) if value is not None else None,headers)
                response=connection.getresponse();status=response.status;body=json.loads(response.read());connection.close()
                return status,body
            try:
                for _ in range(80):
                    try:
                        if request('GET','/api/state')[1]['vision']['valid']:break
                    except OSError:pass
                    time.sleep(.01)
                else:raise AssertionError('probe not ready')
                owner=request('POST','/api/connect',{})[1];session=owner['session'];sequence=0
                def control(op,**fields):
                    nonlocal sequence
                    sequence+=1
                    return request('POST','/api/control',dict(session=session,seq=sequence,op=op,**fields))
                time.sleep(.12);assert control('arm')[0]==200
                video=http.client.HTTPConnection('127.0.0.1',port,timeout=1)
                video.request('GET','/api/video.mjpg',headers={'Authorization':'Bearer 012345'})
                response=video.getresponse()
                while response.readline().strip()!=b'--robotframe':pass
                headers={}
                while True:
                    line=response.readline()
                    if line==b'\r\n':break
                    key,value=line.decode().split(':',1);headers[key.lower()]=value.strip()
                meta=json.loads(base64.b64decode(headers['x-frame-meta']))
                response.read(int(headers['content-length']));response.close();video.close()
                target_id=meta['detections'][0]['id']
                assert control('select',frame_id=meta['frame_id'],target_id=target_id)[0]==200
                assert control('follow')[0]==200
                state=request('GET','/api/state')[1];target=state['selected_target']
                assert not target['ray_bounds_valid'] and target['image_valid'] and target['selection_mono_ms']>0
                assert state['follow_geometry']['image_servo_supported']
                assert state['follow']['active'] and not state['follow']['distance_valid']
                assert state['motor']['linear_mps']==state['motor']['angular_rps']==0
                def observation(distance):
                    target=request('GET','/api/state')[1]['selected_target'];now=int(time.monotonic()*1000)
                    return dict(target_id=target['id'],selection_ms=target['selection_mono_ms'],vision_ms=target['vision_mono_ms'],
                        scan_ms=now,pose_ms=now,matched=True,forward_clear=True,reverse_clear=True,turn_clear=True,
                        distance_m=distance,bearing_right_deg=0,x_body_m=distance,y_body_m=0)
                value=observation(2)
                assert request('POST','/api/navigation/follow-observation',value,False)[0]==401
                assert request('POST','/api/navigation/follow-observation',value)[0]==200
                control('follow');state=request('GET','/api/state')[1]
                assert state['follow']['distance_valid'] and state['motor']['linear_mps']>0
                # Observation POST did not increment the phone's control sequence.
                assert state['control_sequence']==sequence
                assert request('POST','/api/navigation/follow-observation',observation(1.02))[0]==200
                control('follow');state=request('GET','/api/state')[1]
                assert state['mode']=='follow' and state['selected_id']==target_id
                assert state['reason']=='follow_holding_distance' and state['motor']['linear_mps']==0
                old=observation(2);old['selection_ms']+=1
                assert request('POST','/api/navigation/follow-observation',old)[0]==409
                # Cancel through the exact safe-zero path used by the phone.
                assert control('drive',throttle=0,steer=0,hold=False)[0]==200
                request('POST','/api/navigation/follow-observation',observation(2))
                state=request('GET','/api/state')[1]
                assert state['mode']=='idle' and state['motor']['linear_mps']==0 and not state['follow']['active']
                assert control('stop')[0]==200
                assert request('POST','/api/navigation/follow-observation',value)[0]==409
                assert not request('GET','/api/state')[1]['armed']
                print('PASS production HTTP follow: image servo without intrinsics, authentication, source identity, ranging, 1m hold, owner sequence, cancel/late observation')
            finally:
                process.terminate();process.wait(timeout=3)


if __name__=='__main__':main()
