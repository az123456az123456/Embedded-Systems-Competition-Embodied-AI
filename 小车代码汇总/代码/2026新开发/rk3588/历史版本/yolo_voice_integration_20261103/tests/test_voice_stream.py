from pathlib import Path
import base64, hashlib, http.server, json, os, subprocess, tempfile, threading, time
r=Path(__file__).resolve().parents[1];records=[]

def check(case,okay,**more):
 records.append({'case':case,'pass':bool(okay),**more})
 if not okay:raise AssertionError(case)

with tempfile.TemporaryDirectory(prefix='ppe-source-test-') as tmp:
 d=Path(tmp);exe=d/'ppe-voice';log=d/'played.jsonl';status=d/'voice-status.json';token=d/'token';token.write_text('offline-test-only\n')
 include=Path('/home/tan/qiansai/rk3588/ros_localnav_v51_20261103/vendor')
 cmd=['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(include),str(r/'baseline/alerts/src/ppe_voice.cc'),'-o',str(exe)]
 x=subprocess.run(cmd,capture_output=True,text=True);(r/'reports/voice-compile.log').write_text(x.stdout+x.stderr);check('unchanged_RK_voice_source_compiles_on_host',x.returncode==0)
 player=d/'player.sh';player.write_text(f'#!/bin/sh\nprintf "%s\\n" "$1" >> "{log}"\n');player.chmod(0o755)
 requests=[];invalid_sent=threading.Event();allow_valid=threading.Event()
 class Handler(http.server.BaseHTTPRequestHandler):
  def log_message(self,*args):pass
  def do_GET(self):
   requests.append({'method':'GET','path':self.path,'authorization_ok':self.headers.get('Authorization')=='Bearer offline-test-only'})
   self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.end_headers()
   def frame(labels,**overrides):
    meta={'demo':False,'control_eligible':True,'detections_pair_status':'PAIR_COMPLETE','detection_timestamp_ms':time.monotonic_ns()//1000000,'detections':[{'label':x,'score':.9} for x in labels]};meta.update(overrides)
    encoded=base64.b64encode(json.dumps(meta).encode()).decode()
    self.wfile.write(f'--frame\r\nContent-Length: 1\r\nX-Frame-Meta: {encoded}\r\n\r\nx\r\n'.encode());self.wfile.flush()
   try:
    for options in ({'demo':True},{'control_eligible':False},{'detections_pair_status':'PAIR_PARTIAL'},{'detection_timestamp_ms':1}):
     frame(['NO-Mask'],**options);time.sleep(.12)
    invalid_sent.set();allow_valid.wait(3)
    for i in range(5):
     frame(['NO-Hardhat','NO-Mask','NO-Safety Vest']);time.sleep(.12)
    time.sleep(1.2)
    for i in range(3):
     frame(['NO-Hardhat','NO-Mask','NO-Safety Vest']);time.sleep(.1)
    time.sleep(2)
   except (BrokenPipeError,ConnectionResetError):pass
 server=http.server.HTTPServer(('127.0.0.1',0),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
 cfg=json.loads((r/'baseline/alerts/config.json').read_text());cfg.update(port=server.server_port,token_file=str(token),player=str(player),asset_directory=str(d),status_file=str(status),lock_file=str(d/'voice.lock'))
 (d/'config.json').write_text(json.dumps(cfg))
 process=subprocess.Popen([str(exe),'--config',str(d/'config.json'),'--run-seconds','5'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
 try:
  check('invalid_partial_demo_and_stale_stream_observed',invalid_sent.wait(3))
  check('invalid_metadata_does_not_play_audio',not log.exists())
  allow_valid.set();output=process.communicate(timeout=8)[0];(r/'reports/voice-stream.log').write_text(output)
  played=log.read_text().splitlines() if log.exists() else []
  check('each_of_three_real_model_labels_triggers_existing_asset',sorted(Path(x).name for x in played)==['no-hardhat.wav','no-mask.wav','no-vest.wav'],played=[Path(x).name for x in played])
  check('cooldown_suppresses_repeated_same_class',len(played)==3)
  check('listener_uses_only_authenticated_readonly_video_GET',requests and all(x['method']=='GET' and x['path']=='/api/video.mjpg' and x['authorization_ok'] for x in requests),requests=requests)
  check('normal_listener_shutdown',process.returncode==0)
 finally:
  if process.poll() is None:process.terminate();process.wait(timeout=3)
  allow_valid.set();server.shutdown();server.server_close()
  (r/'reports/voice-tests.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in records))
  (r/'reports/voice-summary.json').write_text(json.dumps({'status':'PASS' if records and all(x['pass'] for x in records) else 'FAIL','cases':len(records),'source_sha256':hashlib.sha256((r/'baseline/alerts/src/ppe_voice.cc').read_bytes()).hexdigest(),'scope':'Unchanged voice C++ on host; mock metadata and player, no real audio/YOLO/robot execution'},indent=2)+'\n')
print(json.dumps({'cases':len(records),'failed':sum(not x['pass'] for x in records)}))
