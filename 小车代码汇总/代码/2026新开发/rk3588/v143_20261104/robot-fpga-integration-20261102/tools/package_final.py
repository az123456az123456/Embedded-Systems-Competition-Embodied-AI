from pathlib import Path
import json,hashlib,shutil,tarfile,zipfile
r=Path(__file__).resolve().parents[1];phone=r.parent/'robot_phone_semantic_v135_20261103';out=Path('/home/tan/qiansai/交付/20261103-语义导航-v1.2.35');out.mkdir(parents=True,exist_ok=True)
shutil.copy2(phone/'dist/ACG138-Robot-Control-debug.apk',out/'ACG138-Robot-Control-v1.2.35-semantic.apk')
for name in ('使用说明.txt','技术接线说明.txt'):shutil.copy2(r/name,out/name)
checks=out/'验证记录';checks.mkdir(exist_ok=True)
for name in ('semantic-tests.log','frontier-tests.log','gateway-tests.log','phone-client-tests.log','phone-ui-tests.log','apk-build.log','native-ros-result.json','check-build.console','install.console','installed-hashes.console','final-readonly.console','check-idle.console','patch-generation-receipt.console'):
 p=r/'reports'/name
 if p.exists():shutil.copy2(p,checks/name)
for name in ('apk-manifest.txt','apk-signature.txt'):shutil.copy2(phone/'dist'/name,checks/name)
sources=out/'源码与安装';sources.mkdir(exist_ok=True)
for name in ('semantic-runtime.tar.gz','semantic-build-input.tar.gz','build-on-rk.sh','install-semantic.sh'):shutil.copy2(r/'reports'/name,sources/name)
with tarfile.open(sources/'RK完整源码与测试.tar.gz','w:gz') as t:
 for name in ('gateway','rk_gateway','common','vendor','acg138_frontier','acg138_ros_bridge','acg138_semantic','upstream','deploy','tools','tests'):
  t.add(r/name,arcname=name,filter=lambda x:None if '__pycache__' in x.name or x.name.endswith(('.pyc','.keystore','.jks')) or x.name.endswith('/pair-token.txt') else x)
 for f in (r/'dependencies').glob('*.deb'):t.add(f,arcname='dependencies/'+f.name)
with tarfile.open(sources/'Android源码.tar.gz','w:gz') as t:
 for name in ('src','res','tests','AndroidManifest.xml','build-apk.sh','test-ui-host.sh','test-client-host.sh'):t.add(phone/name,arcname=name)
summary=dict(status='INSTALLED_AND_VERIFIED_NO_VEHICLE_MOTION',FPGA_changed=False,voice_binary_assets_model_preserved=True,
  native_ROS_packages=5,frontier_regressions=341,semantic_lifecycle_and_executor_tests=31,
  gateway_loopback_HTTP_checks=84,phone_client_checks=110,phone_view_checks=390,
  feedback_checks=48,page_indicator_checks=22,android_device_test=False,
  ROS_integration='actual explore_lite + production semantic/explorer nodes, synthetic camera/map/TF/odometry, isolated ROS domain without hardware bridge',
  actual_vehicle_search_test='NOT_RUN',idle_stack_start='UI and phone entered; no motor enable requested; bounded UI exit',
  rollback='/root/model-validation-last-20261102/before-semantic-20261103/rollback.sh',
  APK_upgrade_certificate_matches_v134=True,upstream_commit='326cf8a0b487c34246bb8f3326afbcd69576dc60')
(out/'交付状态.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
(out/'安装与回滚说明.txt').write_text('''RK 本次已经安装，不需要再次运行安装脚本。现有 dh 别名和开机启动将加载新服务，启动时保持空闲。
手机安装同签名APK即可覆盖升级，版本1.2.35 / versionCode48，保留现有设置。
本次验证包括真RK原生编译、隔离合成ROS闭环和实际程序限时空闲启动，未启动真实探索或使能电机。
RK回滚：先停车退出程序，再执行 sh /root/model-validation-last-20261102/before-semantic-20261103/rollback.sh 。这恢复本次之前的Gateway和入口脚本，不改FPGA、模型或语音。
语义ROS源码与install保留在 /acg138-ros2/rootfs/opt/acg138_semantic_20261103 ，回滚后不再由正常入口加载。
源码与安装目录为复算、备份和维护用途，构建脚本针对当前RK Jazzy环境；需要先将semantic-build-input.tar.gz放到 /root/model-validation-last-20261102/ 同名路径。安装脚本拒绝覆盖本次回滚备份。
Android源码不含签名私钥，重建时仍需使用用户现有项目签名密钥。
''')
entries=[]
for p in sorted(out.rglob('*')):
 if p.is_file() and p.name!='SHA256SUMS':entries.append((str(p.relative_to(out)),hashlib.sha256(p.read_bytes()).hexdigest()))
(out/'SHA256SUMS').write_text(''.join(sha+'  '+name+'\n' for name,sha in entries))
zip_path=out.parent/'20261103-语义导航-v1.2.35-完整交付.zip'
with zipfile.ZipFile(zip_path,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in sorted(out.rglob('*')):
  if p.is_file():z.write(p,str(Path(out.name)/p.relative_to(out)))
with zipfile.ZipFile(zip_path) as z:assert z.testzip() is None
print(json.dumps(dict(output=str(out),zip=str(zip_path),bytes=zip_path.stat().st_size,sha256=hashlib.sha256(zip_path.read_bytes()).hexdigest()),ensure_ascii=False))
