#!/usr/bin/env python3
"""Bind v29 production sources, actual ARM64 payload and executed offline evidence."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib,json,re,shutil,subprocess
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT.parent/'robot_layered_v28_1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 reports=ROOT/'reports';reports.mkdir(exist_ok=True)
 logs={
  'navigation-tests.log':('/tmp/v29-rk-navigation-tests.log','PASS 97 compiled profile'),
  'phone-api-publish.log':('/tmp/v29-phone-tests.log','PASS 73 unified HTTP'),
  'dashboard-control.log':('/tmp/v29-rk-dashboard-tests.log','PASS 120 production NAV1/ACK4'),
  'reactive-tests.log':('/tmp/v29-reactive-tests.log','PASS v29 reactive-clearance 26713'),
  'preparation-tests.log':('/tmp/v29-preparation-tests.log','PASS controller/navigation lifecycle'),
  'localization-preflight.log':('/tmp/v29-preflight-tests.log','actual Shared/Gateway localization preflight'),
  'apk-client-tests.log':('/tmp/v29-apk-client-tests.log','PASS 166 production RobotClient'),
  'apk-network-tests.log':('/tmp/v29-apk-network-tests.log','PASS 193 production RobotClient'),
  'export-tests.log':('/tmp/v29-export-tests.log','PASS 15 export checks'),
  'dashboard-arm64-build.log':('/tmp/v29-rk-dashboard-arm64.log','Built build/perception-dashboard.aarch64'),
  'navigation-arm64-build.log':('/tmp/v29-rk-navigation-arm64.log','Leaving directory'),
 }
 for dest,(source,marker) in logs.items():
  source=Path(source);text=source.read_text();assert marker in text,(source,marker)
  assert not re.search(r'(^FAIL |Error [1-9]|Aborted \(core dumped\)|Traceback \(most recent)',text,re.M),source
  shutil.copy2(source,reports/dest)
 suite=json.loads((reports/'navigation-suite-exit.json').read_text());assert suite['exit_code']==0 and not suite['hardware_accessed']
 assert suite['log_sha256']==sha(reports/'navigation-tests.log')
 for name,value in suite['production_source_sha256'].items():assert sha(ROOT/name)==value,name
 assert subprocess.check_output([ROOT/'navigation/build/robot-navigation','--version'],text=True).strip()=='robot-navigation-v29-reactive-clearance'
 assert subprocess.check_output([ROOT/'navigation/build/robot-navigation','--dump-profile'])==(ROOT/'update/profile.json').read_bytes()
 subprocess.run(['sha256sum','-c','SHA256SUMS'],cwd=ROOT/'update',check=True,stdout=subprocess.DEVNULL)
 binaries={}
 for role,relative in [('dashboard','dashboard/build/perception-dashboard.aarch64'),('navigation','navigation/build/robot-navigation.aarch64')]:
  path=ROOT/relative;assert sha(path)==sha(ROOT/'update/payload'/path.name)
  raw=path.read_bytes();assert raw[:6]==b'\x7fELF\x02\x01' and raw[18:20]==b'\xb7\x00'
  dyn=subprocess.check_output(['readelf','-d',path],text=True)
  versions=subprocess.check_output(['readelf','--version-info',path],text=True)
  oldversions=subprocess.check_output(['readelf','--version-info',BASE/relative],text=True)
  numbers=lambda t:sorted(set(re.findall(r'GLIBC_[\d.]+',t)),key=lambda n:tuple(map(int,n.split('_')[1].split('.'))))
  assert numbers(versions)[-1]==numbers(oldversions)[-1]
  binaries[role]={'path':relative,'sha256':sha(path),'bytes':path.stat().st_size,'machine':'AArch64 ELF64 little endian','needed':re.findall(r'Shared library: \[(.*?)\]',dyn),'max_glibc':numbers(versions)[-1],'abi_max_unchanged_from_v28_1':True}
 unchanged=['dashboard/src/main.c','dashboard/src/vision.cc','dashboard/src/camera.cc','dashboard/src/detector.cc','dashboard/src/detector_math.cc','dashboard/src/dashboard.c','dashboard/src/imu_data.c','dashboard/src/imu_data.h','dashboard/src/sensor_data.c','dashboard/src/sensor_data.h','dashboard/phone/controller.cc','dashboard/phone/controller.h','navigation/imu_alignment.cc','navigation/sensor_bridge.cc','navigation/protocol.cc','navigation/vehicle_geometry.h','navigation/run-navigation.sh','navigation/run-dashboard.sh']
 same={}
 for name in unchanged:assert sha(ROOT/name)==sha(BASE/name),name;same[name]=sha(ROOT/name)
 java=list((ROOT/'navigation/android/src').rglob('*.java'));assert java
 for path in java:assert sha(path)==sha(BASE/path.relative_to(ROOT)),path
 installers={}
 for name in ['installer-tests.json','installer-busybox.json']:
  path=ROOT/'update/reports'/name;j=json.loads(path.read_text());assert j['status']=='passed' and j['checks']>=1326 and len(j['cases'])>=279
  assert not j['hardware_accessed'] and not j['arm_programs_executed']
  assert j['dashboard_sha256']==binaries['dashboard']['sha256'] and j['navigation_sha256']==binaries['navigation']['sha256']
  assert j['test_script_sha256']==sha(ROOT/'update/tests/test_installer.py')
  for member,value in j['tested_runtime_files_sha256'].items():assert sha(ROOT/'update'/member)==value,member
  installers[name]={'checks':j['checks'],'cases':len(j['cases']),'shell':j['shell'],'sha256':sha(path)}
 firmware=ROOT.parents[1]/'fpga/robot_control_v29_reactive/softcore/firmware'
 fw={name:sha(firmware/name) for name in ('astar.c','astar.h','nav_protocol.c','nav_protocol.h')}
 monitor_binding={name:sha(ROOT/'update/tools'/name) for name in ('softcore-monitor.c','softcore-monitor.aarch64','verification.json')}
 reports_binding={}
 # Bind only explicitly rerun evidence; copied historical report files are not
 # silently promoted to tests of this version.
 selected=[reports/name for name in logs]+[reports/'navigation-suite-exit.json']
 selected += [ROOT/'navigation/reports'/name for name in ('controller-navigation-lifecycle.log','reactive-legacy.log','reactive-velocity.log','navigation-timeline.csv','vehicle-profile-tests.json')]
 selected += [ROOT/'update/reports'/name for name in ('installer-tests.json','installer-busybox.json')]
 for path in selected:assert path.is_file(),path;reports_binding[str(path.relative_to(ROOT))]=sha(path)
 skip={'build','dist','reports','payload','__pycache__','.git','.local'};sources={}
 for path in sorted(ROOT.rglob('*')):
  rel=path.relative_to(ROOT)
  if not path.is_file() or any(part in skip for part in rel.parts) or path.name=='core' or path.name.startswith('core.') or path.suffix in ('.pyc','.o','.d','.class'):continue
  assert not path.is_symlink(),path;sources[str(rel)]=sha(path)
 prep=(ROOT/'navigation/reports/controller-navigation-lifecycle.log').read_text();checks=int(re.search(r'PASS controller/navigation lifecycle (\d+) checks',prep).group(1))
 result={'result':'PASS','version':'v29-reactive-clearance','created':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds'),'hardware_tested':False,'arm_programs_executed':False,'new_fpga_bitstream_required':True,'apk_update_required':False,'apk_java_sources_unchanged':len(java),'binaries':binaries,'source_sha256':sources,'reports_sha256':reports_binding,'unchanged_camera_model_imu_ui_and_time_sources':same,'firmware_astar_sha256':fw,'passive_monitor_sha256':monitor_binding,'installer_tests':installers,
 'runtime_control':{'required_ack_protocol':4,'ack_timeout_ms':300,'owner_lease_ms':350,'zero_command_period_ms':50,'asynchronous_preparation_timeout_ms':15000,'public_active_semantics':'accepted session = preparing or running including zero SENSOR_WAIT','internal_active_semantics':'current ACK4 and IMU/odometry authorization; transient range invalid permits zero wait only','sonar_transient_wait_s':5,'source_recovery_requires_stopped_new_observations':True,'sonar_invalid_is_never_far_range':True,'old_fpga_fallback':False,'persisted_config_rewritten':False,'high_level_failure_clears_map':False,'NAV1_can_refresh_motor_ack_or_owner_lease':False,'idle_probe_not_fault':True,'latched_localization_rejected_before_claim':True},
 'reactive_policy':{'stop_base_m':.20,'actual_stop_includes':['measured_or_commanded_speed','real_source_age','configured_sonar_latency','configured_reaction','braking_distance'],'forward_clear_floor_m':.35,'forward_clear_extra_m':.04,'clearance_uses_intended_translation_speed':True,'side_translation_clearance_m':.30,'turn_is_continuous':True,'turn_direction_floor_m':.20,'clear_confirmation_distinct_front_observations':2,'scan_max_accumulated_radians':12.566370614359172,'actual_motion_timeout_s':120,'no_progress_s':4,'straight_after_clear':'continue_new_heading','navigation_after_clear_m':.50,'navigation_rejoin':'existing observed-map FPGA A*'},
 'coverage':{'full_navigation_make_test_exit_code':suite['exit_code'],'navcore_cases':40,'passive_motorlink_lifecycle':120,'control_audit':101,'phone_actual_http':73,'apk_actual_client':166,'apk_network':193,'controller_navigation_lifecycle':checks,'reactive_velocity_checks':26713,'reactive_legacy_checks':20152,'wronglog_format_isolation':20},
 'safety_regressions':['two-circle scan across IMU wrap stops explicitly','actual same-side encoder missing-count fault remains latched','turn range gap retains zero owner session and needs post-stop recovery','A* result during invalid range does not cancel session','A* result during SENSOR_RECOVERY preserves FOLLOW_ROUTE/DETOUR','repeated invalid/valid source flap preserves underlying route state','STOP and late plan cannot resurrect task','persistent source five-second timeout requires explicit new Start','latched localization preflight preserves manual arm and does not zero/claim or reset map'],
 'limits':['No physical vehicle or ARM payload execution on hardware','Existing three forward-facing acoustic cones cannot certify all rear/blind space','Brake/reaction/latency remain existing conservative configuration, not fabricated measurements','New base .20 is runtime override of only that user-requested field; disk config is retained','Real encoder/IMU/CPU/lease faults still terminate tasks; auto-disarm is not universally removed','Local mapping is not global closed-loop SLAM']}
 (reports/'rk-verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({'result':'PASS','version':result['version'],'source_files':len(sources),'report_files':len(reports_binding),'binaries':binaries,'new_fpga_bitstream_required':True,'apk_update_required':False},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
