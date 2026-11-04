#!/usr/bin/env python3
"""Static/launch-composition tests with narrow adapters. Not a ROS runtime test."""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'ros2_ws/src';B=SRC/'acg138_bringup';checks=0

def need(condition, text):
    global checks
    checks+=1
    if not condition:
        raise AssertionError(text)
class Action:
    def __init__(self,*args,**kwargs): self.args,self.kw=args,kwargs
class Description:
    def __init__(self,actions): self.entities=actions
for name in ['launch','launch.actions','launch.events','launch.event_handlers','launch.launch_description_sources','launch_ros','launch_ros.actions','ament_index_python','ament_index_python.packages']:
    sys.modules[name]=types.ModuleType(name)
sys.modules['launch'].LaunchDescription=Description
for name in ['DeclareLaunchArgument','OpaqueFunction','EmitEvent','RegisterEventHandler','IncludeLaunchDescription']:
    setattr(sys.modules['launch.actions'],name,Action)
sys.modules['launch.events'].Shutdown=Action
sys.modules['launch.event_handlers'].OnProcessExit=Action
sys.modules['launch.launch_description_sources'].PythonLaunchDescriptionSource=Action
sys.modules['launch_ros.actions'].Node=Action
sys.modules['ament_index_python.packages'].get_package_share_directory=lambda p:str(SRC/p)

def load(path):
    name='fixture_'+path.stem.replace('.','_');spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
for p in SRC.rglob('*.py'):
    if 'vendor' not in p.parts:
        ast.parse(p.read_text(),str(p));checks+=1
packages={}
for p in SRC.glob('*/package.xml'):
    node=ET.parse(p).getroot();packages[node.findtext('name')]=p.parent
need(len(packages)==4,'four local ROS2 packages')
for p in SRC.rglob('*.yaml'):
    if 'vendor' not in p.parts:
        need(isinstance(yaml.safe_load(p.read_text()),dict),'valid YAML '+str(p))
platform=load(B/'launch/platform.launch.py')
ctx=types.SimpleNamespace(launch_configurations={'serial_port':'/dev/rplidar_c1','interface':'eth0','with_base':'true','use_ekf':'false','laser_x':'0.0','laser_y':'0.0','laser_z':'.175','laser_yaw':'0.0'})
a=platform._start(ctx)
need(a[0].kw['executable']=='c1_node','C1 executable matches actual CMake')
need(a[0].kw['parameters'][-1]['baudrate']==460800,'never wrong baud in launch')
need('on_exit' in a[0].kw,'driver failure shuts down whole session for supervisor')
laser=a[1].kw['arguments'];need(laser[laser.index('--z')+1]=='.175','measured laser height')
base=next(x for x in a if x.kw.get('executable')=='base_telemetry_node')
need(base.kw['parameters'][-1]['publish_direct_odom'] is True,'one direct wheel TF publisher')
need(not any(x.kw.get('executable')=='ekf_node' for x in a),'no implicit optional EKF')
ctx.launch_configurations['use_ekf']='true';a=platform._start(ctx)
base=next(x for x in a if x.kw.get('executable')=='base_telemetry_node')
need(base.kw['parameters'][-1]['publish_direct_odom'] is False,'disable direct TF with EKF')
need(sum(x.kw.get('executable')=='odom_health_gate' for x in a)==1,'one health-gated EKF TF publisher')
ekf=yaml.safe_load((B/'config/ekf.yaml').read_text())['ekf_filter_node']['ros__parameters']
need(ekf['publish_tf'] is False,'EKF cannot duplicate TF')
need(sum(ekf['imu0_config'])==1 and ekf['imu0_config'][11],'only gyro yaw-rate fused, no false absolute heading')
ctx.launch_configurations['with_base']='false';a=platform._start(ctx)
need(len(a)==2,'scan-only has no fake odom/base or raw socket process')
need(all('motor' not in x.kw.get('executable','') for x in a),'no motor command producer')
mapping=load(B/'launch/mapping.launch.py');ld=mapping.generate_launch_description()
need(len(ld.entities)>=7,'mapping launch contains lifecycle include and bridge')
ctx.launch_configurations.update(enable_fpga_bridge='true',dry_run='true',exclusive_nav2_owner='false',enable_plan='false')
node=mapping._bridge(ctx)[0];need(node.kw['parameters'][-1]['dry_run'] is True,'no live map upload by default')
need(node.kw['parameters'][-1]['exclusive_nav2_owner'] is False,'cannot seize legacy planning session')
need(node.kw['parameters'][-1]['enable_plan'] is False,'mapping does not automatically request a path')
need(node.kw['parameters'][-1]['host_proc_root']==os.environ.get('ACG138_HOST_PROC_ROOT','/proc'),'process visibility passed from deployment')
ctx.launch_configurations['enable_plan']='true'
need(mapping._bridge(ctx)[0].kw['parameters'][-1]['enable_plan'] is True,'explicit optional planner passed through')
slam=yaml.safe_load((B/'config/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
need(slam['use_sim_time'] is False and slam['base_frame']=='base_link','real TF system-clock mapping')
need(slam['resolution']==.05 and slam['map_update_interval']==1.,'configured map resolution and update')
need(slam['do_loop_closing'] is True,'actual loop-closing SLAM config')
for name in ['localization.launch.py','simulation.launch.py','scan.launch.py']:
    need(bool(load(B/'launch'/name).generate_launch_description().entities),'launch loads '+name)
inputs={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for base in [B,SRC/'acg138_base_telemetry'] for p in sorted(base.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
inputs[str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
report={'result':'PASS','checks':checks,'ROS2_runtime_executed':False,'scope':'XML/YAML/Python parsing and launch composition adapters only; not ROS node compilation','source_sha256':inputs}
(ROOT/'reports/bringup-test.json').write_text(json.dumps(report,indent=2)+'\n')
print('PASS bringup composition / manifest / TF rules',checks,'checks (not ROS runtime)')
