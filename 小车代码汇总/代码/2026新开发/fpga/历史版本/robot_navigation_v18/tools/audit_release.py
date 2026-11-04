#!/usr/bin/env python3
"""Verify this build's sources, constraints, pin attributes and declared STA."""
import argparse
from audit_ram_init import verify as verify_ram
import datetime
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(markup):
    return [[html.unescape(re.sub(r"<[^>]+>", "", cell)).strip()
             for cell in re.findall(r"<t[dh][^>]*>(.*?)</t[dh]>", row, re.S)]
            for row in re.findall(r"<tr[^>]*>(.*?)</tr>", markup, re.S)]


def section(markup, start, end):
    return markup.split('name="' + start + '"', 1)[1].split('name="' + end + '"', 1)[0]


def pin_rows(path):
    table = path.read_text().split('name="Pinout_by_Port_Name"', 1)[1].split('</table>', 1)[0]
    return {row[0]: row for row in rows(table)[1:]}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir',required=True,type=Path)
    args=parser.parse_args()
    project=Path(__file__).resolve().parents[1];build=args.build_dir.resolve()
    name='robot_navigation_v18';pnr=build/'impl/pnr'
    log=(build/'gowin.log').read_text()
    assert 'Bitstream generation completed' in log and '[100%] Power analysis completed' in log
    assert not re.search(r'^ERROR',log,re.M)
    baseline=json.loads((project/'tools/baseline_v17_manifest.json').read_text())
    inputs=re.findall(r'^add_file (\S+)$',(project/'build.tcl').read_text(),re.M)
    gui=[n.attrib['path'] for n in ET.parse(project/(name+'.gprj')).findall('.//File')]
    assert len(set(inputs))==len(inputs) and set(gui)==set(inputs)
    hashes={f:digest(project/f) for f in inputs}
    assert hashes=={f:digest(build/f) for f in inputs}
    changed={f for f,h in baseline['synthesis_inputs'].items() if hashes.get(f)!=h}
    assert changed=={'sensors/ultrasonic_sensor.v','sensors/a02_uart_rx.v','sensors/robot_sensor_capture.v'},changed
    assert set(hashes)-set(baseline['synthesis_inputs'])==set()
    auxiliary={f:digest(project/f) for f in ['build.tcl','project_settings.tcl','camera/ov5640_af.hex','softcore/firmware/firmware.hex','softcore/vendor/upstream.json']}
    assert auxiliary=={f:digest(build/f) for f in auxiliary}
    for f in ['camera/ov5640_af.hex','softcore/vendor/upstream.json']:
        assert auxiliary[f]==baseline['auxiliary_inputs'][f]
    motor=(project/'rtl/motor_remote_control.v').read_text()
    assert "DEFAULT_LEFT_MASK=4'b1010, DEFAULT_INVERT_MASK=4'b1111" in motor
    assert 'motor_remote_control #(.ENCODER_SYNC_ENABLE(1)) motors' in (project/'rtl/robot_fmc_remote_top.v').read_text()
    cst=(project/'robot_fmc_remote.cst').read_text()
    locations=dict(re.findall(r'IO_LOC "([^"]+)" (\w+);',cst))
    pins=pin_rows(pnr/(name+'.pin.html'))
    assert len(pins)==75 and pins.keys()==locations.keys()
    cols={'IO_TYPE':7,'DRIVE':8,'PULL_MODE':9,'BANK_VCCIO':17}
    for port,attrs in re.findall(r'IO_PORT "([^"]+)" ([^;]+);',cst):
        assert pins[port][2].split('/')[0]==locations[port] and pins[port][3]=='Y'
        for key,value in re.findall(r'(\w+)=([\w.]+)',attrs):assert pins[port][cols[key]]==value,(port,key)
    markup=(pnr/(name+'_tr_content.html')).read_text()
    summary=dict(row[:2]for row in rows(section(markup,'STA_Tool_Run_Summary','Clock_Report'))if len(row)==2)
    timing={'setup_violated_endpoints':int(summary['Numbers of Setup Violated Endpoints']),
            'hold_violated_endpoints':int(summary['Numbers of Hold Violated Endpoints']),
            'scope':'Declared internal STA; external camera/RGMII IO not board-signed-off','frequency_table':[]}
    assert timing['setup_violated_endpoints']==timing['hold_violated_endpoints']==0,timing
    for row in rows(section(markup,'Max_Frequency_Report','Total_Negative_Slack_Report'))[1:]:
        if len(row)==6 and row[0].isdigit():
            constraint,actual=[float(v.split('(')[0]) for v in row[2:4]]
            assert actual>=constraint,row
            timing['frequency_table'].append({'clock':row[1],'constraint_mhz':constraint,'fmax_mhz':actual})
    for kind,end in zip(('Setup','Hold','Recovery','Removal'),('Hold_Slack_Table','Recovery_Slack_Table','Removal_Slack_Table','MIN_PULSE_WIDTH_TABLE')):
        slacks=[float(row[1])for row in rows(section(markup,kind+'_Slack_Table',end))if len(row)>=2 and row[0].isdigit()]
        assert slacks and min(slacks)>=0,(kind,slacks[:1])
        timing[kind.lower()+'_minimum_slack_ns']=min(slacks)
    resources={}
    rpt=(pnr/(name+'.rpt.txt')).read_text()
    for key in ['Logic','Register','BSRAM']:
        m=re.search(r'^  '+key+r' +\| (\d+)/(\d+)',rpt,re.M);assert m,key
        resources[key]={'used':int(m[1]),'total':int(m[2]),'baseline_used':baseline['resources'][key]['used']}
    assert resources['Logic']['used']<20000 and resources['BSRAM']['used']<=70
    out=project/'reports';out.mkdir(exist_ok=True)
    verify_ram(project,build)
    target=project/'impl/pnr';target.mkdir(parents=True,exist_ok=True)
    fs=pnr/(name+'.fs');staged=target/(fs.name+'.tmp')
    shutil.copy2(fs,staged);staged.replace(target/fs.name)
    for suffix in ('.pin.html','.power.html','.rpt.html','.rpt.txt','.log','.timing_paths','.tr.html','_tr_cata.html','_tr_content.html'):
        shutil.copy2(pnr/(name+suffix),out/(name+suffix))
    shutil.copy2(build/'gowin.log',out/'gowin.log')
    warnings=[l for l in log.splitlines()if l.startswith('WARN')]
    report={'result':'PASS','date':datetime.date.today().isoformat(),'project':name,'device':'GW5AT-LV60PG484AC1/I0','tool_version':'V1.9.12.03',
        'bitstream':'impl/pnr/'+fs.name,'bitstream_sha256':digest(fs),'hardware_tested':False,
        'left_mask':10,'invert_mask':15,'wheel_positions':{'M1':'right_front','M2':'left_rear','M3':'right_rear','M4':'left_front'},
        'wheel_directions_user_confirmed_in_standalone_v2':True,'synthesis_inputs':hashes,'auxiliary_inputs':auxiliary,
        'baseline_fs_sha256':baseline['bitstream_sha256'],'changed_sources':sorted(changed),
        'unchanged_sources':{f:h for f,h in hashes.items()if f in baseline['synthesis_inputs'] and f not in changed},
        'pins_verified':len(pins),'timing':timing,'resources':resources,'camera_imu_motor_navigation_sources_preserved':True,'navigation_firmware':'NAV2 map upload and A* service',
        'protocols_preserved':['MOT1','ACK1','SNS1','NAV1','NAV2','IMU'],'protocol_added':'SNS1 window_ms=0 sonar event, delta=0; periodic50ms unchanged',
        'softcore':json.loads((project/'softcore/firmware/verification.json').read_text()),
        'firmware_sources':{str(p.relative_to(project)):digest(p) for p in (project/'softcore/firmware').iterdir() if p.suffix in {'.c','.h','.S','.ld'}},
        'encoder_feedback':{'type':'bounded relative wheel-rate integrator','straight_only':True,'min_duty_permille':200,
            'sample_ms':50,'averaging_ms':200,'settle_samples':4,'trim_max_permille':100,'relative_trim_max':0.25,
            'deadband':'max(2 counts per200ms, mean/64)','stale_ms':300,'abs_encoder_counts':True,
            'missing_stalled_illegal_feedback':'clear trims; original PWM feed-forward only',
            'speed_scale_in_rpm_or_mps':False,'automatic_motion_on_boot':False},
        'warnings':warnings,
        'limitations':['Hardware is not tested. MOT2 is continuous camera-bearing visual servo, not an exact open-loop turn. RK owns target selection, mapping and safety.','Requires equal encoder CPR and wheel circumference.','Encoder synchronization cannot eliminate slip, unequal tyre diameters or IMU heading drift.','Physical gains/straightness require on-car testing.']}
    (out/'release_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    summary={'result':'PASS','hardware_tested':False,'bitstream':report['bitstream'],
        'bitstream_sha256':report['bitstream_sha256'],'setup_violations':timing['setup_violated_endpoints'],
        'hold_violations':timing['hold_violated_endpoints'],'timing':timing,'resources':resources,
        'pins_verified':len(pins),'pins_unchanged':hashes['robot_fmc_remote.cst']==baseline['synthesis_inputs']['robot_fmc_remote.cst'],
        'constraints_unchanged':hashes['robot_fmc_remote.sdc']==baseline['synthesis_inputs']['robot_fmc_remote.sdc'],
        'changed_sources':sorted(changed),'details':'release_verification.json'}
    (out/'release-verification.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    (out/'source_bitstream_SHA256SUMS.txt').write_text(''.join(h+'  '+f+'\n' for f,h in {**hashes,**auxiliary,'impl/pnr/'+fs.name:digest(fs)}.items()))
    print(json.dumps({k:report[k]for k in ['bitstream_sha256','pins_verified','resources','timing']},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
