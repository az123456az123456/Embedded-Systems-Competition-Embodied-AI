#!/usr/bin/env python3
"""Verify this build's sources, constraints, pin attributes and declared STA."""
import argparse
from audit_ram_init import verify as verify_ram
from verify_gray_cdc import verify as verify_gray
import datetime
import hashlib
import html
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

PROJECT_NAME = 'robot_control_v26_sonar_ma'
BASELINE_NAME = 'robot_control_v25'
BASELINE_RELEASE_SHA256 = '06d6394a8214c05dd048712b37684c6179c1b3ed211aea60f875f25f74c2d2d5'
BASELINE_BITSTREAM_SHA256 = '30fc6ed2ba1eb5754d16ac780e6fde2cae1b9f1a5c5d3f8a0125d2f2073516f3'
ALLOWED_CHANGED_SOURCES = {
    'sensors/ultrasonic_sensor.v',
    'sensors/ultrasonic_driver.v',
    'sensors/ultrasonic_3ch.v',
}
ALLOWED_NEW_SOURCES = {'sensors/a02_moving_average.v'}
PRESERVED_AUXILIARY = {
    'camera/ov5640_af.hex',
    'softcore/vendor/upstream.json',
    'softcore/firmware/firmware.hex',
}
LOGIC_BUDGET_MAXIMUM = 24000
BSRAM_BUDGET_MAXIMUM = 70
RESOURCE_BUDGET_RATIONALE = (
    'The four timestamped moving-average contributors per sonar channel add state and arithmetic '
    'relative to the v25 rolling-minimum filter. The v26 project Logic budget is explicitly 24000 '
    '(about 40% of the 59904-device capacity), replacing the inherited v25 20000 guard. '
    'The physical device bounds, BSRAM maximum of 70, and zero-violation STA gates remain required.'
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rtl_source_tree(project):
    return {str(p.relative_to(project)): digest(p)
            for folder in ('camera', 'imu', 'rtl', 'sensors')
            for p in sorted((project/folder).rglob('*'))
            if p.is_file() and p.suffix in {'.v', '.sv', '.vh'}}


def rows(markup):
    return [[html.unescape(re.sub(r"<[^>]+>", "", cell)).strip()
             for cell in re.findall(r"<t[dh][^>]*>(.*?)</t[dh]>", row, re.S)]
            for row in re.findall(r"<tr[^>]*>(.*?)</tr>", markup, re.S)]


def section(markup, start, end):
    return markup.split('name="' + start + '"', 1)[1].split('name="' + end + '"', 1)[0]


def pin_rows(path):
    table = path.read_text().split('name="Pinout_by_Port_Name"', 1)[1].split('</table>', 1)[0]
    return {row[0]: row for row in rows(table)[1:]}


def verify_baseline(project):
    """Bind the manifest to the original v25 release and recheck frozen files."""
    baseline = json.loads((project/'tools/baseline_v25_manifest.json').read_text())
    original = (project/baseline['baseline_directory']).resolve()
    release_path = original/baseline['release_verification']
    assert baseline['project'] == BASELINE_NAME and original.name == BASELINE_NAME
    assert digest(release_path) == baseline['release_verification_sha256'] == BASELINE_RELEASE_SHA256
    release = json.loads(release_path.read_text())
    for key in ('project', 'device', 'tool_version', 'bitstream', 'bitstream_sha256',
                'hardware_tested', 'synthesis_inputs', 'auxiliary_inputs',
                'firmware_sources', 'resources'):
        assert baseline[key] == release[key], ('baseline/report mismatch', key)
    for group in ('synthesis_inputs', 'auxiliary_inputs', 'firmware_sources'):
        for path, expected in baseline[group].items():
            assert digest(original/path) == expected, ('frozen v25 changed', path)
    assert baseline['firmware_artifacts'] == {
        'softcore/firmware/'+name: sha for name, sha in release['softcore']['sha256'].items()}
    for path, expected in baseline['firmware_artifacts'].items():
        assert digest(original/path) == expected, ('frozen v25 firmware artifact changed', path)
    assert rtl_source_tree(original) == baseline['rtl_source_tree'], 'frozen v25 RTL source tree changed'
    assert digest(original/baseline['bitstream']) == baseline['bitstream_sha256'] == BASELINE_BITSTREAM_SHA256
    return baseline


def verify_sources(project, baseline, build=None):
    """Reject any removed source, unknown addition, or non-sonar source edit."""
    inputs = re.findall(r'^add_file (\S+)$', (project/'build.tcl').read_text(), re.M)
    gui = [n.attrib['path'] for n in ET.parse(project/(PROJECT_NAME+'.gprj')).findall('.//File')]
    assert len(set(inputs)) == len(inputs) and len(set(gui)) == len(gui)
    assert set(gui) == set(inputs), 'GUI and command-line source inventories differ'
    hashes = {f: digest(project/f) for f in inputs}
    old = baseline['synthesis_inputs']
    removed = set(old)-set(hashes)
    added = set(hashes)-set(old)
    changed = {f for f in set(old)&set(hashes) if hashes[f] != old[f]}
    assert not removed, ('removed synthesis inputs', sorted(removed))
    assert added <= ALLOWED_NEW_SOURCES, ('unapproved added sources', sorted(added))
    assert changed <= ALLOWED_CHANGED_SOURCES, ('unapproved source changes', sorted(changed))
    assert 'sensors/ultrasonic_sensor.v' in changed, 'v26 must contain the sonar moving-average change'
    # An optional source must be in both project inventories when it exists.
    for path in ALLOWED_NEW_SOURCES:
        assert (project/path).is_file() == (path in added), ('unlisted new sonar module', path)
    tree = rtl_source_tree(project)
    original_tree = baseline['rtl_source_tree']
    assert set(tree) == set(original_tree) | added, 'unexpected added or removed RTL source file'
    tree_changed = {f for f in original_tree if tree[f] != original_tree[f]}
    assert tree_changed == changed, ('unapproved non-synthesis RTL changes', sorted(tree_changed-changed))
    auxiliary = {f: digest(project/f) for f in baseline['auxiliary_inputs']}
    for f in PRESERVED_AUXILIARY:
        assert auxiliary[f] == baseline['auxiliary_inputs'][f], ('preserved auxiliary changed', f)
    firmware = {str(p.relative_to(project)): digest(p)
                for p in (project/'softcore/firmware').iterdir()
                if p.suffix in {'.c', '.h', '.S', '.ld'}}
    assert firmware == baseline['firmware_sources'], 'navigation firmware differs from frozen v25'
    for path, expected in baseline['firmware_artifacts'].items():
        assert digest(project/path) == expected, ('firmware artifact differs from frozen v25', path)
    if build is not None:
        assert hashes == {f: digest(build/f) for f in inputs}, 'built sources differ from reviewed sources'
        assert auxiliary == {f: digest(build/f) for f in auxiliary}, 'built auxiliary inputs differ'
        assert firmware == {f: digest(build/f) for f in firmware}, 'built firmware sources differ'
        for path, expected in baseline['firmware_artifacts'].items():
            assert digest(build/path) == expected, ('built firmware artifact differs from frozen v25', path)
        assert rtl_source_tree(build) == tree, 'built RTL tree differs from reviewed sources'
    return hashes, auxiliary, firmware, changed, added


def sonar_contract():
    return {
        'filter': 'rounded_arithmetic_mean_of_latest_4_accepted_valid_observations',
        'default_window': 4, 'supported_windows': [1, 2, 4],
        'warmup_samples': [1, 2, 3, 4],
        'rounding': 'nearest_integer_half_up',
        'window_maximum_contributor_span_ms': 300,
        'maximum_span_inclusive': True,
        'age_semantics': 'age_of_oldest_contributing_observation',
        'contributor_timestamp_capture': 'CRC_acceptance_clock_before_mean_pipeline',
        'pipeline_time_included_in_age': True,
        'normal_publication_latency_clocks': 4,
        'fast_near_publication_latency_clocks': 0,
        'timing_exceptions_added': False,
        'output_is_filtered_estimate': True,
        'output_is_actual_observation': False,
        'fast_near': {'raw_closer_by_at_least_mm': 50, 'raw_at_or_below_mm': 350,
                      'action': 'immediate_raw_output_and_window_reset'},
        'far_step': {'raw_farther_by_more_than_mm': 200, 'valid_confirmations_required': 2,
                     'first_candidate': 'starts_fresh_candidate_window_while_old_output_is_held',
                     'expired_first_candidate': 'evicted_from_mean_without_starving_confirmation'},
        'checksum_numeric_4501_to_65534': 'clamp4500',
        'zero_blind_and_ffff': 'invalid',
        'protocol': 'SNS1 version1 reserved byte63 zero',
        'hardware_tested': False,
    }


def resource_usage(used, total, baseline_resource):
    return {'used': used, 'total': total, 'free': total-used,
            'used_percent': round(100*used/total, 3),
            'baseline_used': baseline_resource['used'],
            'baseline_free': baseline_resource['total']-baseline_resource['used'],
            'delta_from_v25': used-baseline_resource['used']}


def verify_resource_limits(resources):
    for key, usage in resources.items():
        assert 0 <= usage['used'] <= usage['total'], ('physical device resource bound', key, usage)
    assert resources['Logic']['used'] <= LOGIC_BUDGET_MAXIMUM, ('v26 Logic project budget', resources['Logic'])
    assert resources['BSRAM']['used'] <= BSRAM_BUDGET_MAXIMUM, ('BSRAM project budget', resources['BSRAM'])


def synthesis_resource_snapshot(build, baseline):
    """Record preliminary synthesis usage without claiming completed PNR or STA."""
    path = build/'impl/gwsynthesis'/(PROJECT_NAME+'_syn.rpt.html')
    if not path.is_file():
        return None
    resources = {}
    for row in rows(path.read_text()):
        if len(row) < 2 or row[0] not in ('Logic', 'Register', 'BSRAM'):
            continue
        match = re.fullmatch(r'(\d+)(?:\([^)]*\))? / (\d+)', row[1])
        if match:
            resources[row[0]] = resource_usage(int(match[1]), int(match[2]), baseline['resources'][row[0]])
    assert set(resources) == {'Logic', 'Register', 'BSRAM'}, 'missing synthesis resource counts'
    return {'scope': 'Preliminary synthesis resources only; PNR resources and STA remain separate release gates',
            'report_sha256': digest(path), 'resources': resources}


def prepare(project, baseline, source_result, build=None):
    hashes, auxiliary, firmware, changed, added = source_result
    report = {
        'result': 'PASS', 'scope': 'Source and audit preparation only; fresh FPGA build is separately audited',
        'project': PROJECT_NAME, 'hardware_tested': False,
        'baseline_project': BASELINE_NAME,
        'baseline_release_report_sha256': baseline['release_verification_sha256'],
        'baseline_original_bitstream_sha256': baseline['bitstream_sha256'],
        'baseline_original_files_verified': True,
        'allowed_changed_sources': sorted(ALLOWED_CHANGED_SOURCES),
        'allowed_new_sources': sorted(ALLOWED_NEW_SOURCES),
        'changed_sources': sorted(changed), 'new_sources': sorted(added),
        'synthesis_inputs': hashes, 'auxiliary_inputs': auxiliary, 'firmware_sources': firmware,
        'rtl_source_tree': rtl_source_tree(project),
        'firmware_artifacts': baseline['firmware_artifacts'],
        'pins_and_constraints_unchanged': True,
        'release_gates': {'pins': 75, 'setup_violated_endpoints': 0,
                          'hold_violated_endpoints': 0, 'logic_used_maximum': LOGIC_BUDGET_MAXIMUM,
                          'resource_device_capacity_bounds_required': True,
                          'bsram_used_maximum': BSRAM_BUDGET_MAXIMUM, 'cpu_ram_init_words_compared': 16384,
                          'registered_gray_cdc_paths': 88},
        'resource_budget_rationale': RESOURCE_BUDGET_RATIONALE,
        'baseline_resources': {key: resource_usage(value['used'], value['total'], value)
                               for key, value in baseline['resources'].items()},
        'sonar_v26': sonar_contract(),
    }
    if build is not None:
        snapshot = synthesis_resource_snapshot(build, baseline)
        if snapshot is not None:
            report['synthesis_resource_snapshot'] = snapshot
    out = project/'reports'
    out.mkdir(exist_ok=True)
    (out/'audit-preparation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir',type=Path)
    parser.add_argument('--prepare-only',action='store_true',help='Check source provenance without requiring completed FPGA outputs')
    args=parser.parse_args()
    if not args.prepare_only and args.build_dir is None:
        parser.error('--build-dir is required unless --prepare-only is selected')
    project=Path(__file__).resolve().parents[1]
    baseline=verify_baseline(project)
    build=None if args.build_dir is None else args.build_dir.resolve()
    source_result=verify_sources(project,baseline,build)
    preparation=prepare(project,baseline,source_result,build)
    if args.prepare_only:
        print(json.dumps({k:preparation[k] for k in ('result','scope','changed_sources','new_sources')},indent=2))
        return
    hashes,auxiliary,firmware,changed,added=source_result
    name=PROJECT_NAME;pnr=build/'impl/pnr'
    log=(build/'gowin.log').read_text()
    assert 'Bitstream generation completed' in log and '[100%] Power analysis completed' in log
    assert not re.search(r'^ERROR',log,re.M)
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
        resources[key]=resource_usage(int(m[1]),int(m[2]),baseline['resources'][key])
        assert resources[key]['total']==baseline['resources'][key]['total'], ('device capacity changed', key)
    verify_resource_limits(resources)
    out=project/'reports';out.mkdir(exist_ok=True)
    verify_ram(project,build)
    gray=verify_gray(project,build)
    target=project/'impl/pnr';target.mkdir(parents=True,exist_ok=True)
    fs=pnr/(name+'.fs');staged=target/(fs.name+'.tmp')
    shutil.copy2(fs,staged);staged.replace(target/fs.name)
    for suffix in ('.pin.html','.power.html','.rpt.html','.rpt.txt','.log','.timing_paths','.tr.html','_tr_cata.html','_tr_content.html'):
        shutil.copy2(pnr/(name+suffix),out/(name+suffix))
    shutil.copy2(build/'gowin.log',out/'gowin.log')
    warnings=[l for l in log.splitlines()if l.startswith('WARN')]
    report={'result':'PASS','date':datetime.date.today().isoformat(),'project':name,'device':'GW5AT-LV60PG484AC1/I0','tool_version':'V1.9.12.03',
        'bitstream':'impl/pnr/'+fs.name,'bitstream_sha256':digest(fs),'hardware_tested':False,
        'firmware_default_left_mask':10,'invert_mask':15,'phone_left_mask_current_wiring':None,'phone_wiring_policy':'Preserve current working phone left/invert settings; wheel/encoder correspondence requires calibration','wheel_positions_require_encoder_calibration':True,
        'camera_trial':{'target_fps':39.9575491,'hts':1880,'vts':1120,'pixel_clock_hz':84134615.3846,'module_hts_minimum_not_documented':True},'synthesis_inputs':hashes,'auxiliary_inputs':auxiliary,
        'rtl_source_tree':rtl_source_tree(project),
        'baseline_fs_sha256':baseline['bitstream_sha256'],'baseline_release_report_sha256':baseline['release_verification_sha256'],
        'baseline_original_files_verified':True,'changed_sources':sorted(changed),'new_sources':sorted(added),
        'unchanged_sources':{f:h for f,h in hashes.items()if f in baseline['synthesis_inputs'] and f not in changed},
        'pins_verified':len(pins),'timing':timing,'resources':resources,
        'resource_budgets':{'Logic':LOGIC_BUDGET_MAXIMUM,'BSRAM':BSRAM_BUDGET_MAXIMUM},
        'resource_budget_rationale':RESOURCE_BUDGET_RATIONALE,
        'motor_camera_imu_encoder_navigation_firmware_preserved':True, 'mot3_arc_inner_duty_min_ratio':0.5,'navigation_firmware':'NAV2 map upload and A* service',
        'protocols_preserved':['MOT1','ACK1','MOT2','ACK2','SNS1','NAV1','NAV2','IMU'],'protocol_added':None,'sonar_v26':sonar_contract(),
        'softcore':json.loads((project/'softcore/firmware/verification.json').read_text()),
        'firmware_sources':firmware,'firmware_artifacts':baseline['firmware_artifacts'],
        'encoder_feedback':{'type':'bounded relative wheel-rate integrator','straight_only':True,'min_duty_permille':200,
            'sample_ms':50,'averaging_ms':200,'settle_samples':4,'trim_max_permille':100,'relative_trim_max':0.25,
            'deadband':'max(2 counts per200ms, mean/64)','stale_ms':300,'abs_encoder_counts':True,
            'missing_stalled_illegal_feedback':'clear trims; original PWM feed-forward only',
            'speed_scale_in_rpm_or_mps':False,'automatic_motion_on_boot':False},
        'warnings':warnings,'gray_cdc_paths_verified':gray['paths_verified'],
        'limitations':['Hardware is not tested. MOT2 is continuous camera-bearing visual servo, not an exact open-loop turn. RK owns target selection, mapping and safety.','Requires equal encoder CPR and wheel circumference.','Encoder synchronization cannot eliminate slip, unequal tyre diameters or IMU heading drift.','Physical gains/straightness require on-car testing.']}
    (out/'release_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    summary={'result':'PASS','hardware_tested':False,'bitstream':report['bitstream'],
        'bitstream_sha256':report['bitstream_sha256'],'setup_violations':timing['setup_violated_endpoints'],
        'hold_violations':timing['hold_violated_endpoints'],'timing':timing,'resources':resources,
        'resource_budgets':report['resource_budgets'],
        'pins_verified':len(pins),'pins_unchanged':hashes['robot_fmc_remote.cst']==baseline['synthesis_inputs']['robot_fmc_remote.cst'],
        'constraints_unchanged':hashes['robot_fmc_remote.sdc']==baseline['synthesis_inputs']['robot_fmc_remote.sdc'],
        'changed_sources':sorted(changed),'new_sources':sorted(added),'details':'release_verification.json'}
    (out/'release-verification.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    (out/'source_bitstream_SHA256SUMS.txt').write_text(''.join(h+'  '+f+'\n' for f,h in {**hashes,**auxiliary,'impl/pnr/'+fs.name:digest(fs)}.items()))
    print(json.dumps({k:report[k]for k in ['bitstream_sha256','pins_verified','resources','timing']},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
