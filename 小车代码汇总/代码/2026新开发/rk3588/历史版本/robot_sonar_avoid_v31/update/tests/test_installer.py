#!/usr/bin/env python3
"""Actual ARM payload bytes; no executable payload, hardware, or network access."""
from pathlib import Path
import argparse, hashlib, importlib.util, json, os, shutil, subprocess, tempfile
SRC=Path(__file__).resolve().parents[1];WS=SRC.parents[2]
DNAME='perception-dashboard.aarch64';NNAME='robot-navigation.aarch64'
DASHES=[WS/'rk3588/robot_control_v21/dashboard/build'/DNAME,WS/'rk3588/robot_control_v21_1/dashboard/build'/DNAME,WS/'rk3588/robot_sonar_v24/dashboard/build'/DNAME,WS/'rk3588/robot_navigation_v25/dashboard/build'/DNAME,WS/'rk3588/robot_phone_v26/dashboard/build'/DNAME,WS/'rk3588/robot_phone_v26_1/dashboard/build'/DNAME,WS/'rk3588/robot_phone_v26_2/dashboard/build'/DNAME,WS/'rk3588/robot_phone_v26_3/dashboard/build'/DNAME,WS/'rk3588/robot_phone_v26_4/dashboard/build'/DNAME,WS/'rk3588/robot_straight_v27/dashboard/build'/DNAME,WS/'rk3588/robot_straight_v27_1/dashboard/build'/DNAME,WS/'rk3588/robot_straight_v27_2/dashboard/build'/DNAME]
NAVS=[WS/'rk3588/robot_navigation_v23/build'/NNAME,WS/'rk3588/robot_navigation_v23_2/build'/NNAME,WS/'rk3588/robot_sonar_v24/navigation/build'/NNAME,WS/'rk3588/robot_navigation_v25/navigation/build'/NNAME,WS/'rk3588/robot_phone_v26/navigation/build'/NNAME,WS/'rk3588/robot_phone_v26_1/navigation/build'/NNAME,WS/'rk3588/robot_phone_v26_2/navigation/build'/NNAME,WS/'rk3588/robot_phone_v26_3/navigation/build'/NNAME,WS/'rk3588/robot_phone_v26_4/navigation/build'/NNAME,WS/'rk3588/robot_straight_v27/navigation/build'/NNAME,WS/'rk3588/robot_straight_v27_1/navigation/build'/NNAME,WS/'rk3588/robot_straight_v27_2/navigation/build'/NNAME]
BASELINE_COMBINATIONS=[(0,0),(0,1),(1,0),(1,1),(2,2),(3,3),(4,4),(5,5),(6,6),(7,7),(8,8),(9,9),(10,10),(11,11)]
DASHES.append(WS/'rk3588/robot_layered_v28/dashboard/build'/DNAME)
NAVS.append(WS/'rk3588/robot_layered_v28/navigation/build'/NNAME)
BASELINE_COMBINATIONS.append((12,12))
DASHES.append(WS/'rk3588/robot_layered_v28_1/dashboard/build'/DNAME)
NAVS.append(WS/'rk3588/robot_layered_v28_1/navigation/build'/NNAME)
BASELINE_COMBINATIONS.append((13,13))
DASHES.append(WS/'rk3588/robot_reactive_v29/dashboard/build'/DNAME)
NAVS.append(WS/'rk3588/robot_reactive_v29/navigation/build'/NNAME)
BASELINE_COMBINATIONS.append((14,14))
DASHES.append(WS/'rk3588/robot_local_avoid_v29_2/dashboard/build'/DNAME)
NAVS.append(WS/'rk3588/robot_local_avoid_v29_2/navigation/build'/NNAME)
BASELINE_COMBINATIONS.append((15,15))
DASHES.append(WS/'rk3588/robot_sonar_avoid_v30/dashboard/build'/DNAME)
NAVS.append(WS/'rk3588/robot_sonar_avoid_v30/navigation/build'/NNAME)
BASELINE_COMBINATIONS.append((16,16))
NEW_DASH=SRC.parent/'dashboard/build'/DNAME;NEW_NAV=SRC.parent/'navigation/build'/NNAME
BASE=WS/'rk3588/robot_navigation_v23';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--busybox',action='store_true');p.add_argument('--package',type=Path);p.add_argument('--report',type=Path,default=SRC/'reports/installer-tests.json');args=p.parse_args()
    shell=['busybox','sh'] if args.busybox else ['sh'];checks=0;cases=[]
    def ck(value,message=''):
        nonlocal checks
        assert value,message;checks+=1
    spec=importlib.util.spec_from_file_location('seal',SRC/'seal_package.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    ck([sha(f) for f in DASHES]==['b06309212b022e06a269ca86f271e95fd5ca217567902af2a7dae2d51cee52ce','4af680bc190f49e237b95b932762e9bee9b0f3fe9369354169aae735a20e0969','13a8d8d837905fa95f21ee4956c2fff4cb7144b80ab3dcc0d5fcb65ae068ef02','a168471c8f489345106f86df20db1a250ee321d6d913ac7888adba6091138ffa','67571b9407038a2167a558dba2a10cfd4a685a46fcbbdf4d033dc56f9fa6f752','7684fa64ed36cf4be682fb66127983e5d9a551ae7e3a44355a3c561d62be3da9','e9af8f5a2d9a618b3573e09fd000415ac99d09fdc4796eea52126fa799c8be9b','f4156b71b076bfb4a061b17ca8597f9fef9d727aa0c3576ea1fd3411cc60bd7d','9fad208692a1c20fe27e4dea2cfdf94ff3511be2132eccde3b5dacaf17c90519','591be67e4f6f2f0b6acae63c81b667738910ab95e76ff31b8a6467fb606cb4d5','d916c0db1160691e1ed709755ff84e454eed79d934603da87164ddde727ebcab','20fc3935da3efd8de3c19c1eecbd4ddb38b039a62b94811d59fe28e103da555e','67abbb44b82798ad539900c44450ec212120d3bdcb7f32933f62128b9c7ad657','4549aa17d409998e00bb3e3e8b525d0f8b5877d7ebfe1428347e4b748bd18abd','dabec4068898d9da6cc3c0f9b065478a941ebfa0456734fbf899032965e99e38','1387d1cc2dbefee19fc6a07c06f676d938861310e74fe7341bce47d600597cdd','53b4cb02636c9c7807d16156c83dac07f78c2e5e7fa82c523b90d5e82def2926'])
    ck([sha(f) for f in NAVS]==['c6023121d82187a48ccce65ae9961efc12bc0edf763842bd4d7e94afb1ad9518','b6b34ec3ef37a8b16667ba70ae8dc86acece59e5c59710bd02126d8ac9b1ba0c','34a3270bae4c231f419696a124af7cfdf44af4a54291f0f6b82dc65bf6853480','5f64ff58c6325db59192562be0de904db323cfdbf208ba93c6b0cd815e8f439e','8816350671ac2f1f9da6519e2b7863a8e7457b2bb6a878f0727bd20b18c3693c','a1ffde9c9e8cd04a77a0801c71e59e02e3447a9e199cdbf4580e6186957c90d6','f70b9af75f921b67dfae17129b1d70baec6561bf91e3838a82507d44fefe8223','0b43ace53a8c0d173688a3c1ca643fec98e511afd7bef8e15a33eca45f337f77','0911fe2f582469513d2ab60153183c987daaaeeebf93012ce5b909041a5de837','cf273584800659738e8a3b4d2d186bffece08761d580e3b23efbb8f4e565e7de','f62325d941aeda50ef4d61926f3934f8c38c3584c9e9a7766a23a7e5527f67b0','e2885e110d7796bee137b41e84203b110474b03cc4bcf2d0c781e289f0581aa7','a8aa750aa58c32f875cfd08646d625bb3708d4af48be6817e44a9e1ed32891ba','a1bf38d1e8a86af496c34f1cfa180cbb9abd8c329129d01b5d07aeb7ec3d808e','171c25c830ea88e051ac7b2f9fa2d312a70eaeff75bb56fbccafb88e69135f3f','bf393e6fedb7750819790b847605a0fa2480307c0f9699c56386743f0d3f33fc','7a1db8e5c58d36c77461a10da4c014812141eb9d06b6f4309136aa8d555a4fc8'])
    with tempfile.TemporaryDirectory(prefix='navigation-sonar-v31-three-sonar-20261028-install-') as temporary:
        tmp=Path(temporary);package=tmp/'package'
        if args.package:shutil.copytree(args.package,package)
        else:
            package.mkdir();(package/'payload').mkdir()
            for f in mod.RUNTIME_FILES:
                if f.startswith('payload/') or f=='payload.sha256':continue
                (package/f).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(SRC/f,package/f)
            shutil.copy2(NEW_DASH,package/'payload'/DNAME);shutil.copy2(NEW_NAV,package/'payload'/NNAME);mod.seal(package)
        subprocess.run(['sha256sum','-c','SHA256SUMS'],cwd=package,check=True,capture_output=True)
        for f in ('common.sh','install.sh','rollback.sh','diagnose-phone.sh','export-wrong.sh'):subprocess.run([*shell,'-n',str(package/f)],check=True)
        bindings={f:sha(package/f) for f in mod.RUNTIME_FILES};manifest=sha(package/'SHA256SUMS')
        new_dash=sha(package/'payload'/DNAME);new_nav=sha(package/'payload'/NNAME);new_pair=(new_dash+'\n').encode()
        ck(new_dash not in [sha(f) for f in DASHES] and new_nav not in [sha(f) for f in NAVS])
        tools=tmp/'tools';tools.mkdir()
        bodies={'id':'[ "$1" != -u ] || { echo "${TEST_UID:-0}"; exit; }; exec /usr/bin/id "$@"',
                'uname':'[ "$1" != -m ] || { echo "${TEST_ARCH:-aarch64}"; exit; }; exec /usr/bin/uname "$@"',
                'mv':'''last=
for last; do :; done
if [ "${last##*/}" = "${TEST_TARGET:-disabled}" ] && [ ! -f "$TEST_MARKER" ]; then
 : > "$TEST_MARKER"
 case "$TEST_FAILURE" in
 before) exit 73 ;;
 after) /usr/bin/mv "$@"; exit 73 ;;
 corrupt) /usr/bin/mv "$@"; printf corrupt > "$last"; exit 0 ;;
 signal) /usr/bin/mv "$@"; kill -TERM "$PPID"; exit 0 ;;
 esac
fi
exec /usr/bin/mv "$@"'''}
        for f,body in bodies.items():q=tools/f;q.write_text('#!/bin/sh\nset -eu\n'+body+'\n');q.chmod(0o755)
        def fixture(name,dash=0,navver=0,alias=False,cal=False):
            root=tmp/name;app=root/'root'/('actual-app' if alias else 'robot-perception');nav=root/'root'/('actual-nav' if alias else 'robot-navigation')
            app.mkdir(parents=True);nav.mkdir(parents=True);(root/'proc').mkdir()
            if alias:(root/'root/robot-perception').symlink_to(app,target_is_directory=True);(root/'root/robot-navigation').symlink_to(nav,target_is_directory=True)
            (root/'root/perception-dashboard').symlink_to(app,target_is_directory=True)
            shutil.copy2(DASHES[dash],app/DNAME);shutil.copy2(NAVS[navver],nav/NNAME)
            (app/DNAME).chmod(0o751);(nav/NNAME).chmod(0o750)
            (nav/'dashboard.sha256').write_text(sha(DASHES[dash])+'\n');(nav/'dashboard.sha256').chmod(0o640)
            shutil.copy2(BASE/'run-dashboard.sh',app/'run-dashboard.sh');shutil.copy2(BASE/'run-navigation.sh',nav/'run-navigation.sh')
            config=json.loads((BASE/'config.example.json').read_text());config.update(calibration_confirmed=cal,drive_mps_per_full_pwm=.46 if cal else 0.,operator_notes={'保留':[1,True]},reaction_s=.94)
            (nav/'config.json').write_text(json.dumps(config,ensure_ascii=False)+'\n');(nav/'config.json').chmod(0o640)
            for folder,files in ((app,{'run-perception.sh':b'camera40 launcher\n','lib/librknnrt.so':b'rknn','lib/libwayland-client.so.0':b'wayland','model/custom.rknn':b'unchanged model','pair-token.txt':b'048731\n','phone-preferences.json':b'{"left_mask":12,"invert_mask":15}\n'}),(nav,{'map.bin':b'existing map','wrong.log':b'persisted diagnostics\n','wrong.log.1':b'prior diagnostics\n','follow_geometry.json':b'{"measured":true}\n'})):
                for f,data in files.items():q=folder/f;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(data)
            env=dict(os.environ,PATH=str(tools)+':'+os.environ['PATH'],ROBOT_PHONE_V28_1_DESTDIR=str(root),TEST_TOOLS=str(tools))
            return root,app,nav,env
        def state(root):
            result={}
            for q in root.rglob('*'):
                f=str(q.relative_to(root))
                if q.is_file() and not q.is_symlink() and not f.startswith('proc/') and 'robot-phone-v31-three-sonar-20261028-backup.' not in f:
                    s=q.stat();mtime=s.st_mtime_ns
                    if args.busybox and q.name in (DNAME,NNAME,'dashboard.sha256'):mtime//=1000000000
                    result[f]=(sha(q),s.st_mode&0o777,s.st_uid,s.st_gid,mtime)
            return result
        def run(script,env,*argv,ok=True):
            command=[*shell,str(package/script),*map(str,argv)]
            if args.busybox:command=['busybox','sh','-c','id() { "$TEST_TOOLS/id" "$@"; }; uname() { "$TEST_TOOLS/uname" "$@"; }; mv() { "$TEST_TOOLS/mv" "$@"; }; . "$0"',str(package/script),*map(str,argv)]
            r=subprocess.run(command,env=env,capture_output=True,text=True);ck((r.returncode==0)==ok,r.stdout+r.stderr);return r
        def backups(root):return list((root/'root').glob('robot-phone-v31-three-sonar-20261028-backup.*'))
        def clean(root,app,nav):ck(not (root/'root/.robot-control-v21.lock').exists() and not list(app.glob('.navigation-sonar-v31-three-sonar-20261028.*')) and not list(nav.glob('.navigation-sonar-v31-three-sonar-20261028.*')))
        for dash,navver in BASELINE_COMBINATIONS:
            for alias in (False,True):
                for cal in (False,True):
                    root,app,nav,env=fixture(f'normal-{dash}-{navver}-{alias}-{cal}',dash,navver,alias,cal);before=state(root)
                    run('install.sh',env,'--check-only');ck(state(root)==before and not backups(root));run('install.sh',env)
                    ck(sha(app/DNAME)==new_dash and sha(nav/NNAME)==new_nav and (nav/'dashboard.sha256').read_bytes()==new_pair)
                    after=state(root);targets={str(q.relative_to(root)) for q in (app/DNAME,nav/NNAME,nav/'dashboard.sha256')}
                    ck(all(after[k][1:4]==before[k][1:4] for k in targets));ck({k:v for k,v in after.items() if k not in targets}=={k:v for k,v in before.items() if k not in targets})
                    history=backups(root);ck(len(history)==1 and not (history[0]/'config.json').exists());run('install.sh',env);ck(state(root)==after and backups(root)==history)
                    run('rollback.sh',env,history[0]);ck(state(root)==before);run('rollback.sh',env,history[0]);ck(state(root)==before);clean(root,app,nav)
                    cases.append(f'complete three-file exact install/rollback dashboard={dash} navigation={navver} alias={alias} calibrated={cal}')
        for dash,navver in tuple((a,b) for a in range(len(DASHES)) for b in range(len(NAVS)) if (a,b) not in BASELINE_COMBINATIONS):
            root,app,nav,env=fixture(f'reject-mixed-old-{dash}-{navver}',dash,navver);before=state(root)
            run('install.sh',env,ok=False);ck(state(root)==before and not backups(root));cases.append(f'reject mixed known baseline dashboard={dash} navigation={navver}')
        for reason in ('arch','uid','unknown-nav','unknown-dashboard','old-entry','old-orchestrator','pair','pair-no-newline','missing-config','config-symlink','nav-symlink','dashboard-symlink','pair-symlink','public-separate','lock','mixed-dashboard','mixed-navigation','mixed-pair'):
            root,app,nav,env=fixture('reject-'+reason)
            if reason=='arch':env['TEST_ARCH']='x86_64'
            elif reason=='uid':env['TEST_UID']='1000'
            elif reason=='unknown-nav':(nav/NNAME).write_bytes(b'unknown')
            elif reason=='unknown-dashboard':(app/DNAME).write_bytes(b'unknown')
            elif reason=='old-entry':shutil.copy2(WS/'rk3588/acg138-restore-20261023/launchers/run-dashboard.sh',app/'run-dashboard.sh')
            elif reason=='old-orchestrator':shutil.copy2(WS/'rk3588/robot_navigation_v22/run-navigation.sh',nav/'run-navigation.sh')
            elif reason=='pair':(nav/'dashboard.sha256').write_text('0'*64+'\n')
            elif reason=='pair-no-newline':(nav/'dashboard.sha256').write_text(sha(app/DNAME))
            elif reason=='missing-config':(nav/'config.json').unlink()
            elif reason.endswith('-symlink'):
                q={'config-symlink':nav/'config.json','nav-symlink':nav/NNAME,'dashboard-symlink':app/DNAME,'pair-symlink':nav/'dashboard.sha256'}[reason];real=q.with_name(q.name+'.real');q.rename(real);q.symlink_to(real)
            elif reason=='public-separate':(root/'root/perception-dashboard').unlink();(root/'root/perception-dashboard').mkdir()
            elif reason=='lock':(root/'root/.robot-control-v21.lock').mkdir()
            elif reason=='mixed-dashboard':shutil.copy2(package/'payload'/DNAME,app/DNAME)
            elif reason=='mixed-navigation':shutil.copy2(package/'payload'/NNAME,nav/NNAME)
            elif reason=='mixed-pair':(nav/'dashboard.sha256').write_bytes(new_pair)
            before=state(root);run('install.sh',env,ok=False);ck(state(root)==before and not backups(root));cases.append('reject '+reason)
        for who in ('navigation','dashboard','calibrator','launcher'):
            forms=('public','relative') if who=='launcher' else ('direct','deleted') if who=='calibrator' else ('direct','deleted','hardlink')
            for form in forms:
                root,app,nav,env=fixture(f'active-{who}-{form}');q=root/'proc/1234';q.mkdir();target=nav/NNAME if who=='navigation' else app/DNAME
                if who=='calibrator':target=root/'root/encoder-calibrate.aarch64'
                if who=='launcher':q.joinpath('cmdline').write_bytes(b'sh\0'+(str(root/'root/perception-dashboard/run-dashboard.sh') if form=='public' else 'run-dashboard.sh').encode()+b'\0-0\0')
                elif form=='hardlink':os.link(target,q/'exe')
                else:q.joinpath('exe').symlink_to(str(target)+(' (deleted)' if form=='deleted' else ''))
                before=state(root);run('install.sh',env,ok=False);ck(state(root)==before and not backups(root));cases.append(f'reject active {who} {form}')
        for phase in ('install','rollback'):
            for target in (DNAME,NNAME,'dashboard.sha256'):
                for failure in ('before','after','corrupt','signal'):
                    root,app,nav,env=fixture(f'{phase}-{target}-{failure}')
                    if phase=='rollback':run('install.sh',env)
                    before=state(root);env.update(TEST_TARGET=target,TEST_FAILURE=failure,TEST_MARKER=str(tmp/(root.name+'.marker')))
                    run('install.sh',env,ok=False) if phase=='install' else run('rollback.sh',env,backups(root)[0],ok=False)
                    ck(state(root)==before);clean(root,app,nav);cases.append(f'{phase} full-group recovery {target} {failure}')
        for reason in ('later-navigation','later-dashboard','later-pair','later-launcher','bad-backup','bad-receipt'):
            root,app,nav,env=fixture('rollback-'+reason);run('install.sh',env);backup=backups(root)[0]
            q={'later-navigation':nav/NNAME,'later-dashboard':app/DNAME,'later-pair':nav/'dashboard.sha256','later-launcher':app/'run-dashboard.sh','bad-backup':backup/DNAME,'bad-receipt':backup/'receipt.txt'}[reason];q.write_bytes(b'later or corrupt')
            before=state(root);run('rollback.sh',env,backup,ok=False);ck(state(root)==before);cases.append('rollback reject '+reason)
        root,app,nav,env=fixture('later-config');run('install.sh',env);backup=backups(root)[0]
        (nav/'config.json').write_bytes(b'{"calibration_confirmed":false,"custom":{"later":true}}\n');before=state(root);run('install.sh',env);ck(state(root)==before);run('rollback.sh',env,backup)
        after=state(root);targets={str(q.relative_to(root)) for q in (app/DNAME,nav/NNAME,nav/'dashboard.sha256')};ck({k:v for k,v in after.items() if k not in targets}=={k:v for k,v in before.items() if k not in targets});cases.append('later config edits retained during idempotence and complete-pair rollback')
        for filename in (DNAME,NNAME):
            saved=(package/'payload'/filename).read_bytes()
            for reason in ('checksum','elf'):
                root,app,nav,env=fixture('payload-'+filename+'-'+reason);before=state(root)
                if reason=='checksum':(package/'payload'/filename).write_bytes(saved+b'corrupt')
                else:
                    data=bytearray(saved);data[18:20]=b'\x3e\x00';(package/'payload'/filename).write_bytes(data)
                    (package/'payload.sha256').write_text(''.join(f'{sha(package/f)}  {f}\n' for f in mod.PAYLOAD_FILES));(package/'SHA256SUMS').write_text(''.join(f'{sha(package/f)}  {f}\n' for f in mod.RUNTIME_FILES))
                run('install.sh',env,ok=False);ck(state(root)==before);(package/'payload'/filename).write_bytes(saved);mod.seal(package);cases.append(f'reject {filename} {reason}')
        root,app,nav,env=fixture('diagnose-integrity');before=state(root);saved=(package/'diagnose-phone.sh').read_bytes()
        (package/'diagnose-phone.sh').write_bytes(saved+b'\n# changed after sealing\n')
        run('install.sh',env,ok=False);ck(state(root)==before and not backups(root));(package/'diagnose-phone.sh').write_bytes(saved);mod.seal(package);cases.append('reject modified sealed diagnostic script without executing it')
        root,app,nav,env=fixture('wrong-mode');before=state(root);mode=(package/'package-mode.txt').read_bytes();(package/'package-mode.txt').write_text('dashboard-only\n');(package/'SHA256SUMS').write_text(''.join(f'{sha(package/f)}  {f}\n' for f in mod.RUNTIME_FILES))
        run('install.sh',env,ok=False);ck(state(root)==before);(package/'package-mode.txt').write_bytes(mode);mod.seal(package);cases.append('reject any mode outside fixed three-file transaction')
    report={'status':'passed','checks':checks,'cases':cases,'arm_programs_executed':False,'hardware_accessed':False,'payload_kind':'actual final compiled AArch64 binaries; checked/copied only','replaced_files':[DNAME,NNAME,'dashboard.sha256'],'config_tokens_launchers_byte_exact':True,'later_config_edits_retained_on_rollback':True,'shell':'busybox sh' if args.busybox else 'host sh','replaced_file_mtime_precision':'seconds' if args.busybox else 'nanoseconds','dashboard_sha256':new_dash,'navigation_sha256':new_nav,'pair_content_sha256':hashlib.sha256(new_pair).hexdigest(),'test_script_sha256':sha(Path(__file__)),'tested_runtime_manifest_sha256':manifest,'tested_runtime_files_sha256':bindings}
    args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
