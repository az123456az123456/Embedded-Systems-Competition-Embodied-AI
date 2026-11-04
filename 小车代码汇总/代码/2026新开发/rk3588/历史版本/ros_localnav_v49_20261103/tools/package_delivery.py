#!/usr/bin/env python3
"""Package the stopped, reviewed RK integration without promoting its blockers."""
import hashlib
import json
import shutil
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RK = ROOT / 'rk3588/ros_exploration_candidate26_20261101'
PHONE = ROOT / 'rk3588/robot_phone_fpga_navigation_20261102'
MODEL = ROOT / 'rk3588/model_last_e0e54560_20261101'
OUT = ROOT / '交付/20261102-RK手机-candidate26'
CANONICAL = ROOT / '小车代码汇总/代码/2026新开发/上位机/ACG138-FPGA选点导航-v1.2.26'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n')


def add_tree(archive, source, target):
    excluded = {'.git', '__pycache__', 'build', 'build-offline', 'dist', 'install', 'log'}
    for path in sorted(source.rglob('*')):
        relative = path.relative_to(source)
        if excluded.intersection(relative.parts) or not path.is_file():
            continue
        if path.suffix in {'.o', '.a', '.pyc', '.keystore', '.jks'}:
            continue
        if 'pair-token' in path.name or path.name == 'local.properties':
            continue
        archive.add(path, arcname=str(Path(target) / relative), recursive=False)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    overlay = OUT / 'ros-overlay-installed-20261102.tar.gz'
    assert sha(overlay) == '5d43f421489c7393216cba00162d835d5b201ed3567fdb8e4cd3d917359f1b90'
    expected = {
        RK / 'bin/robot-fpga-gateway.aarch64': '94e4313460cbe970100ded810e646977ca6bac560d3097190db1c641f4867101',
        RK / 'perception/perception-dashboard.aarch64': '150cdd31679951071ad1fddcceba8571311e5b7e794e4027433d1f05fd0af8b4',
        PHONE / 'dist/ACG138-Robot-Control-debug.apk': '6948b420da30da5a4acc8adf63f3418a10aaaad0ac3a1668e7cc20a3b804e62b',
        MODEL / 'acg138-last-e0e54560-int8-rk3588.rknn': 'ae4598bc0065d8f411c7307e76d80d1bc6394d83cc4ab0221777fbb68ee158e3',
    }
    for path, wanted in expected.items():
        assert sha(path) == wanted, str(path)

    console = RK / 'reports/board-current/gateway-flow-inspect-open-area.console'
    cases = []
    for line in console.read_text().splitlines():
        if line.startswith('{'):
            cases.append(json.loads(line))
    assert any(c.get('case') == 'actual_COMMIT_banner' and c['passed'] for c in cases)
    assert any(c.get('case') == 'flow_error' and not c['passed'] for c in cases)
    evidence = OUT / '验收记录'
    evidence.mkdir(exist_ok=True)
    (evidence / 'gateway-flow.jsonl').write_text(''.join(json.dumps(c, ensure_ascii=False) + '\n' for c in cases))

    status = json.loads((RK / 'reports/integration-current-status.json').read_text())
    status.update(stage='frontier_execution_chain_compiled_no_functional_test',
                  delivery_requested_without_more_tests=True,
                  release_allowed=False, production_install=False,
                  board_stack_stopped=True,
                  board_stop_evidence='reports/board-current/frontier-compile-only-export.console (idle runtime lock; no current motor ACK measured)',
                  delivery_directory=str(OUT))
    status['passed']['ROS_colcon_4_packages'] = 'Historical five-package overlay; latest acg138_fpga_map_bridge + acg138_frontier compile exit 0 (39.0s), source/installed Python identical; no functional test'
    status['passed']['explicit_ARM_real_motor_ACK'] = 'PASS: lifted bench test; finally STOP+DISARM confirmed by real motor ACK'
    status['passed']['mapping_save_localization_FPGA_COMMIT'] = {
        'status': 'PASS', 'map_session': '4887846635154463992', 'map_revision': 1,
        'bitmap_crc32': 3179004831, 'evidence': str(console.relative_to(RK)),
    }
    status['latest_fix'] = {
        'name': 'Frontier execution chain: bounded MAP_BEGIN BUSY wait, generation-bound status and phase timeouts, persistent logs',
        'files': ['acg138_fpga_map_bridge/src/nav3_wire.cpp',
                  'acg138_frontier/acg138_frontier/node.py',
                  'acg138_frontier/acg138_frontier/mapping_node.py', 'deploy/run-stack.sh'],
        'board_install': 'APPLIED; backup retained in model-validation-last-20261102/before-frontier-execution-chain-fix',
        'compile': {'status': 'PASS', 'packages': 2, 'seconds': 39.0,
                    'evidence': 'reports/board-current/frontier-execution-chain-build-status.console'},
        'post_fix_board_functional_test': 'NOT_RUN at user request',
        'services_restarted_after_fix': False,
        'scope': 'Existing Frontier selection retained; RK coordination only; FPGA/wire protocol/executor/safety unchanged',
        'preceding_camera_startup_barrier': 'Retained; post-fix board functional test NOT_RUN',
    }
    status['final_motor_ack'] = 'Historical mapping test ended DISARMED_ZERO; current session only compiled/exported files after reboot, no motor command or new ACK measurement'
    status['historical_test_scope'] = 'All existing functional PASS/BLOCKED entries predate this Frontier execution-chain fix; they do not validate the new code'
    status['not_passed_or_unverified'] = [
        {'item': 'point preview/START/ARRIVED/END on real FPGA', 'status': 'BLOCKED',
         'reason': 'Real committed local bitmap does not mark the current robot cell traversable. Test deliberately stopped before GOAL/START; no synthetic map substituted.'},
        {'item': 'autonomous moving exploration after Frontier execution-chain fix', 'status': 'NOT_RUN',
         'reason': 'User requested no tests. Only compile, source/install identity and packaging were performed; no nodes or motion started.'},
        {'item': 'real Android 11+ UI/video/storage', 'status': 'NOT_RUN'},
        {'item': 'ground tracking/avoidance', 'status': 'NOT_RUN', 'reason': 'User will perform ground avoidance testing.'},
    ]
    status['display_scope'] = 'ROS-derived 6.4m local window; no global-map panorama; internal FPGA replan-path refresh not implemented'
    for path in (RK / 'reports/integration-current-status.json', RK / 'staging/status.json', OUT / '交付状态.json'):
        dump(path, status)
    scope = json.loads((RK / 'reports/software-only-scope-current.json').read_text())
    scope['blocked'] = status['not_passed_or_unverified']
    scope['ROS_build'] = status['passed']['ROS_colcon_4_packages']
    scope['latest_fix'] = status['latest_fix']
    dump(RK / 'reports/software-only-scope-current.json', scope)

    phone_status = json.loads((PHONE / 'reports/release-verification.json').read_text())
    phone_status.update(RK_full_integration='PARTIAL: real mapping/save/localization/COMMIT PASS; real point START/ARRIVED blocked by current real map',
                        release_status='DELIVERED_WITH_KNOWN_BLOCKER', production_navigation_acceptance='NOT_PASSED')
    phone_status['RK_latest_startup_fix'] = 'APPLIED; board functional test NOT_RUN at user request; APK unchanged'
    phone_status['RK_latest_frontier_fix'] = 'APPLIED + COMPILED; board functional test NOT_RUN at user request; APK unchanged'
    for path in (PHONE / 'reports/release-verification.json', CANONICAL / 'release-verification.json', OUT / '手机验证.json'):
        dump(path, phone_status)
    shutil.copy2(PHONE / 'dist/ACG138-Robot-Control-debug.apk', OUT / 'ACG138-Control-v1.2.26.apk')

    note = '''本交付面向现有 RK3588 Buildroot + Jazzy 独立用户空间和 candidate26 位流。
交付目的：提供本轮 RK/ROS/手机程序、已编译文件、源码和真实验收记录；不把尚未完成的导航验收写成通过。

2026-11-02 最新更新：落实 Frontier 探索执行链。
点击探索建图后，SLAM 先产生初始局部图；RK 从“已知自由且邻接未知”的边界簇中筛选可达目标；FPGA 独立规划、跟踪、避障和到达；每段到达后停车取消旧任务，再更新局部图、选择下一个前沿。不等待整张地图建完，也没有改成无图随机行驶。
保留既有 Frontier 算法：按可达距离和边界规模排序，目标必须在已确认的膨胀局部自由格中；无可达边界会明确报错，不把它算作探索完成。
日志证实先前 MAP_BEGIN 收到 BUSY，而 RK 将失败锁存后未恢复。本次仅对首次 MAP_BEGIN 的无副作用 BUSY 拒绝增加有界 1500ms 等待、50ms 间隔、同一事务重试；FPGA 每次仍检查真实停车资格。持续 BUSY 则明确失败，不继续传图/目标/START。日志不能证明当时 BUSY 的具体物理原因。
将实际前沿选择/等待/导航/错误状态按 generation 和源年龄关联到现有手机建图状态；保留失败原因，避免 SLAM 已启动却笼统显示“正在探索”。日志持久保存在 /acg138-ros2/data/logs/integration，/tmp 旧路径保留为软链接，启动时轮换超过 8MiB 的日志。
两个修改包已在 RK 编译成功（39.0s），安装后的 Python 文件与源码完全一致。按用户要求，本次没有运行功能测试、启动节点、发送运动指令或重新量测电机 ACK。自主探索仍未实机验收，不能宣称已经保证运动成功。
备份目录：/root/model-validation-last-20261102/before-frontier-execution-chain-fix。FPGA candidate26、手机 APK、YOLO/NPU 代码均不变。

2026-11-02 启动修复：针对“RK 尚未接入探索建图服务”，调整为相机准备完成→ROS/NAV3 握手→Gateway/YOLO。旧顺序中，相机脚本的 eth0 down/up 会破坏已打开的 NAV3 套接字。
只修改 deploy/run-stack.sh 和 perception/run-perception.sh。相机准备阶段不启动 native Dashboard；等待实际 ISP 进程出现，避免残留 NV12 格式被误当作本次准备完成。第二阶段禁止重新初始化 ISP。
已替换当前 RK 独立目录中的脚本并保留旧脚本备份。用户要求改完不再测试：修复后没有启动服务，没有做实板功能验证。下列 PASS 均为此前记录，不能代表这次修复已经实板通过。手机 APK 不变。

一、历史验收状态（不代表最新 Frontier 修改已通过）
已通过：启动空闲、显式使能与真实电机 ACK；手机 HTTP 开始建图；真实雷达/SLAM 静态地图；结束建图后停车、保存、重定位、停车下发地图至 FPGA；COMMIT 成功及会话/版本/CRC 对账。
此前静态验收的 COMMIT：map_session=4887846635154463992，revision=1，bitmap_crc32=3179004831。
选点闭环未通过：真实局部地图中当前车体格不可通行，测试在发目标/START 前终止。没有替换为合成地图、放宽膨胀或关闭保护。
选点预览、显式开始、结束/手动接管的 RK/手机逻辑已有离线检查；不能因此认定实车选点已通过。
自主探索移动、实际 Android 11+、手机视频延迟及落地导航/避障未验收。用户本轮要求直接交付，不再追加测试。
此前完整建图测试结束时电机 ACK 为停用、零输出。本次排查进程也已退出，修复后未重新量测电机 ACK，不自动恢复服务或运动。

二、文件
host-runtime-reviewed-20261102.tar.gz：ARM64 Dashboard、Gateway、启动脚本和挂载助手。解到 /root 后产生 robot-fpga-integration-20261102/。
ros-overlay-installed-20261102.tar.gz：从 RK 收回的已编译五包独立 overlay。固定前缀 /opt/acg138_exploration_20261102，不是新的 rootfs。
source-rk-phone-20261102.tar.gz：本轮 RK、五个 ROS 包、公共头、测试、构建脚本和手机源码；不包含密钥/配对令牌、旧 FPGA 工程或主机 SDK。
ACG138-Control-v1.2.26.apk：包名 cn.acg138.robotphone，versionCode=39，沿用原签名；覆盖安装保留已有设置，未做手机真机验收。
model/：已应用的 last.pt 对应 INT8 RKNN 模型和标签。
验收记录/：本次真实测试、退出记录及相关离线记录；交付状态.json 是当前状态，不拼接历史失败为 PASS。
SHA256SUMS：交付文件校验值。

三、当前板上启动（已部署独立目录，无需重建 rootfs，不自动使能）
保持原 candidate26；先退出旧 robot-navigation、旧 dashboard 和旧 ROS 栈，避免双执行器或串口冲突。
在 RK 上执行：
PERCEPTION_CAMERA_PROBE_BINARY=/root/robot-perception/camera-link-probe.aarch64 sh /root/robot-fpga-integration-20261102/deploy/run-stack.sh --phone-fps auto --motor-interface eth0 --imu-interface eth0
前台 Ctrl+C 退出本套进程。原生产入口未被覆盖，没有安装开机自启动。
手机填写 RK 当前 Wi-Fi 地址和 8080 端口，沿用原配对设置；地址以板子当前实际值为准。
手机“探索建图”开始会话；“结束建图”停车并保存/加载；只有真实 COMMIT 对账成功才显示“地图已载入 FPGA”。
选点只预览；点“开始”才申请 FPGA 执行；点“结束”撤销，之后重新选点仍需重新点开始。当前真实地图阻塞尚未解除，不能保证选点可执行。

四、恢复这份增量包（只用于同一已安装 Jazzy 的板子）
先退出本套程序并确认停车。备份 /root/robot-fpga-integration-20261102 和 /acg138-ros2/rootfs/opt/acg138_exploration_20261102 后，再恢复同名独立目录。
tar -xzf host-runtime-reviewed-20261102.tar.gz -C /root
mkdir -p /acg138-ros2/rootfs/opt/acg138_exploration_20261102
tar -xzf ros-overlay-installed-20261102.tar.gz -C /acg138-ros2/rootfs/opt/acg138_exploration_20261102
继续沿用原 /root/robot-perception/pair-token.txt、原相机 ISP/V4L2 通路、/root/rknn_yolov8_demo/lib、原 /opt/ros/jazzy 和 /opt/acg138_ros。
不替换 Buildroot 的 /lib、/usr、内核或驱动。没有这些既有依赖的板子不能只解压本包运行。
源码中 deploy/build-overlay.sh 用于现有 chroot 内重新编译五个独立包；PC 侧 C++ 构建仍需现有 aarch64-none-linux-gnu SDK。
若板端日期落后源码年份，构建脚本会核对 Python 源码/安装文件；不能只凭 colcon 退出码忽略旧缓存。

五、架构和保留项
FPGA 的 RTL、固件、位流均未改；RK 仅传地图/位姿/目标/开始/取消和既有许可，不发送 ROS 速度，不实现路径跟踪或绕障。
500ms/30s/250ms/60ms 保护、硬件抢占、原子提交、超声硬停不变。
地图仅停车取消旧任务后加载；运行中不换图。
CPR=60527.7；编码器方向=(-1,1,-1,1)；雷达居中、高 0.175m、前向；C1=460800。
隔离包装脚本和 entrypoint 固化 FastRTPS + SUBNET；原系统未重刷。
相机推理源码保留原通路、每眼 640×640、三 NPU worker、完整同帧双目结果才用于跟随。
当前导航显示为 ROS 派生 6.4m 局部地图窗口；全局地图全景及 FPGA 内部重规划后新路径主动刷新未实现。
违规标签持续严格超过 0.5 秒时保存一张；支持 no_vest/no_mask/no_helmet（含 no_halmet 别名）；按手机日期归档，消失再出现才重新触发。

六、回滚
退出新进程后可使用仍保留的原入口和原程序；不要同时运行两套。FPGA 仍是同一 candidate26，无需烧录。
本包没有正式通过选点闭环或落地导航验收；详见交付状态.json 和验收记录/gateway-flow.jsonl。
'''
    (OUT / '交付说明.txt').write_text(note)
    (RK / 'reports/board-current/remaining-acceptance.md').write_text(
        '本轮按用户要求直接交付，不追加测试。此记录供集成人员核对剩余验收。\n\n'
        '| 项目 | 状态 | 证据 |\n| --- | --- | --- |\n'
        '| 静态建图→结束→保存→定位→真实 FPGA COMMIT | PASS | gateway-flow-inspect-open-area.console |\n'
        '| 真实选点→开始→到达→结束 | BLOCKED | 同文件：current_real_map_cell_blocked；未发 START |\n'
        '| 自主探索移动/落地跟踪避障 | NOT_RUN | 架空测试，不证明真实平移 |\n'
        '| Android 11+ 真机/视频延迟 | NOT_RUN | 只有构建、原签名和离线检查 |\n'
        '| 停车、退出、锁及挂载清理 | PASS | delivery-stop.console、delivery-export.console |\n\n'
        '上述为此前证据。最新 Frontier 执行链和相机/ROS 启动顺序修复已部署；两个 ROS 包仅编译及文件一致性核对完成。修复后功能测试 NOT_RUN（用户要求不再测试），节点未启动、未发送运动指令。\n\n'
        '详见交付目录中的交付说明.txt、交付状态.json。FPGA 不变，原生产入口未覆盖。\n')

    with tarfile.open(OUT / 'host-runtime-reviewed-20261102.tar.gz', 'w:gz') as archive:
        prefix = 'robot-fpga-integration-20261102'
        for rel in ('bin/robot-fpga-gateway.aarch64', 'perception/perception-dashboard.aarch64',
                    'perception/run-dashboard.sh', 'perception/run-perception.sh',
                    'deploy/run-stack.sh', 'deploy/run-chroot.sh', 'deploy/exec-chroot.sh',
                    'deploy/ros-entrypoint.sh', 'deploy/bin/mountns-exec.aarch64'):
            archive.add(RK / rel, arcname=prefix + '/' + rel, recursive=False)

    with tarfile.open(OUT / 'source-rk-phone-20261102.tar.gz', 'w:gz') as archive:
        for rel in ('acg138_nav_interfaces', 'acg138_base_telemetry', 'acg138_fpga_map_bridge',
                    'acg138_frontier', 'acg138_ros_bridge', 'common', 'vendor', 'gateway',
                    'rk_gateway', 'perception', 'deploy', 'tests', 'host-tests', 'tools', 'docs'):
            add_tree(archive, RK / rel, 'rk/' + rel)
        archive.add(RK / 'run_wire_tests.py', arcname='rk/run_wire_tests.py')
        add_tree(archive, PHONE, 'phone')

    (OUT / 'model').mkdir(exist_ok=True)
    shutil.copy2(MODEL / 'acg138-last-e0e54560-int8-rk3588.rknn', OUT / 'model/acg138-last-e0e54560-int8-rk3588.rknn')
    shutil.copy2(MODEL / 'labels.txt', OUT / 'model/acg138-last-e0e54560-labels.txt')
    records = (
        'board-current/gateway-flow-inspect-open-area.console', 'board-current/delivery-stop.console',
        'board-current/delivery-export.console', 'board-current/delivery-ros-download.json',
        'board-current/clean-overlay-status-4.console', 'board-current/odom-build-status-2.console',
        'board-current/map-topic-inspect.console', 'board-current/slam-input-audit-reviewed.console',
        'board-current/saved-map-audit.console', 'gateway-fpga-current.json',
        'board-current/mapping-service-after-idle.console',
        'board-current/mapping-network-inspect.console',
        'board-current/install-camera-startup-fix.console',
        'board-current/delivery-clean-exited-session.console',
        'camera-startup-barrier.log', 'camera-startup-barrier-fix.json',
        'generation-adapter/normal/summary.json', 'generation-adapter/asan-ubsan/summary.json',
        'delegation/summary.json', 'frontier-mapping-current.log',
        'odom-publication-clock-normal.json', 'odom-publication-clock-asan.json',
        'odom-protection-regression.log', 'software-only-scope-current.json',
        'frontier-execution-chain-fix.json',
        'board-current/frontier-execution-chain-build-status.console',
        'board-current/frontier-compile-only-export.console',
        'board-current/frontier-overlay-download.json',
        'board-current/exploration-persisted-evidence.console',
    )
    for rel in records:
        source = RK / 'reports' / rel
        if source.is_file():
            target = evidence / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        else:
            raise FileNotFoundError(source)

    shutil.copy2(RK / 'staging/camera-startup-barrier-fix.tar.gz', OUT / 'camera-startup-barrier-fix.tar.gz')
    shutil.copy2(RK / 'staging/frontier-execution-chain-fix-20261102.tar.gz', OUT / 'frontier-execution-chain-fix-20261102.tar.gz')

    manifest = {str(p.relative_to(OUT)): {'sha256': sha(p), 'bytes': p.stat().st_size}
                for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in {'manifest.json', 'SHA256SUMS'}}
    dump(OUT / 'manifest.json', {'scope': 'incremental RK/phone handoff; known blocker preserved', 'files': manifest})
    files = sorted(p for p in OUT.rglob('*') if p.is_file() and p.name != 'SHA256SUMS')
    (OUT / 'SHA256SUMS').write_text(''.join(sha(p) + '  ' + str(p.relative_to(OUT)) + '\n' for p in files))
    # Read each archive after closing it: identity/integrity checking, no test run.
    for path in OUT.glob('*.tar.gz'):
        with tarfile.open(path) as archive:
            for member in archive:
                if member.isfile():
                    stream = archive.extractfile(member)
                    while stream.read(1024 * 1024):
                        pass
    bundle = OUT.with_suffix('.tar.gz')
    with tarfile.open(bundle, 'w:gz') as archive:
        archive.add(OUT, arcname=OUT.name)
    bundle.with_name(bundle.name+'.sha256').write_text(sha(bundle)+'  '+bundle.name+'\n')
    print(json.dumps({'bundle': str(bundle), 'bytes': bundle.stat().st_size,
                      'sha256': sha(bundle), 'point_navigation': 'BLOCKED'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
