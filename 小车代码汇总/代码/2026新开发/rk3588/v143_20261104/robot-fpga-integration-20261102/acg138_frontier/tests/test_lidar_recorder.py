import subprocess
from pathlib import Path
from acg138_frontier.lidar_recorder import MIB, command, quota_reason, directory_bytes, TOPICS


def test_only_observation_topics():
    assert '/scan' in TOPICS and '/scan_mapping' in TOPICS and '/scan_clearing' in TOPICS
    assert '/tf_static' in TOPICS and '/odom' in TOPICS and '/imu/data' in TOPICS
    assert not any(t in TOPICS for t in ('/cmd_vel', '/fpga/start', '/fpga/goal'))
    cmd = command('/tmp/test', '/tmp/qos.yaml', 32*MIB)
    assert cmd[:3] == ['ros2', 'bag', 'record']
    assert cmd[cmd.index('--max-cache-size')+1] == str(2*MIB)


def test_limits_and_reserve():
    q = lambda used, total, free: quota_reason(used, total, free, 256*MIB,512*MIB,512*MIB)
    assert q(239*MIB,495*MIB,529*MIB) is None
    assert q(240*MIB,495*MIB,529*MIB) == 'session_size_limit'
    assert q(0,496*MIB,529*MIB) == 'total_size_limit'
    assert q(0,0,528*MIB) == 'low_disk_space'


def test_no_delete_no_follow_symlinks(tmp_path):
    root=tmp_path/'recordings'; root.mkdir()
    (root/'old.db3').write_bytes(b'x'*12)
    (tmp_path/'outside').write_bytes(b'x'*1000)
    (root/'pointer').symlink_to(tmp_path/'outside')
    assert directory_bytes(root) == 12
    assert (root/'old.db3').read_bytes() == b'x'*12


def test_flag_stripped_without_reordering_other_arguments():
    script = Path(__file__).resolve().parents[2]/'deploy/run-stack.sh'
    preamble=script.read_text().split('if [ "${1:-}" = --help ]; then')[0]
    for args, enabled, expected in [([], 'false', []), (['--log'],'true',[]),
        (['--model','a b','--log','--seconds','9'],'true',['--model','a b','--seconds','9']),
        (['--log','--log','--model','a b'],'true',['--model','a b']),
        (['--phone-fps','auto'],'false',['--phone-fps','auto'])]:
        result=subprocess.check_output(['sh','-c',preamble+'\nprintf "%s\\n" "$RECORD_LIDAR"; for v do printf "%s\\n" "$v"; done','test',*args],text=True)
        assert result.splitlines()==[enabled,*expected]


def test_launch_default_off_and_no_environment_override():
    p=Path(__file__).resolve().parents[1]/'launch/integrated.launch.py'
    text=p.read_text()
    assert "DeclareLaunchArgument('record_lidar', default_value='false')" in text
    assert "condition=IfCondition(LaunchConfiguration('record_lidar'))" in text
