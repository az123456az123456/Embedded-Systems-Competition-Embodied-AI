#!/usr/bin/env python3
"""Exercise the real camera launcher; mocks cannot send motor/network traffic."""
import importlib.util
import json
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('fixture', HERE / 'launcher_fixture.py')
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


class BarrierTests(fixture.LauncherTest):
    def setUp(self):
        super().setUp()
        self.env['PERCEPTION_CAMERA_PROBE'] = '0'
        self.env.pop('PERCEPTION_PREPARE_ONLY', None)

    def test_barrier_reuses_ready_isp_without_starting_native(self):
        value = self.run_launcher(changes={'MOCK_ISP_RUNNING': '1', 'PERCEPTION_PREPARE_ONLY': '1'})
        self.assertIn('CAMERA_PREPARED', value.stdout)
        self.assertFalse(self.result.exists())
        self.assertFalse(self.commands_seen('nohup'))

    def test_barrier_stale_nv12_does_not_prove_new_isp_started(self):
        value = self.run_launcher(changes={'MOCK_ISP_RUNNING': '0', 'PERCEPTION_PREPARE_ONLY': '1'}, expected=1, startup=True)
        self.assertIn('not ready', value.stderr)
        self.assertFalse(self.result.exists())
        self.assertEqual(len(self.commands_seen('v4l2-ctl')), 3)

    def test_barrier_cold_start_waits_for_receiver_after_network_setup(self):
        code = fixture.DISPATCH.replace('if os.environ.get("MOCK_ISP_RUNNING") == "1":', 'if count >= 3:')
        self.dispatcher.write_text(code)
        value = self.run_launcher(changes={'PERCEPTION_PREPARE_ONLY': '1'}, startup=True)
        self.assertIn('CAMERA_PREPARED', value.stdout)
        self.assertEqual(len(self.commands_seen('v4l2-ctl')), 2)
        self.assertFalse(self.result.exists())

    def test_barrier_native_pass_never_reinitializes_network(self):
        self.run_launcher(args=['--npu-workers', '3'], changes={'PERCEPTION_PREPARE_ISP': '0', 'PERCEPTION_PREPARE_ONLY': '0'})
        self.assertFalse(self.commands_seen('nohup'))
        self.assertFalse(self.commands_seen('ps'))
        self.assertEqual(self.option('--npu-workers'), '3')

    def test_barrier_invalid_flag_has_no_side_effects(self):
        value = self.run_launcher(changes={'PERCEPTION_PREPARE_ONLY': '2'}, expected=2)
        self.assertIn('PERCEPTION_PREPARE_ONLY', value.stderr)
        self.assertFalse(self.events())

    def test_barrier_stack_orders_prep_before_ros_and_blocks_second_isp(self):
        text = (HERE.parents[1] / 'deploy/run-stack.sh').read_text()
        self.assertLess(text.index('PERCEPTION_PREPARE_ONLY=1 sh'), text.index('setsid sh "$BASE/deploy/run-chroot.sh"'))
        self.assertIn('PERCEPTION_PREPARE_ONLY=1 sh "$BASE/perception/run-dashboard.sh" "$@" 9>&-', text)
        self.assertIn('setsid env PERCEPTION_PREPARE_ISP=0 PERCEPTION_PREPARE_ONLY=0', text)


if __name__ == '__main__':
    # Only this change's cases; do not include inherited historical contracts.
    names = [n for n in dir(BarrierTests) if n.startswith('test_barrier_')]
    result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(BarrierTests(n) for n in names))
    report = {'cases': len(names), 'failed': len(result.failures), 'errors': len(result.errors),
              'pass': result.wasSuccessful(), 'scope': 'camera/network startup barrier; no motion'}
    print(json.dumps(report))
    raise SystemExit(0 if result.wasSuccessful() else 1)
