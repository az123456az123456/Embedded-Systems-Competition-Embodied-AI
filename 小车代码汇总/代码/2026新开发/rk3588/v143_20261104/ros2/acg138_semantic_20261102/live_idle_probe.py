"""Authorized static integration test: read state, send preview instructions only."""
import json
from pathlib import Path
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    node = Node('semantic_static_integration_probe')
    states, results, goals, exploration = [], [], [], []
    node.create_subscription(String, '/fpga/state', lambda m: states.append(json.loads(m.data)), 10)
    node.create_subscription(String, '/semantic/result', lambda m: results.append(json.loads(m.data)), 10)
    node.create_subscription(PoseStamped, '/fpga/goal', lambda m: goals.append(m), 10)
    node.create_subscription(String, '/exploration/state', lambda m: exploration.append(json.loads(m.data)), 10)
    commands = node.create_publisher(String, '/semantic/command', 1)
    params = node.create_client(GetParameters, '/semantic_navigation/get_parameters')

    def until(condition, timeout=15.):
        deadline = time.monotonic()+timeout
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not condition():
            raise RuntimeError('static integration probe deadline expired')

    def idle_state():
        value = states[-1] if states else {}
        return (value.get('available') is True and value.get('state_stale') is False and
                value.get('state') in ('IDLE', 'ARRIVED', 'CANCELLED') and
                value.get('owner') == 0 and not value.get('flags', 1) & 1 and
                value.get('permit_remaining_ms') == 0)

    try:
        until(lambda: commands.get_subscription_count() > 0 and params.service_is_ready())
        request = GetParameters.Request(names=['publish_goals', 'fixture_mode'])
        future = params.call_async(request)
        until(future.done)
        values = future.result().values
        assert len(values) == 2 and all(v.type == 1 and not v.bool_value for v in values), values
        until(idle_state)
        until(lambda: bool(exploration))
        assert exploration[-1]['active'] is False and exploration[-1]['state'] == 'IDLE'
        assert exploration[-1]['drive']['motion_requested'] is False
        token = Path('/run/acg138-ros-bridge/semantic/token').read_text().strip()
        request = urllib.request.Request('http://127.0.0.1:8082/api/state',
                                         headers={'Authorization': 'Bearer '+token})
        with urllib.request.urlopen(request, timeout=3) as response:
            backend = json.load(response)
        vision = backend.get('vision', {})
        assert vision.get('detection_enabled') is False, vision
        initial = states[-1]
        terminal = []
        for instruction in ('person', 'red bottle', 'chair', 'cardboard box'):
            assert idle_state(), states[-1]
            start = len(results)
            commands.publish(String(data=instruction))
            until(lambda: any(r.get('status') in ('REJECTED', 'PREVIEW') for r in results[start:]))
            result = next(r for r in reversed(results[start:]) if r.get('status') in ('REJECTED', 'PREVIEW'))
            terminal.append(result)
            assert 'presence' in result, result
            assert result.get('goal_published') is False and not goals, result
            assert idle_state(), states[-1]
        output = {'passed': True, 'publish_goals': False, 'fixture_mode': False,
                  'backend_vision': vision, 'initial_fpga_state': initial,
                  'final_fpga_state': states[-1], 'results': terminal,
                  'exploration_idle': exploration[-1],
                  'ros_nodes': [name for name, namespace in node.get_node_names_and_namespaces()],
                  'observed_fpga_goals': len(goals), 'probe_sent_start': False}
        print('LIVE_IDLE_PROBE '+json.dumps(output, ensure_ascii=False), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
