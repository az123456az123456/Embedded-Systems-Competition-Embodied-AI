"""Construct real ROS adapters in preview mode, without any hardware producer."""
import json,os
import rclpy
from rclpy.utilities import get_rmw_implementation_identifier
from acg138_frontier.mapping_node import MappingCoordinator
from acg138_frontier.node import FrontierNode
from acg138_frontier.mapping_explorer import MappingExplorer

assert os.environ['RMW_IMPLEMENTATION']=='rmw_fastrtps_cpp'
assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']=='SUBNET'
rclpy.init()
mapping=MappingCoordinator()
frontier=FrontierNode()
explorer=MappingExplorer()
try:
    assert mapping.dry_run and mapping.bridge is None and mapping.slam is None
    assert mapping.session.state=='IDLE' and frontier.dry_run
    assert explorer.dry_run and not explorer.active
    for node in (mapping,frontier,explorer):rclpy.spin_once(node,timeout_sec=.01)
    assert mapping.bridge is None and mapping.slam is None
    print(json.dumps({'pass':True,'real_ROS_adapter_construction':True,
        'RMW':get_rmw_implementation_identifier(),'discovery':os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'],
        'mapping_state':mapping.session.state,'frontier_worker_constructed':True,'exploration_active':explorer.active,
        'dry_run':True,'hardware_nodes_started':0}))
finally:
    explorer.destroy_node();frontier.destroy_node();mapping.destroy_node();rclpy.shutdown()
