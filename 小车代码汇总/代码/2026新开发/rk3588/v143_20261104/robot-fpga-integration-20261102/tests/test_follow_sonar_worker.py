"""Load the real worker with lightweight ROS imports; sonar mode must not touch scan/TF or post observations."""
import importlib.util
import sys
import types
import unittest
from pathlib import Path

names=('rclpy','rclpy.node','rclpy.time','rclpy.qos','nav_msgs','nav_msgs.msg','sensor_msgs','sensor_msgs.msg','tf2_ros')
for name in names:
    sys.modules[name]=types.ModuleType(name)
sys.modules['rclpy.node'].Node=object
sys.modules['rclpy.time'].Time=object
sys.modules['rclpy.qos'].qos_profile_sensor_data=object()
sys.modules['nav_msgs.msg'].Odometry=object
sys.modules['sensor_msgs.msg'].LaserScan=object
sys.modules['tf2_ros'].Buffer=sys.modules['tf2_ros'].TransformListener=object
package=Path(__file__).resolve().parents[1]/'acg138_semantic/acg138_semantic'
if not package.is_dir():package=Path(__file__).resolve().parents[1]/'install/lib/python3.12/site-packages/acg138_semantic'
pkg=types.ModuleType('sonar_worker_fixture');pkg.__path__=[str(package)]
sys.modules[pkg.__name__]=pkg
spec=importlib.util.spec_from_file_location('sonar_worker_fixture.follow_node',package/'follow_node.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class WorkerTests(unittest.TestCase):
    def test_sonar_does_not_read_scan_or_post_lidar(self):
        class Worker:
            def __init__(self):self.calls=[];self.touched=[]
            def request(self,path,value=None):
                self.calls.append(path)
                return {'follow':{'distance_source':'front_sonar'},'selected_target':{'image_valid':True}}
            def __getattr__(self,name):
                self.touched.append(name)
                raise AssertionError('sonar touched lidar dependency: '+name)
        worker=Worker();module.FollowLidar.observe(worker)
        self.assertEqual(worker.calls,['/api/state'])
        self.assertEqual(worker.touched,[])

if __name__=='__main__':unittest.main()
