"""Execute production coordinator methods; replace only ROS/process/clock I/O."""
import ast,json,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import acg138_frontier.mapping_core as core

class Node:
    def __init__(self,*a):pass
    def declare_parameter(self,name,value):return S(value=value)
    def create_publisher(self,*a):return S(publish=lambda m:self.messages.append(json.loads(m.data)))
    def create_subscription(self,*a):pass
    def create_service(self,*a):pass
    def create_client(self,*a):return S(service_is_ready=lambda:True,call_async=Mock())
    def create_timer(self,*a):pass
    def get_logger(self):return S(info=lambda m:None)

class ButtonStartTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        (Path(self.tmp.name)/'run').mkdir()
        # Compile the unchanged production class rather than a parallel lifecycle.
        source=Path(core.__file__).with_name('mapping_node.py').read_text()
        cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef))
        ns=dict(Node=Node,Path=lambda p:Path(self.tmp.name)/str(p).lstrip('/'),
                MappingSession=core.MappingSession,RequestLedger=core.RequestLedger,
                mapping_start_ready=core.mapping_start_ready,time=time,json=json,
                Buffer=lambda:None,TransformListener=lambda *a:None,
                QoSProfile=lambda **kw:None,ReliabilityPolicy=S(RELIABLE=0),DurabilityPolicy=S(TRANSIENT_LOCAL=0),
                OccupancyGrid=object,PoseWithCovarianceStamped=object,String=lambda **kw:S(**kw),Execution=S(Request=lambda:S()),
                Trigger=S(Request=lambda:S()),Pause=object,GetParameters=object,SerializePoseGraph=object,
                GetState=S(Request=object),ChangeState=S(Request=object),
                SlamLifecycle=lambda *a:S(ready=False,state=0,reason='not_started',timeouts=0,tick=lambda t:None))
        exec(compile(ast.Module(body=[cls],type_ignores=[]),'<production mapping_node.py>','exec'),ns)
        C=ns['MappingCoordinator'];self.spawn=[]
        def bridge(node):self.spawn.append('bridge');node.bridge=S(poll=lambda:None)
        def slam(node,mode):self.spawn.append(mode);node.slam=S(poll=lambda:None);node.slam_mode=mode
        C.spawn_bridge=bridge;C.spawn_slam=slam
        C.declare_parameter=lambda obj,name,value:S(value=False if name=='dry_run' else value)
        C.fresh_tf=lambda obj:(_ for _ in ()).throw(ValueError('fresh actual SLAM TF required'))
        self.node=C();self.node.messages=[]
    def tearDown(self):self.tmp.cleanup()
    def request(self,i=1):return S(request_id=i,run_generation=9,created_monotonic_ns=time.monotonic_ns(),valid_for_ms=500)
    def reply(self):return S(success=False,status=0,message='')
    def test_startup_has_no_slam_or_exploration(self):
        self.assertEqual(self.spawn,['bridge']);self.node.tick()
        m=self.node.messages[-1];self.assertTrue(m['ready']);self.assertFalse(m['slam_ready']);self.assertFalse(m['active'])
        self.assertEqual(m['reason'],'press_mapping_start');self.node.frontier_start.call_async.assert_not_called()
    def test_button_admitted_before_map_but_no_motion(self):
        r=self.node.start_service(self.request(),self.reply());self.assertTrue(r.success)
        self.assertEqual(self.spawn,['bridge']);self.node.tick();self.assertEqual(self.spawn,['bridge','mapping'])
        self.assertEqual(self.node.session.state,'STARTING');self.node.frontier_start.call_async.assert_not_called()
        self.assertFalse(self.node.messages[-1]['slam_ready'])
    def test_lifecycle_and_map_and_fresh_tf_all_required(self):
        self.node.start_service(self.request(),self.reply());self.node.tick()
        self.node.latest_map=object();self.node.tick();self.node.frontier_start.call_async.assert_not_called()
        self.node.slam_lifecycle.ready=True;self.node.tick();self.node.frontier_start.call_async.assert_not_called()
        self.node.fresh_tf=lambda:10
        self.node.tick();self.node.frontier_start.call_async.assert_called_once()
    def test_stop_before_timer_never_spawns_mapping(self):
        self.node.start_service(self.request(),self.reply());self.node.stop_service(self.request(2),self.reply())
        self.node.tick();self.assertEqual(self.spawn,['bridge']);self.node.frontier_start.call_async.assert_not_called()
    def test_duplicate_button_does_not_restart(self):
        q=self.request();self.node.start_service(q,self.reply());self.node.start_service(q,self.reply())
        self.node.tick();self.node.tick();self.assertEqual(self.spawn,['bridge','mapping'])
    def test_dead_bridge_not_admitted(self):
        self.node.bridge=S(poll=lambda:1);self.node.tick();self.assertFalse(self.node.messages[-1]['ready'])
    def test_admission_not_execution(self):
        for state in ('IDLE','LOADED','ERROR'):self.assertTrue(core.mapping_start_ready(False,True,True,state))
        for state in ('STARTING','EXPLORING','STOPPING','SAVING','LOADING'):self.assertFalse(core.mapping_start_ready(False,True,True,state))
        for args in ((True,True,True,'IDLE'),(False,False,True,'IDLE'),(False,True,False,'IDLE')):
            self.assertFalse(core.mapping_start_ready(*args))
