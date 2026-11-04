"""Execute production callbacks with only ROS transport/clock stand-ins."""
import ast,copy,json,math,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
from acg138_frontier.core import Grid
from acg138_frontier.dynamic_map import DynamicMap
from acg138_frontier.scan_pair import ClearingScanCache

class Stamp:
    def __init__(self,nanoseconds):self.nanoseconds=nanoseconds
    @classmethod
    def from_msg(cls,m):return cls(m.sec*1000000000+m.nanosec)
    def to_msg(self):return S(sec=self.nanoseconds//1000000000,nanosec=self.nanoseconds%1000000000)

def message(x=-.607,y=-.613,width=60,height=60,stamp=1200000000):
    return S(header=S(frame_id='map',stamp=Stamp(stamp).to_msg()),
             info=S(width=width,height=height,resolution=.03,
                    origin=S(position=S(x=x,y=y,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),data=[-1]*(width*height))

class ProductionDynamicNodeTests(unittest.TestCase):
    def setUp(self):
        source=Path(__file__).parents[1]/'acg138_frontier/dynamic_map_node.py'
        tree=ast.parse(source.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        ns=dict(Node=object,copy=copy,json=json,math=math,time=time,Time=Stamp,Grid=Grid,String=lambda **kw:S(**kw))
        exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),ns)
        n=ns['DynamicMapNode'].__new__(ns['DynamicMapNode']);self.n=n
        n.layer=DynamicMap(clear_range=3.);n.session=1000000000;n.mode='mapping';n.raw=None;n.scan=None;n.pending_map=None
        n.reference=None;n.reference_stamp=0;n.dirty=False;n.publish_at=0.;n.clock=0;n.clearing=ClearingScanCache()
        n.process_ms=n.last_process_ms=0;n.status='waiting';n.get_logger=lambda:Mock()
        self.now=1300000000;n.get_clock=lambda:S(now=lambda:Stamp(self.now))
        self.maps=[];self.diags=[];n.pub=S(publish=self.maps.append);n.diag=S(publish=self.diags.append)
    def tick(self):self.n.publish_at=0.;self.n.tick()
    def test_cells_origin_size_and_stamp_publish_atomically_after_growth(self):
        self.n.map_cb(message());self.tick()
        self.n.map_cb(message(x=-.617,y=-.631,width=80,height=70));self.tick()
        out=self.maps[-1];g=self.n.layer.grid
        self.assertEqual(len(out.data),out.info.width*out.info.height)
        self.assertEqual((out.info.width,out.info.height,out.info.origin.position.x,out.info.origin.position.y),
                         (g.width,g.height,g.origin_x,g.origin_y))
        self.assertEqual(Stamp.from_msg(out.header.stamp).nanoseconds,1200000000)
        self.assertEqual(self.n.layer.resets,1)
    def test_expansion_does_not_make_old_map_fresh(self):
        self.n.map_cb(message());self.now=10000000000;self.tick()
        self.assertEqual(Stamp.from_msg(self.maps[-1].header.stamp).nanoseconds,1200000000)
    def test_previous_session_map_ignored(self):
        self.n.map_cb(message(stamp=999999999));self.tick();self.assertFalse(self.maps)
    def test_new_mapping_session_clears_history_and_previous_extent(self):
        self.n.map_cb(message());self.tick()
        self.n.session_cb(S(data=json.dumps(dict(slam_session_start_ns='2000000000',slam_mode='mapping'))))
        self.assertIsNone(self.n.layer.grid);self.assertIsNone(self.n.raw)
    def test_clock_reversal_still_resets_evidence(self):
        self.n.map_cb(message());self.tick();self.now=1250000000;self.tick()
        self.assertEqual(self.n.layer.last_reset_reason,'source_clock_reversal')
    def test_capacity_error_keeps_last_valid_map(self):
        self.n.map_cb(message());old=self.n.layer.grid
        self.n.map_cb(message(x=2000.,y=2000.))
        self.assertEqual(self.n.status,'stable_map_capacity_exceeded');self.assertIs(self.n.layer.grid,old)

if __name__=='__main__':unittest.main()
