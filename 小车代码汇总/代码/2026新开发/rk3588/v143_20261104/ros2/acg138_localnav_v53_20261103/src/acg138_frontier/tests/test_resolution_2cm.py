import math,struct,unittest
from acg138_frontier.core import Grid
from acg138_frontier.dynamic_map import DynamicMap
from acg138_frontier.exploration_motion import plan_frontier

class Resolution2cmTests(unittest.TestCase):
 def test_ros_float32_two_cm_is_accepted(self):
  r=struct.unpack('<f',struct.pack('<f',.02))[0]
  self.assertLess(r,.02)
  g=Grid(200,200,r,-2,-2,tuple(0 if 10<x<190 and 10<y<190 else -1 for y in range(200) for x in range(200)))
  d=DynamicMap();d.bind(g)
  p=plan_frontier((g.width,g.height,r,g.origin_x,g.origin_y,g.yaw,g.cells),(0,0,0),[])
  self.assertGreater(len(p['path']),1)
  self.assertTrue(all(p['safe'][g.cell(*xy)] for xy in p['path']))
 def test_wall_support_physical_gap_unchanged(self):
  for r in (.02,.05,.1):
   g=Grid(100,100,r,0,0,(0,)*10000);d=DynamicMap();d.bind(g)
   points=[(.5/r,(.5+i*.08)/r) for i in range(4)]
   cells,segments=d._wall_cells(points);self.assertEqual(segments,3);self.assertTrue(cells)
   _,segments=d._wall_cells([(.5/r,(.5+i*.12)/r) for i in range(4)]);self.assertEqual(segments,0)
 def test_two_cm_obstacle_clears_only_after_observations(self):
  g=Grid(200,200,.02,-2,-2,(0,)*40000);d=DynamicMap();d.bind(g)
  for i,r in enumerate((.7,.7,1.5,1.5,1.5)):
   self.assertTrue(d.observe(1_000_000_000+i*100_000_000,1_010_000_000+i*100_000_000,(0,0,0),[r],0,.01,.05,12))
   if i==3:self.assertEqual(d.cells()[g.cell(.7,0)],100)
  self.assertEqual(d.cells()[g.cell(.7,0)],0)
