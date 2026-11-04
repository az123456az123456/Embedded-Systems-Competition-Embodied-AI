import math,unittest
from acg138_frontier.core import Grid
from acg138_frontier.dynamic_map import DynamicMap

class WallSupportTests(unittest.TestCase):
 def setUp(self):
  self.g=Grid(240,240,.05,-6,-6,(0,)*57600);self.m=DynamicMap();self.m.bind(self.g)
 def test_four_contiguous_surface_points_rasterized(self):
  p=[(120.,100.2+i*1.6) for i in range(4)]
  c,n=self.m._wall_cells(p);self.assertEqual(n,3)
  self.assertTrue(all(y*240+120 in c for y in range(100,106)))
 def test_three_points_do_not_create_surface(self):
  self.assertEqual(self.m._wall_cells([(120,100),(120,101.6),(120,103.2)]),(set(),0))
 def test_missing_ray_breaks_surface(self):
  p=[(120.,100+i*1.6) for i in range(4)];p[1]=None
  self.assertEqual(self.m._wall_cells(p),(set(),0))
 def test_door_gap_not_bridged(self):
  p=[(120.,y) for y in [100,101.6,103.2,104.8,110,111.6,113.2,114.8]]
  c,n=self.m._wall_cells(p);self.assertEqual(n,6)
  self.assertFalse(any(y*240+120 in c for y in range(105,110)))
 def test_bent_and_range_discontinuous_points_rejected(self):
  for p in [[(120,100),(121.6,100),(121.6,101.6),(123.2,101.6)],[(120,100),(120,101.6),(124,103.2),(124,104.8)]]:
   self.assertEqual(self.m._wall_cells(p),(set(),0))
 def test_coarse_raster_no_long_segment(self):
  self.m.bind(Grid(120,120,.1,-6,-6,(0,)*14400))
  self.assertEqual(self.m._wall_cells([(60.,50+i*1.2) for i in range(4)]),(set(),0))
 def test_no_wrap_around_first_last_beams(self):
  self.assertEqual(self.m._wall_cells([(120,100),(120,101.6),None,(120,103.2),(120,104.8)]),(set(),0))
 def test_observation_history_and_clearing(self):
  # Vertical surface x=4.03m, 0.02 rad between consecutive valid rays (~8cm).
  angles=[-.03,-.01,.01,.03];ranges=[4.03/math.cos(a) for a in angles]
  for j in range(2):self.assertTrue(self.m.observe(1_000_000_000+j*100_000_000,1_010_000_000+j*100_000_000,(0,0,0),ranges,angles[0],.02,.05,12))
  d=self.m.diagnostics();self.assertEqual(d['supported_wall_segments'],3);self.assertGreater(d['supported_wall_extra_cells'],0)
  idx=self.g.cell(4.03,0.);self.assertEqual(self.m.cells()[idx],100)
  # Only newly observed free rays erase it. No timer-only deletion.
  for j in range(3):self.m.observe(1_200_000_000+j*100_000_000,1_210_000_000+j*100_000_000,(0,0,0),[5.],0.,.02,.05,12)
  self.assertEqual(self.m.cells()[idx],0)
if __name__=='__main__':unittest.main()
