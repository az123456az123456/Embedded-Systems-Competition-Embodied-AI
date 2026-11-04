"""Known map evidence never disappears on same-session grid replacement."""
import random,unittest
from dataclasses import replace
from acg138_frontier.core import Grid
from acg138_frontier.dynamic_map import DynamicMap

class KnownRetention(unittest.TestCase):
 def setUp(self):
  self.g=Grid(120,120,.03,-1.8,-1.8,(-1,)*14400)
  self.m=DynamicMap(clear_range=3);self.m.bind(self.g)
 def test_slam_known_cells_survive_unknown_shrink_growth(self):
  values=list(self.g.cells)
  for x,y,v in [(.3,0,0),(.9,0,100),(-.3,.4,0)]:values[self.g.cell(x,y)]=v
  self.m.bind(replace(self.g,cells=tuple(values)))
  for g in [self.g,Grid(30,30,.03,-.45,-.45,(-1,)*900),Grid(150,140,.03,-1.813,-1.827,(-1,)*21000)]:
   self.m.bind(g)
   for x,y,v in [(.3,0,0),(.9,0,100),(-.3,.4,0)]:self.assertEqual(self.m.cells()[self.m.grid.cell(x,y)],v)
 def test_one_second_cells_survive_100_moving_bounds_and_TF_corrections(self):
  for k in range(13):
   t=1000000000+k*100000000
   self.assertTrue(self.m.observe(t,t,(0,0,0),[.9],0,.01,.05,12))
  saved={self.m.grid.xy(i):int(v) for i,v in enumerate(self.m.history.values) if v>=0}
  self.assertTrue(any(v==0 for v in saved.values()));self.assertTrue(any(v==100 for v in saved.values()))
  rng=random.Random(31103)
  for k in range(100):
   g=Grid(160,150,.03,-1.8-rng.random()*.15,-1.8-rng.random()*.15,(-1,)*24000)
   self.m.bind(g);self.m.reference_correction()
   cells=self.m.cells()
   self.assertTrue(all(cells[self.m.grid.cell(*xy)]==v for xy,v in saved.items()))
 def test_confirmed_short_observations_no_longer_lost_to_reference_correction(self):
  for k in range(3):
   t=1000000000+k*100000000;self.m.observe(t,t,(0,0,0),[1.],0,.01,.05,12)
  self.assertEqual(self.m.history.free_count,0)
  self.m.reference_correction()
  self.assertEqual(self.m.cells()[self.m.grid.cell(.3,0)],0)
 def test_new_known_obstacle_then_new_clear_evidence_still_wins(self):
  cells=list(self.g.cells);i=self.g.cell(.9,0);cells[i]=0
  self.m.bind(replace(self.g,cells=tuple(cells)))
  cells[i]=100;self.m.bind(replace(self.g,cells=tuple(cells)))
  self.assertEqual(self.m.cells()[i],100)
  for k in range(3):
   t=1000000000+k*100000000;self.m.observe(t,t,(0,0,0),[1.5],0,.01,.05,12)
  self.assertEqual(self.m.cells()[i],0)
 def test_unknown_no_return_never_become_free(self):
  for k in range(20):
   t=1000000000+k*100000000;self.m.observe(t,t,(0,0,0),[float('nan')],0,.01,.05,12)
  self.m.bind(self.g);self.assertTrue(all(v==-1 for v in self.m.cells()))
 def test_explicit_new_session_releases_known_cells(self):
  self.m.bind(replace(self.g,cells=(0,)*14400))
  self.m.grid=None;self.m.reset('mapping_session_changed');self.m.bind(self.g)
  self.assertTrue(all(v==-1 for v in self.m.cells()))
if __name__=='__main__':unittest.main()
