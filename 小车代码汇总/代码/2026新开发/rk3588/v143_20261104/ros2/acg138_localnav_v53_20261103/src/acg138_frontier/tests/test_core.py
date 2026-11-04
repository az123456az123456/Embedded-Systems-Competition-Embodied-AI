import unittest, math, random
from acg138_frontier.core import *

def maps():
    cells=[-1]*10000
    for y in range(10,90):
        for x in range(10,90):cells[y*100+x]=0
    g=Grid(100,100,.1,-5,-5,tuple(cells))
    l=Grid(64,64,.1,-3.2,-3.2,tuple(100 if min(i%64,i//64,63-i%64,63-i//64)<4 else 0 for i in range(4096)))
    return g,l

def identity(**kw):
    d=dict(session=1,run=2,goal="01"*16,plan=3,map_epoch=1,revision=2,localization_epoch=1,nav_epoch=7);d.update(kw);return Identity(**d)

class FrontierTests(unittest.TestCase):
    def test_outside_uses_reachable_inset(self):
        g,l=maps();s=select_frontier(g,l,(0,0));self.assertIsNotNone(s.target)
        t=s.target;self.assertTrue(t.intermediate);self.assertEqual(l.cells[t.local_cell],0)
        self.assertEqual(l.cell(t.x,t.y),t.local_cell);self.assertGreaterEqual(min(t.local_cell%64,t.local_cell//64,63-t.local_cell%64,63-t.local_cell//64),3)
    def test_no_frontier_closed_map(self):
        g,l=maps();g=Grid(100,100,.1,-5,-5,(0,)*10000)
        self.assertEqual(select_frontier(g,l,(0,0)).reason,"NO_FRONTIER")
    def test_unknown_is_not_free(self):
        g,l=maps();g=Grid(100,100,.1,-5,-5,(-1,)*10000)
        self.assertEqual(select_frontier(g,l,(0,0)).reason,"BLOCKED_START")
    def test_blocked_start_no_free_bubble(self):
        g,l=maps();l=Grid(64,64,.1,-3.2,-3.2,(100,)*4096)
        self.assertEqual(select_frontier(g,l,(0,0)).reason,"BLOCKED_START")
    def test_wrong_local_shape(self):
        g,l=maps()
        with self.assertRaises(ValueError):select_frontier(g,Grid(32,32,.2,-3.2,-3.2,(0,)*1024),(0,0))
    def test_bad_values(self):
        g,l=maps()
        with self.assertRaises(ValueError):select_frontier(Grid(100,100,.1,-5,-5,(255,)*10000),l,(0,0))
    def test_rotated_map(self):
        g,l=maps();g=Grid(100,100,.1,5,-5,g.cells,math.pi/2)
        self.assertIsNotNone(select_frontier(g,l,(0,0)).target)
    def test_exclusion(self):
        g,l=maps();s=select_frontier(g,l,(0,0));t=select_frontier(g,l,(0,0),[s.target.frontier_key])
        self.assertNotEqual(s.target,t.target)
    def test_disconnected_frontier_not_selected(self):
        g,l=maps();c=[100]*10000
        for y in range(45,56):
            for x in range(45,56):c[y*100+x]=0
        for y in range(5,15):
            for x in range(5,15):c[y*100+x]=0 if x>5 else -1
        self.assertEqual(select_frontier(Grid(100,100,.1,-5,-5,tuple(c)),l,(0,0)).reason,"NO_FRONTIER")
    def test_standoff_inside_window(self):
        g,l=maps();c=[-1]*10000
        for y in range(30,70):
            for x in range(30,70):c[y*100+x]=0
        g=Grid(100,100,.1,-5,-5,tuple(c));s=select_frontier(g,l,(0,0))
        self.assertIsNotNone(s.target);self.assertFalse(s.target.intermediate)
        self.assertEqual(g.cells[g.cell(s.target.x,s.target.y)],0)
    def test_random_obstacles_local_reachability(self):
        g,l=maps();rng=random.Random(26)
        for _ in range(20):
            c=list(l.cells)
            for i in range(4096):
                if rng.random()<.12:c[i]=100
            c[l.cell(0,0)]=0;local=Grid(64,64,.1,-3.2,-3.2,tuple(c));s=select_frontier(g,local,(0,0))
            if s.target:self.assertEqual(c[s.target.local_cell],0)

class StateTests(unittest.TestCase):
    def ready(self):
        e=Explorer();gen=e.begin(100,True);g,l=maps();s=select_frontier(g,l,(0,0));self.assertTrue(e.map_ready(gen,s,True,True));self.assertTrue(e.planned(gen,identity(),True));return e,gen
    def test_explicit_authority(self):
        with self.assertRaises(ValueError):Explorer().begin(0,False)
    def test_preview_is_not_navigation(self):
        e,n=self.ready();self.assertEqual(e.mode,Mode.SELECTING);self.assertEqual(e.phase,"WAIT_START")
    def test_normal_cycle(self):
        e,n=self.ready();self.assertTrue(e.started(n,identity(),200,True))
        self.assertEqual(e.observe(identity(),"ARRIVED",13,1,250,30),"CANCEL_AND_CONFIRM_STOP")
        self.assertFalse(e.stopped(n,False));self.assertTrue(e.stopped(n,True));self.assertEqual(e.mode,Mode.SELECTING)
    def test_stop_invalidates_delayed_start(self):
        e,n=self.ready();e.stop();self.assertFalse(e.started(n,identity(),200,True));self.assertEqual(e.mode,Mode.IDLE)
    def test_stop_invalidates_delayed_map(self):
        e=Explorer();n=e.begin(0,True);e.stop();g,l=maps();self.assertFalse(e.map_ready(n,select_frontier(g,l,(0,0)),True,True))
    def test_stale_wrong_run_state(self):
        e,n=self.ready();e.started(n,identity(),200,True)
        self.assertEqual(e.observe(identity(run=1),"ARRIVED",13,1,250,10),"IGNORE")
        self.assertEqual(e.observe(identity(),"ARRIVED",13,1,250,251),"IGNORE")
    def test_manual_cancel_never_retries(self):
        e,n=self.ready();e.started(n,identity(),200,True);self.assertEqual(e.observe(identity(),"CANCELLED",5,1,250,10),"STOP")
        self.assertFalse(e.stopped(n,True));self.assertEqual(e.mode,Mode.ERROR)
    def test_no_route_retries_after_stop(self):
        e,n=self.ready();e.started(n,identity(),200,True);self.assertEqual(e.observe(identity(),"PLANNING",3,1,250,10),"CANCEL_AND_CONFIRM_STOP")
        self.assertTrue(e.stopped(n,True));self.assertEqual(e.mode,Mode.SELECTING)
    def test_timeout(self):
        e,n=self.ready();e.started(n,identity(),200,True);self.assertEqual(e.tick(451,0),"STOP");self.assertEqual(e.reason,"STATE_STALE")
    def test_no_progress(self):
        e=Explorer();e.begin(0,True);self.assertEqual(e.tick(300000,0),"STOP")
    def test_crc_and_stop_both_needed(self):
        e=Explorer();n=e.begin(0,True);g,l=maps();s=select_frontier(g,l,(0,0))
        self.assertFalse(e.map_ready(n,s,True,False));self.assertFalse(e.map_ready(n,s,False,True))
    def test_duplicate_state_does_not_refresh(self):
        e,n=self.ready();e.started(n,identity(),200,True);e.observe(identity(),"FOLLOWING",0,1,250,10)
        self.assertEqual(e.observe(identity(),"FOLLOWING",0,1,450,10),"IGNORE");self.assertEqual(e.tick(501,0),"STOP")

if __name__=="__main__":unittest.main()
