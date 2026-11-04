import math
import unittest
from acg138_frontier.exploration_motion import footprint_cells, plan_frontier


def room():
    w=80;cells=[-1]*(w*w)
    for y in range(10,70):
        for x in range(10,70):cells[y*w+x]=0
    return (w,w,.05,-2.,-2.,0.,cells)


class FootprintInflationTests(unittest.TestCase):
    def test_start_inflation_exemption_does_not_allow_body_collision(self):
        g=room();g[6][40*80+42]=100
        with self.assertRaisesRegex(ValueError,'robot_cell_blocked'):
            plan_frontier(g,(0,0,0),[],self_footprint_exemption=False)
        p=plan_frontier(g,(0,0,0),[])
        # This occupied .10..15m cell lies inside the physical body. Preserve
        # the inflation-only start mask, but never select an unexecutable route.
        self.assertTrue(p['safe'][40*80+40])
        self.assertEqual(p['path'],[])
        self.assertEqual(p['reason'],'frontiers_require_blocked_initial_motion')
        self.assertFalse(p['safe'][40*80+42])

    def test_occupied_uncertain_under_start_never_erased(self):
        for value in (26,100):
            with self.subTest(value=value):
                g=room();g[6][40*80+40]=value
                with self.assertRaisesRegex(ValueError,'robot_cell_blocked'):
                    plan_frontier(g,(0,0,0),[])

    def test_body_mask_is_oriented_and_finite(self):
        g=room()
        # Longitudinal offset is asymmetric, so quarter-turn mask must rotate.
        a=footprint_cells(g,(0,0,0));b=footprint_cells(g,(0,0,math.pi/2))
        self.assertEqual({(i%80)*80+(79-i//80) for i in a},b)
        self.assertNotEqual(a,b)
        for i in a:
            self.assertLess(abs((i%80+.5)*.05-2),.20)
            self.assertLess(abs((i//80+.5)*.05-2),.175)

    def test_rotated_source_map_and_pose_match(self):
        g=room();g[6][40*80+42]=100;p=plan_frontier(g,(0,0,.4),[])
        angle=.73;c,s=math.cos(angle),math.sin(angle)
        rotated=(*g[:3],c*g[3]-s*g[4],s*g[3]+c*g[4],angle,g[6])
        q=plan_frontier(rotated,(0,0,.4+angle),[])
        self.assertEqual(p['safe'],q['safe'])

    def test_outside_body_still_inflated_unknown_still_blocked(self):
        g=room();g[6][40*80+44]=-1
        p=plan_frontier(g,(0,0,0),[]);mask=footprint_cells(g,(0,0,0))
        for i,v in enumerate(g[6]):
            if v<0:self.assertFalse(p['safe'][i])
        # Cell .325,.025 is outside the body but within .15m of unknown square.
        i=40*80+46;self.assertNotIn(i,mask);self.assertFalse(p['safe'][i])
        self.assertTrue(p['safe'][40*80+52])

    def test_exemption_does_not_make_route_through_enclosing_wall(self):
        g=room()
        for y in range(34,47):
            for x in range(34,47):
                if x in (34,46) or y in (34,46):g[6][y*80+x]=100
        p=plan_frontier(g,(0,0,0),[])
        self.assertEqual(p['path'],[])

    def test_replanning_does_not_retain_old_exemption(self):
        g=room();g[6][40*80+42]=100
        old=plan_frontier(g,(0,0,0),[]);new=plan_frontier(g,(-.5,0,0),[])
        self.assertTrue(old['safe'][40*80+40]);self.assertFalse(new['safe'][40*80+40])

    def test_source_and_window_edge_not_cleared(self):
        g=room()
        with self.assertRaisesRegex(ValueError,'robot_cell_blocked'):
            plan_frontier(g,(-1.99,0,0),[])

    def test_invalid_parameters_rejected(self):
        for kw in ({'radius':-1},{'radius':float('nan')},{'body_width':0},{'body_front':float('inf')}):
            with self.subTest(kw=kw),self.assertRaises(ValueError):plan_frontier(room(),(0,0,0),[],**kw)

if __name__=='__main__':unittest.main()
