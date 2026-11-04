import json,math,unittest
from pathlib import Path
from acg138_frontier.exploration_motion import (unobserved_body_cells,footprint_cells,
    plan_frontier,route_clear_latest,body_motion_block)

def sample():
    d=json.loads((Path(__file__).parent/'fixtures/unknown_under_body_20261103.json').read_text());m=d['map']
    return (m['width'],m['height'],m['resolution'],*m['origin'],tuple(m['cells'])),d['pose']

def room(res=.03):
    w=100;return (w,w,res,-w*res/2,-w*res/2,0.,[0]*w*w)

def cell(g,x,y):return math.floor((y-g[4])/g[2])*g[0]+math.floor((x-g[3])/g[2])

class SelfUnknownTests(unittest.TestCase):
    def test_actual_saved_map_plans_without_changing_unknown_cell(self):
        g,pose=sample();before=tuple(g[6]);p=plan_frontier(g,pose,[])
        self.assertEqual(p['self_unknown_cells'],1);self.assertTrue(p['path'])
        cursor=1
        if len(p['path'])>2 and math.dist(p['path'][1],pose[:2])<.08:cursor+=1
        target=p['path'][cursor];a=math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2]
        rot=abs(math.atan2(math.sin(a),math.cos(a)))>.35
        self.assertTrue(route_clear_latest(g,pose,p['path'],cursor))
        self.assertIsNone(body_motion_block(g,pose,target,rot))
        self.assertEqual(g[6],before);self.assertEqual(p['grid'][6],before)
        self.assertTrue(all(before[i]==-1 for i in unobserved_body_cells(g,pose)))
    def test_real_obstacle_and_uncertain_under_body_always_block(self):
        for v in (26,100):
            g=room();g[6][cell(g,.06,0)]=v
            self.assertEqual(unobserved_body_cells(g,(0,0,0)),set())
            self.assertEqual(body_motion_block(g,(0,0,0),(.1,0),False)['kind'],'occupied')
            self.assertEqual(body_motion_block(g,(0,0,0),(0,.1),True)['kind'],'occupied')
    def test_center_unknown_exempt_but_partial_edge_unknown_not_exempt(self):
        g=room();center=cell(g,0,0);edge=cell(g,.155,0)
        g[6][center]=g[6][edge]=-1
        self.assertEqual(unobserved_body_cells(g,(0,0,0)),{center})
        self.assertEqual(body_motion_block(g,(0,0,0),(.1,0),False)['cell'],[edge%g[0],edge//g[0]])
    def test_external_unknown_still_blocks_rotation_translation_and_live_route(self):
        for x,y,rot in ((.22,0,True),(.22,0,False)):
            g=room();i=cell(g,x,y);g[6][i]=-1
            self.assertNotIn(i,unobserved_body_cells(g,(0,0,0)))
            self.assertEqual(body_motion_block(g,(0,0,0),(.2,0),rot)['kind'],'unknown')
            self.assertFalse(route_clear_latest(g,(0,0,0),[(0,0),(.4,0)],1))
    def test_mask_does_not_follow_old_world_cells_after_motion(self):
        g=room();i=cell(g,.06,0);g[6][i]=-1
        self.assertIn(i,unobserved_body_cells(g,(0,0,0)))
        pose=(-.22,0,0)
        self.assertNotIn(i,unobserved_body_cells(g,pose))
        self.assertEqual(body_motion_block(g,pose,(.3,0),False)['kind'],'unknown')
    def test_rotated_source_and_pose_have_same_exemption(self):
        g,pose=sample();a=.73;c,s=math.cos(a),math.sin(a)
        q=(*g[:3],c*g[3]-s*g[4],s*g[3]+c*g[4],a,g[6]);p=(c*pose[0]-s*pose[1],s*pose[0]+c*pose[1],pose[2]+a)
        self.assertEqual(unobserved_body_cells(g,pose),unobserved_body_cells(q,p))
        self.assertEqual(plan_frontier(g,pose,[])['safe'],plan_frontier(q,p,[])['safe'])
    def test_coarse_cells_crossing_body_boundary_not_exempt(self):
        for res in (.02,.03,.05,.1,.25):
            g=room(res);i=cell(g,0,0);g[6][i]=-1
            mask=unobserved_body_cells(g,(0,0,0))
            self.assertEqual(i in mask,res<=.1)
    def test_disabling_footprint_exemption_retains_strict_start_rejection(self):
        g,pose=sample()
        with self.assertRaisesRegex(ValueError,'robot_cell_blocked'):
            plan_frontier(g,pose,[],self_footprint_exemption=False)

if __name__=='__main__':unittest.main()
