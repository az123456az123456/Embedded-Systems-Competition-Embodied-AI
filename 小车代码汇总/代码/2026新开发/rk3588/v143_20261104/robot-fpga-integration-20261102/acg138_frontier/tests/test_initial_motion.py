import json,math,unittest
from pathlib import Path
from acg138_frontier.exploration_motion import plan_frontier,body_motion_clear,body_motion_block,route_clear_latest

def sample():
    d=json.loads((Path(__file__).parent/'fixtures/blocked_initial_turn.json').read_text());m=d['map']
    g=(m['width'],m['height'],m['resolution_m'],*m['origin'],tuple({'.':0,'#':100,'?':-1}[v] for v in m['cells']))
    return g,d['pose'],d['old_path']

def initial(pose,path):
    cursor=1
    if len(path)>2 and math.dist(path[1],pose[:2])<.08:cursor+=1
    target=path[cursor];angle=math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2]
    return cursor,target,abs(math.atan2(math.sin(angle),math.cos(angle)))>.35

class InitialMotionTests(unittest.TestCase):
    def test_saved_failure_keeps_obstacle_but_selects_executable_alternative(self):
        g,pose,old=sample();cur,t,rot=initial(pose,old)
        self.assertTrue(route_clear_latest(g,pose,old,cur));self.assertTrue(rot)
        self.assertFalse(body_motion_clear(g,pose,t,rot))
        p=plan_frontier(g,pose,[]);cur,t,rot=initial(pose,p['path'])
        self.assertTrue(body_motion_clear(g,pose,t,rot));self.assertFalse(rot)
        self.assertTrue(route_clear_latest(g,pose,p['path'],cur))
        self.assertGreater(p['initial_motion_rejections'],0)
        # All input obstacles survive selection; this is not a map exemption.
        self.assertEqual(p['grid'][6],g[6])
    def test_unknown_at_turn_boundary_is_not_ignored(self):
        g,pose,old=sample();g=(*g[:6],tuple(-1 if v>25 else v for v in g[6]))
        _,t,rot=initial(pose,old);detail=body_motion_block(g,pose,t,rot)
        self.assertEqual(detail['kind'],'unknown');self.assertEqual(detail['motion'],'rotation')
        p=plan_frontier(g,pose,[])
        if p['path']:
            _,t,rot=initial(pose,p['path']);self.assertTrue(body_motion_clear(g,pose,t,rot))
    def test_obstacle_inside_body_rejects_all_initial_motions(self):
        g,pose,_=sample();cells=list(g[6]);w,h,res,ox,oy,_,_=g
        cells[math.floor((pose[1]-oy)/res)*w+math.floor((pose[0]-ox)/res)+1]=100
        p=plan_frontier((*g[:6],tuple(cells)),pose,[])
        self.assertEqual(p['path'],[]);self.assertEqual(p['reason'],'frontiers_require_blocked_initial_motion')
    def test_rotated_map_preserves_admissible_motion(self):
        g,pose,_=sample();a=.73;c,s=math.cos(a),math.sin(a)
        rg=(*g[:3],c*g[3]-s*g[4],s*g[3]+c*g[4],a,g[6])
        rp=(c*pose[0]-s*pose[1],s*pose[0]+c*pose[1],pose[2]+a)
        p=plan_frontier(rg,rp,[]);_,t,rot=initial(rp,p['path'])
        self.assertTrue(body_motion_clear(rg,rp,t,rot))
    def test_coarse_map_cursor_matches_actual_controller(self):
        for res in (.02,.03,.05,.1,.25):
            w=60;cells=tuple(0 if 8<x<52 and 8<y<52 else -1 for y in range(w) for x in range(w))
            g=(w,w,res,-w*res/2,-w*res/2,0.,cells);pose=(0.,0.,.2)
            p=plan_frontier(g,pose,[])
            if p['path']:
                _,t,rot=initial(pose,p['path']);self.assertTrue(body_motion_clear(g,pose,t,rot))

if __name__=='__main__':unittest.main()
