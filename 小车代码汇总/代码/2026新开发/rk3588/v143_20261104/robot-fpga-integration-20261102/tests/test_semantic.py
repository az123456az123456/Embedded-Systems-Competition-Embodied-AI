import copy,unittest,math
from acg138_semantic.core import SearchSession,TargetEvidence,parse_command,LABELS
from acg138_frontier.exploration_motion import plan_frontier

def frame(t,fid=1,label='NO-Mask',eye=0,epoch=1):
    off=eye*500
    return dict(demo=False,control_eligible=True,detections_pair_status='PAIR_COMPLETE',
        detection_timestamp_ms=round(t*1000),detections_frame_id=fid,source_epoch=epoch,width=1000,height=600,
        detections=[dict(label='Person',eye=eye,score=.9,box=[off+100,50,off+300,550]),
                    dict(label=label,eye=eye,score=.9,box=[off+150,80,off+200,130])])

CONFIRM_TIMES = tuple(100.1 + n*.2 for n in range(6))

class Tests(unittest.TestCase):
    def start(self,target='NO-Mask'):
        s=SearchSession();s.start(target,'t',12,100000000000,500,100.01);return s
    def test_supported_commands(self):
        self.assertEqual(parse_command('找到未戴口罩的人。'),'NO-Mask')
        self.assertEqual(parse_command('找到未戴安全帽的人'),'NO-Hardhat')
        self.assertEqual(parse_command('找到未穿反光衣的人'),'NO-Safety Vest')
        for label in LABELS:self.assertEqual(parse_command(label),label)
    def test_no_substring_negation(self):
        for text in ('不要找到未戴口罩的人','找到戴口罩的人','找到红色水瓶','找到未戴口罩的人然后前进',None):
            with self.assertRaises(ValueError):parse_command(text)
    def test_expired_exact_boundary(self):
        with self.assertRaises(ValueError):SearchSession().start('Person','t',1,100000000000,500,100.5)
    def test_future_command(self):
        with self.assertRaises(ValueError):SearchSession().start('Person','t',1,101000000000,500,100.)
    def test_no_implicit_start(self):self.assertFalse(SearchSession().gate(100)['allow_motion'])
    def test_camera_and_executor_required_without_mapping(self):
        s=self.start();s.tick(100.1,True);self.assertEqual(s.state,'WAIT_EXECUTOR')
        s.observe(frame(100.11),100.12);s.tick(100.13,False);self.assertEqual(s.state,'WAIT_EXECUTOR')
        s.tick(100.14,True);self.assertEqual(s.state,'SEARCHING');self.assertTrue(s.gate(100.14)['allow_motion'])
    def test_continuous_one_second_then_stop_ack(self):
        s=self.start()
        for n,t in enumerate(CONFIRM_TIMES[:-1]):
            s.observe(frame(t,n+1),t+.01)
            self.assertIsNone(s.match)
        s.observe(frame(CONFIRM_TIMES[-1],6),101.11)
        self.assertEqual(s.state,'STOPPING');self.assertFalse(s.gate(101.12)['allow_motion'])
        self.assertFalse(s.status(101.12,True)['found']);s.tick(101.15,stopped=True)
        s.tick(101.66,stopped=True);self.assertEqual(s.state,'FOUND')
    def test_300ms_target_is_not_success(self):
        s=self.start()
        for n,t in enumerate((100.1,100.26,100.42)):s.observe(frame(t,n+1),t+.01)
        self.assertIsNone(s.match);self.assertNotEqual(s.state,'STOPPING')
    def test_right_eye_can_find_target(self):
        s=self.start()
        for n,t in enumerate(CONFIRM_TIMES):s.observe(frame(t,n+1,eye=1),t+.01)
        self.assertEqual(s.match['eye'],1);self.assertEqual(s.state,'STOPPING')
    def test_duplicate_frame_does_not_confirm(self):
        s=self.start()
        for t in (100.11,100.2,100.4):s.observe(frame(100.1),t)
        self.assertEqual(s.state,'WAIT_EXECUTOR')
    def test_precommand_evidence_rejected(self):
        s=self.start()
        for i in range(3):s.observe(frame(99.8+i*.05,i+1),100.02+i*.05)
        self.assertEqual(s.evidence.last_valid,0)
    def test_partial_and_demo_rejected(self):
        for key,value in (('control_eligible',False),('demo',True),('detections_pair_status','PAIR_PARTIAL')):
            s=self.start()
            for i,t in enumerate(CONFIRM_TIMES):
                m=frame(t,i+1);m[key]=value;s.observe(m,t+.01)
            self.assertIsNone(s.match)
    def test_no_person_no_negative_confirmation(self):
        s=self.start()
        for n,t in enumerate(CONFIRM_TIMES):
            m=frame(t,n+1);m['detections']=m['detections'][1:];s.observe(m,t+.01)
        self.assertIsNone(s.match)
    def test_positive_conflict_rejected(self):
        s=self.start()
        for n,t in enumerate(CONFIRM_TIMES):
            m=frame(t,n+1);m['detections'].append(dict(label='Mask',eye=0,score=.9,box=[150,80,200,130]));s.observe(m,t+.01)
        self.assertIsNone(s.match)
    def test_source_epoch_resets_confirmation(self):
        s=self.start()
        for n,t in enumerate(CONFIRM_TIMES):s.observe(frame(t,n+1,epoch=n+1),t+.01)
        self.assertIsNone(s.match)
    def test_gap_resets_evidence(self):
        s=self.start()
        for n,t in enumerate((100.1,100.26,100.7)):s.observe(frame(t,n+1),t+.01)
        self.assertIsNone(s.match)
    def test_unrelated_person_cannot_inherit(self):
        s=self.start()
        for n,t in enumerate(CONFIRM_TIMES):
            m=frame(t,n+1)
            if n==4:
                for d in m['detections']:d['box']=[d['box'][0]+180,d['box'][1],d['box'][2]+180,d['box'][3]]
            s.observe(m,t+.01)
        self.assertIsNone(s.match)
    def test_generic_vehicle_confirm(self):
        s=self.start('vehicle')
        for n,t in enumerate(CONFIRM_TIMES):s.observe(frame(t,n+1,label='vehicle'),t+.01)
        self.assertEqual(s.state,'STOPPING')
    def test_target_disappearance_restarts_one_second(self):
        for target in ('NO-Mask','vehicle'):
            with self.subTest(target=target):
                s=self.start(target)
                for n in range(10):
                    t=100.1+n*.2;m=frame(t,n+1,label=target)
                    if n==4:m['detections']=[]
                    s.observe(m,t+.01)
                    self.assertIsNone(s.match)
                s.observe(frame(102.1,11,label=target),102.11)
                self.assertEqual(s.state,'STOPPING')
    def test_evidence_gap_cannot_count_toward_one_second(self):
        for target in ('NO-Mask','vehicle'):
            s=self.start(target)
            for n,t in enumerate((100.1,100.3,100.5,100.9,101.1,101.3)):
                s.observe(frame(t,n+1,label=target),t+.01)
            self.assertIsNone(s.match)
    def test_camera_stale_stops_and_gate_does_not_renew_it(self):
        s=self.start();s.observe(frame(100.1),100.11);s.tick(100.12,True)
        self.assertFalse(s.gate(101.11)['allow_motion']);s.tick(101.11,True);self.assertEqual(s.state,'STOPPING')
    def test_without_stop_ack_never_found(self):
        s=self.start();s.stop('FOUND','target_confirmed_waiting_stop',100.1)
        s.tick(106,stopped=False);self.assertEqual(s.state,'ERROR');self.assertFalse(s.status(106,True)['found'])
    def test_interrupted_stop_dwell_restarts(self):
        s=self.start();s.stop('FOUND','target_confirmed_waiting_stop',100.1)
        s.tick(100.2,stopped=True);s.tick(100.5,stopped=False);s.tick(100.7,stopped=True)
        s.tick(101,stopped=True);self.assertEqual(s.state,'STOPPING')
    def test_no_resume_after_cancel(self):
        s=self.start();s.stop('CANCELLED','manual_takeover',100.1);s.tick(100.2,stopped=True);s.tick(100.8,stopped=True)
        s.tick(101,mapping_ready=True);self.assertEqual(s.state,'CANCELLED');self.assertFalse(s.gate(101)['allow_motion'])
    def test_no_announcement(self):self.assertFalse(self.start().status(100.1,True)['speech_added'])
    def test_external_frontier_unknown_never_traversed(self):
        cells=[-1]*6400
        for y in range(10,70):
            for x in range(10,70):cells[y*80+x]=0
        g=(80,80,.05,-2,-2,0,tuple(cells));p=plan_frontier(g,(0,0,0),[],requested_goal=(1.5,0))
        self.assertTrue(p['path'])
        for x,y in p['path']:
            self.assertEqual(cells[math.floor((y+2)/.05)*80+math.floor((x+2)/.05)],0)
    def test_unreachable_external_goal_does_not_choose_other_frontier(self):
        g=(80,80,.05,-2,-2,0,tuple([0]*6400))
        p=plan_frontier(g,(0,0,0),[],requested_goal=(30,30));self.assertFalse(p['path'])
        self.assertEqual(p['reason'],'external_frontier_unreachable')
if __name__=='__main__':unittest.main()
