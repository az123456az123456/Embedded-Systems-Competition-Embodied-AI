import tempfile,unittest
from pathlib import Path
from acg138_frontier.mapping_core import MappingSession,save_grid,RequestLedger,slam_parameters,current_tf_pose
from acg138_frontier.core import Grid

class MappingTests(unittest.TestCase):
    def test_live_composed_TF_keeps_measured_source_age(self):
        value=current_tf_pose(0,0,0,21390417,1000000000)
        self.assertEqual(value['source_monotonic_ns'],'978609583')
        self.assertEqual(value['xyyaw'],[0,0,0])
    def test_TF_250ms_boundary_still_enforced(self):
        self.assertIsNotNone(current_tf_pose(1,2,.1,250000000,1000000000))
        for age in (-1,250000001,251000000):
            with self.assertRaises(ValueError):current_tf_pose(1,2,.1,age,1000000000)
    def test_invalid_TF_not_filled_with_zero(self):
        with self.assertRaises(ValueError):current_tf_pose(float('nan'),0,0,1,100)
    def test_mapping_mode_preserves_existing_tuning(self):
        old={'slam_toolbox':{'ros__parameters':{'resolution':.05,'mode':'mapping'}}}
        result=slam_parameters(old,'mapping');self.assertEqual(result['slam_toolbox']['ros__parameters']['resolution'],.05)
        self.assertNotIn('map_name',old['slam_toolbox']['ros__parameters'])
    def test_localization_mode_uses_actual_saved_graph_and_pose(self):
        v=slam_parameters({'slam_toolbox':{'ros__parameters':{'mode':'mapping'}}},'localization','/data/maps/site/graph',(1,2,.1))
        p=v['slam_toolbox']['ros__parameters'];self.assertEqual(p['mode'],'localization');self.assertEqual(p['map_start_pose'],[1,2,.1]);self.assertEqual(p['map_file_name'],'/data/maps/site/graph')
    def test_missing_graph_never_fills_zero_pose(self):
        for graph,pose in ((None,(1,2,3)),('/graph',None),('/graph',(1,float('nan'),3))):
            with self.assertRaises(ValueError):slam_parameters({'slam_toolbox':{'ros__parameters':{}}},'localization',graph,pose)
    def test_repeated_request_reuses_original_reply_without_restarting(self):
        r=RequestLedger();self.assertIsNone(r.begin('start',1,1,1000,500,1001))
        r.finish(1,True,0,'accepted')
        self.assertEqual(r.begin('start',1,1,1000,500,10**12),(True,0,'accepted'))
    def test_request_replay_cannot_change_operation_or_source_time(self):
        r=RequestLedger();r.begin('start',1,1,1000,500,1001);r.finish(1,True,0,'accepted')
        with self.assertRaises(ValueError):r.begin('stop',1,1,1000,500,1001)
        with self.assertRaises(ValueError):r.begin('start',1,1,1001,500,1002)
    def test_stop_request_blocks_late_older_start(self):
        r=RequestLedger();r.begin('stop',10,2,1000,500,1001);r.finish(10,True,0,'stopping')
        with self.assertRaises(ValueError):r.begin('start',9,1,1000,500,1001)
    def test_request_exact_ttl_and_capacity(self):
        for age,valid in ((499999999,True),(500000000,False),(500000001,False),(-1,False)):
            r=RequestLedger()
            if valid:self.assertIsNone(r.begin('start',1,1,1000,500,1000+age))
            else:
                with self.assertRaises(ValueError):r.begin('start',1,1,1000,500,1000+age)
        r=RequestLedger()
        for n in range(1,20):r.begin('stop',n,n,1000,500,1001);r.finish(n,True,0,'ok')
        self.assertEqual(len(r.cache),8)
        with self.assertRaises(ValueError):r.begin('stop',1,1,1000,500,1001)
    def prepared(self):
        s=MappingSession();g=s.start(0,True);self.assertTrue(s.ready(g,10,True,20));g=s.stop(20)
        self.assertTrue(s.stopped(g,30,True));self.assertTrue(s.saved(g,40,999,True,True));self.assertTrue(s.localized(g,50,7,20,True));return s,g
    def data(self):
        return ({'source_map_stamp_ns':999,'uploaded':True,'state_crc_verified':True,'dry_run':False,'map_load_state':'FROZEN','map_revision':2,'map_epoch':1,'bitmap_crc32':123},
                {'session_id':7,'map_revision':2,'map_epoch':1,'active_crc32':123,'available':True,'state_stale':False,'stream_fault':False,'host_receipt_age_ms':10,'pose_age_ms':20})
    def test_stopped_start_only(self):
        with self.assertRaises(ValueError):MappingSession().start(0,False)
    def test_start_does_not_mean_exploring(self):
        s=MappingSession();g=s.start(0,True);self.assertEqual(s.state,'STARTING');self.assertFalse(s.ready(g,10,True,251))
    def test_stop_invalidates_late_slam_start(self):
        s=MappingSession();g=s.start(0,True);s.stop(10);self.assertFalse(s.ready(g,20,True,0));self.assertEqual(s.state,'STOPPING')
    def test_cannot_save_before_cancelled_stopped(self):
        s=MappingSession();g=s.stop(0);self.assertFalse(s.stopped(g,10,False));self.assertEqual(s.state,'STOPPING')
    def test_graph_and_grid_both_required(self):
        s=MappingSession();g=s.stop(0);s.stopped(g,1,True);self.assertFalse(s.saved(g,2,99,False,True));self.assertEqual(s.state,'SAVING')
    def test_commit_needs_ack_crc_and_new_session(self):
        for field,value in [('uploaded',False),('state_crc_verified',False),('source_map_stamp_ns',998),('bitmap_crc32',124)]:
            s,g=self.prepared();m,f=self.data();m[field]=value;self.assertFalse(s.committed(g,m,f));self.assertEqual(s.state,'LOADING')
    def test_not_loaded_by_an_old_session_or_old_callback(self):
        s,g=self.prepared();m,f=self.data();f['session_id']=6;self.assertFalse(s.committed(g,m,f));f['session_id']=7;self.assertFalse(s.committed(g-1,m,f))
    def test_loaded_only_after_actual_matching_commit(self):
        s,g=self.prepared();m,f=self.data();self.assertTrue(s.committed(g,m,f));self.assertEqual(s.state,'LOADED');self.assertEqual(s.loaded_revision,2)
    def test_expired_pose_blocks_loaded_banner(self):
        s,g=self.prepared();m,f=self.data();f['pose_age_ms']=245;self.assertFalse(s.committed(g,m,f))
    def test_timeout_never_claims_success(self):
        s=MappingSession();s.start(0,True);s.tick(30000);self.assertEqual(s.state,'ERROR')
    def test_fixed_grid_preserves_orientation_and_values(self):
        with tempfile.TemporaryDirectory() as tmp:
            g=Grid(2,2,.1,1,2,(0,100,-1,30));p=save_grid(Path(tmp)/'site',g)
            self.assertTrue((p/'map.pgm').read_bytes().endswith(bytes([205,205,254,0])))
            self.assertIn('0.1',(p/'map.yaml').read_text());self.assertIn('"cells":[0,100,-1,30]',(p/'map.json').read_text())
