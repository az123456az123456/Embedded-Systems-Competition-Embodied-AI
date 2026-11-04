"""YOLO label co-presence must revoke motion independently of display/tracking."""
import copy
import ast
import json
from pathlib import Path
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import acg138_semantic.core as core
from acg138_semantic.core import SearchSession


def result(ms, fid, labels=('Person','NO-Safety Vest'), score=.35, epoch=1):
    return dict(demo=False,control_eligible=True,detections_pair_status='PAIR_COMPLETE',
                detection_timestamp_ms=ms,detections_frame_id=fid,source_epoch=epoch,
                # No display boxes, eye, ID or layout are needed by the voice rule.
                detections=[dict(label=label,score=score) for label in labels])


class VestStopTests(unittest.TestCase):
    def start(self, target='NO-Safety Vest'):
        s=SearchSession();s.start(target,'vest',12,100000000000,500,100.01)
        return s

    def observe(self, s, ms, fid, **kwargs):
        s.observe(result(ms,fid,**kwargs),ms/1000.+.01)
        s.tick(ms/1000.+.01,executor_ready=True)

    def test_pair_over_half_second_revokes_motion_before_stop_ack(self):
        s=self.start()
        for fid,ms in enumerate((100100,100300,100500,100600),1):
            self.observe(s,ms,fid)
            self.assertNotEqual(s.state,'STOPPING')
            self.assertTrue(s.gate(ms/1000.+.01)['allow_motion'])
        self.observe(s,100601,5)
        self.assertEqual(s.state,'STOPPING')
        self.assertFalse(s.gate(100.62)['allow_motion'])
        self.assertEqual(s.match['labels'],['Person','NO-Safety Vest'])
        self.assertFalse(s.status(100.62,True)['found'])
        s.tick(100.7,stopped=True);s.tick(101.21,stopped=True)
        self.assertEqual(s.state,'FOUND')
        self.assertFalse(s.gate(102.)['allow_motion'])

    def test_either_label_alone_or_positive_vest_never_stops(self):
        for labels in (('Person',),('NO-Safety Vest',),('Person','Safety Vest')):
            with self.subTest(labels=labels):
                s=self.start()
                for fid,ms in enumerate(range(100100,101301,100),1):
                    self.observe(s,ms,fid,labels=labels)
                self.assertEqual(s.state,'SEARCHING');self.assertIsNone(s.match)

    def test_missing_pair_restarts_confirmation(self):
        s=self.start()
        for fid,ms in enumerate((100100,100300,100500),1):self.observe(s,ms,fid)
        self.observe(s,100600,4,labels=('Person',))
        for fid,ms in enumerate((100700,100900,101100,101200),5):
            self.observe(s,ms,fid);self.assertIsNone(s.match)
        self.observe(s,101201,9);self.assertEqual(s.state,'STOPPING')

    def test_duplicate_result_and_time_alone_do_not_confirm(self):
        s=self.start();self.observe(s,100100,1)
        for now in (100.2,100.4,100.7):
            s.observe(result(100100,1),now);s.tick(now,executor_ready=True)
            self.assertIsNone(s.match)

    def test_invalid_or_untrusted_result_never_stops(self):
        changes=[dict(demo=True),dict(control_eligible=False),dict(detections_pair_status='PAIR_PARTIAL'),
                 dict(detections=[dict(label='Person',score=True),dict(label='NO-Safety Vest',score=.9)]),
                 dict(source_epoch=None),dict(detections_frame_id=0)]
        for change in changes:
            s=self.start()
            for fid,ms in enumerate(range(100100,101301,100),1):
                m=result(ms,fid);m.update(change);s.observe(m,ms/1000.+.01)
            self.assertIsNone(s.match)

    def test_low_score_and_stale_results_cannot_confirm(self):
        for score,lag in ((.349,.01),(.9,1.01)):
            s=self.start()
            for fid,ms in enumerate(range(100100,101301,100),1):
                s.observe(result(ms,fid,score=score),ms/1000.+lag)
            self.assertIsNone(s.match)

    def test_epoch_and_long_gap_restart_timer(self):
        for stamps,epochs in (((100100,100300,100500,100700),(1,2,3,4)),
                              ((100100,100300,101400,101600),(1,1,1,1))):
            s=self.start()
            for fid,(ms,epoch) in enumerate(zip(stamps,epochs),1):
                self.observe(s,ms,fid,epoch=epoch)
            self.assertIsNone(s.match)

    def test_pre_task_result_cannot_start_half_second_timer(self):
        s=self.start()
        s.observe(result(99900,1),100.02)
        for fid,ms in enumerate((100100,100300,100500,100600),2):self.observe(s,ms,fid)
        self.assertIsNone(s.match)

    def test_display_and_box_motion_do_not_affect_label_presence(self):
        s=self.start()
        for fid,ms in enumerate((100100,100300,100500,100601),1):
            m=result(ms,fid);m.update(width=1000,height=600,display_detections=[])
            for d in m['detections']:
                d.update(id=fid,eye=fid%2,box=[fid*60,10,fid*60+20,40],selectable=False)
            s.observe(m,ms/1000.+.01)
        self.assertEqual(s.state,'STOPPING')

    def test_display_hold_and_partial_results_do_not_count(self):
        s=self.start()
        for fid,ms in enumerate(range(100100,101301,100),1):
            m=result(ms,fid,labels=())
            m['display_detections']=copy.deepcopy(result(ms,fid)['detections'])
            m['partial_detections']=copy.deepcopy(m['display_detections'])
            s.observe(m,ms/1000.+.01)
        self.assertIsNone(s.match)

    def test_pair_does_not_override_a_different_search_target(self):
        s=self.start('NO-Mask')
        for fid,ms in enumerate(range(100100,101301,100),1):self.observe(s,ms,fid)
        self.assertIsNone(s.match)

    def test_production_tick_publishes_closed_gate_and_requests_executor_stop(self):
        # Exercise the real coordinator tick without importing ROS or sending
        # any command to hardware. This checks observation -> gate -> stop.
        source=Path(core.__file__).with_name('node.py').read_text()
        cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef))
        clock=S(value=100.02);logger=Mock()
        ns=dict(Node=object,time=S(monotonic=lambda:clock.value),json=json,
                String=lambda **kwargs:S(**kwargs),Trigger=S(Request=lambda:S()))
        exec(compile(ast.Module(body=[cls],type_ignores=[]),'<production semantic node>','exec'),ns)
        n=ns['SemanticSearch'].__new__(ns['SemanticSearch'])
        n.session=self.start();n.lock=threading.Lock();n.session_lock=threading.RLock()
        n.latest_meta=None;n.stop_requested_task='';n.logged_state=None;n.dry_run=False
        n.stopped=lambda now:False;n.get_logger=lambda:logger
        gates=[];states=[]
        n.gate_pub=S(publish=lambda m:gates.append(json.loads(m.data)))
        n.state_pub=S(publish=lambda m:states.append(json.loads(m.data)))
        n.stop_client=S(service_is_ready=lambda:True,call_async=Mock())
        for fid,ms in enumerate((100100,100300,100500,100601),1):
            clock.value=ms/1000.+.01
            n.latest_meta=result(ms,fid)
            n.exploration=dict(gateway_generation='12',active=True,semantic_external=True)
            n.exploration_received=clock.value;n.tick()
            if fid<4:
                self.assertTrue(gates[-1]['allow_motion']);n.stop_client.call_async.assert_not_called()
        self.assertEqual(states[-1]['state'],'STOPPING')
        self.assertFalse(gates[-1]['allow_motion'])
        n.stop_client.call_async.assert_called_once()
        n.stopped=lambda now:True
        for now in (100.7,101.21):
            clock.value=now;n.tick()
        self.assertEqual(states[-1]['state'],'FOUND')
        self.assertFalse(gates[-1]['allow_motion'])
        n.stop_client.call_async.assert_called_once()


if __name__=='__main__':unittest.main()
