import unittest
from concurrent.futures import Future
from types import SimpleNamespace as NS
from acg138_frontier.slam_lifecycle import SlamLifecycle

class Client:
    def __init__(self):self.ready=True;self.calls=[];self.removed=[]
    def service_is_ready(self):return self.ready
    def call_async(self, request):
        f=Future();self.calls.append((request,f));return f
    def remove_pending_request(self, f):self.removed.append(f)

class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.get=Client();self.change=Client();self.now=0
        self.s=SlamLifecycle(self.get,self.change,lambda:NS(),lambda:NS(transition=NS(id=0)))
    def state(self, value):
        self.s.tick(self.now);self.get.calls[-1][1].set_result(NS(current_state=NS(id=value)))
        self.now+=.01;self.s.tick(self.now)
    def next(self, dt=1):self.now+=dt;self.s.tick(self.now)
    def test_normal_configure_activate_observed(self):
        self.state(1);self.next();self.assertEqual(self.change.calls[-1][0].transition.id,1)
        self.change.calls[-1][1].set_result(NS(success=True));self.next();self.assertFalse(self.s.ready)
        self.now+=1;self.state(2);self.next();self.assertEqual(self.change.calls[-1][0].transition.id,3)
        self.change.calls[-1][1].set_result(NS(success=True));self.next();self.now+=1;self.state(3)
        self.assertTrue(self.s.ready)
    def test_lost_configure_reply_queries_before_retry(self):
        self.state(1);self.next();lost=self.change.calls[-1][1];self.next(5)
        self.assertTrue(lost.cancelled());self.assertEqual(self.s.timeouts,1)
        self.now+=1;self.state(2);self.next()
        self.assertEqual([q.transition.id for q,f in self.change.calls],[1,3])
    def test_lost_activate_reply_observed_active_no_reactivation(self):
        self.state(2);self.next();self.next(5);self.now+=1;self.state(3)
        self.assertTrue(self.s.ready);self.assertEqual(len(self.change.calls),1)
    def test_inflight_configuring_does_not_repeat_transition(self):
        self.state(10);self.next();self.assertEqual(len(self.change.calls),0)
    def test_get_timeout_does_not_grant_ready(self):
        self.s.tick(0);self.s.tick(2);self.assertFalse(self.s.ready)
        self.assertIn('get_reply_timeout',self.s.reason);self.assertEqual(len(self.change.calls),0)
    def test_rejected_transition_reobserves(self):
        self.state(1);self.next();self.change.calls[-1][1].set_result(NS(success=False));self.next()
        self.next();self.assertEqual(len(self.change.calls),1);self.assertEqual(len(self.get.calls),2)
    def test_reset_discards_previous_child_rpc(self):
        self.s.tick(0);f=self.get.calls[-1][1];self.s.reset()
        self.assertTrue(f.cancelled());self.assertFalse(self.s.ready);self.assertIsNone(self.s.state)
    def test_missing_service_and_finalized_never_ready(self):
        self.get.ready=False;self.s.tick(0);self.assertEqual(self.get.calls,[])
        self.get.ready=True;self.now=1;self.state(4);self.next()
        self.assertFalse(self.s.ready);self.assertEqual(self.change.calls,[])
    def test_service_exception_not_latched_forever(self):
        self.s.tick(0);self.get.calls[-1][1].set_exception(RuntimeError('DDS response lost'));self.next()
        self.now+=1;self.state(3);self.assertTrue(self.s.ready)

if __name__=='__main__':unittest.main()
