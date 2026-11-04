import math,unittest
from acg138_frontier.recovery import Retreat,reverse_block,rear_scan_clear
from test_live_route import grid,obstacle,ProductionExplorerTests

class ReverseSafety(unittest.TestCase):
 def test_front_blocked_rear_free(self):
  g=grid();obstacle(g,.26,0)
  self.assertIsNone(reverse_block(g,(0,0,0)))
 def test_obstacle_unknown_side_and_map_edge_reject(self):
  for value in (-1,26,100):
   g=grid();obstacle(g,-.3,0,value);self.assertIsNotNone(reverse_block(g,(0,0,0)))
  g=grid();obstacle(g,-.24,.15);self.assertIsNotNone(reverse_block(g,(0,0,0)))
  self.assertEqual(reverse_block(grid(),(-1.4,0,0)),'reverse_outside_map')
 def test_reverse_rotates_with_body_not_world_x(self):
  g=grid();obstacle(g,0,-.3)
  self.assertIsNotNone(reverse_block(g,(0,0,math.pi/2)))
  self.assertIsNone(reverse_block(g,(0,0,0)))
 def test_rear_lidar_obstacle_no_echo_and_unknown_fail(self):
  args=(-math.pi,2*math.pi/360,.05,12.,0.,0.,0.)
  self.assertTrue(rear_scan_clear([4.]*360,*args))
  for value in (.3,float('nan'),float('inf')):
   rays=[4.]*360
   for i in list(range(46))+list(range(315,360)):rays[i]=value
   self.assertFalse(rear_scan_clear(rays,*args))
 def test_brake_reverse_real_distance_stop(self):
  r=Retreat();self.assertTrue(r.begin((0,0,0),10))
  self.assertEqual(r.update(grid(),(0,0,0),(0,0,0),10.1,True),(False,'reverse_brake_before_motion'))
  self.assertEqual(r.update(grid(),(0,0,0),(-.05,0,0),10.3,True),(True,'reversing_to_clear_turn'))
  self.assertEqual(r.update(grid(),(-.15,0,0),(-.15,0,0),10.8,True),(False,'reverse_complete_replan'))
 def test_map_correction_is_not_odometry_progress(self):
  r=Retreat();r.begin((0,0,0),10)
  self.assertTrue(r.update(grid(),(-.16,0,0),(0,0,0),10.3,True)[0])
  self.assertEqual(r.update(grid(),(-.16,0,0),(0,0,0),11.6,True),(False,'reverse_no_progress_stop'))
 def test_fresh_obstacle_interrupts_active_reverse(self):
  r=Retreat();r.begin((0,0,0),10);g=grid();obstacle(g,-.3,0)
  self.assertEqual(r.update(g,(0,0,0),(0,0,0),10.3,True),(False,'reverse_obstacle'))
 def test_wrong_direction_or_unexpected_turn_stops(self):
  for odom in ((.04,0,0),(-.1,.06,0),(-.1,0,.2),(-.3,0,0)):
   r=Retreat();r.begin((0,0,0),10)
   self.assertEqual(r.update(grid(),(0,0,0),odom,10.3,True),(False,'recovery_pose_deviation'))
 def test_manual_cancel_and_bounded_attempts(self):
  r=Retreat()
  for _ in range(3):self.assertTrue(r.begin((0,0,0),10));r.cancel()
  self.assertFalse(r.begin((0,0,0),10));self.assertFalse(r.active)

class RecoveryProduction(ProductionExplorerTests):
 def test_no_frontier_waiting_state_retries_recovery(self):
  self.n.plan={'path':[],'reason':'frontiers_require_blocked_initial_motion'}
  self.n.planned_map=self.n.grid_seq;self.n.state='NO_FRONTIER'
  self.n.recovery_step=lambda pose,now:(True,'reversing_to_clear_turn')
  self.n.tick();self.assertEqual(self.messages[-1]['state'],'WAITING')
  self.assertFalse(self.messages[-1]['drive']['motion_requested'])
  self.n.tick();self.assertEqual(self.messages[-1]['drive']['throttle'],-.18)
 def test_map_jump_does_not_reset_reverse_attempt_limit(self):
  self.n.retreat.attempts=3;self.n.forward_progress_pose=(0,0,0)
  self.n.pose=lambda:((.25,0,0),0)
  self.n.tick();self.assertEqual(self.n.retreat.attempts,3)
 def test_real_forward_progress_resets_reverse_attempt_limit(self):
  self.n.retreat.attempts=3;self.n.forward_progress_pose=(0,0,0)
  self.n.odom_pose=lambda:(.25,0,0)
  self.n.tick();self.assertEqual(self.n.retreat.attempts,0)
 def test_blocked_first_tick_is_zero_before_recovery(self):
  g=grid();obstacle(g,.2,.1);self.n.grid=g;self.n.checked_map=self.n.grid_seq
  self.n.recovery_step=lambda pose,now:(True,'reversing_to_clear_turn')
  self.n.tick();self.assertFalse(self.messages[-1]['drive']['motion_requested'])
  self.n.tick();self.assertEqual(self.messages[-1]['drive']['throttle'],-.18)
 def test_stale_pose_cannot_enter_reverse(self):
  self.n.retreat.begin((0,0,0),10)
  self.n.pose=lambda:(_ for _ in ()).throw(ValueError('waiting_fresh_pose'))
  self.n.tick();self.assertFalse(self.messages[-1]['drive']['motion_requested']);self.assertFalse(self.n.retreat.active)
if __name__=='__main__':unittest.main()
