import cn.acg138.robotphone.FollowPresentation;
import org.json.JSONObject;

/** Parsing and wording tests use only RK-provided geometry, never local steering math. */
public final class FollowPresentationTests {
    static int checks;
    static void check(boolean ok,String text){if(!ok)throw new AssertionError(text);checks++;}
    static JSONObject target(long id){return new JSONObject().put("id",id).put("eye",0).put("u_px",480.5).put("v_px",600).put("angle_valid",true).put("turn_angle_deg",10);}
    public static void main(String[] args) {
        check(FollowPresentation.reason("lease_expired").contains("RK 未及时收到运动指令"),"backend motion lease is distinguished from removed phone HTTP ACK-age timer");
        check(FollowPresentation.reason("motor_link_stale").contains("RK 与 FPGA"),"hardware motor link cause is identified without conflating Wi-Fi ACK latency");
        JSONObject state=new JSONObject().put("selected_target",target(42));
        check(FollowPresentation.target(state,0).isEmpty(),"no selected target means no geometry");
        check(FollowPresentation.target(state,41).equals("目标 #41 · 等待角度"),"old target cannot provide current bearing");
        check(FollowPresentation.target(state,42).contains("右转 10.0°"),"positive angle is right");
        state.optJSONObject("selected_target").put("turn_angle_deg",-10);
        check(FollowPresentation.target(state,42).contains("左转 10.0°"),"negative angle is left");
        state.optJSONObject("selected_target").put("turn_angle_deg",0);
        check(FollowPresentation.target(state,42).contains("正前方"),"zero received angle displayed as front");
        state.optJSONObject("selected_target").put("turn_angle_deg",Double.NaN).put("u_px",Double.POSITIVE_INFINITY);
        String invalid=FollowPresentation.target(state,42);
        check(!invalid.contains("NaN")&&!invalid.contains("Infinity")&&!invalid.contains("正前方"),"invalid numbers cannot become a usable bearing");
        check(FollowPresentation.beforeFollow(new JSONObject(),42).isEmpty(),"old RK API remains compatible");
        state.put("follow_geometry",new JSONObject().put("configured",false));
        check(FollowPresentation.beforeFollow(state,42).contains("镜头尚未标定"),"unconfigured service cannot start calibrated follow");
        state.optJSONObject("follow_geometry").put("configured",true).put("lookahead_m",1.0);
        state.optJSONObject("selected_target").put("angle_valid",false).put("geometry_reason","outside_calibrated_span");
        check(FollowPresentation.beforeFollow(state,42).contains("超出已标定视角"),"outside calibration blocks start");
        String[] faults={"calibration_required","bearing_outside_calibration","angle_protocol_waiting","obstacle_front","obstacle_side","sonar_stale","yaw_history_missing","view_not_supported","target_lost","target_reached","vision_stale","imu_stale","bearing_invalid","camera_uncalibrated","follow_geometry_invalid","follow_geometry_disabled","outside_calibrated_span","bearing_too_large","turn_too_large","invalid_camera_eye","nonfinite_or_invalid_bearing_input"};
        for(String f:faults){check(FollowPresentation.stoppedReason(f),"fault cancels follow: "+f);check(!FollowPresentation.reason(f).equals(f),"fault has Chinese reason: "+f);}
        check(!FollowPresentation.stoppedReason("following")&&!FollowPresentation.stoppedReason(null),"healthy or absent reason does not stop follow");
        state.put("motor",new JSONObject().put("ack_protocol",2));
        String diag=FollowPresentation.diagnostics(state,42);
        check(diag.contains("角度控制")&&diag.contains("不是目标实测距离"),"diagnostics separate virtual lookahead from distance measurement");
        state.optJSONObject("motor").put("ack_protocol",1);
        check(FollowPresentation.diagnostics(state,42).contains("旧版轮速控制"),"legacy ACK identified");
        state.optJSONObject("motor").put("angle_protocol_supported",true);
        check(FollowPresentation.diagnostics(state,42).contains("角度控制")&&!FollowPresentation.diagnostics(state,42).contains("旧版"),"latest MOT1 ACK does not erase confirmed MOT2 capability");
        state.optJSONObject("motor").put("ack_protocol",2).put("angle_protocol_supported",false);
        check(FollowPresentation.diagnostics(state,42).contains("角度控制未确认"),"explicit current capability overrides a historical ACK number");
        state.put("follow",new JSONObject().put("active",true).put("distance_valid",true).put("distance_m",.42).put("status","follow_holding_distance"));
        check(FollowPresentation.target(state,42).contains("0.42 m")&&FollowPresentation.target(state,42).contains("持续跟随"),"ranged distance and continuous hold displayed");
        for(String code:new String[]{"follow_wait_target","follow_wait_lidar","follow_wait_range","follow_wait_sonar","follow_wait_motor","follow_wait_imu","follow_wait_command","follow_holding_distance","follow_backing","follow_aligning","follow_obstacle_front","follow_obstacle_side","follow_obstacle_rear"}) {
            check(!FollowPresentation.stoppedReason(code),"transient hold keeps follow task: "+code);
            check(!FollowPresentation.reason(code).equals(code),"follow state translated: "+code);
        }
        state.optJSONObject("follow_geometry").put("configured",false).put("image_servo_supported",true);
        check(FollowPresentation.beforeFollow(state,42).isEmpty(),"pixel servo has no intrinsic calibration gate");
        check(FollowPresentation.diagnostics(state,0).contains("多个候选"),"approximate association limits are explained");
        check(!FollowPresentation.stoppedReason("follow_wait_image"),"temporary image delay preserves follow");
        state.optJSONObject("follow").put("distance_source","front_sonar");
        check(FollowPresentation.target(state,42).contains("前超声 0.42 m"),"front echo source is visible beside distance");
        check(FollowPresentation.diagnostics(state,42).contains("前超声波测距"),"help identifies actual front sonar distance source");
        check(FollowPresentation.reason("follow_wait_lidar").contains("障碍检查"),"lidar wait is clearance, not target ranging");
        System.out.println("PASS "+checks+" follow presentation checks");
    }
}
