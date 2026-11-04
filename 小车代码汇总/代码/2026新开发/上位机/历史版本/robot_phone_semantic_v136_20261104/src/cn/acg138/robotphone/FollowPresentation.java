package cn.acg138.robotphone;

import java.util.Locale;
import org.json.JSONObject;

/** Presents RK-computed geometry; the phone never converts pixels into motion. */
public final class FollowPresentation {
    private FollowPresentation() { }
    public static String reason(String code) {
        if (code == null) return "";
        switch (code) {
            case "command_timeout_30s": return "30秒未收到有效指令，电机已停用";
            case "motion_command_stale": return "控制指令暂缓，已暂停运动";
            case "arc_protocol_waiting": case "arc_protocol_lost":
            case "softcore_not_ready": case "velocity_protocol_lost": return "等待 FPGA 控制就绪，运动已暂停，使能保持";
            case "lease_expired": return "RK 未及时收到运动指令，已停车；请检查通信后重新使能";
            case "motor_link_not_ready": case "motor_link_stale": return "RK 与 FPGA 电机链路暂未就绪，运动已暂停，使能保持";
            case "owner_disconnected": return "RK 控制会话已超时，已停车";
            case "camera_uncalibrated": case "follow_geometry_disabled":
            case "calibration_required": return "镜头尚未标定，请先完成 RK 端角度标定";
            case "outside_calibrated_span": case "bearing_too_large": case "turn_too_large":
            case "bearing_outside_calibration": return "目标超出已标定视角，请重新选取目标";
            case "bearing_invalid": case "follow_geometry_invalid": case "invalid_camera_eye":
            case "nonfinite_or_invalid_bearing_input": return "目标角度无效，请检查镜头标定";
            case "angle_protocol_waiting": return "等待 FPGA 角度控制确认，请核对 v17 固件";
            case "obstacle_front": return "前方有障碍，跟随已停止";
            case "obstacle_side": return "转向侧有障碍，跟随已停止";
            case "sonar_stale": return "超声波数据未更新，跟随已停止";
            case "yaw_history_missing": return "等待与画面同步的 IMU 航向";
            case "view_not_supported": return "当前摄像头显示模式不支持角度跟随";
            case "target_lost": return "目标已消失，请重新选框并点击跟随";
            case "target_reached": return "已到目标附近，跟随已停止";
            case "vision_stale": return "摄像头信息未更新，跟随已停止";
            case "imu_stale": return "IMU 数据未更新，跟随已停止";
            default: return code;
        }
    }
    public static boolean stoppedReason(String code) {
        switch (code == null ? "" : code) {
            case "camera_uncalibrated": case "follow_geometry_disabled": case "outside_calibrated_span":
            case "bearing_too_large": case "turn_too_large": case "bearing_invalid":
            case "follow_geometry_invalid": case "invalid_camera_eye": case "nonfinite_or_invalid_bearing_input":
            case "calibration_required": case "bearing_outside_calibration":
            case "angle_protocol_waiting": case "obstacle_front": case "obstacle_side":
            case "sonar_stale": case "yaw_history_missing": case "view_not_supported":
            case "target_lost": case "target_reached": case "vision_stale": case "imu_stale":
                return true;
            default: return false;
        }
    }
    public static String target(JSONObject state, long selected) {
        if (selected <= 0) return "";
        String title = "目标 #" + selected;
        JSONObject target = state.optJSONObject("selected_target");
        if (target == null || target.optLong("id", 0) != selected) return title + " · 等待角度";
        int eye = target.optInt("eye", -1);
        title += eye == 0 ? " · 左目" : eye == 1 ? " · 右目" : "";
        double u = target.optDouble("u_px", Double.NaN);
        if (finite(u) && u >= 0) title += String.format(Locale.US, " u=%.1f", u);
        double angle = target.optDouble("turn_angle_deg", Double.NaN);
        if (target.optBoolean("angle_valid", false) && finite(angle)) {
            title += Math.abs(angle) < .05 ? " · 正前方" : String.format(Locale.US, " · %s %.1f°", angle > 0 ? "右转" : "左转", Math.abs(angle));
        } else {
            String code = target.optString("geometry_reason", "");
            JSONObject geometry = state.optJSONObject("follow_geometry");
            if (code.isEmpty() && geometry != null) code = geometry.optString("status", "");
            title += " · " + (code.isEmpty() ? "等待角度" : reason(code));
        }
        return title;
    }
    /** Empty permits the normal backend validation. Missing fields support older RK releases. */
    public static String beforeFollow(JSONObject state, long selected) {
        if (state == null) return "";
        JSONObject geometry = state.optJSONObject("follow_geometry");
        if (geometry != null && !geometry.optBoolean("configured", false)) return reason("calibration_required");
        JSONObject target = state.optJSONObject("selected_target");
        if (target != null && target.optLong("id", 0) == selected && !target.optBoolean("angle_valid", false)) {
            String code = target.optString("geometry_reason", "");
            if (stoppedReason(code)) return reason(code);
        }
        return "";
    }
    public static String diagnostics(JSONObject state, long selected) {
        if (state == null) return "目标角度由 RK3588 计算，手机只发送选框与跟随操作。";
        JSONObject geometry = state.optJSONObject("follow_geometry");
        if (geometry == null) return "当前 RK 服务未提供角度标定信息。";
        String text = geometry.optBoolean("configured", false) ? "镜头角度：已标定" : "镜头角度：未标定";
        JSONObject motor = state.optJSONObject("motor");
        int protocol = motor == null ? 0 : motor.optInt("ack_protocol", 0);
        boolean capabilityKnown = motor != null && motor.has("angle_protocol_supported");
        boolean angleSupported = capabilityKnown ? motor.optBoolean("angle_protocol_supported", false) : protocol == 2;
        text += "；FPGA 确认协议：" + (angleSupported ? "角度控制" : capabilityKnown ? "角度控制未确认" : protocol == 1 ? "旧版轮速控制" : "等待确认");
        double distance = geometry.optDouble("lookahead_m", Double.NaN);
        if (finite(distance) && distance > 0) text += String.format(Locale.US, "\n安装偏置补偿前视距离：%.2f m（不是目标实测距离）", distance);
        if (selected > 0) text += "\n" + target(state, selected);
        return text;
    }
    private static boolean finite(double value) { return !Double.isNaN(value) && !Double.isInfinite(value); }
}
