package cn.acg138.robotphone;

import org.json.JSONObject;

/** Optional extension of the existing /api/state navigation object. */
public final class MappingPresentation {
    private MappingPresentation() { }
    public static JSONObject mapping(JSONObject state) {
        JSONObject nav = state == null ? null : state.optJSONObject("navigation");
        return nav == null ? null : nav.optJSONObject("mapping");
    }
    public static boolean supported(JSONObject state) {
        JSONObject nav = state == null ? null : state.optJSONObject("navigation");
        return nav != null && nav.optBoolean("supports_exploration_mapping", false)
                && nav.optInt("exploration_protocol_version", 0) == 1;
    }
    public static boolean busy(JSONObject state) {
        JSONObject m = mapping(state);
        if (m == null) return false;
        String s = m.optString("state", "");
        return m.optBoolean("active", false) || "STOPPING".equals(s)
                || "SAVING".equals(s) || "LOCALIZING".equals(s) || "LOADING".equals(s)
                || ("LOADED".equals(s) && !confirmedLoaded(state));
    }
    public static boolean confirmedLoaded(JSONObject state) {
        JSONObject m=mapping(state);
        return supported(state)&&m!=null&&"LOADED".equals(m.optString("state"))
                && !m.optBoolean("active",true)&&m.optBoolean("map_commit_confirmed",false);
    }
    public static boolean loadFailed(JSONObject state) {
        JSONObject m=mapping(state);
        return m!=null && "ERROR".equals(m.optString("state"))
                && m.optBoolean("saved_map_available",false) && !m.optBoolean("map_commit_confirmed",false);
    }
    public static boolean loaded(JSONObject state, NavigationMap map) {
        JSONObject m = mapping(state);
        // A locally drawn map or an accepted HTTP request is not a COMMIT ACK.
        return confirmedLoaded(state) && m != null && map != null && map.available && map.selectionReady
                && "LOADED".equals(m.optString("state")) && !m.optBoolean("active")
                && m.optBoolean("map_commit_confirmed", false)
                && map.sessionId.equals(m.optString("loaded_map_session", ""))
                && map.revision == m.optLong("loaded_map_revision", -1);
    }
    private static String reason(String code) {
        switch(code) {
            case "waiting_fresh_pose": return "定位数据更新";
            case "waiting_fresh_full_scan": case "waiting_lidar_scan": return "雷达数据更新";
            case "waiting_map": return "雷达地图";
            case "robot_cell_blocked_after_020m_inflation": return "当前位置在膨胀障碍内";
            case "rotation_clearance_blocked": return "周围空间不足，已停车";
            case "front_obstacle_replan": return "前方障碍，重新选路";
            case "frontiers_require_blocked_initial_motion": return "已找到目标，但起步转向或车身前方空间未通过检查，正在等待地图更新";
            case "no_reachable_frontier_waiting_map": return "暂无可达探索边界";
            case "frontier_planning_budget_exceeded": return "地图选路计算超时，正在重算";
            default: return code;
        }
    }
    public static String status(JSONObject state) {
        JSONObject m = mapping(state);
        if (m == null) return "等待 RK 确认探索状态";
        switch (m.optString("state", "")) {
            case "EXPLORING": return "探索请求已接受，等待运动指令";
            case "LOCALIZING": return "地图已保存，正在确认定位";
            case "STARTING": return "正在启动探索建图";
            case "SELECTING": return "探索建图 · 正在选择目标";
            case "NAVIGATING": return m.optBoolean("drive_command_acknowledged",false)
                    ? "Frontier 探索 · 运动指令已确认" : "Frontier 探索 · 等待控制确认";
            case "WAITING": return "探索等待：" + reason(m.optString("reason", ""));
            case "MAPPING": return "探索请求已接受，正在准备";
            case "ARRIVED": return "探索建图 · 已到达，准备下一目标";
            case "STOPPING": return "正在停车，等待结束建图";
            case "SAVING": return "已停车，正在保存地图";
            case "LOADING": return "已停车，正在载入 FPGA";
            case "LOADED": return confirmedLoaded(state)?"地图已载入 FPGA":"地图处理完成，等待载入核对";
            case "NO_FRONTIER": return "探索等待：" + reason(m.optString("reason", "no_reachable_frontier_waiting_map"));
            case "ERROR":
                if(loadFailed(state)) {
                    String why=m.optString("reason", "");
                    return why.contains("MAP_BEGIN_STOP_NOT_CONFIRMED")
                            ? "地图已保存；FPGA 未确认停稳，未载入。保持停车，点击结束建图重试"
                            : "地图已保存但未载入 FPGA，请点击结束建图重试："+why;
                }
                return "探索建图失败：" + m.optString("reason", "请检查 RK 状态");
            case "IDLE": return m.optBoolean("slam_ready",false)
                    ? "探索已停止 · 可结束建图保存地图" : "点击探索建图，启动建图与探索";
            default: return "等待 RK 确认探索状态";
        }
    }
}
