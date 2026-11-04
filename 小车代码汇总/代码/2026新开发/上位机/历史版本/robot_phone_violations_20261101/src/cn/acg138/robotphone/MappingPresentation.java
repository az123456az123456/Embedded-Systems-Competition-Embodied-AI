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
                || "SAVING".equals(s) || "LOADING".equals(s);
    }
    public static boolean loaded(JSONObject state, NavigationMap map) {
        JSONObject m = mapping(state);
        // A locally drawn map or an accepted HTTP request is not a COMMIT ACK.
        return supported(state) && m != null && map != null && map.available
                && "LOADED".equals(m.optString("state")) && !m.optBoolean("active")
                && m.optBoolean("map_commit_confirmed", false)
                && map.sessionId.equals(m.optString("loaded_map_session", ""))
                && map.revision == m.optLong("loaded_map_revision", -1);
    }
    public static String status(JSONObject state) {
        JSONObject m = mapping(state);
        if (m == null) return "等待 RK 确认探索状态";
        switch (m.optString("state", "")) {
            case "STARTING": return "正在启动探索建图";
            case "SELECTING": return "探索建图 · 正在选择目标";
            case "NAVIGATING": return "探索建图 · 正在探索";
            case "ARRIVED": return "探索建图 · 已到达，准备下一目标";
            case "STOPPING": return "正在停车，等待结束建图";
            case "SAVING": return "已停车，正在保存地图";
            case "LOADING": return "已停车，正在载入 FPGA";
            case "LOADED": return "地图处理完成，等待载入核对";
            case "NO_FRONTIER": return "探索已暂停，可结束建图保存地图";
            case "ERROR": return "探索建图失败：" + m.optString("reason", "请检查 RK 状态");
            case "IDLE": return "探索建图已停止";
            default: return "等待 RK 确认探索状态";
        }
    }
}
