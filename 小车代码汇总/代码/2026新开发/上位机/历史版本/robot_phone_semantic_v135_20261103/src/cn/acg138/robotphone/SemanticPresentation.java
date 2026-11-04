package cn.acg138.robotphone;
import org.json.JSONObject;
public final class SemanticPresentation {
    public static JSONObject state(JSONObject root) {
        JSONObject nav=root==null?null:root.optJSONObject("navigation");
        return nav==null?null:nav.optJSONObject("semantic");
    }
    public static boolean active(JSONObject root) {
        JSONObject s=state(root);return s!=null&&s.optBoolean("active",false);
    }
    public static String status(JSONObject root) {
        JSONObject s=state(root);if(s==null||s.optString("task","").isEmpty())return "";
        switch(s.optString("state","")) {
            case "WAIT_MAPPING":return "正在启动语义搜索，等待地图和 YOLO";
            case "SEARCHING":return "正在探索寻找："+label(s.optString("target",""));
            case "STOPPING":return "target_confirmed_waiting_stop".equals(s.optString("reason",""))?"已确认目标，正在停车":"搜索正在结束，等待停车确认";
            case "FOUND":return "已找到"+label(s.optString("target",""))+"，已确认停车";
            case "NOT_FOUND":return "搜索结束，未找到目标";
            case "CANCELLED":return "语义搜索已取消";
            case "ERROR":return "语义搜索停止："+reason(s.optString("reason",""));
            default:return "";
        }
    }
    public static String label(String value) {
        switch(value){
            case "NO-Mask":return "未戴口罩的人";case "NO-Hardhat":return "未戴安全帽的人";
            case "NO-Safety Vest":return "未穿反光衣的人";case "Person":return "人员";
            case "Hardhat":return "安全帽";case "Mask":return "口罩";case "Safety Vest":return "反光衣";
            case "machinery":return "机械";case "vehicle":return "车辆";default:return value;
        }
    }
    public static String reason(String value) {
        if(value.startsWith("stop_not_confirmed"))return "未取得停车确认，请检查车辆";
        if(value.equals("yolo_observation_stale"))return "YOLO 数据已过期";
        if(value.equals("exploration_owner_or_heartbeat_lost"))return "探索连接中断";
        if(value.equals("explore_lite_process_exited"))return "探索进程退出";
        if(value.equals("semantic_start_timeout"))return "地图或识别启动超时";
        return value;
    }
}
