package cn.acg138.robotphone;

import org.json.JSONArray;
import org.json.JSONObject;
import java.util.Locale;

/** Display only: SNS1 diagnostics retain an accepted observation, never raw UART. */
public final class SonarPresentation {
    private static final long MAX_STATE_AGE_MS=1000;
    private SonarPresentation() { }
    public static String line(JSONObject state,boolean connected,long receivedAt,long now) {
        JSONObject navigation=state==null?null:state.optJSONObject("navigation");
        JSONObject health=navigation==null?null:navigation.optJSONObject("sonar_health");
        return "L:"+range(health,"left",1,connected,receivedAt,now)+
               "  F:"+range(health,"front",0,connected,receivedAt,now)+
               "  R:"+range(health,"right",2,connected,receivedAt,now);
    }
    private static JSONObject channel(JSONObject health,String name,int id) {
        JSONArray channels=health==null?null:health.optJSONArray("channels");
        if(channels==null)return null;
        for(int i=0;i<Math.min(3,channels.length());i++) {
            JSONObject c=channels.optJSONObject(i);
            if(c!=null&&name.equals(c.optString("name"))&&number(c,"channel")==id)return c;
        }
        return null;
    }
    private static double number(JSONObject value,String key) {
        Object raw=value==null?null:value.opt(key);
        if(!(raw instanceof Number))return Double.NaN;
        double n=((Number)raw).doubleValue();return Double.isFinite(n)?n:Double.NaN;
    }
    private static String range(JSONObject health,String name,int id,
                                boolean connected,long receivedAt,long now) {
        if(!connected)return "—";
        JSONObject c=channel(health,name,id);if(c==null)return "—";
        String status=c.optString("status","");
        if(status.equals("not_received")||status.equals("not_observed")||!health.optBoolean("packet_received"))return "—";
        if(receivedAt<=0||now<receivedAt||now-receivedAt>MAX_STATE_AGE_MS)return "过期";
        if(status.equals("packet_expired")||status.equals("source_expired"))return "过期";
        if(status.equals("fpga_invalid")||status.equals("host_time_invalid")||status.equals("outside_configured_range"))return "无效";
        if(!status.equals("valid"))return "—";
        if(!c.optBoolean("navigation_usable")||!c.optBoolean("valid")||c.optBoolean("fault"))return "无效";
        // Gateway.decorate obtains navigation_state live, separately from the
        // dashboard cache. Its age must not be added a second time here.
        double elapsed=now-receivedAt;
        double packetAge=number(health,"packet_age_ms"),sourceAge=number(c,"total_age_ms");
        double packetLimit=number(health,"navigation_packet_timeout_ms"),sourceLimit=number(health,"navigation_source_timeout_ms");
        // Keep presentation ages finite even if a malformed configuration arrives.
        if(!Double.isFinite(packetAge)||packetAge<0||!Double.isFinite(sourceAge)||sourceAge<0||
           !Double.isFinite(packetLimit)||packetLimit<=0||!Double.isFinite(sourceLimit)||sourceLimit<=0)return "过期";
        if(packetAge+elapsed>Math.min(500,packetLimit)||sourceAge+elapsed>Math.min(1000,sourceLimit))return "过期";
        double mm=number(c,"last_accepted_mm");
        if(!Double.isFinite(mm)||mm<30||mm>=65535||mm!=Math.rint(mm))return "—";
        return mm>4000?">4m":String.format(Locale.US,"%.2fm",mm/1000.);
    }
}
