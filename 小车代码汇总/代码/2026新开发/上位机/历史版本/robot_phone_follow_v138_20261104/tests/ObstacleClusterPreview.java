import cn.acg138.robotphone.*;
import org.json.*;
import android.content.Context;
public final class ObstacleClusterPreview {
 public static void main(String[] args)throws Exception{
  android.os.SystemClock.setElapsedRealtimeForTest(10000);
  GuiHostTests.Actions actions=new GuiHostTests.Actions();DashboardView v=new DashboardView(new Context(),actions);
  GuiHostTests.draw(v,2340,1080,null);GuiHostTests.tap(v,"navigation");v.connection(true);v.flags(false,"manual");
  char[] cells=new char[224*224];java.util.Arrays.fill(cells,'.');
  // A thin staircase wall, a compact adjacent cluster and an independent cluster.
  for(int y=50;y<180;y++){int x=165+(y/14)%2;cells[y*224+x]='#';}
  for(int y=135;y<146;y++)for(int x=65;x<77;x++)if((x-71)*(x-71)+(y-140)*(y-140)<30)cells[y*224+x]='#';
  for(int y=80;y<85;y++)for(int x=50;x<54;x++)cells[y*224+x]='#';
  JSONObject data=GuiHostTests.mapData("loaded-preview",9).put("map_commit_confirmed",true).put("view_scope","6.4m_local_window")
   .put("render_map",new JSONObject().put("width",224).put("height",224).put("resolution_m",.03)
    .put("origin",new JSONArray(-3.36,-3.36,0)).put("session","saved-slam").put("cells",new String(cells)));
  v.navigation(NavigationMap.parse(data,10000));
  v.state(new JSONObject().put("navigation",new JSONObject().put("supports_exploration_mapping",true).put("exploration_protocol_version",1)
   .put("mapping",new JSONObject().put("state","LOADED").put("active",false).put("map_commit_confirmed",true).put("loaded_map_session","loaded-preview").put("loaded_map_revision",9))));
  GuiHostTests.draw(v,2340,1080,args[0]);
 }
}
