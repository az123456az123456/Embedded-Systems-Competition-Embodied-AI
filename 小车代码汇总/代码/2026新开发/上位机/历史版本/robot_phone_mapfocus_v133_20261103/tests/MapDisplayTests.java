import cn.acg138.robotphone.*;
import org.json.*;
public class MapDisplayTests {
 static int checks=0;
 static void check(boolean c){if(!c)throw new AssertionError("display check "+checks);checks++;}
 static NavigationMap map(double x,double span,String session){
  String cells=new String(new char[4096]).replace('\0','.');
  JSONObject m=NavigationMapTests.map().put("origin",new JSONArray(x,x)).put("cells",cells).put("resolution_m",span/64).put("session_id",session);
  return NavigationMap.parse(m,1000);
 }
 static JSONObject nativeMap(boolean committed,String session){
  String local=new String(new char[4096]).replace('\0','.');
  String full=new String(new char[700*700]).replace('\0','.');
  return NavigationMapTests.map().put("origin",new JSONArray(-2.8,-4.1)).put("cells",local).put("navigable",local)
   .put("snapshot_age_ms",0).put("sensor_age_ms",0).put("resolution_m",.1).put("session_id",committed?"fpga-42":"preview")
   .put("state",committed?"LOADED":"MAPPING").put("map_commit_confirmed",committed)
   .put("view_scope",committed?"6.4m_local_window":"global_overview")
   .put("render_map",new JSONObject().put("width",700).put("height",700).put("resolution_m",.03)
     .put("origin",new JSONArray(-10,-10,0)).put("cells",full).put("session",session));
 }
 static void committedFocus(){
  MapViewport v=new MapViewport();NavigationMap global=NavigationMap.parse(nativeMap(false,"slam-a"),1000);
  v.update(global,1000);check(v.span==21&&v.x==-10);
  NavigationMap local=NavigationMap.parse(nativeMap(true,"slam-a"),1000);
  v.update(local,1010);check(v.span==21);v.update(local,1160);check(v.span>6.4&&v.span<21);
  v.update(local,1310);check(Math.abs(v.span-6.4)<1e-9&&Math.abs(v.x+2.8)<1e-9&&Math.abs(v.y+4.1)<1e-9);
  check(local.displayWidth()==700&&local.displayHeight()==700&&local.raster.resolution==.03);
  for(int i=0;i<50;i++){v.update(local,1400+i);check(v.span==6.4&&v.x==-2.8&&v.y==-4.1);}
  double wx=.35,wy=.45;NavigationTarget t=NavigationTarget.fromScreen(local,1500,(wx-v.x)/v.span*640,
    640-(wy-v.y)/v.span*640,0,0,640,640,v.x,v.y,v.span);
  check(t.cellX==31&&t.cellY==45);check(Math.abs(t.x-wx)<1e-8&&Math.abs(t.y-wy)<1e-8);
  v.toggleOverview(local,1500);v.update(local,1800);check(v.span==21&&!v.locallyFocused());
  // A refresh of the same COMMIT must not undo the user's overview choice.
  v.update(local,1810);check(v.span==21);
  v.toggleOverview(local,1810);v.update(local,2110);check(v.span==6.4&&v.locallyFocused());
  v.update(global,2120);check(v.span==6.4); // brief ACK/metadata gap does not zoom out
  v.toggleOverview(local,2200);v.update(local,2500);check(v.span==21);
  NavigationMap next=NavigationMap.parse(nativeMap(true,"slam-a").put("map_revision",local.revision+1),2500);
  v.update(next,2510);v.update(next,2810);check(v.span==6.4&&v.locallyFocused());
  v.update(NavigationMap.parse(nativeMap(false,"slam-b"),2900),2900);check(v.span==21&&v.x==-10);
  check(!NavigationMap.parse(nativeMap(false,"s").put("state","LOADED"),1000).localWindowConfirmed);
  check(!NavigationMap.parse(nativeMap(true,"s").put("view_scope","global_overview"),1000).localWindowConfirmed);
  check(!NavigationMap.parse(nativeMap(true,"s").put("resolution_m",.2),1000).localWindowConfirmed);
  local.prepareDisplay(null);NavigationMap same=NavigationMap.parse(nativeMap(true,"slam-a").put("pose",new JSONArray(1,2,1)),1001);
  same.prepareDisplay(local);check(local.preparedDisplayPixels()==same.preparedDisplayPixels());
  JSONObject changed=nativeMap(true,"slam-a");String original=changed.optJSONObject("render_map").optString("cells","");
  changed.optJSONObject("render_map").put("cells","#"+original.substring(1));
  NavigationMap other=NavigationMap.parse(changed,1002);other.prepareDisplay(same);
  check(other.preparedDisplayPixels()!=same.preparedDisplayPixels());check(other.raster.cells.charAt(0)=='#');
  check(local.raster.cells.equals(original)); // no filtering of selection/saved map
  for(int w=1;w<=3;w++)for(int h=1;h<=3;h++)check(MapDisplayFilter.pixels(new String(new char[w*h]).replace('\0','.'),w,h).length==w*h);
 }
 public static void main(String[] args){
  committedFocus();
  MapViewport v=new MapViewport();NavigationMap first=map(-3.2,6.4,"a");v.update(first,1000);
  check(v.x==-3.2&&v.span==6.4);NavigationMap bigger=map(-3.23,6.46,"a");v.update(bigger,1010);
  check(v.x==-3.2&&v.span==6.4);v.update(bigger,1160);check(v.span==6.4);
  v.update(bigger,1310);double x=v.x,y=v.y,s=v.span;
  for(int i=0;i<100;i++){
   v.update(i%2==0?bigger:first,1400+i);check(v.x==x&&v.y==y&&v.span==s);
   check(!v.animating(1400+i));
  }
  v.toggleOverview(bigger,1500);v.update(bigger,1650);check(v.span>6.4&&v.span<6.46);
  v.update(bigger,1800);check(v.x==-3.23&&v.span==6.46);
  v.update(map(-5,10,"a"),1810);v.update(map(-5,10,"a"),2200);check(v.x==-3.23&&v.span==6.46);
  // Same world point must map to the same navigable cell under a changed view.
  double wx=.35,wy=.45,px=(wx-v.x)/v.span*640,py=640-(wy-v.y)/v.span*640;
  NavigationTarget a=NavigationTarget.fromScreen(first,1500,px,py,0,0,640,640,v.x,v.y,v.span);
  NavigationTarget b=NavigationTarget.fromScreen(first,1500,(wx+3.2)*100,640-(wy+3.2)*100,0,0,640,640);
  check(a.cellX==b.cellX&&a.cellY==b.cellY);
  v.update(map(-1,2,"b"),2300);check(v.x==-1&&v.span==2);
  String cells="?????"+"?...."+"?.#.."+"?...."+"?????";int[] pixels=MapDisplayFilter.pixels(cells,5,5);
  for(int row=0;row<5;row++)for(int col=0;col<5;col++){
   char c=cells.charAt(row*5+col);int g=pixels[(4-row)*5+col]&255;
   check(c=='#'?g==255:c=='?'?g==24:g>=88);
  }
  check((pixels[2*5+3]&255)>88);check(cells.charAt(12)=='#');
  System.out.println("PASS "+checks+" stable viewport, point selection and display-only Gaussian checks");
 }
}
