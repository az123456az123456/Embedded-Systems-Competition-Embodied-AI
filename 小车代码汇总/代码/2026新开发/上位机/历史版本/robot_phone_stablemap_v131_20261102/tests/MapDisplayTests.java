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
 public static void main(String[] args){
  MapViewport v=new MapViewport();NavigationMap first=map(-3.2,6.4,"a");v.update(first,1000);
  check(v.x==-3.2&&v.span==6.4);NavigationMap bigger=map(-3.23,6.46,"a");v.update(bigger,1010);
  check(v.x==-3.2&&v.span==6.4);v.update(bigger,1160);check(v.span>6.4);
  v.update(bigger,1310);double x=v.x,y=v.y,s=v.span;
  for(int i=0;i<100;i++){v.update(first,1400+i);check(v.x==x&&v.y==y&&v.span==s);}
  // Same world point must map to the same navigable cell under a changed view.
  double wx=.35,wy=.45,px=(wx-v.x)/v.span*640,py=640-(wy-v.y)/v.span*640;
  NavigationTarget a=NavigationTarget.fromScreen(first,1500,px,py,0,0,640,640,v.x,v.y,v.span);
  NavigationTarget b=NavigationTarget.fromScreen(first,1500,(wx+3.2)*100,640-(wy+3.2)*100,0,0,640,640);
  check(a.cellX==b.cellX&&a.cellY==b.cellY);
  v.update(map(-1,2,"b"),2000);check(v.x==-1&&v.span==2);
  String cells="?????"+"?...."+"?.#.."+"?...."+"?????";int[] pixels=MapDisplayFilter.pixels(cells,5,5);
  for(int row=0;row<5;row++)for(int col=0;col<5;col++){
   char c=cells.charAt(row*5+col);int g=pixels[(4-row)*5+col]&255;
   check(c=='#'?g==255:c=='?'?g==24:g>=88);
  }
  check((pixels[2*5+3]&255)>88);check(cells.charAt(12)=='#');
  System.out.println("PASS "+checks+" stable viewport, point selection and display-only Gaussian checks");
 }
}
