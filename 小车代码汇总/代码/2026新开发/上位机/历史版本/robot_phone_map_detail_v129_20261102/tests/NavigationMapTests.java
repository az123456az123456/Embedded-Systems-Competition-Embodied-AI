import cn.acg138.robotphone.*;
import org.json.*;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicInteger;

public final class NavigationMapTests {
    static final AtomicInteger activeRequests=new AtomicInteger(),peakRequests=new AtomicInteger();
    static int checks;static volatile int code=200;static volatile String response;static volatile String auth,method,path;
    static void check(boolean ok,String label){if(!ok)throw new AssertionError(label);checks++;}
    static JSONObject map(){char[] cells=new char[4096];java.util.Arrays.fill(cells,'?');cells[0]='#';cells[63*64]='.';
        return new JSONObject().put("schema",1).put("available",true).put("pose_valid",true).put("width",64).put("height",64).put("resolution_m",.1)
        .put("origin",new JSONArray(-3.2,-3.2)).put("cells",new String(cells)).put("pose",new JSONArray(0,0,Math.PI/2))
        .put("path",new JSONArray(new JSONArray(0,0),new JSONArray(1,1))).put("trajectory",new JSONArray(new JSONArray(-1,0),new JSONArray(0,0)))
        .put("goal",new JSONArray(1,1)).put("state","tracking").put("reason","").put("map_revision",21).put("session_id","boot-reset-1")
        .put("snapshot_age_ms",120).put("sensor_age_ms",20).put("updated_monotonic_ms",999999999999L);}
    static void rejected(JSONObject o,String label){boolean bad=false;try{NavigationMap.parse(o,100);}catch(IllegalArgumentException e){bad=true;}check(bad,label);}
    static class Connection extends HttpURLConnection {
        Connection(URL u){super(u);}public void connect(){}public boolean usingProxy(){return false;}public void disconnect(){if(counted){counted=false;activeRequests.decrementAndGet();}}
        private boolean counted;
        public int getResponseCode(){counted=true;int n=activeRequests.incrementAndGet();peakRequests.accumulateAndGet(n,Math::max);
            auth=getRequestProperty("Authorization");NavigationMapTests.method=getRequestMethod();path=url.getPath();return code;}
        public InputStream getInputStream(){return new ByteArrayInputStream(response.getBytes(StandardCharsets.UTF_8));}
    }
    public static void main(String[] args)throws Exception {
        if(args.length>0){
            String json=new String(java.nio.file.Files.readAllBytes(java.nio.file.Paths.get(args[0])),StandardCharsets.UTF_8);
            NavigationMap fixture=NavigationMap.parse(new JSONObject(json),android.os.SystemClock.elapsedRealtime());
            check(fixture.available&&fixture.width==64&&fixture.height==64&&fixture.cells.length()==4096,"actual RK C++ serializer fixture parsed successfully");
        }
        char[] nativeCells=new char[1000000];java.util.Arrays.fill(nativeCells,'.');nativeCells[999999]='#';
        JSONObject raster=new JSONObject().put("width",1000).put("height",1000).put("resolution_m",.02)
            .put("origin",new JSONArray(-10,-10,.2)).put("cells",new String(nativeCells));
        NavigationMap detailed=NavigationMap.parse(map().put("render_map",raster),1000);
        check(detailed.width==64&&detailed.displayWidth()==1000&&detailed.displayCells().charAt(999999)=='#',"native detail never replaces navigation geometry");
        check(detailed.raster.yaw==.2,"native grid orientation preserved");
        rejected(map().put("render_map",raster.put("width",1001)),"native grid allocation bounded");
        rejected(map().put("render_map",raster.put("width",1000).put("cells","#")),"native grid length mismatch");
        NavigationMap m=NavigationMap.parse(map(),1000);
        check(m.width==64&&m.height==64&&m.cells.charAt(0)=='#'&&m.cells.charAt(4032)=='.',"schema and original y-up cell indexing");
        check(m.pose[2]==Math.PI/2&&m.path[1][1]==1&&m.trajectory[0][0]==-1&&m.goal[0]==1,"metric pose/path/trajectory/goal preserved");
        check(m.ageMs(1100)==220&&m.fresh(2880)&&!m.fresh(2881),"age uses local receipt delta plus server age, not foreign clock");
        check(!m.fresh(999)&&m.poseFresh(2980)&&!m.poseFresh(2981),"clock reversal and stale pose rejected");
        check(!NavigationMap.parse(map().put("sensor_age_ms",Long.MAX_VALUE),100).poseFresh(101),"sensor age overflow cannot make pose fresh");
        check(!NavigationMap.parse(map().put("snapshot_age_ms",Long.MAX_VALUE),100).fresh(101),"snapshot overflow cannot make map fresh");
        rejected(map().put("schema",2),"unknown schema");rejected(map().put("width",65),"invalid dimensions");
        rejected(map().put("cells","#"),"bad cell count");rejected(map().put("cells",new String(new char[4096])),"unknown cell token");
        rejected(map().put("resolution_m",0),"zero resolution");rejected(map().put("origin",new JSONArray(Double.NaN,0)),"NaN coordinate");
        rejected(map().put("pose",new JSONArray(0,0)),"pose needs yaw");rejected(map().put("path",new JSONArray(new JSONArray(1))),"bad path point");
        rejected(map().put("snapshot_age_ms",-1),"required server freshness");rejected(map().put("session_id",""),"required map coordinate session");
        rejected(map().put("trajectory",new JSONArray(new Object[4097])),"bounded trajectory allocation");
        JSONObject unavailable=new JSONObject().put("schema",1).put("available",false).put("reason","waiting-sensors");
        check(!NavigationMap.parse(unavailable,100).available,"unavailable response requires no invented cells/pose");
        JSONObject noGoal=map();noGoal.remove("goal");check(NavigationMap.parse(noGoal,100).goal==null,"no goal supported");
        char[] free=new char[4096];java.util.Arrays.fill(free,'.');
        NavigationMap freeMap=NavigationMap.parse(map().put("cells",new String(free)),1000);
        JSONObject fullRaster=new JSONObject().put("width",1000).put("height",1000).put("resolution_m",.02)
            .put("origin",new JSONArray(-10,-10,0)).put("cells",new String(nativeCells));
        NavigationMap fullMap=NavigationMap.parse(map().put("cells",new String(free)).put("render_map",fullRaster),1000);
        check(fullMap.viewSpan==20&&fullMap.viewOriginX==-10&&fullMap.width==64,"full ROS viewport independent of FPGA local crop");
        NavigationTarget fullTarget=NavigationTarget.fromScreen(fullMap,1100,170,180,10,20,330,340);
        check(fullTarget.cellX==32&&fullTarget.cellY==32,"global pixel converts into acknowledged local cell");
        boolean outsideWindow=false;
        try{NavigationTarget.fromScreen(fullMap,1100,250,180,10,20,330,340);}catch(IllegalArgumentException e){outsideWindow=e.getMessage().contains("6.4");}
        check(outsideWindow,"full-map point outside FPGA window is rejected, never clipped");
        for(int y=0;y<64;y+=7)for(int x=0;x<64;x+=5) {
            NavigationTarget t=NavigationTarget.fromScreen(freeMap,1100,10+(x+.5)*5,20+(63-y+.5)*5,10,20,330,340);
            check(t.cellX==x&&t.cellY==y&&Math.abs(t.x-(-3.2+(x+.5)*.1))<1e-10&&Math.abs(t.y-(-3.2+(y+.5)*.1))<1e-10,"screen/world roundtrip all quadrants");
        }
        double[][] invalid={{9,200},{330,200},{200,20},{200,341},{Double.NaN,100},{100,Double.POSITIVE_INFINITY}};
        for(double[] q:invalid){boolean denied=false;try{NavigationTarget.fromScreen(freeMap,1100,q[0],q[1],10,20,330,340);}catch(IllegalArgumentException e){denied=true;}check(denied,"outside or nonfinite map touch rejected");}
        boolean stale=false;try{NavigationTarget.fromScreen(freeMap,4000,100,100,10,20,330,340);}catch(IllegalArgumentException e){stale=true;}check(stale,"stale map cannot form a destination");
        NavigationMap blocked=NavigationMap.parse(map(),1000);
        boolean denied=false;try{NavigationTarget.fromScreen(blocked,1100,12,338,10,20,330,340);}catch(IllegalArgumentException e){denied=true;}check(denied,"occupied destination blocked");
        denied=false;try{NavigationTarget.fromScreen(blocked,1100,170,180,10,20,330,340);}catch(IllegalArgumentException e){denied=true;}check(denied,"unknown destination blocked");
        char[] inflated=new char[4096];java.util.Arrays.fill(inflated,'#');
        NavigationMap inflatedMap=NavigationMap.parse(map().put("cells",new String(free)).put("navigable",new String(inflated)),1000);
        denied=false;try{NavigationTarget.fromScreen(inflatedMap,1100,170,180,10,20,330,340);}catch(IllegalArgumentException e){denied=e.getMessage().contains("太近");}check(denied,"server inflated clearance mask honoured");
        rejected(map().put("navigable","."),"bad navigable mask count");
        rejected(map().put("navigable",new String(new char[4096])),"bad navigable mask tokens");
        URL.setURLStreamHandlerFactory(p->p.equals("http")?new URLStreamHandler(){protected URLConnection openConnection(URL u){return new Connection(u);}}:null);
        response=map().toString();AtomicInteger good=new AtomicInteger(),bad=new AtomicInteger();
        NavigationMapClient client=new NavigationMapClient("http://192.168.1.1:8080","123456",new NavigationMapClient.Listener(){public void map(NavigationMap m){good.incrementAndGet();}public void unavailable(String s){bad.incrementAndGet();}});
        try {
        client.start();client.start();Thread.sleep(550);
        check(good.get()>=3&&bad.get()==0,"100ms map updates and successful actual parser delivery");
        check(peakRequests.get()==1,"duplicate start never overlaps map requests");
        check("Bearer 123456".equals(auth)&&"GET".equals(method)&&"/api/navigation/map".equals(path),"authenticated read-only map API");
        code=404;Thread.sleep(550);check(bad.get()>0,"missing navigation service is reported without motor calls");
        client.close();int count=good.get()+bad.get();Thread.sleep(650);check(good.get()+bad.get()==count,"polling stops on close");
        check(NavigationTarget.reason("avoid_selected_side_physical_clearance_insufficient").equals("当前近距不足，停车等待安全转向"),"physical proximity is a current clearance wait, not proof that no safe angle exists");
        check(NavigationTarget.reason("AVOID_SELECT").equals("避开近侧，优先较宽方向"),"selection label follows current policy instead of incorrectly turning toward the smaller reading");
        check(!NavigationTarget.reason("AVOID_TURN_CLEAR").equals("AVOID_TURN_CLEAR"),"continuous sonar turn state has a Chinese label");
        System.out.println("PASS "+checks+" navigation map parsing, bounded data, coordinate/freshness and polling checks");
        } finally {client.close();}
    }
}
