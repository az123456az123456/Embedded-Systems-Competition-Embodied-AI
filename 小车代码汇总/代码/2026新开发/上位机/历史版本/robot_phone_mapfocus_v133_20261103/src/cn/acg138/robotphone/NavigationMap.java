package cn.acg138.robotphone;

import org.json.JSONArray;
import org.json.JSONObject;

/** Validated immutable schema-1 map. Server and Android monotonic clocks are never compared. */
public final class NavigationMap {
    public static final long MAX_AGE_MS = 2000;
    public final boolean available, poseValid, localWindowConfirmed;
    public final int width, height;
    public final Raster raster;
    public final double resolution, originX, originY, viewOriginX, viewOriginY, viewSpan;
    public final String cells, navigable, state, reason, sessionId;
    public final double[] pose, goal;
    public final double[][] path, trajectory;
    public final long revision, receivedAt, snapshotAgeMs, sensorAgeMs;
    private volatile int[] displayPixels;
    private NavigationMap(boolean available, boolean poseValid, int width, int height,
            double resolution, double[] origin, String cells, String navigable, String state, String reason,
            String session, long revision, long receivedAt, long snapshotAge, long sensorAge,
            double[] pose, double[] goal, double[][] path, double[][] trajectory, Raster raster, boolean localWindowConfirmed) {
        this.raster=raster;
        this.localWindowConfirmed=localWindowConfirmed;
        double minx=origin[0],miny=origin[1],span=width*resolution;
        if(raster!=null){
            minx=raster.x;miny=raster.y;double maxx=minx,maxy=miny,c=Math.cos(raster.yaw),s=Math.sin(raster.yaw);
            for(int ix=0;ix<=1;ix++)for(int iy=0;iy<=1;iy++){
                double a=ix*raster.width*raster.resolution,b=iy*raster.height*raster.resolution;
                double x=raster.x+c*a-s*b,y=raster.y+s*a+c*b;
                minx=Math.min(minx,x);maxx=Math.max(maxx,x);miny=Math.min(miny,y);maxy=Math.max(maxy,y);
            }
            span=Math.max(maxx-minx,maxy-miny);
        }
        viewOriginX=minx;viewOriginY=miny;viewSpan=span;
        this.available=available;this.poseValid=poseValid;this.width=width;this.height=height;
        this.resolution=resolution;this.originX=origin[0];this.originY=origin[1];this.cells=cells;this.navigable=navigable;
        this.state=state;this.reason=reason;this.sessionId=session;this.revision=revision;
        this.receivedAt=receivedAt;this.snapshotAgeMs=snapshotAge;this.sensorAgeMs=sensorAge;
        this.pose=pose;this.goal=goal;this.path=path;this.trajectory=trajectory;
    }
    /** Display-only native ROS grid. Selection still uses acknowledged 64x64 cells. */
    public static final class Raster {
        public final int width,height;public final double resolution,x,y,yaw;public final String cells,session;
        private Raster(int w,int h,double r,double[] o,String c,String session){width=w;height=h;resolution=r;x=o[0];y=o[1];yaw=o[2];cells=c;this.session=session;}
        static Raster parse(JSONObject o){
            if(o==null)return null;
            double dw=o.optDouble("width",-1),dh=o.optDouble("height",-1);
            if(dw<1||dh<1||dw>1000000||dh>1000000||dw!=Math.floor(dw)||dh!=Math.floor(dh)||dw*dh>1000000)
                throw new IllegalArgumentException("原始地图尺寸无效");
            int w=(int)dw,h=(int)dh;double r=finite(o.optDouble("resolution_m",Double.NaN));
            if(r<.01||r>1000)throw new IllegalArgumentException("原始地图比例无效");
            String c=o.optString("cells","");if(c.length()!=w*h)throw new IllegalArgumentException("原始地图长度错误");
            for(int i=0;i<c.length();i++)if("?.#".indexOf(c.charAt(i))<0)throw new IllegalArgumentException("原始地图内容错误");
            String session=o.optString("session","");if(session.length()>128)throw new IllegalArgumentException("地图会话无效");
            return new Raster(w,h,r,point(o.optJSONArray("origin"),3),c,session);
        }
    }
    public int displayWidth(){return raster==null?width:raster.width;}
    public int displayHeight(){return raster==null?height:raster.height;}
    public String displayCells(){return raster==null?cells:raster.cells;}
    /** Prepared on the map network worker, never changes occupancy or geometry. */
    public void prepareDisplay(NavigationMap previous) {
        if(!available||displayPixels!=null)return;
        if(previous!=null&&previous.displayPixels!=null&&displayWidth()==previous.displayWidth()
                &&displayHeight()==previous.displayHeight()&&displayCells().equals(previous.displayCells()))
            displayPixels=previous.displayPixels;
        else displayPixels=MapDisplayFilter.pixels(displayCells(),displayWidth(),displayHeight());
    }
    public int[] preparedDisplayPixels(){return displayPixels;}
    public long ageMs(long now) {
        long elapsed=now-receivedAt;
        return elapsed<0 || elapsed>Long.MAX_VALUE-snapshotAgeMs ? Long.MAX_VALUE : snapshotAgeMs+elapsed;
    }
    public boolean fresh(long now) { return available && ageMs(now)<=MAX_AGE_MS; }
    public boolean poseFresh(long now) { long elapsed=now-receivedAt; return poseValid && sensorAgeMs<=MAX_AGE_MS && elapsed>=0 && elapsed<=MAX_AGE_MS-sensorAgeMs; }
    private static double finite(double x) {
        if(Double.isNaN(x)||Double.isInfinite(x)||Math.abs(x)>1000000)throw new IllegalArgumentException("地图坐标无效");
        return x;
    }
    private static double[] point(JSONArray a,int count) {
        if(a==null||a.length()!=count)throw new IllegalArgumentException("地图坐标格式错误");
        double[] p=new double[count];for(int i=0;i<count;i++)p[i]=finite(a.optDouble(i,Double.NaN));return p;
    }
    private static double[][] points(JSONArray a,int limit) {
        if(a==null||a.length()>limit)throw new IllegalArgumentException("地图路径格式错误");
        double[][] p=new double[a.length()][];for(int i=0;i<p.length;i++)p[i]=point(a.optJSONArray(i),2);return p;
    }
    public static NavigationMap parse(JSONObject o,long requestedAt) {
        if(o.optInt("schema",-1)!=1)throw new IllegalArgumentException("导航地图版本不兼容");
        boolean available=o.optBoolean("available",false);
        String state=o.optString("state","waiting"),reason=o.optString("reason",""),session=o.optString("session_id","");
        long age=o.optLong("snapshot_age_ms",-1),sensorAge=o.optLong("sensor_age_ms",-1),revision=o.optLong("map_revision",-1);
        if(!available)return new NavigationMap(false,false,0,0,0,new double[]{0,0},"","",state,reason,session,revision,requestedAt,
                MAX_AGE_MS+1,MAX_AGE_MS+1,new double[]{0,0,0},null,new double[0][],new double[0][],null,false);
        if(age<0||sensorAge<0||revision<0||session.isEmpty()||session.length()>128)
            throw new IllegalArgumentException("地图状态字段缺失");
        int w=o.optInt("width",-1),h=o.optInt("height",-1);
        double resolution=finite(o.optDouble("resolution_m",Double.NaN));
        if(w!=64||h!=64||resolution<.01||resolution>1000)throw new IllegalArgumentException("地图尺寸无效");
        String cells=o.optString("cells","");if(cells.length()!=w*h)throw new IllegalArgumentException("地图栅格长度错误");
        for(int i=0;i<cells.length();i++)if("?.#".indexOf(cells.charAt(i))<0)throw new IllegalArgumentException("地图栅格内容错误");
        String navigable=o.optString("navigable", "");
        if(!navigable.isEmpty()) {
            if(navigable.length()!=w*h)throw new IllegalArgumentException("可通行栅格长度错误");
            for(int i=0;i<navigable.length();i++)if(".#".indexOf(navigable.charAt(i))<0)throw new IllegalArgumentException("可通行栅格内容错误");
        }
        double[] origin=point(o.optJSONArray("origin"),2),pose=point(o.optJSONArray("pose"),3);
        JSONArray goalJson=o.optJSONArray("goal");double[] goal=goalJson==null?null:point(goalJson,2);
        boolean local=o.optBoolean("map_commit_confirmed",false)
                &&"6.4m_local_window".equals(o.optString("view_scope",""))&&Math.abs(resolution-.1)<1e-9;
        return new NavigationMap(true,o.optBoolean("pose_valid",false),w,h,resolution,origin,cells,navigable,state,reason,session,revision,
                requestedAt,age,sensorAge,pose,goal,points(o.optJSONArray("path"),4096),points(o.optJSONArray("trajectory"),4096),Raster.parse(o.optJSONObject("render_map")),local);
    }
}
