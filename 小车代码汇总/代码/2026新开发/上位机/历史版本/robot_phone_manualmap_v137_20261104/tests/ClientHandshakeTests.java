import cn.acg138.robotphone.RobotClient;
import cn.acg138.robotphone.NavigationMap;
import cn.acg138.robotphone.NavigationTarget;
import cn.acg138.robotphone.NavigationMapClient;
import org.json.JSONObject;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.BooleanSupplier;

/** Runs production RobotClient executors and HTTP parsing against a delayed-ACK transport.
 * The URL transport is in-memory; this does not claim a real phone/FPGA hardware test. */
public final class ClientHandshakeTests {
    static final List<String> operations = new CopyOnWriteArrayList<>(), messages = new CopyOnWriteArrayList<>();
    static volatile long configuredAt, ackDelayMs=450;
    static volatile boolean serverArmed, rejectArm, navReady=true;
    static volatile int navProtocol=2;
    static volatile boolean supportsStraight,straightActive,stateSessionRejected;
    static volatile boolean supportsMapping,fpgaSource,manualMapping;
    static volatile String mappingState="IDLE";
    static volatile JSONObject semanticState=new JSONObject();
    static final List<String> navigationOps=new CopyOnWriteArrayList<>();
    static volatile long serverSelected;
    static volatile String serverMode="idle",serverReason="waiting";
    static volatile int follows;
    static volatile JSONObject geometry;
    static volatile boolean slowMapStarted,slowMapFinished;
    static volatile JSONObject lastUiState,lastNavigation;
    static volatile boolean navStarted,navFinished,navActive,navPreparing;
    static volatile String navReason="",navState="tracking",navReject="";
    static volatile long disarmDelayMs=0,disarmCompletedAt=0,mappingStopAt=0,navDelayMs=0,lastCancelSequence,lastZeroSequence;
    static volatile int navRequests,idles;
    static volatile int armCalls, nonzeroDrive, zeroDrive;
    static int checks;
    static void check(boolean ok,String label){if(!ok)throw new AssertionError(label);checks++;}
    static long now(){return System.nanoTime()/1000000;}
    static void await(BooleanSupplier ok,long timeout,String label)throws Exception{long deadline=now()+timeout;while(!ok.getAsBoolean()&&now()<deadline)Thread.sleep(10);check(ok.getAsBoolean(),label);}
    static JSONObject handle(String path,String body){
        if(path.equals("/api/connect"))return new JSONObject().put("ok",true).put("session","session-test-12345");
        if(path.equals("/api/navigation/map")){slowMapStarted=true;try{Thread.sleep(1200);}catch(InterruptedException ignored){}slowMapFinished=true;return new JSONObject().put("ok",true).put("schema",1).put("available",false);}
        if(path.equals("/api/navigation")) {
            JSONObject thisNavigation=new JSONObject(body);lastNavigation=thisNavigation;navRequests++;navStarted=true;navigationOps.add(thisNavigation.optString("op"));
            try{Thread.sleep(navDelayMs);}catch(InterruptedException ignored){}
            navFinished=true;
            if(!navReject.isEmpty())return new JSONObject().put("ok",false).put("error",navReject);
            if("mapping-stop".equals(thisNavigation.optString("op"))){mappingStopAt=now();mappingState="STOPPING";navActive=false;return new JSONObject().put("ok",true).put("executed",false);}
            if("semantic-start".equals(thisNavigation.optString("op"))){
                semanticState=new JSONObject().put("task","semantic-fixture").put("target","NO-Mask").put("state","WAIT_EXECUTOR").put("gateway_generation","12").put("active",true);
                navActive=true;return new JSONObject().put("ok",true).put("executed",false).put("gateway_generation","12");
            }
            if("manual-mapping-start".equals(thisNavigation.optString("op"))){
                manualMapping=true;mappingState="MANUAL";navActive=false;
                return new JSONObject().put("ok",true).put("gateway_generation","25");
            }
            if("mapping-start".equals(thisNavigation.optString("op"))){
                if(!serverArmed||!supportsMapping)return new JSONObject().put("ok",false).put("error","exploration_not_ready");
                mappingState="SELECTING";navActive=true;return new JSONObject().put("ok",true).put("executed",false);
            }
            if(thisNavigation.optLong("intent_seq")<lastCancelSequence)return new JSONObject().put("ok",false).put("error","stale_navigation_intent");
            if(fpgaSource&&"goal".equals(thisNavigation.optString("op"))){navActive=false;navReady=true;navState="PREVIEW_READY";return new JSONObject().put("ok",true).put("preview_only",true);}
            navActive=true;straightActive="straight".equals(thisNavigation.optString("op"));return new JSONObject().put("ok",true);
        }
        if(path.equals("/api/state")&&stateSessionRejected)return new JSONObject().put("ok",false).put("error","invalid_session");
        boolean link=configuredAt>0&&now()-configuredAt>=ackDelayMs;
        if(path.equals("/api/state"))return new JSONObject().put("ok",true).put("armed",serverArmed).put("selected_id",serverSelected).put("reason",serverReason).put("mode",serverMode).put("navigation",new JSONObject().put("execution_source",fpgaSource?"FPGA_candidate26":"legacy").put("active",navActive).put("preparing",navPreparing).put("supports_manual_mapping",supportsMapping).put("supports_exploration_mapping",supportsMapping).put("exploration_protocol_version",1).put("mapping",new JSONObject().put("state",mappingState).put("strategy",manualMapping?"manual":"frontier").put("gateway_generation",manualMapping?"25":"0").put("active",(manualMapping&&!"LOADED".equals(mappingState))||navActive&&supportsMapping)).put("semantic",semanticState).put("supports_straight_assist",supportsStraight).put("straight_assist_active",straightActive&&navActive&&!navPreparing).put("state",navState).put("reason",navReason).put("can_start",navReady).put("supports_start_stop",navProtocol>=2).put("start_protocol_version",navProtocol).put("readiness_reason",navReady?"ready":"start_not_verified_free")).put("follow_geometry",geometry).put("motor",new JSONObject().put("link_ok",link).put("reason","test_no_fpga_ack"));
        JSONObject q=new JSONObject(body);String op=q.optString("op");operations.add(op);
        if(op.equals("idle"))idles++;
        else {lastCancelSequence=q.optLong("seq");navActive=false;}
        if(op.equals("configure")){configuredAt=now();serverArmed=false;serverSelected=0;}
        if(op.equals("arm")){armCalls++;if(!link||rejectArm)return new JSONObject().put("ok",false).put("error",rejectArm?"test_arm_rejected":"test_early_arm");serverArmed=true;}
        if(op.equals("disarm")){try{Thread.sleep(disarmDelayMs);}catch(InterruptedException ignored){}disarmCompletedAt=now();}
        if(op.equals("stop")||op.equals("disarm"))serverArmed=false;
        if(op.equals("select")){serverSelected=q.optLong("target_id");serverReason="target_selected";serverMode="idle";}
        if(op.equals("follow")){follows++;if(!serverReason.equals("obstacle_front")){serverMode="follow";serverReason="following";}}
        if(op.equals("drive")){if(q.optDouble("throttle")==0&&q.optDouble("steer")==0){zeroDrive++;lastZeroSequence=q.optLong("seq");}else nonzeroDrive++;}
        return new JSONObject().put("ok",true);
    }
    static class Connection extends HttpURLConnection {
        final ByteArrayOutputStream posted=new ByteArrayOutputStream();byte[] response;volatile boolean closed;int code=200;
        Connection(URL url){super(url);}public void connect(){}public boolean usingProxy(){return false;}public void disconnect(){closed=true;}
        public OutputStream getOutputStream(){return posted;}
        void prepare(){if(response!=null||url.getPath().equals("/api/video.mjpg"))return;JSONObject result=handle(url.getPath(),new String(posted.toByteArray(),StandardCharsets.UTF_8));if(!result.optBoolean("ok"))code=409;response=result.toString().getBytes(StandardCharsets.UTF_8);}
        public int getResponseCode(){prepare();return code;}
        public InputStream getErrorStream(){prepare();return new ByteArrayInputStream(response);}
        public InputStream getInputStream(){prepare();if(!url.getPath().equals("/api/video.mjpg"))return new ByteArrayInputStream(response);return new InputStream(){public int read()throws IOException{while(!closed){try{Thread.sleep(10);}catch(InterruptedException ex){throw new IOException("closed");}}return -1;}};}
    }
    static RobotClient start()throws Exception{
        semanticState=new JSONObject();manualMapping=false;supportsMapping=false;fpgaSource=false;mappingState="IDLE";navigationOps.clear();configuredAt=0;serverArmed=false;navReady=true;navProtocol=2;supportsStraight=straightActive=stateSessionRejected=false;serverSelected=0;serverMode="idle";serverReason="waiting";follows=0;geometry=null;lastUiState=null;operations.clear();messages.clear();armCalls=nonzeroDrive=zeroDrive=0;navRequests=idles=0;navStarted=navFinished=navActive=navPreparing=false;lastNavigation=null;lastCancelSequence=lastZeroSequence=0;
        RobotClient client=new RobotClient("http://192.168.1.1:8080","012345",new RobotClient.Listener(){public void message(String s){messages.add(s);}public void connected(boolean b){}public void state(JSONObject s){lastUiState=s;}public void frame(RobotClient.Frame f){}});
        client.start();await(client::isConnected,1000,"client handshake connected");return client;
    }
    static NavigationTarget target(long age) {
        char[] cells=new char[4096];Arrays.fill(cells,'.');
        JSONObject data=new JSONObject().put("schema",1).put("available",true).put("pose_valid",true).put("width",64).put("height",64).put("resolution_m",.1)
            .put("origin",new org.json.JSONArray(-3.2,-3.2)).put("cells",new String(cells)).put("pose",new org.json.JSONArray(0,0,0))
            .put("path",new org.json.JSONArray()).put("trajectory",new org.json.JSONArray()).put("state","tracking").put("reason","")
            .put("map_revision",55).put("session_id","map-test-123").put("snapshot_age_ms",0).put("sensor_age_ms",0);
        long now=android.os.SystemClock.elapsedRealtime();NavigationMap map=NavigationMap.parse(data,now-age);
        return NavigationTarget.fromScreen(map,now-age,100,150,0,0,320,320);
    }
    static void waitStopAck(RobotClient c)throws Exception {
        java.lang.reflect.Field f=RobotClient.class.getDeclaredField("disarmsInFlight");f.setAccessible(true);
        java.util.concurrent.atomic.AtomicInteger n=(java.util.concurrent.atomic.AtomicInteger)f.get(c);
        await(()->n.get()==0,1800,"all stop responses settled before a new user ARM tap");
    }
    static void fpgaPointFlow()throws Exception {
        ackDelayMs=20;rejectArm=false;navDelayMs=0;RobotClient c=start();
        c.arm(10,15);await(c::isArmed,1000,"FPGA point test explicitly armed");
        int oldBackendRequests=navRequests;c.navigate(target(0));c.startNavigation();Thread.sleep(100);
        check(navRequests==oldBackendRequests,"legacy backend cannot auto execute selected destination or start RK follower");
        fpgaSource=true;
        await(()->lastUiState!=null&&"FPGA_candidate26".equals(lastUiState.optJSONObject("navigation").optString("execution_source")),700,"FPGA capability received");
        int before=navRequests;c.startNavigation();Thread.sleep(100);
        check(navRequests==before,"FPGA start without endpoint rejected locally");
        int zeroBefore=zeroDrive;c.navigate(target(0));
        await(()->navigationOps.contains("goal"),700,"FPGA selection sends GOAL only");
        await(()->c.mode().equals("preview")&&messages.stream().anyMatch(v->v.contains("路线仅预览")),700,"FPGA preview does not become active navigation");
        Thread.sleep(120);check(!navActive&&zeroDrive==zeroBefore,"preview neither starts nor emits preempting zero DRIVE");
        check(lastNavigation.has("x")&&lastNavigation.has("y")&&lastNavigation.optLong("map_revision")==55,"preview carries exact goal and displayed map identity");
        c.startNavigation();await(()->navigationOps.contains("start"),700,"only explicit start requests execution");
        check(zeroDrive==zeroBefore&&!lastNavigation.has("x")&&!lastNavigation.has("y"),"START uses prepared FPGA goal without DRIVE or synthetic destination");
        c.endNavigation();await(()->!c.isArmed()&&!serverArmed,700,"end revokes local and server authorization");
        waitStopAck(c);c.arm(10,15);await(c::isArmed,1000,"explicit rearm after end");
        before=navRequests;c.startNavigation();Thread.sleep(150);check(navRequests==before,"end clears preview; rearm does not replay old start");
        c.navigate(target(0));await(()->c.mode().equals("preview"),500,"new endpoint remains preview after end");
        c.manual(.5f,0,false);await(()->nonzeroDrive>0,500,"manual takeover remains available during preview");
        before=navRequests;c.startNavigation();Thread.sleep(150);check(navRequests==before,"manual takeover invalidates previous preview");
        c.stop(true);await(()->!serverArmed,600,"stop before late-response trials");
        waitStopAck(c);c.arm(10,15);await(c::isArmed,1000,"explicit rearm before delayed preview");
        navStarted=navFinished=false;navDelayMs=700;
        c.navigate(target(0));await(()->navStarted,500,"preview HTTP is in flight");
        c.endNavigation();await(()->!serverArmed,500,"End uses independent stop lane during delayed preview");
        await(()->navFinished,1200,"old preview response completed");Thread.sleep(100);
        check(!c.isArmed()&&"manual".equals(c.mode()),"late preview cannot restore an ended intent");
        navDelayMs=0;waitStopAck(c);c.arm(10,15);await(c::isArmed,1000,"explicit rearm before delayed START");
        navStarted=navFinished=false;messages.clear();c.navigate(target(0));await(()->navFinished&&messages.stream().anyMatch(v->v.contains("路线仅预览"))&&c.mode().equals("preview"),700,"new explicit preview accepted");
        navDelayMs=700;navStarted=navFinished=false;c.startNavigation();await(()->navStarted,500,"START HTTP is in flight");
        c.endNavigation();await(()->!serverArmed,500,"End interrupts a delayed START");
        await(()->navFinished,1200,"old START response completed");Thread.sleep(100);
        check(!c.isArmed()&&!navActive&&"manual".equals(c.mode()),"late START cannot revive stopped task or change mode");
        navDelayMs=0;c.close();Thread.sleep(120);
    }
    static void manualMappingFlow()throws Exception {
        RobotClient c=start();final RobotClient first=c;supportsMapping=true;fpgaSource=true;
        c.arm(10,15);await(c::isArmed,1500,"manual mapping arm");
        await(()->lastUiState.optJSONObject("navigation").optBoolean("supports_manual_mapping"),500,"manual support");
        int requests=navRequests;c.startManualMapping();
        await(()->manualMapping&&first.mode().equals("manual_mapping"),700,"manual mapping accepted");
        check(navRequests==requests+1&&!navActive,"manual mapping has no autonomous navigation");
        c.startMapping();c.startSemantic("找到未戴口罩的人");c.follow();Thread.sleep(150);
        check(navRequests==requests+1&&follows==0,"pending manual mapping blocks autonomous jobs");
        c.setManualMode(true);check("hold".equals(c.mode()),"mapping preserves heading-hold mode");
        c.manual(.4f,0,false);await(()->nonzeroDrive>0,700,"joystick drives during mapping");
        check(c.mode().equals("manual_mapping"),"joystick keeps mapping intent");
        c.releaseStick(false);Thread.sleep(180);check(c.mode().equals("manual_mapping"),"release keeps mapping alive");
        c.endMapping();await(()->mappingStopAt>0,1800,"manual finish uses mapping-stop");
        check(!c.isArmed()&&c.mode().equals("manual"),"manual finish disarms");c.close();Thread.sleep(120);
        c=start();supportsMapping=true;final RobotClient late=c;c.arm(10,15);await(c::isArmed,1500,"late manual arm");
        await(()->lastUiState.optJSONObject("navigation").optBoolean("supports_manual_mapping"),500,"late manual support");
        navDelayMs=450;c.startManualMapping();await(()->navStarted,500,"manual start in flight");c.endMapping();
        await(()->mappingStopAt>0,2000,"late start ordered before finish");
        check(late.mode().equals("manual")&&!late.isArmed(),"late manual reply cannot resurrect intent");
        navDelayMs=0;c.close();Thread.sleep(120);
    }
    static void mappingFlow()throws Exception {
        ackDelayMs=20;rejectArm=false;navDelayMs=0;navReject="";RobotClient c=start();
        Thread.sleep(100);check(navRequests==0,"connection and polling do not start mapping");
        c.startMapping();Thread.sleep(60);check(navRequests==0&&armCalls==0,"unarmed mapping never arms");
        c.arm(10,15);await(c::isArmed,1000,"explicit arm before mapping");
        c.startMapping();Thread.sleep(60);check(navRequests==0&&messages.contains("RK 尚未接入探索建图服务，请更新 RK 整合程序"),"legacy RK gets clear unavailable message and no fallback request");
        supportsMapping=true;await(()->lastUiState.optJSONObject("navigation").optBoolean("supports_exploration_mapping"),500,"mapping capability observed");
        final RobotClient mappingClient=c;
        int zeroBefore=zeroDrive,armBefore=armCalls;
        c.startMapping();await(()->navRequests==1&&"mapping".equals(mappingClient.mode()),500,"explicit tap starts one mapping request");
        check(lastNavigation.optString("op").equals("mapping-start")&&lastNavigation.optLong("nav_seq")>0,"same HTTP channel with dedicated mapping operation and sequence");
        check(zeroDrive==zeroBefore&&nonzeroDrive==0&&armCalls==armBefore,"mapping does not send legacy DRIVE or implicit ARM");
        c.startMapping();Thread.sleep(100);check(navRequests==1,"repeated tap does not issue duplicate starts");
        c.startNavigation();Thread.sleep(60);check(navRequests==1,"normal navigation cannot silently replace mapping");
        disarmDelayMs=250;disarmCompletedAt=mappingStopAt=0;
        c.endMapping();check(!c.isArmed()&&c.mode().equals("manual"),"ending immediately invalidates local motion intent");
        await(()->navigationOps.contains("mapping-stop")&&!serverArmed,800,"end sends dedicated mapping-stop and independent STOP");
        check(disarmCompletedAt>0&&mappingStopAt>=disarmCompletedAt,"map-finalize cannot race ahead of delayed DISARM acknowledgement");disarmDelayMs=0;
        mappingState="LOADING";await(()->messages.stream().anyMatch(x->x.contains("正在载入 FPGA")),700,"load progress follows manual mode after End");
        mappingState="ERROR";await(()->messages.stream().anyMatch(x->x.contains("探索建图失败")),700,"load error replaces accepted-request message");
        check(messages.stream().noneMatch(x->x.contains("地图已载入")),"accepted HTTP does not claim saved/loaded completion");
        c.close();Thread.sleep(100);
        c=start();final RobotClient late=c;supportsMapping=true;mappingState="IDLE";c.arm(10,15);await(c::isArmed,1000,"arm delayed-start case");
        await(()->lastUiState.optJSONObject("navigation").optBoolean("supports_exploration_mapping"),500,"delayed case capability");
        navDelayMs=500;c.startMapping();await(()->navStarted,500,"start in flight");
        c.endMapping();c.arm(10,15);check(!c.isArmed(),"pending end forbids rearm");
        await(()->!serverArmed,400,"STOP is independent of slow navigation response");
        await(()->navigationOps.contains("mapping-stop"),1000,"stop ordered after in-flight start");
        Thread.sleep(600);check(!late.isArmed()&&late.mode().equals("manual")&&!navActive,"late start cannot revive stopped intent");
        check(navigationOps.equals(Arrays.asList("mapping-start","mapping-stop")),"no autonomous retry or straight fallback");
        c.close();Thread.sleep(100);navDelayMs=0;
    }
    static void preparingSessionFlow()throws Exception {
        ackDelayMs=20;rejectArm=false;navDelayMs=0;navReject="";RobotClient c=start();supportsStraight=true;
        c.arm(10,15);await(c::isArmed,1000,"prepare compatibility explicit arm");
        await(()->lastUiState!=null&&lastUiState.optJSONObject("navigation").optBoolean("supports_straight_assist"),500,"prepare capability reaches installed client");
        navPreparing=true;navState="PREPARING";navReason="waiting_softcore_calibration";
        c.startNavigation();await(()->navFinished&&navActive,500,"asynchronous preparing start accepted immediately");
        int idleBefore=idles,nonzeroBefore=nonzeroDrive,requests=navRequests;
        Thread.sleep(2200);
        check(c.isArmed()&&c.mode().equals("navigation")&&navActive,"unmodified production client retains PREPARING active session for2s calibration");
        check(idles-idleBefore>=20&&nonzeroDrive==nonzeroBefore&&navRequests==requests,"PREPARING keeps independent idle leases, no raw drive or repeated starts");
        check(lastUiState.optJSONObject("navigation").optBoolean("preparing")&&!lastUiState.optJSONObject("navigation").optBoolean("straight_assist_active"),"preparing metadata does not claim physical straight motion");
        navPreparing=false;navState="STRAIGHT_ADVANCE";navReason="heading_hold";
        await(()->lastUiState.optJSONObject("navigation").optBoolean("straight_assist_active"),500,"ready transition reaches original APK without source changes");
        check(c.isArmed()&&c.mode().equals("navigation"),"ready transition preserves explicit accepted intent");
        c.endNavigation();await(()->!serverArmed&&!navActive,500,"STOP terminates prepared session");
        c.arm(10,15);await(c::isArmed,1000,"rearm before preparing background cancellation");navPreparing=true;navState="PREPARING";navStarted=navFinished=false;c.startNavigation();
        await(()->navFinished&&navActive,500,"second preparation accepted");c.background();await(()->serverArmed&&!navActive,500,"background pauses preparation without waiting for ready or disarming");
        navPreparing=false;c.foreground();Thread.sleep(120);check(c.isArmed()&&c.mode().equals("manual")&&!navActive,"foreground/CPU ready retains authorization but cannot restart canceled preparation");
        c.close();Thread.sleep(100);
    }
    static void semanticFlow()throws Exception {
        RobotClient c=start();final RobotClient current=c;fpgaSource=true;supportsMapping=true;
        c.startSemantic("找到未戴口罩的人");Thread.sleep(70);check(navRequests==0,"semantic search never implicitly arms");
        c.arm(10,15);await(c::isArmed,1000,"explicit arm for semantic search");
        await(()->lastUiState!=null&&lastUiState.optJSONObject("navigation").optString("execution_source").equals("FPGA_candidate26"),500,"observe FPGA adapter");
        c.startSemantic("找到未戴口罩的人");await(()->navRequests==1,500,"one semantic task request");
        check(lastNavigation.optString("op").equals("semantic-start")&&lastNavigation.optString("text").equals("找到未戴口罩的人"),"Chinese target sent unchanged to RK");
        semanticState=new JSONObject().put("task","old-task").put("gateway_generation","11").put("state","FOUND").put("active",false);
        Thread.sleep(250);check("semantic".equals(current.mode()),"old FOUND cannot end the new semantic generation or select mapping");
        check(nonzeroDrive==0,"phone does not track semantic route or send nonzero drive");
        int requestsBefore=navRequests,idleBefore=idles;
        c.startMapping();c.startNavigation();c.startSemantic("找到未戴口罩的人");
        Thread.sleep(150);check(navRequests==requestsBefore&&idles>idleBefore,"semantic keeps idle lease and blocks implicit mapping or duplicate starts");
        semanticState=new JSONObject().put("task","semantic-fixture").put("target","NO-Mask").put("state","FOUND").put("gateway_generation","12").put("active",false).put("found",true);
        navActive=false;await(()->"manual".equals(current.mode()),700,"confirmed found ends local search intent");
        await(()->messages.stream().anyMatch(x->x.contains("已找到未戴口罩的人")),500,"found result shown to user");
        int requests=navRequests;Thread.sleep(100);check(navRequests==requests,"found task never restarts automatically");c.close();
    }
    public static void main(String[] args)throws Exception{
        URL.setURLStreamHandlerFactory(protocol->protocol.equals("http")?new URLStreamHandler(){protected URLConnection openConnection(URL u){return new Connection(u);}}:null);
        RobotClient client=start();final RobotClient first=client;
        client.select(100,50);await(()->messages.stream().anyMatch(s->s.contains("目标 #50 已确认")),700,"unarmed selection accepted for pixel calibration");
        check(!client.isArmed()&&follows==0&&nonzeroDrive==0,"unarmed selection never sends motion");
        client.follow();check(messages.get(messages.size()-1).equals("请先使能电机"),"unarmed follow remains blocked");
        client.arm(10,0);
        await(()->configuredAt>0,500,"configuration POST sent");Thread.sleep(180);
        check(armCalls==0&&!client.isArmed(),"production client waits beyond old 50ms arm race");
        await(client::isArmed,1400,"delayed FPGA ACK enables once");check(armCalls==1,"only one ARM POST after ACK");
        NavigationMapClient maps=new NavigationMapClient("http://192.168.1.1:8080","012345",new NavigationMapClient.Listener(){public void map(NavigationMap m){}public void unavailable(String s){}});
        maps.start();await(()->slowMapStarted,500,"map request starts on dedicated worker");
        client.manual(.75f,0,false);await(()->nonzeroDrive>0,500,"held forward reaches production DRIVE request");
        int heldStart=nonzeroDrive;Thread.sleep(550);check(nonzeroDrive-heldStart>=8,"holding a direction repeatedly sends DRIVE at 50ms intervals");
        check(!slowMapFinished,"map response is still delayed while control heartbeats continue");maps.close();
        client.releaseStick(false);await(()->zeroDrive>0,500,"release sends explicit zero DRIVE");
        int previous=nonzeroDrive;Thread.sleep(150);check(nonzeroDrive==previous&&client.isArmed(),"release keeps stopped arm lease without repeating motion");
        final JSONObject queuedBeforeTap=lastUiState;client.select(101,51);
        check(first.selectedTarget()==51&&queuedBeforeTap.optLong("selected_id",-1)==0,"UI delivery getter sees fresh tap despite already queued old state");
        await(()->messages.stream().anyMatch(s->s.contains("目标 #51 已确认")),700,"production select receives ACK");
        Thread.sleep(500);check(first.selectedTarget()==51&&lastUiState.optLong("selected_id")==51,"confirmed selection survives idle heartbeats and repeated polls");
        final JSONObject queuedBeforeChange=lastUiState;client.select(102,52);
        check(first.selectedTarget()==52&&queuedBeforeChange.optLong("selected_id")==51,"replacement tap wins over queued previous-target state");
        await(()->messages.stream().anyMatch(s->s.contains("目标 #52 已确认")),700,"replacement selection ACK received");
        geometry=new JSONObject().put("configured",false).put("status","calibration_required");
        await(()->lastUiState!=null&&lastUiState.optJSONObject("follow_geometry")!=null,500,"geometry status received");
        client.follow();Thread.sleep(100);check(follows==0&&client.mode().equals("manual"),"uncalibrated geometry never starts follow");
        check(messages.get(messages.size()-1).contains("镜头尚未标定"),"calibration requirement explained in Chinese");
        geometry=new JSONObject().put("configured",true).put("status","ready");
        await(()->lastUiState.optJSONObject("follow_geometry").optBoolean("configured"),500,"calibrated state received");
        client.follow();await(()->follows>=2,600,"explicit Follow repeats its lease while active");
        serverReason="obstacle_front";serverMode="idle";
        await(()->first.mode().equals("manual"),600,"server obstacle stop cancels local follow intention");
        int stoppedFollows=follows;Thread.sleep(250);check(follows==stoppedFollows,"idle polls do not automatically restart stopped follow");
        check(first.selectedTarget()==52,"obstacle stop retains selected target while backend retains it");
        serverReason="target_selected";client.follow();await(()->follows>stoppedFollows,500,"only a new explicit tap restarts follow");
        serverSelected=0;serverReason="target_lost";serverMode="idle";await(()->first.selectedTarget()==0,700,"true target loss clears current selection getter");
        int lostFollows=follows;Thread.sleep(200);check(follows==lostFollows&&first.mode().equals("manual"),"target loss never repeats Follow commands");
        client.follow();check(messages.get(messages.size()-1).equals("请先点击视频中的目标框"),"lost target cannot start follow");
        client.stop(true);await(()->!serverArmed,500,"stop disarms server");client.close();Thread.sleep(100);
        client=start();final RobotClient canceled=client;client.arm(10,0);await(()->configuredAt>0,500,"pending configuration started");client.stop(true);Thread.sleep(650);
        check(armCalls==0&&!canceled.isArmed(),"STOP cancels pending configure so a later ACK never arms");client.close();Thread.sleep(100);
        ackDelayMs=5000;client=start();final RobotClient timeout=client;client.arm(10,0);
        await(()->messages.stream().anyMatch(s->s.contains("test_no_fpga_ack")),2500,"timeout exposes FPGA ACK failure reason");
        check(armCalls==0&&!timeout.isArmed(),"missing ACK never sends ARM");client.close();Thread.sleep(100);
        ackDelayMs=100;rejectArm=true;client=start();final RobotClient rejected=client;client.arm(10,0);
        await(()->messages.contains("test_arm_rejected"),1500,"backend ARM rejection remains visible");check(!rejected.isArmed(),"rejected ARM never changes armed flag");client.close();
        mappingFlow();manualMappingFlow();fpgaPointFlow();semanticFlow();
        System.out.println("PASS "+checks+" production RobotClient checks: delayed ACK, DRIVE/release, STOP race, missing ACK, visible rejection and current selection delivery");
    }
}
