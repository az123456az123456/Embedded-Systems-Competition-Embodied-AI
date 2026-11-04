import cn.acg138.robotphone.*;
import org.json.JSONObject;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import javax.imageio.ImageIO;

/** Faults reach production RobotClient, including actual threads and multipart/JPEG parsing.
 * The HTTP adapter is deliberately in-memory: this is not a claim of RF/hardware testing. */
public final class ClientNetworkTests {
    static final List<String> operations=new CopyOnWriteArrayList<>(),messages=new CopyOnWriteArrayList<>();
    static final List<JSONObject> diagnosticRequests=new CopyOnWriteArrayList<>();
    static final List<JSONObject> acceptedControls=new CopyOnWriteArrayList<>();
    static volatile java.util.concurrent.CountDownLatch heldDriveEntered,heldDriveRelease;
    static final List<Boolean> connectedEvents=new CopyOnWriteArrayList<>();
    static volatile boolean serverArmed,configured,stateFails,blockControl,controlEntered,controlFinished;
    static volatile boolean delayedState,delayedStateEntered,delayedStateFinished,invalidControl;
    static volatile int stateCode=200,videoMode,videoAttempts,zeroDrives,nonzeroDrives,armCalls,connectCalls;
    static volatile int singleControlLoss;
    static volatile boolean injectArmCache, armInFlight;
    static volatile JSONObject stateOverride;
    static volatile long oneStateDelayMs, stateRequestedAt;
    static volatile JSONObject lastDeliveredState;
    static final AtomicInteger stateDeliveries=new AtomicInteger();
    static volatile long serverSequence,lastZeroDriveSequence;
    static volatile RobotClient.Frame lastFrame;
    static final AtomicInteger states=new AtomicInteger();
    static int checks;
    static long now(){return System.nanoTime()/1000000;}
    static void check(boolean condition,String label){if(!condition)throw new AssertionError(label);checks++;}
    static void await(BooleanSupplier condition,long timeout,String label)throws Exception {
        long deadline=now()+timeout;while(!condition.getAsBoolean()&&now()<deadline)Thread.sleep(5);
        check(condition.getAsBoolean(),label);
    }
    static void delay(long ms){long end=now()+ms;while(now()<end)try{Thread.sleep(Math.min(10,Math.max(1,end-now())));}catch(InterruptedException ignored){}}
    static JSONObject ok(){return new JSONObject().put("ok",true);}
    static JSONObject state(boolean armed){return ok().put("armed",armed).put("mode","manual").put("reason","idle").put("selected_id",0).put("motor",new JSONObject().put("link_ok",configured));}
    static byte[] jpg;
    static byte[] frameBytes()throws Exception {
        JSONObject meta=new JSONObject().put("frame_id",42).put("width",8).put("height",6).put("detections_frame_id",39).put("detection_timestamp_ms",900);
        String head="--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "+jpg.length+"\r\nX-Frame-Id: 42\r\nX-Frame-Meta: "+Base64.getEncoder().encodeToString(meta.toString().getBytes(StandardCharsets.UTF_8))+"\r\n\r\n";
        ByteArrayOutputStream out=new ByteArrayOutputStream();out.write(head.getBytes(StandardCharsets.US_ASCII));out.write(jpg);out.write("\r\n".getBytes(StandardCharsets.US_ASCII));return out.toByteArray();
    }
    static final class Connection extends HttpURLConnection {
        final ByteArrayOutputStream posted=new ByteArrayOutputStream();
        byte[] response;int code=200;volatile boolean disconnected;int attempt;
        Connection(URL url){super(url);}
        public void connect(){}public boolean usingProxy(){return false;}public void disconnect(){disconnected=true;}
        public OutputStream getOutputStream(){return posted;}
        void prepare()throws IOException {
            if(response!=null)return;
            String path=url.getPath();
            if(path.equals("/api/video.mjpg")) {attempt=++videoAttempts;code=videoMode==3?429:videoMode==4?401:200;response=new byte[0];return;}
            JSONObject answer;
            if(path.equals("/api/connect")){connectCalls++;answer=ok().put("session","network-test-session-12345");}
            else if(path.equals("/api/state")) {
                states.incrementAndGet();code=stateCode;
                if(stateFails)throw new SocketTimeoutException("injected telemetry packet loss");
                boolean oldArmed=serverArmed;
                stateRequestedAt=now();
                JSONObject override=stateOverride;
                long addedDelay=oneStateDelayMs;oneStateDelayMs=0;
                if(addedDelay>0)delay(addedDelay);
                if(override!=null)answer=new JSONObject(override.toString());
                else if(delayedState){delayedState=false;delayedStateEntered=true;delay(700);delayedStateFinished=true;answer=state(oldArmed);}
                else answer=state(serverArmed);
            } else {
                JSONObject body=new JSONObject(new String(posted.toByteArray(),StandardCharsets.UTF_8));
                String op=body.optString("op");operations.add(op);
                if(body.has("client_reason")) diagnosticRequests.add(new JSONObject()
                    .put("op",op).put("reason",body.optString("client_reason"))
                    .put("id",body.optLong("client_diag_id")).put("at",body.optLong("client_mono_ms"))
                    .put("state_age",body.optLong("client_state_age_ms")).put("lease_age",body.optLong("client_lease_age_ms")));
                if(invalidControl){code=403;answer=new JSONObject().put("ok",false).put("error","not_control_owner");}
                else {
                    long seq=body.optLong("seq");
                    java.util.concurrent.CountDownLatch entered=heldDriveEntered,release=heldDriveRelease;
                    if(op.equals("drive")&&(body.optDouble("throttle")!=0||body.optDouble("steer")!=0)&&entered!=null&&release!=null){
                        heldDriveEntered=null;entered.countDown();
                        try{if(!release.await(250,java.util.concurrent.TimeUnit.MILLISECONDS))throw new IOException("test in-flight DRIVE latch timed out");}
                        catch(InterruptedException ex){throw new IOException("test in-flight DRIVE interrupted",ex);}
                    }
                    if(seq>serverSequence){
                        acceptedControls.add(new JSONObject(body.toString()));serverSequence=seq;
                        if(op.equals("configure")){configured=true;serverArmed=false;}
                        if(op.equals("arm")){
                            serverArmed=true;armCalls++;
                            if(injectArmCache){
                                injectArmCache=false;armInFlight=true;
                                stateOverride=state(false).put("gateway_state_age_ms",0).put("gateway_control_pending",true);
                                delay(160);armInFlight=false;
                            }
                        }
                        if(op.equals("stop")||op.equals("disarm"))serverArmed=false;
                        if(op.equals("drive")){if(body.optDouble("throttle")==0&&body.optDouble("steer")==0){zeroDrives++;lastZeroDriveSequence=seq;}else nonzeroDrives++;}
                    }
                    if((op.equals("drive")||op.equals("idle"))&&singleControlLoss>0){singleControlLoss--;delay(160);throw new SocketTimeoutException("one lost ACK");}
                    if((op.equals("drive")||op.equals("idle"))&&blockControl){controlEntered=true;delay(700);controlFinished=true;}
                    answer=ok();
                }
            }
            response=answer.toString().getBytes(StandardCharsets.UTF_8);
        }
        public int getResponseCode()throws IOException{prepare();return code;}
        public InputStream getErrorStream(){try{prepare();}catch(IOException ex){return null;}return new ByteArrayInputStream(response);}
        public InputStream getInputStream()throws IOException {
            prepare();if(!url.getPath().equals("/api/video.mjpg"))return new ByteArrayInputStream(response);
            if(videoMode==1)return new ByteArrayInputStream(new byte[0]);
            if(videoMode==2)throw new SocketTimeoutException("injected video timeout");
            final InputStream parked=new InputStream(){
                public int read()throws IOException {while(!disconnected){try{Thread.sleep(5);}catch(InterruptedException ex){throw new IOException("closed");}}return -1;}
                @Override public int read(byte[] bytes,int off,int len)throws IOException{return read();}
            };
            if(videoMode==5){try{return new SequenceInputStream(new ByteArrayInputStream(frameBytes()),parked);}catch(Exception ex){throw new IOException(ex);}}
            return parked;
        }
    }
    static RobotClient start()throws Exception { return start(false); }
    static RobotClient start(boolean expectVideoAuthFailure)throws Exception {
        serverArmed=configured=stateFails=blockControl=controlEntered=controlFinished=false;
        delayedState=delayedStateEntered=delayedStateFinished=invalidControl=false;
        injectArmCache=armInFlight=false;stateOverride=null;oneStateDelayMs=0;lastDeliveredState=null;stateDeliveries.set(0);
        stateCode=200;singleControlLoss=0;zeroDrives=nonzeroDrives=armCalls=connectCalls=0;serverSequence=lastZeroDriveSequence=0;heldDriveEntered=heldDriveRelease=null;acceptedControls.clear();
        operations.clear();messages.clear();connectedEvents.clear();diagnosticRequests.clear();lastFrame=null;states.set(0);
        RobotClient c=new RobotClient("http://192.168.1.1:8080","012345",new RobotClient.Listener(){
            public void message(String value){messages.add(value);}public void connected(boolean value){connectedEvents.add(value);}
            public void state(JSONObject value){lastDeliveredState=value;stateDeliveries.incrementAndGet();}public void frame(RobotClient.Frame value){lastFrame=value;}
        });
        c.start();
        if(expectVideoAuthFailure)await(()->connectCalls==1,700,"handshake was attempted before video authorization denial");
        else {await(c::isConnected,700,"handshake establishes session");await(()->states.get()>0,700,"initial telemetry fetched");}
        return c;
    }
    static void arm(RobotClient c)throws Exception{c.arm(10,15);await(c::isArmed,700,"explicit user arm confirmed");}
    static void finish(RobotClient c)throws Exception{c.close();Thread.sleep(100);}
    static void videoIndependent(int failureMode,String label)throws Exception {
        videoMode=failureMode;videoAttempts=0;RobotClient c=start();arm(c);
        c.manual(.6f,.1f,false);await(()->nonzeroDrives>0,500,label+" control begins");
        await(()->videoAttempts>=3,1600,label+" retries video with backoff");
        check(c.isConnected()&&c.isArmed()&&serverArmed,label+" does not disconnect/disarm healthy control");
        check(!connectedEvents.contains(false)&&!operations.contains("stop")&&!operations.contains("disarm"),label+" never emits global STOP");
        long previousZero=lastZeroDriveSequence;c.releaseStick(false);
        long zeroBarrier=awaitAcknowledgedZero(c,previousZero,label+" release confirms zero ACK");
        int prior=nonzeroDrives;Thread.sleep(160);
        check(c.isArmed()&&nonzeroDrives==prior,label+" normal release preserves arm and never old throttle after zero ACK");
        check(noNonzeroAfter(zeroBarrier),label+" no accepted nonzero sequence follows confirmed zero barrier");
        videoMode=5;await(()->lastFrame!=null,3500,label+" actual multipart decoder recovers after retry");
        check(lastFrame.id==42&&lastFrame.metadata.optLong("frame_id")==42&&Arrays.equals(lastFrame.jpeg,jpg),label+" exact same-frame JPEG reaches archive interface");
        check(armCalls==1&&connectCalls==1,label+" recovery never re-arms or silently creates session");finish(c);
    }
    static long awaitAcknowledgedZero(RobotClient c,long previousZero,String label)throws Exception {
        java.lang.reflect.Field field=RobotClient.class.getDeclaredField("completedSequence");field.setAccessible(true);
        java.util.concurrent.atomic.AtomicLong completed=(java.util.concurrent.atomic.AtomicLong)field.get(c);
        await(()->lastZeroDriveSequence>previousZero&&completed.get()>=lastZeroDriveSequence,500,label);
        return lastZeroDriveSequence;
    }
    static boolean noNonzeroAfter(long barrier) {
        for(JSONObject q:acceptedControls)if(q.optLong("seq")>barrier&&q.optString("op").equals("drive")&&
            (q.optDouble("throttle")!=0||q.optDouble("steer")!=0))return false;
        return true;
    }
    static void releaseWithInflightDrive()throws Exception {
        videoMode=0;RobotClient c=start();arm(c);
        java.util.concurrent.CountDownLatch entered=new java.util.concurrent.CountDownLatch(1),release=new java.util.concurrent.CountDownLatch(1);
        heldDriveEntered=entered;heldDriveRelease=release;c.manual(.6f,.1f,false);
        check(entered.await(500,java.util.concurrent.TimeUnit.MILLISECONDS),"real nonzero DRIVE held in transport before server acceptance");
        int before=nonzeroDrives;long previousZero=lastZeroDriveSequence;
        c.releaseStick(false);release.countDown();
        long barrier=awaitAcknowledgedZero(c,previousZero,"in-flight DRIVE completes before confirmed release zero ACK");
        check(nonzeroDrives==before+1,"one pre-release in-flight DRIVE is allowed to finish before zero barrier");
        int after=nonzeroDrives;Thread.sleep(180);
        check(c.isArmed()&&nonzeroDrives==after&&noNonzeroAfter(barrier),"after confirmed zero ACK no old throttle is accepted or repeated");
        check(!operations.contains("stop")&&!operations.contains("disarm"),"normal release with legal in-flight DRIVE preserves existing arm lease");finish(c);
    }
    static void stateTransientAndStale()throws Exception {
        videoMode=0;RobotClient c=start();arm(c);stateFails=true;Thread.sleep(400);
        check(c.isConnected()&&c.isArmed(),"short telemetry timeout does not destroy control lease");
        stateFails=false;Thread.sleep(300);check(c.isConnected()&&c.isArmed(),"telemetry recovers without manual reconnection");
        stateFails=true;await(()->!c.isArmed(),1900,"bounded telemetry age eventually forces safe stop");
        await(()->!serverArmed,500,"telemetry age stop reaches server");check(c.isConnected(),"stale read-only state preserves retrying session");checkDiagnostic("telemetry_timeout",true,1500);
        stateFails=false;Thread.sleep(250);check(!c.isArmed()&&armCalls==1,"fresh state never automatically rearms after stale stop");
        arm(c);check(armCalls==2,"new explicit arm can recover after valid state");finish(c);
    }
    static void controlFailures()throws Exception {
        videoMode=0;RobotClient c=start();arm(c);singleControlLoss=1;c.manual(.5f,0,false);Thread.sleep(300);
        check(c.isConnected()&&c.isArmed(),"single lost ACK recovered within unchanged 350ms lease");
        check(!connectedEvents.contains(false),"isolated lost control response is not a session disconnect");
        blockControl=true;long started=now();await(()->controlEntered,400,"long control response fault is in flight");
        await(()->!c.isConnected()&&!c.isArmed(),430,"independent watchdog clears connection while HTTP remains blocked");
        check(now()-started<650&&!controlFinished,"safety watchdog is not blocked by synchronized HTTP");
        await(()->controlFinished,900,"late control ACK eventually arrives");Thread.sleep(100);
        check(!c.isConnected()&&!c.isArmed()&&c.mode().equals("manual")&&armCalls==1,"late control ACK cannot rearm or restore throttle");
        await(()->!serverArmed,700,"best-effort STOP/disarm runs after blocked command returns");checkDiagnostic("control_ack_timeout",true,0);finish(c);
    }
    static void authFailures()throws Exception {
        videoMode=0;RobotClient c=start();arm(c);stateCode=401;final RobotClient unauthorized=c;
        await(()->!unauthorized.isConnected()&&!unauthorized.isArmed(),500,"401 is a real session failure even on telemetry");
        check(connectedEvents.contains(false),"authentication failure reports disconnected");finish(c);
        videoMode=0;c=start();arm(c);invalidControl=true;final RobotClient owner=c;
        await(()->!owner.isConnected()&&!owner.isArmed(),500,"403 owner loss invalidates whole control session");finish(c);
        videoMode=4;c=start(true);final RobotClient denied=c;await(()->!denied.isConnected(),500,"video authentication denial is not retried forever");finish(c);
    }
    static void backgroundCloseAndLateState()throws Exception {
        videoMode=1;videoAttempts=0;RobotClient c=start();arm(c);
        await(()->videoAttempts>0,500,"video retry worker active before background");c.background();Thread.sleep(100);
        int attempts=videoAttempts;Thread.sleep(650);check(videoAttempts==attempts&&!c.isArmed(),"background cancels video retry without rearming");checkDiagnostic("app_background",true,0);
        final int backgroundAttempts=attempts;c.foreground();await(()->videoAttempts>backgroundAttempts,1000,"foreground may restore video only");
        check(!c.isArmed()&&armCalls==1,"foreground never restores arm or throttle");
        c.close();Thread.sleep(100);attempts=videoAttempts;Thread.sleep(650);check(videoAttempts==attempts&&!c.isConnected(),"close cancels all later reconnects");
        videoMode=0;c=start();arm(c);delayedState=true;await(()->delayedStateEntered,500,"old armed telemetry request is delayed");
        c.background();c.foreground();await(()->delayedStateFinished,1000,"old telemetry returns after foreground epoch change");
        Thread.sleep(100);check(!c.isArmed()&&c.mode().equals("manual"),"late pre-background state cannot revive cleared intent");finish(c);
    }
    static long longField(RobotClient client,String name)throws Exception {
        java.lang.reflect.Field field=RobotClient.class.getDeclaredField(name);field.setAccessible(true);return field.getLong(client);
    }
    static void setField(RobotClient client,String name,Object value)throws Exception {
        java.lang.reflect.Field field=RobotClient.class.getDeclaredField(name);field.setAccessible(true);field.set(client,value);
    }
    static void cachedStateValidation()throws Exception {
        java.lang.reflect.Method parse=RobotClient.class.getDeclaredMethod("parseStateSourceAt",JSONObject.class,long.class);parse.setAccessible(true);
        check(((Long)parse.invoke(null,ok(),1000L))==1000,"legacy state without cache fields remains compatible");
        check(((Long)parse.invoke(null,ok().put("gateway_state_age_ms",300),1000L))==700,"300ms cached state has original source timestamp");
        check(((Long)parse.invoke(null,ok().put("gateway_state_age_ms",100.0).put("gateway_control_pending",false),1000L))==900,"integral finite numeric cache age accepted");
        Object[] invalid={"0",null,true,-1,0.5,301,Double.NaN,Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY};
        for(Object age:invalid)check(((Long)parse.invoke(null,ok().put("gateway_state_age_ms",age),1000L))==-1,"invalid cache age type/range rejected: "+age);
        check(((Long)parse.invoke(null,ok().put("gateway_state_age_ms",100),50L))==-1,"cache age cannot precede monotonic epoch");
        check(((Long)parse.invoke(null,ok().put("gateway_state_age_ms",0).put("gateway_control_pending",true),1000L))==-1,"pending control cache is not authoritative state");
        check(((Long)parse.invoke(null,ok().put("gateway_control_pending","false"),1000L))==-1,"malformed pending flag fails closed");
        videoMode=0;RobotClient c=start();injectArmCache=true;c.arm(10,15);
        await(()->armInFlight,700,"real ARM request is in flight during cached old disarmed poll");
        await(c::isArmed,700,"ARM response succeeds while telemetry cache remains pending");
        long previousAt=longField(c,"latestStateAt");int delivered=stateDeliveries.get();Thread.sleep(260);
        check(c.isArmed()&&serverArmed&&c.isConnected(),"pending pre-ARM armed=false cache cannot cancel newly enabled motors");
        check(longField(c,"latestStateAt")==previousAt&&stateDeliveries.get()==delivered,"pending replies never renew source freshness or reach UI");
        setField(c,"mode","follow");setField(c,"followStartedAt",now()-10);setField(c,"followAcceptedAt",now()-10);setField(c,"followAcknowledged",true);
        Thread.sleep(160);check(c.mode().equals("follow"),"pending cache cannot cancel follow through old mode or selected_id");
        setField(c,"mode","navigation");setField(c,"navigationAcceptedAt",now()-10);setField(c,"navigationOperation","start");
        stateOverride=state(false).put("gateway_control_pending",true).put("gateway_state_age_ms",0).put("navigation",new JSONObject().put("active",false));
        Thread.sleep(160);check(c.mode().equals("navigation"),"pending cache cannot cancel accepted navigation");
        stateOverride=null;await(()->stateDeliveries.get()>delivered,500,"fresh nonpending state resumes after backend command completes");
        check(c.isArmed(),"fresh recovery preserves existing valid arm without implicit second ARM");
        c.endNavigation();await(()->!serverArmed,500,"test stops navigation explicitly");checkDiagnostic("navigation_end",true,0);arm(c);
        stateFails=true;Thread.sleep(250);stateOverride=state(true).put("gateway_state_age_ms",100).put("gateway_control_pending",false);oneStateDelayMs=180;stateFails=false;
        JSONObject old=lastDeliveredState;await(()->lastDeliveredState!=old&&lastDeliveredState.has("gateway_state_age_ms"),800,"delayed cached state reaches parser");
        long latest=longField(c,"latestStateAt"),request=stateRequestedAt;
        check(Math.abs((request-100)-latest)<=30,"cache freshness is request start minus source age, not receipt time");
        stateOverride=null;await(()->lastDeliveredState!=null&&!lastDeliveredState.has("gateway_state_age_ms"),500,"legacy fresh state restores again");
        for(Object age:new Object[]{"0",-1,0.5,301,null}){
            long before=longField(c,"latestStateAt");int prior=stateDeliveries.get();
            stateOverride=state(false).put("gateway_state_age_ms",age);Thread.sleep(150);
            check(c.isArmed()&&stateDeliveries.get()==prior&&longField(c,"latestStateAt")==before,"actual malformed/stale cache cannot disarm or renew state: "+age);
            stateOverride=null;await(()->stateDeliveries.get()>prior,500,"real fresh state accepted after bad cache");
        }
        stateOverride=state(false).put("gateway_state_age_ms",301);
        await(()->!c.isArmed(),1900,"repeated stale cache cannot keep telemetry watchdog alive forever");
        check(c.isConnected(),"cache staleness stops safely without destroying healthy control session");
        stateOverride=null;Thread.sleep(250);check(!c.isArmed(),"later fresh cache recovery never automatically rearms");finish(c);
    }
    static void checkDiagnostic(String reason,boolean disarms,long minimumStateAge)throws Exception {
        await(()->diagnosticRequests.stream().anyMatch(d->reason.equals(d.optString("reason"))&&(!disarms||"disarm".equals(d.optString("op")))),700,"stop diagnostic delivered: "+reason);
        JSONObject first=null,second=null;
        for(JSONObject d:diagnosticRequests)if(reason.equals(d.optString("reason"))) {
            check(d.optLong("at")>0&&d.optLong("id")>0,"diagnostic source clock/id present");
            check(d.optLong("state_age")>=minimumStateAge,"diagnostic retains original state age: "+reason);
            if("stop".equals(d.optString("op")))first=d;
            if("disarm".equals(d.optString("op")))second=d;
        }
        if(disarms)check(first!=null&&second!=null&&first.optLong("id")==second.optLong("id")&&first.optLong("at")==second.optLong("at"),"STOP and DISARM identify same captured client cause");
    }
    static void diagnosticExplicitAndCancel()throws Exception {
        videoMode=0;RobotClient c=start();arm(c);c.stop(true);
        checkDiagnostic("explicit_stop",true,0);check(!c.isArmed(),"logging leaves explicit STOP behavior unchanged");finish(c);
        c=start();arm(c);c.releaseStick(true);checkDiagnostic("touch_cancel",true,0);check(!c.isArmed(),"pointer cancellation still disarms");finish(c);
        c=start();arm(c);c.manual(.5f,0,false);await(()->nonzeroDrives>0,500,"release diagnostic begins actual motion");c.releaseStick(false);
        checkDiagnostic("joystick_release",false,0);check(c.isArmed()&&serverArmed,"normal zero release metadata never disarms");finish(c);
    }
    public static void main(String[] args)throws Exception {
        Thread.setDefaultUncaughtExceptionHandler((thread,error)->{error.printStackTrace();System.exit(1);});
        java.awt.image.BufferedImage image=new java.awt.image.BufferedImage(8,6,java.awt.image.BufferedImage.TYPE_INT_RGB);
        ByteArrayOutputStream jpeg=new ByteArrayOutputStream();ImageIO.write(image,"jpg",jpeg);jpg=jpeg.toByteArray();
        URL.setURLStreamHandlerFactory(protocol->protocol.equals("http")?new URLStreamHandler(){protected URLConnection openConnection(URL url){return new Connection(url);}}:null);
        check(ConnectionPolicy.leaseFresh(1349,1000)&&!ConnectionPolicy.leaseFresh(1350,1000),"control lease boundary remains 350 ms");
        check(!ConnectionPolicy.leaseFresh(999,1000),"negative/backwards timing cannot count as fresh");
        check(ConnectionPolicy.telemetryFresh(2499,1000)&&!ConnectionPolicy.telemetryFresh(2500,1000),"telemetry loss has finite 1500ms horizon");
        check(ConnectionPolicy.nextVideoRetry(1600)==3000,"video retry bounded at 3 seconds");
        videoIndependent(1,"video EOF");videoIndependent(2,"video timeout");videoIndependent(3,"video 429");
        releaseWithInflightDrive();stateTransientAndStale();controlFailures();authFailures();backgroundCloseAndLateState();cachedStateValidation();diagnosticExplicitAndCancel();
        System.out.println("PASS "+checks+" production RobotClient network fault checks: independent video retries, exact JPEG, control lease, telemetry recovery, auth, late responses, background and close");
    }
}
