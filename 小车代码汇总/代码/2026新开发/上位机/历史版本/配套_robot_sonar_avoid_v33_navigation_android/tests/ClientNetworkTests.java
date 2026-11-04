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
    static volatile java.util.concurrent.CountDownLatch heldDriveEntered,heldDriveRelease,heldVideoEntered,heldVideoRelease,heldStateEntered,heldStateRelease;
    static final List<Boolean> connectedEvents=new CopyOnWriteArrayList<>();
    static volatile boolean serverArmed,configured,stateFails,blockControl,controlEntered,controlFinished,allNetworkFails,jsonNetworkFails;
    static volatile boolean delayedState,delayedStateEntered,delayedStateFinished,invalidControl;
    static volatile int stateCode=200,videoMode,videoAttempts,zeroDrives,nonzeroDrives,armCalls,connectCalls;
    static volatile int singleControlLoss;
    // Host-only backend authority model. Actual Gateway/Controller lease
    // enforcement is exercised separately by test_mobile_backend_lease_v33.cc.
    static volatile boolean backendSafetyEnabled;
    static volatile long backendAcceptedAt;
    static final AtomicInteger backendSafetyStops=new AtomicInteger();
    static final Object backendSafetyLock=new Object();
    static volatile long sustainedControlDelayMs;
    static volatile String delayedOperation="";
    static volatile long delayedOperationMs;
    static volatile boolean delayedOperationEntered,delayedOperationFinished;
    static final List<Long> delayedControlStarts=new CopyOnWriteArrayList<>();
    static volatile boolean injectArmCache, armInFlight;
    static volatile JSONObject stateOverride;
    static volatile long oneStateDelayMs, stateRequestedAt;
    static volatile JSONObject lastDeliveredState;
    static final AtomicInteger stateDeliveries=new AtomicInteger();
    static volatile long serverSequence,lastZeroDriveSequence;
    static volatile RobotClient.Frame lastFrame;
    static final AtomicInteger states=new AtomicInteger(),videoFrames=new AtomicInteger();
    static int checks;
    static long now(){return System.nanoTime()/1000000;}
    static void check(boolean condition,String label){if(!condition)throw new AssertionError(label);checks++;}
    static void await(BooleanSupplier condition,long timeout,String label)throws Exception {
        long deadline=now()+timeout;while(!condition.getAsBoolean()&&now()<deadline)Thread.sleep(5);
        check(condition.getAsBoolean(),label);
    }
    static void delay(long ms){long end=now()+ms;while(now()<end)try{Thread.sleep(Math.min(10,Math.max(1,end-now())));}catch(InterruptedException ignored){}}
    static JSONObject ok(){return new JSONObject().put("ok",true);}
    static JSONObject state(boolean armed){return ok().put("armed",armed).put("mode","manual").put("reason",!armed&&backendSafetyStops.get()>0?"lease_expired":"idle").put("selected_id",0).put("motor",new JSONObject().put("link_ok",configured));}
    static void enableBackendSafety(){synchronized(backendSafetyLock){backendSafetyEnabled=true;backendAcceptedAt=now();}}
    static void backendAccepted(){synchronized(backendSafetyLock){if(backendSafetyEnabled&&serverArmed)backendAcceptedAt=now();}}
    static void backendArmed(){synchronized(backendSafetyLock){serverArmed=true;if(backendSafetyEnabled)backendAcceptedAt=now();}}
    static void backendSafetyClock(){
        Thread safety=new Thread(()->{while(true){synchronized(backendSafetyLock){if(backendSafetyEnabled&&serverArmed&&backendAcceptedAt>0&&now()-backendAcceptedAt>=350){serverArmed=false;backendSafetyStops.incrementAndGet();}}delay(5);}},"host-backend-authority-model");
        safety.setDaemon(true);safety.start();
    }
    static byte[] jpg;
    static byte[] frameBytes()throws Exception { return frameBytes(42); }
    static byte[] frameBytes(long frameId)throws Exception {
        JSONObject meta=new JSONObject().put("frame_id",frameId).put("width",8).put("height",6).put("detections_frame_id",39).put("detection_timestamp_ms",900);
        String head="--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "+jpg.length+"\r\nX-Frame-Id: "+frameId+"\r\nX-Frame-Meta: "+Base64.getEncoder().encodeToString(meta.toString().getBytes(StandardCharsets.UTF_8))+"\r\n\r\n";
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
            if(allNetworkFails || (jsonNetworkFails&&!path.equals("/api/video.mjpg")))throw new SocketTimeoutException("requested HTTP channel unavailable");
            if(path.equals("/api/video.mjpg")) {attempt=++videoAttempts;code=videoMode==3?429:videoMode==4?401:200;response=new byte[0];return;}
            JSONObject answer;
            if(path.equals("/api/connect")){connectCalls++;answer=ok().put("session","network-test-session-12345");}
            else if(path.equals("/api/state")) {
                states.incrementAndGet();code=stateCode;
                if(stateFails)throw new SocketTimeoutException("injected telemetry packet loss");
                boolean oldArmed=serverArmed;
                stateRequestedAt=now();
                java.util.concurrent.CountDownLatch stateEntered=heldStateEntered,stateRelease=heldStateRelease;
                JSONObject heldSnapshot=null;
                if(stateEntered!=null&&stateRelease!=null){
                    heldStateEntered=null;heldSnapshot=state(oldArmed);stateEntered.countDown();
                    try{if(!stateRelease.await(800,java.util.concurrent.TimeUnit.MILLISECONDS))throw new IOException("held source-state fixture deadline");}
                    catch(InterruptedException ex){throw new IOException("held source-state fixture interrupted",ex);}
                }
                JSONObject override=stateOverride;
                long addedDelay=oneStateDelayMs;oneStateDelayMs=0;
                if(addedDelay>0)delay(addedDelay);
                if(heldSnapshot!=null)answer=heldSnapshot;
                else if(override!=null)answer=new JSONObject(override.toString());
                else if(delayedState){delayedState=false;delayedStateEntered=true;delay(700);delayedStateFinished=true;answer=state(oldArmed);}
                else answer=state(serverArmed);
            } else {
                JSONObject body=new JSONObject(new String(posted.toByteArray(),StandardCharsets.UTF_8));
                String op=body.optString("op");operations.add(op);
                if(body.has("client_reason")) diagnosticRequests.add(new JSONObject()
                    .put("op",op).put("reason",body.optString("client_reason"))
                    .put("id",body.optLong("client_diag_id")).put("at",body.optLong("client_mono_ms"))
                    .put("state_age",body.optLong("client_state_age_ms")).put("lease_age",body.optLong("client_lease_age_ms"))
                    .put("control_send_ms",body.optLong("client_control_send_ms")).put("control_ack_ms",body.optLong("client_control_ack_ms"))
                    .put("control_rtt_ms",body.optLong("client_control_rtt_ms")).put("tick_gap_ms",body.optLong("client_tick_gap_ms"))
                    .put("tick_run_ms",body.optLong("client_tick_run_ms")).put("scheduler_delay_ms",body.optLong("client_scheduler_delay_ms")));
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
                            backendArmed();armCalls++;
                            if(injectArmCache){
                                injectArmCache=false;armInFlight=true;
                                stateOverride=state(false).put("gateway_state_age_ms",0).put("gateway_control_pending",true);
                                delay(160);armInFlight=false;
                            }
                        }
                        if(op.equals("stop")||op.equals("disarm"))serverArmed=false;
                        if(op.equals("drive")){if(body.optDouble("throttle")==0&&body.optDouble("steer")==0){zeroDrives++;lastZeroDriveSequence=seq;}else nonzeroDrives++;}
                        backendAccepted();
                    }
                    if((op.equals("drive")||op.equals("idle"))&&singleControlLoss>0){singleControlLoss--;delay(160);throw new SocketTimeoutException("one lost ACK");}
                    if((op.equals("drive")||op.equals("idle"))&&sustainedControlDelayMs>0){delayedControlStarts.add(now());delay(sustainedControlDelayMs);}
                    if((op.equals("drive")||op.equals("idle"))&&blockControl){controlEntered=true;delay(700);controlFinished=true;}
                    if(op.equals(delayedOperation)&&delayedOperationMs>0){delayedOperationEntered=true;delay(delayedOperationMs);delayedOperationFinished=true;}
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
            if(videoMode==7) {
                final byte[] packet;try{packet=frameBytes();}catch(Exception ex){throw new IOException(ex);}
                final java.util.concurrent.CountDownLatch entered=heldVideoEntered,release=heldVideoRelease;
                return new InputStream(){int offset;boolean waited;
                    public int read()throws IOException{byte[] one=new byte[1];return read(one,0,1)<0?-1:one[0]&255;}
                    public int read(byte[] bytes,int at,int count)throws IOException{
                        if(!waited){waited=true;entered.countDown();try{if(!release.await(2000,java.util.concurrent.TimeUnit.MILLISECONDS))throw new IOException("buffered video fixture deadline");}catch(InterruptedException ex){throw new IOException(ex);}}
                        if(offset==packet.length)return -1;int size=Math.min(count,packet.length-offset);
                        System.arraycopy(packet,offset,bytes,at,size);offset+=size;return size;
                    }
                };
            }
            if(videoMode==6) {
                return new InputStream(){int offset;long nextId=42;byte[] packet;
                    private void packet()throws IOException{try{packet=frameBytes(nextId++);}catch(Exception ex){throw new IOException(ex);}}
                    public int read()throws IOException{byte[] one=new byte[1];return read(one,0,1)<0?-1:one[0]&255;}
                    public int read(byte[] bytes,int at,int count)throws IOException{
                        if(disconnected)return -1;if(packet==null)packet();
                        if(offset==packet.length){delay(100);offset=0;packet();}
                        if(disconnected)return -1;int size=Math.min(count,packet.length-offset);
                        System.arraycopy(packet,offset,bytes,at,size);offset+=size;return size;
                    }
                };
            }
            if(videoMode==5){try{return new SequenceInputStream(new ByteArrayInputStream(frameBytes()),parked);}catch(Exception ex){throw new IOException(ex);}}
            return parked;
        }
    }
    static RobotClient start()throws Exception { return start(false); }
    static RobotClient start(boolean expectVideoAuthFailure)throws Exception {
        serverArmed=configured=stateFails=blockControl=controlEntered=controlFinished=allNetworkFails=jsonNetworkFails=false;
        delayedState=delayedStateEntered=delayedStateFinished=invalidControl=false;
        injectArmCache=armInFlight=false;stateOverride=null;oneStateDelayMs=0;lastDeliveredState=null;stateDeliveries.set(0);
        stateCode=200;singleControlLoss=0;sustainedControlDelayMs=0;delayedControlStarts.clear();zeroDrives=nonzeroDrives=armCalls=connectCalls=0;serverSequence=lastZeroDriveSequence=0;heldDriveEntered=heldDriveRelease=heldStateEntered=heldStateRelease=null;acceptedControls.clear();
        synchronized(backendSafetyLock){backendSafetyEnabled=false;backendAcceptedAt=0;backendSafetyStops.set(0);}
        delayedOperation="";delayedOperationMs=0;delayedOperationEntered=delayedOperationFinished=false;
        operations.clear();messages.clear();connectedEvents.clear();diagnosticRequests.clear();lastFrame=null;states.set(0);videoFrames.set(0);
        RobotClient c=new RobotClient("http://192.168.1.1:8080","012345",new RobotClient.Listener(){
            public void message(String value){messages.add(value);}public void connected(boolean value){connectedEvents.add(value);}
            public void state(JSONObject value){lastDeliveredState=value;stateDeliveries.incrementAndGet();}public void frame(RobotClient.Frame value){lastFrame=value;videoFrames.incrementAndGet();}
        });
        c.start();
        if(expectVideoAuthFailure)await(()->connectCalls==1,700,"handshake was attempted before video authorization denial");
        else {await(c::isConnected,700,"handshake establishes session");await(()->states.get()>0,700,"initial telemetry fetched");}
        return c;
    }
    static void arm(RobotClient c)throws Exception{
        c.arm(10,15);
        try{await(c::isArmed,700,"explicit user arm confirmed");}
        catch(AssertionError failure){System.err.println("ARM FAILURE messages="+messages+" operations="+operations+" state="+lastDeliveredState);throw failure;}
    }
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
        check(c.isConnected()&&c.isArmed(),"single lost ACK does not replace authoritative fresh backend state");
        check(!connectedEvents.contains(false),"isolated lost control response is not a session disconnect");
        enableBackendSafety();blockControl=true;long started=now();await(()->controlEntered,400,"long control response fault is in flight");
        await(()->!serverArmed&&backendSafetyStops.get()>0,550,"independent backend350ms source authority expires while phone HTTP remains blocked");
        await(()->!c.isArmed(),400,"fresh authoritative backend disarm removes local motor authorization");
        check(c.isConnected(),"backend source lease loss preserves independent authenticated session");
        check(now()-started<700&&!controlFinished,"backend safety and source telemetry are independent of blocked phone control response");
        await(()->controlFinished,900,"late control ACK eventually arrives");Thread.sleep(100);
        check(c.isConnected()&&!c.isArmed()&&c.mode().equals("manual")&&armCalls==1,"late control ACK cannot rearm or restore throttle while session remains connected");
        check(!serverArmed&&backendSafetyStops.get()>0,"actual model authority stays disarmed after old reply");
        check(diagnosticRequests.stream().noneMatch(d->d.optString("reason").equals("control_ack_timeout")),"removed duplicate phone ACK-age watchdog emits no artificial STOP");
        blockControl=false;Thread.sleep(150);arm(c);check(armCalls==2&&connectCalls==1,"new explicit arm after ACK loss recovers on same session");finish(c);
    }
    static void sustainedSubTimeoutControlLatency()throws Exception {
        videoMode=0;RobotClient c=start();arm(c);c.manual(.5f,0,false);
        await(()->nonzeroDrives>0,500,"fresh motion precedes sustained legal HTTP delay");
        sustainedControlDelayMs=160;
        long started=now();
        while(now()-started<2200){
            Thread.sleep(5);
            check(c.isConnected()&&c.isArmed()&&serverArmed,"continuous160ms valid responses retain fresh authoritative backend state");
        }
        check(delayedControlStarts.size()>=10,"sustained delay exercises many actual serial production control requests");
        check(armCalls==1&&connectCalls==1&&!operations.contains("stop")&&!operations.contains("disarm"),"legal latency never silently rearms or replaces session");
        long maxGap=0;
        for(int i=1;i<delayedControlStarts.size();++i)maxGap=Math.max(maxGap,delayedControlStarts.get(i)-delayedControlStarts.get(i-1));
        check(maxGap<190,"continuous delayed requests are not charged an extra50ms idle after each response");
        sustainedControlDelayMs=0;c.stop(true);
        await(()->!serverArmed,500,"STOP still wins after saturated serial control cadence");
        check(!c.isArmed()&&c.isConnected(),"cadence fix preserves explicit STOP and transport separation");
        finish(c);
    }
    static void controlCadenceBoundary(long delayMs)throws Exception {
        videoMode=0;RobotClient c=start();arm(c);c.manual(.5f,0,false);
        await(()->nonzeroDrives>0,500,"fresh motion precedes cadence boundary experiment");
        sustainedControlDelayMs=delayMs;long started=now();
        while(now()-started<2200){Thread.sleep(5);check(c.isConnected()&&c.isArmed()&&serverArmed,"continuous"+delayMs+"ms responses with fresh armed backend must not be revoked by a duplicate phone ACK-age timer");}
        check(delayedControlStarts.size()>=10,"continuous delay covers many actual serial production requests");
        check(!operations.contains("stop")&&!operations.contains("disarm"),"continuous valid backend acknowledgements require no implicit recovery");
        System.out.println("BOUNDARY "+new JSONObject().put("rtt_ms",delayMs).put("result","stays_armed")
                .put("duration_ms",now()-started).put("requests",delayedControlStarts.size())
                .put("response_evidence_ms",ConnectionPolicy.RESPONSE_EVIDENCE_MS).put("http_timeout_ms",ConnectionPolicy.CONTROL_HTTP_TIMEOUT_MS));
        sustainedControlDelayMs=0;c.stop(true);await(()->!serverArmed,500,"explicit STOP still terminates continuous response experiment");
        finish(c);
    }
    static void sameMillisecondArmObservation()throws Exception {
        videoMode=0;RobotClient c=start();
        final long clock=now();android.os.SystemClock.setElapsedRealtimeForTest(clock);
        java.util.concurrent.CountDownLatch entered=new java.util.concurrent.CountDownLatch(1),release=new java.util.concurrent.CountDownLatch(1);
        heldStateEntered=entered;heldStateRelease=release;
        check(entered.await(500,java.util.concurrent.TimeUnit.MILLISECONDS),"actual read-only telemetry captured armed=false BEFORE explicit ARM");
        arm(c);check(longField(c,"armedAt")==clock,"explicit ARM and held old observation share one millisecond source timestamp");
        int delivered=stateDeliveries.get();release.countDown();
        await(()->stateDeliveries.get()>delivered,500,"latched old telemetry is delivered only AFTER successful ARM");
        check(c.isArmed()&&c.isConnected()&&serverArmed,"same-millisecond pre-ARM armed=false observation cannot revoke a newly confirmed ARM");
        check(!operations.contains("stop")&&!operations.contains("disarm")&&armCalls==1,"ambiguous old observation causes no artificial STOP or recovery ARM");
        serverArmed=false;android.os.SystemClock.setElapsedRealtimeForTest(clock+1);
        await(()->!c.isArmed(),500,"a genuinely NEWER armed=false observation immediately removes motor authorization");
        check(c.isConnected(),"true backend disarm keeps authenticated session independent");
        serverArmed=true;android.os.SystemClock.setElapsedRealtimeForTest(clock+2);Thread.sleep(120);
        check(!c.isArmed()&&armCalls==1,"later armed=true observation cannot resurrect locally revoked intent");
        c.stop(true);await(()->!serverArmed,500,"explicit STOP still reaches backend after millisecond boundary experiment");
        android.os.SystemClock.setElapsedRealtimeForTest(-1);finish(c);
        System.out.println("PASS source-time equality fixture: pre-ARM same-ms false cannot cancel, newer false does stop, later true never rearms");
    }
    static long completedControlSequence(RobotClient c)throws Exception {
        java.lang.reflect.Field f=RobotClient.class.getDeclaredField("completedSequence");f.setAccessible(true);return ((java.util.concurrent.atomic.AtomicLong)f.get(c)).get();
    }
    static void singleLateResponseCannotAuthorize()throws Exception {
        videoMode=0;RobotClient c=start();long previous=completedControlSequence(c),ack=longField(c,"lastControlAckAt");
        delayedOperation="select";delayedOperationMs=450;c.select(42,7);
        await(()->delayedOperationEntered,500,"single selection response is held beyond350ms evidence budget");
        await(()->delayedOperationFinished,700,"old response eventually arrives while state remains fresh");Thread.sleep(100);
        check(completedControlSequence(c)==previous&&longField(c,"lastControlAckAt")==ack,"old single response cannot advance confirmed command sequence or valid ACK clock");
        check(c.selectedTarget()==0&&c.isConnected()&&!c.isArmed(),"old selection acknowledgement cannot establish a target or arm motion");finish(c);
        c=start();delayedOperation="arm";delayedOperationMs=450;c.arm(10,15);
        await(()->delayedOperationEntered,700,"single ARM response is held beyond350ms evidence budget");
        final RobotClient arming=c;long began=now();while(now()-began<650){Thread.sleep(5);check(!arming.isArmed(),"late ARM response never creates a transient local motor authorization");}
        await(()->!serverArmed,500,"rejected stale ARM evidence triggers genuine STOP/disarm");
        check(c.isConnected()&&armCalls==1&&!c.isArmed(),"stale ARM fails closed without reconnect or automatic retry ARM");finish(c);
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
        int attempts=videoAttempts;Thread.sleep(650);check(videoAttempts==attempts&&!c.isArmed(),"background cancels video retry without rearming");checkDiagnostic("activity_pause",true,0);
        final int backgroundAttempts=attempts;c.foreground();await(()->videoAttempts>backgroundAttempts,1000,"foreground may restore video only");
        check(!c.isArmed()&&armCalls==1,"foreground never restores arm or throttle");
        c.close();Thread.sleep(100);attempts=videoAttempts;Thread.sleep(650);check(videoAttempts==attempts&&!c.isConnected(),"close cancels all later reconnects");
        videoMode=0;c=start();arm(c);delayedState=true;await(()->delayedStateEntered,500,"old armed telemetry request is delayed");
        c.background();c.foreground();await(()->delayedStateFinished,1000,"old telemetry returns after foreground epoch change");
        Thread.sleep(100);check(!c.isArmed()&&c.mode().equals("manual"),"late pre-background state cannot revive cleared intent");finish(c);
    }
    static void windowFocusPreservesSession()throws Exception {
        videoMode=5;videoAttempts=0;RobotClient c=start();arm(c);
        c.manual(.5f,0,false);await(()->nonzeroDrives>0,500,"motion begins before transient window focus loss");
        int beforeConnect=connectCalls,beforeArm=armCalls;
        c.windowFocusChanged(false);
        check(!c.isArmed()&&c.isConnected(),"window focus loss clears movement but retains connection immediately");
        checkDiagnostic("window_focus_lost",true,0);
        await(()->!serverArmed,500,"window focus loss sends genuine STOP and DISARM to RK");
        int stateBefore=states.get(),videoBefore=videoAttempts,moveBefore=nonzeroDrives;
        c.arm(10,15);c.manual(.7f,0,false);c.rotate(true);c.startStraightAssist();
        Thread.sleep(1700);
        check(c.isConnected()&&!connectedEvents.contains(false),"focus-only pause longer than telemetry limit preserves authenticated session");
        check(states.get()>=stateBefore+10,"read-only status continues during focus-only pause");
        check(connectCalls==beforeConnect&&armCalls==beforeArm&&!serverArmed&&nonzeroDrives==moveBefore,"focus-only pause rejects new motion and never reconnects or rearms");
        check(videoAttempts>=videoBefore,"video session remains independent of focus-only pause");
        c.windowFocusChanged(true);Thread.sleep(180);
        check(c.isConnected()&&!c.isArmed()&&armCalls==beforeArm&&c.mode().equals("manual"),"focus restoration never restores previous throttle or navigation");
        arm(c);c.manual(.4f,0,false);await(()->nonzeroDrives>moveBefore,500,"explicit user arm and new stick can resume after focus restoration");
        c.close("activity_pause");await(()->!serverArmed,500,"real Activity pause still disarms server");
        check(!c.isConnected()&&!c.isArmed(),"real Activity pause closes connection and cannot retain control");
        checkDiagnostic("activity_pause",true,0);Thread.sleep(100);
    }
    static void serverDisarmRetainsSession()throws Exception {
        videoMode=6;videoAttempts=0;RobotClient c=start();arm(c);
        c.manual(.4f,0,false);await(()->nonzeroDrives>0,500,"motion starts before backend automatic disarm");
        // Fresh authoritative WAIT/STOP from RK, with a continuing multipart
        // JPEG stream and telemetry. No radio or Android-device validation is claimed.
        serverArmed=false;
        stateOverride=state(false).put("reason","obstacle_wait").put("navigation",new JSONObject().put("active",false).put("state","AVOID_WAIT"));
        await(()->!c.isArmed(),700,"authoritative backend disarm clears local motor state");
        long began=now();int statesBefore=states.get(),framesBefore=videoFrames.get(),armsBefore=armCalls,connectsBefore=connectCalls,movesBefore=nonzeroDrives;
        while(now()-began<31000) {
            check(c.isConnected()&&!c.isArmed(),"repeated backend disarm keeps connection without motor authorization");
            Thread.sleep(100);
        }
        check(states.get()>statesBefore+150,"telemetry continues more than30s in repeated backend-disarmed WAIT");
        check(videoFrames.get()>framesBefore+150,"fresh JPEG frames keep arriving throughout30s backend-disarmed WAIT");
        check(lastFrame!=null&&now()-lastFrame.received<1000&&Arrays.equals(lastFrame.jpeg,jpg),"latest WAIT JPEG is fresh and exact-source bytes are retained");
        check(connectCalls==connectsBefore&&armCalls==armsBefore&&nonzeroDrives==movesBefore,"backend disarm never reconnects/rearms/replays throttle");
        check(!connectedEvents.contains(false),"backend disarm never emits session-disconnected event");
        stateOverride=null;Thread.sleep(200);arm(c);
        check(connectCalls==connectsBefore&&armCalls==armsBefore+1,"explicit arm restores operation on unchanged session after long WAIT");
        c.manual(.3f,0,false);await(()->nonzeroDrives>movesBefore,500,"explicit fresh stick resumes after backend disarm");
        finish(c);
    }
    static void videoOnlyLivenessDoesNotAuthorizeMotion()throws Exception {
        videoMode=6;videoAttempts=0;RobotClient c=start();arm(c);
        await(()->videoFrames.get()>1,600,"fresh continuous JPEG stream is active before JSON channel faults");
        long previousLease=longField(c,"lastLeaseAt"),previousState=longField(c,"latestStateAt");enableBackendSafety();
        int framesBefore=videoFrames.get(),connectsBefore=connectCalls,armsBefore=armCalls;
        jsonNetworkFails=true;
        await(()->!serverArmed&&backendSafetyStops.get()>0,550,"JSON control loss stops backend350ms authority while video remains fresh");
        await(()->!c.isArmed(),1500,"without source-state observations phone independently clears authorization at bounded telemetry age");
        Thread.sleep(2200);
        check(c.isConnected()&&!c.isArmed(),"fresh validated video alone retains disarmed authenticated transport beyond1500ms");
        check(videoFrames.get()>framesBefore+15&&lastFrame!=null&&now()-lastFrame.received<500,"video-only liveness uses actual new JPEG frames");
        check(longField(c,"lastLeaseAt")==previousLease&&longField(c,"latestStateAt")==previousState,"video liveness never refreshes motor lease or source telemetry freshness");
        check(connectCalls==connectsBefore&&armCalls==armsBefore,"video-only activity never reconnects or rearms");
        java.lang.reflect.Field field=RobotClient.class.getDeclaredField("videoConnection");field.setAccessible(true);
        HttpURLConnection current=(HttpURLConnection)field.get(c);videoMode=1;if(current!=null)current.disconnect();
        await(()->!c.isConnected(),1900,"after new JPEGs also stop, genuine loss of all successful channels disconnects");
        jsonNetworkFails=false;Thread.sleep(150);check(!c.isConnected()&&!c.isArmed()&&armCalls==armsBefore,"channel recovery cannot revive a disconnected motor session");
        finish(c);
    }
    static void cancelledVideoGenerationCannotKeepAlive()throws Exception {
        heldVideoEntered=new java.util.concurrent.CountDownLatch(1);heldVideoRelease=new java.util.concurrent.CountDownLatch(1);
        videoMode=7;videoAttempts=0;RobotClient c=start();
        check(heldVideoEntered.await(700,java.util.concurrent.TimeUnit.MILLISECONDS),"complete old video payload held before acceptance");
        jsonNetworkFails=true;videoMode=1;c.background();c.foreground();
        long before=longField(c,"lastTransportAt");int delivered=videoFrames.get();
        heldVideoRelease.countDown();Thread.sleep(250);
        check(longField(c,"lastTransportAt")==before&&videoFrames.get()==delivered,"buffered cancelled-generation frame cannot refresh transport or reach UI");
        await(()->!c.isConnected(),1700,"orphan video frame cannot extend otherwise dead transport");
        jsonNetworkFails=false;finish(c);
    }
    static void totalTransportLoss()throws Exception {
        videoMode=0;RobotClient c=start();arm(c);enableBackendSafety();allNetworkFails=true;
        await(()->!serverArmed&&backendSafetyStops.get()>0,550,"all-network loss first removes backend motor authority within350ms");
        await(()->!c.isArmed()&&!c.isConnected(),1800,"continued loss of all transport ends phone authorization and session at finite1500ms age");
        int armsBefore=armCalls;allNetworkFails=false;Thread.sleep(250);
        check(!c.isConnected()&&!c.isArmed()&&armCalls==armsBefore,"late network recovery never reconnects or rearms automatically");
        finish(c);
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
            check(d.optLong("control_send_ms")>0&&d.optLong("control_ack_ms")>0,"control send and verified ACK source clocks captured before asynchronous STOP");
            check(d.optLong("control_rtt_ms")>=0&&d.optLong("tick_gap_ms")>=0&&d.optLong("tick_run_ms")>=0&&d.optLong("scheduler_delay_ms")>=0,"bounded control RTT and scheduling durations captured");
            check(d.optLong("state_age")>=minimumStateAge,"diagnostic retains original state age: "+reason);
            if("stop".equals(d.optString("op")))first=d;
            if("disarm".equals(d.optString("op")))second=d;
        }
        if(disarms){
            check(first!=null&&second!=null&&first.optLong("id")==second.optLong("id")&&first.optLong("at")==second.optLong("at"),"STOP and DISARM identify same captured client cause");
            check(first.optLong("control_send_ms")==second.optLong("control_send_ms")&&first.optLong("control_ack_ms")==second.optLong("control_ack_ms")&&first.optLong("control_rtt_ms")==second.optLong("control_rtt_ms"),"queued STOP and DISARM retain one immutable pre-stop control timing snapshot");
        }
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
        backendSafetyClock();
        if(args.length==1&&args[0].equals("cadence")){
            sustainedSubTimeoutControlLatency();controlFailures();releaseWithInflightDrive();
            System.out.println("PASS "+checks+" production control cadence / genuine missing ACK / STOP race checks");return;
        }
        if(args.length==1&&args[0].equals("state")){
            stateTransientAndStale();System.out.println("PASS "+checks+" production telemetry recovery / explicit re-arm checks");return;
        }
        if(args.length==1&&args[0].equals("boundaries")){
            controlCadenceBoundary(190);controlCadenceBoundary(195);
            System.out.println("PASS "+checks+" production190ms/195ms repeated response checks; backend authority remains unchanged");return;
        }
        if(args.length==1&&(args[0].equals("boundary170")||args[0].equals("boundary190"))){
            controlCadenceBoundary(args[0].equals("boundary170")?170:190);
            System.out.println("PASS "+checks+" production continuous response checks; no duplicate phone ACK-age withdrawal");return;
        }
        if(args.length==1&&args[0].equals("arm-clock")){
            sameMillisecondArmObservation();System.out.println("PASS "+checks+" production same-millisecond ARM observation checks");return;
        }
        if(args.length==1&&args[0].equals("v33-authority")){
            singleLateResponseCannotAuthorize();controlFailures();releaseWithInflightDrive();sameMillisecondArmObservation();
            System.out.println("PASS "+checks+" v33 backend authority / old single response / STOP and source-time checks");return;
        }
        check(ConnectionPolicy.responseEvidenceFresh(1349,1000)&&!ConnectionPolicy.responseEvidenceFresh(1350,1000),"single old response evidence remains bounded350ms without a repeating phone ACK-age timer");
        check(!ConnectionPolicy.responseEvidenceFresh(999,1000),"negative/backwards timing cannot count as fresh");
        check(ConnectionPolicy.telemetryFresh(2499,1000)&&!ConnectionPolicy.telemetryFresh(2500,1000),"telemetry loss has finite 1500ms horizon");
        check(ConnectionPolicy.nextVideoRetry(1600)==3000,"video retry bounded at 3 seconds");
        videoIndependent(1,"video EOF");videoIndependent(2,"video timeout");videoIndependent(3,"video 429");
        sameMillisecondArmObservation();singleLateResponseCannotAuthorize();sustainedSubTimeoutControlLatency();controlCadenceBoundary(190);controlCadenceBoundary(195);
        releaseWithInflightDrive();stateTransientAndStale();controlFailures();authFailures();backgroundCloseAndLateState();windowFocusPreservesSession();serverDisarmRetainsSession();videoOnlyLivenessDoesNotAuthorizeMotion();cancelledVideoGenerationCannotKeepAlive();totalTransportLoss();cachedStateValidation();diagnosticExplicitAndCancel();
        System.out.println("PASS "+checks+" production RobotClient network fault checks: independent video retries, exact JPEG, control lease, telemetry recovery, auth, late responses, background and close");
    }
}
