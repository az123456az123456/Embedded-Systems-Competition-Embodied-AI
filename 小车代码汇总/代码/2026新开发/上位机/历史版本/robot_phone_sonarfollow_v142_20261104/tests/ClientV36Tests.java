import cn.acg138.robotphone.*;
import org.json.JSONObject;
import java.io.*;
import java.net.*;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import javax.imageio.ImageIO;

/** Runs production RobotClient, real executors, HTTP parsing, and fault transport.
 * Only the HTTP adapter is simulated; this is not a physical WLAN/APK test. */
public final class ClientV36Tests extends ClientNetworkTests {
    static boolean booleanField(RobotClient client,String name) {
        try { java.lang.reflect.Field f=RobotClient.class.getDeclaredField(name);f.setAccessible(true);return f.getBoolean(client); }
        catch(Exception ex){throw new AssertionError(ex);}
    }
    static void atomicArmObservationRace() throws Exception {
        videoMode=0; RobotClient c=start();
        CountDownLatch entered=new CountDownLatch(1), release=new CountDownLatch(1);
        JSONObject.beforeArmedBooleanForTest=()->{
            JSONObject.beforeArmedBooleanForTest=null; entered.countDown();
            try { if(!release.await(1500,TimeUnit.MILLISECONDS))throw new AssertionError("ARM race fixture release deadline"); }
            catch(InterruptedException ex){throw new AssertionError(ex);}
        };
        check(entered.await(700,TimeUnit.MILLISECONDS),"poll paused between ARM epoch comparison and armed=false read");
        c.arm(10,15); await(()->armCalls==1,700,"ARM backend ACK arrives while old poll comparison is pending");
        check(!c.isArmed(),"ARM write waits for atomic false-observation section instead of changing its epoch midway");
        release.countDown(); await(c::isArmed,700,"new ARM commits after pre-ARM observation finishes");
        Thread.sleep(150); check(c.isArmed()&&serverArmed,"old false observation cannot revoke newly committed ARM");
        finish(c);
    }
    static void temporaryStateFailure() throws Exception {
        videoMode=0; RobotClient c=start(); arm(c);
        c.manual(.55f,0,false); await(()->nonzeroDrives>0,500,"motion before telemetry outage");
        stateFails=true;
        await(()->diagnosticRequests.stream().anyMatch(d->d.optString("reason").equals("telemetry_timeout")),2100,"stale telemetry emits zero-motion barrier");
        check(c.isArmed()&&serverArmed&&c.isConnected(),"1.5-second stale state does not revoke armed or disconnect");
        check(!operations.contains("stop")&&!operations.contains("disarm"),"state retry never emits implicit disarm");
        int before=nonzeroDrives; Thread.sleep(400);
        check(nonzeroDrives==before&&c.mode().equals("manual"),"old stick is not replayed during telemetry outage");
        stateFails=false; Thread.sleep(300);
        check(c.isArmed()&&serverArmed&&nonzeroDrives==before,"state recovery preserves authority but never restores throttle");
        c.manual(.4f,0,false); await(()->nonzeroDrives>before,600,"new explicit stick works after telemetry recovers"); finish(c);
    }
    static void rejectionAndTimeoutPause() throws Exception {
        videoMode=0; RobotClient c=start(); arm(c);
        c.manual(.4f,0,false); await(()->nonzeroDrives>0,500,"drive before transient rejection");
        transientControlCode=409;
        await(()->diagnosticRequests.stream().anyMatch(d->d.optString("reason").equals("control_rejected")),700,"409 pauses only motion");
        check(c.isArmed()&&serverArmed&&c.isConnected()&&c.mode().equals("manual"),"ordinary 409 keeps authenticated armed session");
        check(!operations.contains("stop")&&!operations.contains("disarm"),"409 does not send STOP/disarm");
        int before=nonzeroDrives; singleControlLoss=1; c.manual(.6f,0,false);
        await(()->diagnosticRequests.stream().anyMatch(d->d.optString("reason").equals("control_response_retry")),900,"lost response clears current intent");
        check(c.isArmed()&&serverArmed&&c.mode().equals("manual"),"response timeout retains authorization");
        int at=nonzeroDrives; Thread.sleep(250); check(nonzeroDrives==at&&at>=before,"timeout does not replay old motion"); finish(c);
    }
    static void focusAndMultitouch() throws Exception {
        videoMode=0; RobotClient c=start(); arm(c);
        c.manual(.5f,0,false); await(()->nonzeroDrives>0,500,"drive before second pointer cancellation");
        c.releaseStick(true); await(()->zeroDrives>0,500,"pointer cancellation transmits zero");
        check(c.isArmed()&&serverArmed&&c.mode().equals("manual"),"second pointer cancels gesture without disarming");
        int before=nonzeroDrives; c.windowFocusChanged(false); Thread.sleep(180);
        c.manual(.8f,0,false); c.rotate(true); c.startMapping(); Thread.sleep(1600);
        check(c.isConnected()&&c.isArmed()&&serverArmed&&nonzeroDrives==before,"focus loss retains authority while blocking motion");
        check(!operations.contains("stop")&&!operations.contains("disarm"),"focus/multitouch never emits implicit STOP");
        c.windowFocusChanged(true); Thread.sleep(200); check(nonzeroDrives==before,"focus restore never replays gesture");
        c.manual(.3f,0,false); await(()->nonzeroDrives>before,600,"fresh gesture resumes without rearm"); finish(c);
    }
    static void stopOvertakesSlowIo() throws Exception {
        videoMode=0; RobotClient c=start(); arm(c);
        blockControl=true; c.manual(.7f,0,false); await(()->controlEntered,700,"slow accepted drive HTTP is in flight");
        long began=now(); c.stop(true);
        await(()->!serverArmed,200,"STOP reaches backend independently of slow drive response");
        check(now()-began<200&&!controlFinished,"STOP is not blocked by control HTTP I/O lock");
        await(()->controlFinished,1000,"old HTTP response finishes after STOP"); Thread.sleep(100);
        check(!c.isArmed()&&!serverArmed&&armCalls==1,"late drive reply cannot restore authorization");
        finish(c);
        c=start(); arm(c); CountDownLatch entered=new CountDownLatch(1), release=new CountDownLatch(1);
        heldDriveEntered=entered; heldDriveRelease=release; c.manual(.5f,0,false);
        check(entered.await(500,TimeUnit.MILLISECONDS),"drive held before acceptance");
        c.stop(true); await(()->!serverArmed,200,"higher STOP sequence accepted before held drive");
        int moves=nonzeroDrives; release.countDown(); Thread.sleep(200);
        check(nonzeroDrives==moves&&!c.isArmed()&&!serverArmed,"lower late sequence cannot cross STOP barrier"); finish(c);
    }
    static void delayedArmAndOldState() throws Exception {
        videoMode=0; RobotClient c=start(); delayedOperation="arm"; delayedOperationMs=450; c.arm(10,15);
        await(()->delayedOperationEntered,700,"ARM request uses slower valid network");
        await(c::isArmed,1000,"450ms accepted current ARM is usable with 800ms HTTP timeout");
        long armSeq=longField(c,"armedSequence");
        stateOverride=state(false).put("control_sequence",armSeq+100).put("logical_armed",true);
        Thread.sleep(180); check(c.isArmed(),"temporary physical ACK disarm cannot override logical authority");
        stateOverride=state(false).put("control_sequence",armSeq-1); Thread.sleep(220);
        check(c.isArmed()&&serverArmed,"old GET sequence cannot revoke a newer ARM");
        stateOverride=null; c.stop(true); await(()->!serverArmed,500,"manual stop disarms");
        stateOverride=state(true).put("control_sequence",armSeq+100); Thread.sleep(150);
        check(!c.isArmed(),"GET armed=true never rearms after manual STOP"); finish(c);
        c=start(); delayedOperation="arm"; delayedOperationMs=700; c.arm(10,15);
        await(()->delayedOperationEntered,700,"ARM delayed before explicit stop"); c.stop(true);
        await(()->!serverArmed,200,"explicit STOP overtakes delayed ARM response");
        await(()->delayedOperationFinished,1000,"late ARM response arrives"); Thread.sleep(100);
        check(!c.isArmed()&&!serverArmed,"late accepted ARM response cannot rearm stopped UI or backend"); finish(c);
    }
    static void preArmGetNeverRevokesNewArm() throws Exception {
        videoMode=0; RobotClient c=start(); refreshHeldSequence=true;
        CountDownLatch entered=new CountDownLatch(1), release=new CountDownLatch(1);
        heldStateEntered=entered; heldStateRelease=release;
        check(entered.await(500,TimeUnit.MILLISECONDS),"pre-ARM false GET captured before authorization");
        arm(c); int delivered=stateDeliveries.get(); release.countDown();
        await(()->stateDeliveries.get()>delivered,700,"old GET arrives with misleading updated sequence");
        check(c.isArmed()&&serverArmed,"pre-ARM GET cannot revoke new ARM even if cache seq is updated independently");
        finish(c);
    }
    static void safePauseKeepsSelection() throws Exception {
        videoMode=0; RobotClient c=start(); arm(c);
        c.select(42,7); await(()->messages.contains("目标 #7 已确认，按“跟随目标”开始"),700,"selected target acknowledged");
        c.pauseMotion("settings_open"); Thread.sleep(80);
        check(c.isArmed()&&serverArmed&&c.selectedTarget()==7,"settings pause retains selected orange target and authority");
        c.releaseRotation(true); Thread.sleep(80);
        check(c.isArmed()&&serverArmed,"rotation pointer cancel does not disarm"); finish(c);
    }
    static void activeFollowSurvivesOneLostReply() throws Exception {
        videoMode=0;RobotClient c=start();arm(c);c.select(42,7);
        await(()->messages.contains("目标 #7 已确认，按“跟随目标”开始"),700,"follow target acknowledged");
        stateOverride=state(true).put("mode","follow").put("selected_id",7).put("reason","following");
        c.follow();
        await(()->messages.contains("目标跟随已启动，保持约 0.4 米；再次点击取消"),700,"follow ACK replaces stale selection-only prompt");
        await(()->booleanField(c,"followAcknowledged"),700,"fresh follow telemetry observed");
        singleControlLoss=1;
        await(()->singleControlLoss==0,700,"one follow response lost");
        Thread.sleep(280);
        check(c.mode().equals("follow")&&c.isArmed(),"one lost reply retains explicitly active follow while telemetry is fresh");
        check(diagnosticRequests.stream().noneMatch(d->d.optString("reason").equals("control_response_retry")),"follow retry does not inject a cancelling zero-drive command");
        stateOverride=state(true).put("mode","follow").put("selected_id",7).put("reason","follow_wait_command");
        Thread.sleep(180);check(c.mode().equals("follow"),"waiting for fresh command is a hold, not task completion");
        c.follow();check(c.mode().equals("manual"),"same button still cancels retained follow");
        await(()->zeroDrives>0,600,"follow cancel sends zero barrier");
        int sent=operations.size();Thread.sleep(180);
        check(operations.subList(sent,operations.size()).stream().noneMatch(op->op.equals("follow")),"late healthy telemetry cannot restart cancelled follow");
        stateOverride=null;finish(c);
    }
    static void noJsonDoesNotRenewLease() throws Exception {
        videoMode=6; RobotClient c=start(); arm(c); enableBackendSafety();
        await(()->videoFrames.get()>1,600,"continuous valid video before control outage");
        long lease=longField(c,"lastLeaseAt"); jsonNetworkFails=true; long began=now();
        while(now()-began<30200) {
            Thread.sleep(100);
            if(now()-began<29500)check(serverArmed,"backend authority lasts until actual30-second command horizon");
            check(c.isConnected(),"video keeps transport while command lease is independent");
        }
        await(()->!serverArmed&&backendSafetyStops.get()>0,500,"30 seconds without accepted command revokes backend authority");
        check(videoFrames.get()>200&&longField(c,"lastLeaseAt")==lease,"GET/video neither renew command lease nor fake ACK");
        jsonNetworkFails=false;
        await(()->!c.isArmed(),1000,"fresh authoritative lease expiry updates phone");
        check(c.isConnected()&&armCalls==1,"lease expiry never disconnects or automatically rearms"); finish(c);
    }
    static void temporaryBackground() throws Exception {
        videoMode=1; RobotClient c=start(); arm(c);
        c.manual(.4f,0,false); await(()->nonzeroDrives>0,500,"motion before temporary background");
        c.background(); Thread.sleep(180); int before=nonzeroDrives, attempts=videoAttempts;
        Thread.sleep(650);
        check(c.isConnected()&&c.isArmed()&&serverArmed&&videoAttempts==attempts,"short Activity pause retains session and authority while suspending video");
        check(!operations.contains("stop")&&!operations.contains("disarm"),"temporary background does not implicitly disarm");
        c.foreground(); Thread.sleep(200);
        check(nonzeroDrives==before&&c.mode().equals("manual"),"return to app never restores prior motion");
        c.close(); await(()->!serverArmed,500,"explicit disconnect immediately disarms");
        check(!c.isConnected()&&!c.isArmed(),"closed session cannot retain authority"); Thread.sleep(100);
    }
    static void resumeRequiresNewAuthority() throws Exception {
        videoMode=0; RobotClient c=start(); arm(c); c.background();
        await(()->zeroDrives>0,500,"background zero barrier reached backend");
        serverArmed=false; stateFails=true; int before=acceptedControls.size(); c.foreground();
        c.manual(.8f,0,false); c.rotate(true); Thread.sleep(240);
        check(acceptedControls.size()==before&&c.mode().equals("manual"),"resume cannot renew or move before new authoritative state");
        stateFails=false; await(()->!c.isArmed(),700,"fresh post-resume expired lease clears old local authority");
        check(armCalls==1&&!serverArmed,"resume never implicitly rearms expired owner"); finish(c);
    }
    static void genuineTransportLoss() throws Exception {
        videoMode=0; RobotClient c=start(); arm(c); allNetworkFails=true;
        setField(c,"lastTransportAt",now()-ConnectionPolicy.CONNECTION_LOSS_MS+180);
        await(()->!c.isConnected()&&!c.isArmed(),700,"true30-second all-channel loss disconnects and disarms");
        allNetworkFails=false; Thread.sleep(100); check(!c.isConnected()&&armCalls==1,"late recovery cannot reconnect or rearm automatically"); finish(c);
    }
    public static void main(String[] args) throws Exception {
        Thread.setDefaultUncaughtExceptionHandler((thread,error)->{error.printStackTrace();System.exit(1);});
        java.awt.image.BufferedImage image=new java.awt.image.BufferedImage(8,6,java.awt.image.BufferedImage.TYPE_INT_RGB);
        ByteArrayOutputStream jpeg=new ByteArrayOutputStream(); ImageIO.write(image,"jpg",jpeg); jpg=jpeg.toByteArray();
        URL.setURLStreamHandlerFactory(protocol->protocol.equals("http")?new URLStreamHandler(){protected URLConnection openConnection(URL url){return new Connection(url);}}:null);
        backendSafetyClock();
        if(args.length==1&&args[0].equals("arm-race")){atomicArmObservationRace();System.out.println("PASS "+checks+" deterministic ARM-vs-state atomicity checks");return;}
        if(args.length==1&&args[0].equals("follow-retry")){activeFollowSurvivesOneLostReply();System.out.println("PASS "+checks+" active follow retry checks");return;}
        atomicArmObservationRace();activeFollowSurvivesOneLostReply();
        check(ConnectionPolicy.CONTROL_HTTP_TIMEOUT_MS==800,"normal HTTP budget supports delayed RK responses");
        check(ConnectionPolicy.CONNECTION_LOSS_MS==30000,"all-channel loss horizon is30 seconds");
        videoIndependent(1,"video EOF"); videoIndependent(2,"video timeout"); videoIndependent(3,"video429");
        temporaryStateFailure(); rejectionAndTimeoutPause(); focusAndMultitouch();
        stopOvertakesSlowIo(); delayedArmAndOldState(); preArmGetNeverRevokesNewArm(); sameMillisecondArmObservation(); safePauseKeepsSelection(); authFailures();
        temporaryBackground(); resumeRequiresNewAuthority(); genuineTransportLoss(); noJsonDoesNotRenewLease();
        System.out.println("PASS "+checks+" v36 production client checks: 30s accepted-command lease, slow HTTP, stale telemetry, multi-touch/focus, STOP precedence, old GET/ARM, exact JPEG and reconnect separation");
    }
}
