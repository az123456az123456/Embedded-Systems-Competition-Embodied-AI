package cn.acg138.robotphone;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.SystemClock;
import org.json.JSONArray;
import org.json.JSONObject;
import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicInteger;

/** Video, telemetry and motor commands use separate threads and HTTP connections. */
public final class RobotClient {
    public interface Listener {
        void message(String message);
        void connected(boolean value);
        void state(JSONObject state);
        void frame(Frame frame);
        default void navigation(String message, NavigationTarget target) { message(message); }
    }
    public static final class Frame {
        public final Bitmap image;
        public final JSONObject metadata;
        /** The immutable source JPEG belonging to metadata/id; never reused by the decoder. */
        public final byte[] jpeg;
        public final long id, received;
        Frame(Bitmap image, JSONObject metadata, long id) { this(image, metadata, id, null); }
        Frame(Bitmap image, JSONObject metadata, long id, byte[] jpeg) {
            this(image, metadata, id, jpeg, SystemClock.elapsedRealtime());
        }
        Frame(Bitmap image, JSONObject metadata, long id, byte[] jpeg, long received) {
            this.image = image; this.metadata = metadata; this.id = id; this.jpeg = jpeg;
            this.received = received;
        }
    }
    private final String base, token;
    private final Listener listener;
    private final ScheduledExecutorService control = Executors.newSingleThreadScheduledExecutor();
    private final ScheduledExecutorService telemetry = Executors.newSingleThreadScheduledExecutor();
    private final ExecutorService video = Executors.newSingleThreadExecutor();
    private final ExecutorService videoDecoder = Executors.newSingleThreadExecutor();
    private final Object decodeWake = new Object();
    private PendingVideo pendingVideo;
    private static final class PendingVideo {
        final MultipartReader.Part part; final JSONObject meta;
        final long epoch, videoEpoch, received;
        PendingVideo(MultipartReader.Part p, JSONObject m, long e, long ve, long r) {
            part=p;meta=m;epoch=e;videoEpoch=ve;received=r;
        }
    }
    private final ExecutorService navigation = Executors.newSingleThreadExecutor();
    private final ExecutorService emergency = Executors.newCachedThreadPool();
    // This watchdog cannot be held up by an HTTP response or JPEG decoding.
    private final ScheduledExecutorService health = Executors.newSingleThreadScheduledExecutor();
    private final Object videoWake = new Object();
    private final AtomicBoolean started = new AtomicBoolean();
    private final AtomicLong sessionGeneration = new AtomicLong(), videoGeneration = new AtomicLong();
    // lastLeaseAt is retained as the original request-source clock for log
    // compatibility only; it does not authorize or revoke phone motor state.
    private volatile long connectedAt, lastLeaseAt, lastTelemetryWarningAt, lastControlWarningAt, lastTransportAt;
    // Diagnostic clocks only. These values never authorize or renew movement.
    private volatile long lastControlSendAt, lastControlAckAt, lastControlRttMs;
    private volatile long lastControlTickAt, lastControlTickGapMs, lastControlTickRunMs, lastControlSchedulerDelayMs;
    private final ConcurrentLinkedQueue<JSONObject> actions = new ConcurrentLinkedQueue<>();
    private final AtomicLong sequence = new AtomicLong(), completedSequence = new AtomicLong(), navigationSequence = new AtomicLong();
    private volatile NavigationTarget navigationTarget;
    private volatile String navigationOperation="";
    private volatile boolean mappingStopPending;
    private volatile long latestStateAt;
    private final AtomicLong diagnosticSequence = new AtomicLong();
    private volatile long navigationAcceptedAt;
    private volatile String navigationStatus="";
    private final AtomicLong safetyGeneration = new AtomicLong();
    private final AtomicInteger disarmsInFlight = new AtomicInteger();
    private final Object navigationLock = new Object(), commandOrder = new Object();
    private final AtomicBoolean failed = new AtomicBoolean();
    private final SelectionGate selection = new SelectionGate();
    private final ArmHandshake armHandshake = new ArmHandshake();
    private volatile boolean closed, connected, armed, foreground = true, windowFocused = true;
    private volatile String session = "", mode = "manual";
    private volatile float throttle, steer;
    private volatile long armedAt, armedSequence, followStartedAt, followAcceptedAt;
    private volatile boolean telemetryPaused, resumeNeedsState;
    private volatile long foregroundResumedAt;
    private volatile boolean followAcknowledged;
    private volatile JSONObject latestState;
    private volatile HttpURLConnection videoConnection;
    public RobotClient(String base, String token, Listener listener) {
        this.base = LanAddress.normalize(base); this.token = token; this.listener = listener;
    }
    public boolean isConnected() { return connected && !closed; }
    public boolean isArmed() { return armed && isConnected(); }
    public String mode() { return mode; }
    public String followDiagnostics() { return FollowPresentation.diagnostics(latestState, selection.display()); }
    // Read at UI delivery time: a queued telemetry snapshot may predate a tap.
    public long selectedTarget() { return selection.display(); }
    public void start() {
        if (closed || !started.compareAndSet(false, true)) return;
        control.execute(() -> {
            try {
                JSONObject answer = request("POST", "/api/connect", object("client", "Android native 1.2.25"), 1500);
                session = answer.getString("session");
                if (session.length() < 8 || session.length() > 256) throw new IOException("会话无效");
                if (closed || !foreground) { bestEffortDisarm(); return; }
                sessionGeneration.incrementAndGet(); connectedAt = SystemClock.elapsedRealtime(); connected = true; armed = false;
                listener.connected(true); listener.message("已连接，电机未使能");
                telemetry.scheduleWithFixedDelay(this::poll, 0, 100, TimeUnit.MILLISECONDS);
                video.execute(this::stream);
                videoDecoder.execute(this::decodeVideo);
                // Keep one serialized worker and read CURRENT intent each tick.
                // Fixed rate avoids extra post-response idle. Actual motor
                // leases belong to the RK/FPGA, where command arrival is known;
                // phone response latency cannot revoke a fresh backend owner.
                // Requests cannot overlap or extend old/late motion authority.
                control.scheduleAtFixedRate(this::tick, 0, 50, TimeUnit.MILLISECONDS);
                health.scheduleWithFixedDelay(this::checkHealth, 20, 20, TimeUnit.MILLISECONDS);
            } catch (Exception ex) { networkFailure(ex); }
        });
    }
    public void arm(int leftMask, int invertMask) {
        if (!isConnected() || !foreground || !windowFocused) return;
        if (isArmed()) { listener.message("电机已使能"); return; }
        if (mappingStopPending) { listener.message("正在结束建图，请等待请求确认"); return; }
        if (disarmsInFlight.get() != 0) { listener.message("停车请求确认中，请稍后再使能"); return; }
        if (latestStateAt > 0 && !ConnectionPolicy.telemetryFresh(SystemClock.elapsedRealtime(), latestStateAt)) {
            listener.message("状态尚未恢复，请稍后再使能电机"); return;
        }
        if (Integer.bitCount(leftMask & 15) != 2) { listener.message("请选择两个左侧电机"); return; }
        if (!armHandshake.begin(safetyGeneration.get())) { listener.message("正在等待 FPGA 确认，请稍候…"); return; }
        throttle = steer = 0; mode = "manual";
        selection.clear(SystemClock.elapsedRealtime());
        queue(object("op", "configure", "left_mask", leftMask, "invert_mask", invertMask));
        listener.message("正在配置电机，等待 FPGA 确认…");
    }
    public void manual(float throttle, float steer, boolean hold) {
        if (!motionAllowed()) return;
        cancelNavigationIntent();
        this.throttle = Math.max(-1, Math.min(1, throttle));
        this.steer = Math.max(-1, Math.min(1, steer));
        this.mode = hold ? "hold" : "manual";
    }
    public void releaseStick(boolean cancel) {
        cancelNavigationIntent();
        throttle = steer = 0;
        if (cancel) { pauseMotion("touch_cancel"); return; }
        mode = "manual";
        safetyGeneration.incrementAndGet(); actions.clear();
        if (isArmed()) immediateStop(false, "joystick_release");
    }
    public void rotate(boolean clockwise) {
        cancelNavigationIntent();
        if (!isArmed()) { listener.message("请先使能电机"); return; }
        if (!motionAllowed()) { listener.message("运动已暂停，请等待状态恢复"); return; }
        throttle = steer = 0;
        mode = clockwise ? "rotate_cw" : "rotate_ccw";
    }
    public void releaseRotation(boolean cancel) {
        // Invalidate a queued rotate before STOP can be overtaken. A normal
        // release and pointer cancellation stop intent while preserving authorization.
        if (cancel) { pauseMotion("rotation_cancel"); return; }
        throttle = steer = 0; mode = "manual";
        safetyGeneration.incrementAndGet(); actions.clear();
        if (isArmed()) immediateStop(false, "rotation_release");
    }
    public void select(long frameId, long targetId) {
        cancelNavigationIntent();
        if (!isConnected()) return;
        // A fresh tap cancels all previous movement intent. Only the explicit
        // Follow button may start following the new target after its ACK.
        throttle = steer = 0; mode = "manual";
        long request = selection.request(targetId);
        queue(object("op", "select", "frame_id", frameId, "target_id", targetId, "_selection", request));
        listener.message("正在确认目标 #" + targetId + "…");
    }
    public void follow() {
        cancelNavigationIntent();
        if (!isArmed()) { listener.message("请先使能电机"); return; }
        if (!motionAllowed()) { listener.message("运动已暂停，请等待状态恢复"); return; }
        if (selection.pending()) { listener.message("正在确认选框，请稍后再按跟随"); return; }
        if (selection.confirmed() <= 0) { listener.message("请先点击视频中的目标框"); return; }
        String blocked = FollowPresentation.beforeFollow(latestState, selection.confirmed());
        if (!blocked.isEmpty()) { listener.message(blocked); return; }
        throttle = steer = 0; followAcknowledged = false;
        followAcceptedAt = 0; followStartedAt = SystemClock.elapsedRealtime(); mode = "follow";
    }
    public void setManualMode(boolean hold) {
        cancelNavigationIntent();
        throttle = steer = 0; mode = hold ? "hold" : "manual";
        if (isArmed()) queue(object("op", "drive", "throttle", 0, "steer", 0, "hold", false));
    }
    private void cancelNavigationIntent() {
        synchronized(navigationLock){
        if(navigationTarget!=null||!navigationOperation.isEmpty()||"navigation".equals(mode)||"mapping".equals(mode)) {
            navigationTarget=null;navigationOperation="";navigationAcceptedAt=0;safetyGeneration.incrementAndGet();actions.clear();
        }
        }
    }
    private boolean fpgaNavigation() {
        JSONObject nav=latestState==null?null:latestState.optJSONObject("navigation");
        return nav!=null&&"FPGA_candidate26".equals(nav.optString("execution_source"));
    }
    public void startNavigation() {
        if(!fpgaNavigation()){listener.navigation("请先更新 RK 的 FPGA 自主导航适配，当前仅可手动操控",null);return;}
        if ("mapping".equals(mode) || mappingStopPending || MappingPresentation.busy(latestState)) {
            listener.navigation(MappingPresentation.status(latestState)+"；完成载入后才能导航",null);return;
        }
        if(MappingPresentation.loadFailed(latestState)){listener.navigation(MappingPresentation.status(latestState),null);return;}
        if(!isArmed()){listener.navigation("请先连接并使能电机，再开始导航",null);return;}
        if (!foreground || !windowFocused || resumeNeedsState) return;
        if("navigation".equals(mode)){listener.navigation("导航已在运行，可随时结束或使用摇杆接管",navigationTarget);return;}
        long now=SystemClock.elapsedRealtime();
        if(latestState==null||now<latestStateAt||now-latestStateAt>1000){listener.navigation("状态未更新，请稍后再开始导航",null);return;}
        JSONObject nav=latestState.optJSONObject("navigation");
        if(nav==null||!nav.optBoolean("supports_start_stop")||nav.optInt("start_protocol_version",0)<2){
            listener.navigation("请先安装 RK 导航 v23 更新，再使用开始导航",null);return;
        }
        if(nav.has("can_start")&&!nav.optBoolean("can_start")){
            listener.navigation(NavigationTarget.reason(nav.optString("readiness_reason","navigation_not_ready")),null);return;
        }
        if(fpgaNavigation()&&(navigationTarget==null||!"goal".equals(navigationOperation))){
            listener.navigation("请先选择终点并预览路线，再点击开始导航",null);return;
        }
        beginNavigation("start",fpgaNavigation()?navigationTarget:null);
    }
    public void startMapping() {
        synchronized(navigationLock) {
            if(!isArmed()){listener.navigation("请先连接并使能电机，再探索建图",null);return;}
            if(!foreground || !windowFocused || resumeNeedsState)return;
            if(mappingStopPending){listener.navigation("正在结束建图，请等待请求确认",null);return;}
            if("mapping".equals(mode)||MappingPresentation.busy(latestState)){
                listener.navigation("探索建图正在运行或处理中，请先结束建图",null);return;
            }
            if("navigation".equals(mode)||"follow".equals(mode)||mode.startsWith("rotate_")){
                listener.navigation("请先停止当前任务，再探索建图",null);return;
            }
            long now=SystemClock.elapsedRealtime();
            if(latestState==null||now<latestStateAt||now-latestStateAt>1000){
                listener.navigation("状态未更新，请稍后再探索建图",null);return;
            }
            if(!MappingPresentation.supported(latestState)){
                listener.navigation("RK 尚未接入探索建图服务，请更新 RK 整合程序",null);return;
            }
            beginNavigation("mapping-start",null);
        }
    }
    public void endMapping() {
        synchronized(navigationLock) {
            if(mappingStopPending)return;
            if(!isConnected()){listener.navigation("连接已断开，无法确认结束建图，请检查 RK",null);return;}
            mappingStopPending=true;
            // Revoke local intent immediately. STOP uses the independent emergency
            // lane; saving/loading is ordered after any in-flight mapping-start.
            final String endingSession=session;
            final long stopSeq=navigationSequence.incrementAndGet();
            final CompletableFuture<Boolean> stopped=stop(true,"navigation_end");
            listener.navigation("正在停车并结束建图，等待 RK 确认…",null);
            try { navigation.execute(() -> {
                try {
                    if(closed || !endingSession.equals(session))return;
                    // Both STOP and DISARM must finish before Gateway prepares
                    // the quiet map-upload phase. A late DISARM would release
                    // quiet mode and restart MOT probes, resetting FPGA's
                    // measured-stop window on every probe.
                    if(!stopped.get(2500,TimeUnit.MILLISECONDS))
                        throw new IOException("stop/disarm not acknowledged");
                    if(closed || !endingSession.equals(session))return;
                    request("POST","/api/navigation",object("op","mapping-stop",
                            "session",endingSession,"nav_seq",stopSeq),1200);
                    listener.navigation("结束建图请求已接收，等待停车、保存和载入确认",null);
                } catch(Exception ex) {
                    if(sessionFailure(ex))networkFailure(ex);
                    else if(!closed)listener.navigation("结束建图未确认，请重试；尚不能确认地图已保存",null);
                } finally { mappingStopPending=false; }
            }); } catch(java.util.concurrent.RejectedExecutionException ignored) {mappingStopPending=false;}
        }
    }
    public void endNavigation() {
        if("mapping".equals(mode)||mappingStopPending){endMapping();return;}
        // Invalidate locally before the independent emergency thread sends the
        // serialized STOP/disarm. A late start/goto cannot restore this intent.
        synchronized(navigationLock){stop(true, "navigation_end");listener.navigation("导航已结束，电机已停用",null);}
    }
    public void navigate(NavigationTarget target) {
        if(!fpgaNavigation()){listener.navigation("请先更新 RK 的 FPGA 自主导航适配，选点不会启动旧导航",null);return;}
        if("mapping".equals(mode)||mappingStopPending||MappingPresentation.busy(latestState)){
            listener.navigation(MappingPresentation.status(latestState)+"；完成载入后才能选点",null);return;
        }
        if(MappingPresentation.loadFailed(latestState)){listener.navigation(MappingPresentation.status(latestState),null);return;}
        if(!motionAllowed()){listener.navigation("请先使能电机并等待状态恢复，再选择目的地",null);return;}
        if(target==null||!target.fresh(SystemClock.elapsedRealtime())){listener.navigation("地图已过期，请重新选择",null);return;}
        beginNavigation("goal",target);
    }
    private void beginNavigation(String operation,NavigationTarget target) {
        synchronized(navigationLock){
        if(!isArmed())return;
        long generation=safetyGeneration.incrementAndGet(),navSeq=navigationSequence.incrementAndGet();
        actions.clear();throttle=steer=0;mode="mapping-start".equals(operation)?"mapping":"goal".equals(operation)?"preview":"navigation";navigationTarget=target;navigationOperation=operation;navigationAcceptedAt=0;
        selection.clear(SystemClock.elapsedRealtime());
        if("mapping-start".equals(operation)||fpgaNavigation()) {
            // Native FPGA exploration is a task request, never a legacy DRIVE,
            // claim or an implicit ARM. The Gateway owns NVA1/START admission.
            listener.navigation("mapping-start".equals(operation)?"正在请求探索建图，等待 RK 确认…":"goal".equals(operation)?"正在预览路线，不会行驶":"正在请求 FPGA 开始导航…",target);
            submitNavigation(navSeq,generation,completedSequence.get(),operation,target);
            return;
        }
        // The successful zero DRIVE is the server-validated intent barrier.
        // Navigation uses an independent worker; 50ms idle leases keep flowing.
        queue(object("op","drive","throttle",0,"steer",0,"hold",false,"_navigation",navSeq,
                     "_navigation_op",operation,"_motion_generation",generation));
        listener.navigation("start".equals(operation)?"正在开始导航…":"正在提交目的地…",target);
        }
    }
    private boolean currentNavigation(long navSeq,long generation,String operation,NavigationTarget target) {
        return generation==safetyGeneration.get()&&navSeq==navigationSequence.get()&&isArmed()&&foreground&&windowFocused&&
                ("navigation".equals(mode)||"mapping".equals(mode)||"preview".equals(mode))&&operation.equals(navigationOperation)&&navigationTarget==target;
    }
    private void submitNavigation(long navSeq,long generation,long intentSeq,String operation,NavigationTarget target) {
        try { navigation.execute(() -> {
            if(!currentNavigation(navSeq,generation,operation,target))return;
            try {
                JSONObject requestBody=object("op",operation,"session",session,"intent_seq",intentSeq,"nav_seq",navSeq);
                if("goto".equals(operation)||"goal".equals(operation)){
                    if(target==null||!target.fresh(SystemClock.elapsedRealtime()))throw new ApiException("map_snapshot_expired");
                    requestBody.put("x",target.x);requestBody.put("y",target.y);
                    requestBody.put("map_session",target.map.sessionId);requestBody.put("map_revision",target.map.revision);
                }
                request("POST","/api/navigation",requestBody,1200);
                synchronized(navigationLock){if(currentNavigation(navSeq,generation,operation,target)) {
                    navigationAcceptedAt=SystemClock.elapsedRealtime();navigationStatus="";
                    listener.navigation("mapping-start".equals(operation)?"探索请求已接收，等待 RK 开始建图":"goal".equals(operation)?"终点已提交，路线仅预览；点击开始才行驶":"start".equals(operation)?"导航已开始，正在规划路线":"目的地已接受，正在规划路线",target);
                }}
            } catch(Exception ex) {
                if (sessionFailure(ex)) { networkFailure(ex); return; }
                synchronized(navigationLock){if(currentNavigation(navSeq,generation,operation,target)) {
                    pauseMotion("navigation_rejected");listener.navigation(NavigationTarget.reason(ex instanceof ApiException ? ((ApiException)ex).code : ex.getMessage()),null);
                }}
            }
        }); } catch(java.util.concurrent.RejectedExecutionException ignored) { }
    }
    public void imuAction(boolean calibrate) {
        pauseMotion(calibrate ? "imu_calibrate" : "imu_zero");
        final long generation = safetyGeneration.get();
        emergency.execute(() -> {
            if (!closed && isConnected() && generation == safetyGeneration.get()) {
                try {
                    command(object("op", "drive", "throttle", 0, "steer", 0, "hold", false, "_safe_zero", true, "_motion_generation", generation));
                    command(object("op", calibrate ? "calibrate" : "zero", "_motion_generation", generation));
                }
                catch (Exception ex) { if (sessionFailure(ex)) networkFailure(ex); else if (!closed) listener.message(ex.getMessage()); }
            }
        });
    }
    private boolean motionAllowed() {
        if (!isArmed() || !foreground || !windowFocused || resumeNeedsState) return false;
        if (!ConnectionPolicy.telemetryFresh(SystemClock.elapsedRealtime(), latestStateAt)) {
            checkHealth(); return false;
        }
        return true;
    }
    public void pauseMotion(String reason) {
        synchronized(navigationLock) {
            navigationTarget = null; navigationOperation = ""; navigationAcceptedAt = 0;
            throttle = steer = 0; mode = "manual";
            safetyGeneration.incrementAndGet(); actions.clear(); armHandshake.cancel();
            selection.cancelPending(SystemClock.elapsedRealtime());
            // Keep the selected target for display. Starting follow/navigation again
            // always needs a fresh explicit tap, never a recovered HTTP reply.
            immediateStop(false, reason);
        }
    }
    private void queue(JSONObject action) {
        if (actions.size() < 12 && !closed) {
            try { action.put("_generation", safetyGeneration.get()); } catch (Exception ignored) { }
            actions.offer(action);
        }
    }
    public void stop(boolean disarm) { stop(disarm, "explicit_stop"); }
    CompletableFuture<Boolean> stop(boolean disarm, String reason) {
        synchronized(navigationLock){
        navigationTarget=null;navigationOperation="";navigationAcceptedAt=0;
        throttle = steer = 0; mode = "manual";
        safetyGeneration.incrementAndGet();
        armHandshake.cancel();
        selection.clear(SystemClock.elapsedRealtime());
        actions.clear();
        if (disarm) armed = false;
        return immediateStop(disarm, reason);
        }
    }
    private void immediateStop(boolean disarm) { immediateStop(disarm, "stop_request"); }
    private CompletableFuture<Boolean> immediateStop(boolean disarm, String reason) {
        final CompletableFuture<Boolean> confirmed=new CompletableFuture<>();
        if (session.isEmpty()) { confirmed.complete(false); return confirmed; }
        final long at=SystemClock.elapsedRealtime(), event=diagnosticSequence.incrementAndGet();
        final long stateAge=latestStateAt>0&&at>=latestStateAt?at-latestStateAt:-1;
        final long leaseAge=lastLeaseAt>0&&at>=lastLeaseAt?at-lastLeaseAt:-1;
        final JSONObject timing=object("client_control_send_ms",lastControlSendAt,
                "client_control_ack_ms",lastControlAckAt,"client_control_rtt_ms",lastControlRttMs,
                "client_tick_gap_ms",lastControlTickGapMs,"client_tick_run_ms",lastControlTickRunMs,
                "client_scheduler_delay_ms",lastControlSchedulerDelayMs);
        final String cause=reason;
        final long stopGeneration=safetyGeneration.get();
        if (disarm) disarmsInFlight.incrementAndGet();
        try {
            emergency.execute(() -> {
                boolean acknowledged=false;
                try {
                    // Backend STOP also disarms. Zero DRIVE stops a released
                    // hold button without unexpectedly clearing its arm state.
                    JSONObject first=disarm ? object("op", "stop") : object("op", "drive", "throttle", 0, "steer", 0, "hold", false, "_safe_zero", true);
                    if (!disarm) first.put("_motion_generation", stopGeneration);
                    attachStopDiagnostics(first,cause,event,at,stateAge,leaseAge,timing);
                    command(first);
                    if (disarm) {
                        JSONObject second=object("op", "disarm");
                        attachStopDiagnostics(second,cause,event,at,stateAge,leaseAge,timing);
                        command(second);
                    }
                    acknowledged=true;
                } catch (Exception ignored) { /* Server watchdog also expires the lease. */ }
                finally { if (disarm) disarmsInFlight.decrementAndGet(); confirmed.complete(acknowledged); }
            });
        } catch (java.util.concurrent.RejectedExecutionException ignored) { if (disarm) disarmsInFlight.decrementAndGet(); confirmed.complete(false); }
        return confirmed;
    }
    private static void attachStopDiagnostics(JSONObject command,String reason,long event,long at,long stateAge,long leaseAge,JSONObject timing) throws Exception {
        command.put("client_reason",reason);command.put("client_diag_id",event);
        command.put("client_mono_ms",at);command.put("client_state_age_ms",stateAge);
        command.put("client_lease_age_ms",leaseAge);
        for(String key:new String[]{"client_control_send_ms","client_control_ack_ms","client_control_rtt_ms",
                "client_tick_gap_ms","client_tick_run_ms","client_scheduler_delay_ms"})command.put(key,timing.optLong(key));
    }
    /** A system window may take focus while this Activity remains visible. Stop
     * every motion intent, but retain the authenticated session and telemetry.
     * Focus restoration never re-arms or replays an autonomous operation. */
    public void windowFocusChanged(boolean hasFocus) {
        if (closed || windowFocused == hasFocus) return;
        windowFocused = hasFocus;
        if (!hasFocus) {
            pauseMotion("window_focus_lost");
            listener.message("控制界面暂时失焦，已暂停运动；连接与电机使能保持");
        }
    }
    public void background() { background("activity_pause"); }
    private void background(String reason) {
        foreground = false; resumeNeedsState = true; videoGeneration.incrementAndGet(); pauseMotion(reason);
        HttpURLConnection stream = videoConnection;
        if (stream != null) stream.disconnect();
        synchronized (videoWake) { videoWake.notifyAll(); }
    }
    public void foreground() {
        foregroundResumedAt = SystemClock.elapsedRealtime(); resumeNeedsState = true;
        foreground = true; // A new authoritative GET must precede any renewed control.
        synchronized (videoWake) { videoWake.notifyAll(); }
    }
    private void tick() {
        if (!isConnected() || !foreground || resumeNeedsState) return;
        final long tickStarted=SystemClock.elapsedRealtime();
        lastControlTickGapMs=lastControlTickAt>0&&tickStarted>=lastControlTickAt?tickStarted-lastControlTickAt:0;
        lastControlSchedulerDelayMs=Math.max(0,lastControlTickGapMs-Math.max(50,lastControlTickRunMs));
        lastControlTickAt=tickStarted;
        String attemptedOp = "";
        final long tickEpoch = sessionGeneration.get(), tickGeneration = safetyGeneration.get();
        try {
            JSONObject action = actions.poll();
            if (action != null) {
                String op = action.optString("op"); attemptedOp = op;
                long generation = action.optLong("_generation", -1);
                long selectionRequest = action.optLong("_selection", -1);
                long navSeq=action.optLong("_navigation",-1);
                String navOp=action.optString("_navigation_op","");
                action.remove("_generation");
                action.remove("_selection");
                action.remove("_navigation");
                action.remove("_navigation_op");
                if (generation != safetyGeneration.get()) return;
                command(action);
                NavigationTarget target=navigationTarget;
                if(navSeq>0&&currentNavigation(navSeq,generation,navOp,target))
                    submitNavigation(navSeq,generation,completedSequence.get(),navOp,target);
                if ("configure".equals(op)) armHandshake.configurationSent(generation, SystemClock.elapsedRealtime());
                if ("select".equals(op) && selection.acknowledge(selectionRequest, SystemClock.elapsedRealtime()))
                    listener.message("目标 #" + selection.confirmed() + " 已确认，按“跟随目标”开始");
                return;
            }
            if (armHandshake.pending()) {
                final long generation = safetyGeneration.get();
                long stateStarted = SystemClock.elapsedRealtime();
                JSONObject status = request("GET", "/api/state", null, ConnectionPolicy.CONTROL_HTTP_TIMEOUT_MS);
                JSONObject motor = status.optJSONObject("motor");
                boolean usableState = parseStateSourceAt(status, stateStarted) >= 0;
                int result = armHandshake.observe(generation, usableState && motor != null && motor.optBoolean("link_ok", false), SystemClock.elapsedRealtime());
                if (result == ArmHandshake.TIMED_OUT) {
                    String reason = motor == null ? "没有 FPGA 状态" : motor.optString("reason", motor.optString("status", "未收到匹配的电机配置 ACK"));
                    throw new ApiException("FPGA 电机连接未就绪：" + reason + "；请核对烧录文件与板内网口");
                }
                if (result == ArmHandshake.READY && foreground && !closed && generation == safetyGeneration.get()) {
                    attemptedOp = "arm";
                    command(object("op", "arm", "_motion_generation", generation));
                    synchronized (navigationLock) {
                        if (foreground && !closed && generation == safetyGeneration.get()) {
                            armedAt = SystemClock.elapsedRealtime(); armedSequence = completedSequence.get(); armed = true;
                            listener.message("电机已使能，推动摇杆运动，松手即停");
                        }
                    }
                }
                return;
            }
            if (!isArmed()) return;
            long intentGeneration = safetyGeneration.get();
            String intent = windowFocused && !telemetryPaused ? mode : "manual";
            JSONObject drive = intent.startsWith("rotate_") ? object("op", "rotate", "direction", "rotate_cw".equals(intent) ? "cw" : "ccw", "speed", ControlPolicy.ROTATION_SPEED) :
                    "follow".equals(intent) ? object("op", "follow") :
                    (!windowFocused || telemetryPaused || (throttle == 0 && steer == 0)) ? object("op", "idle") :
                    object("op", "drive", "throttle", throttle, "steer", steer, "hold", "hold".equals(mode));
            attemptedOp = drive.optString("op");
            drive.put("_motion_generation", intentGeneration);
            command(drive);
        } catch (ApiException ex) {
            if (closed || !foreground || !connected || tickEpoch != sessionGeneration.get() || tickGeneration != safetyGeneration.get()) return;
            if (sessionFailure(ex)) { networkFailure(ex); return; }
            pauseMotion("control_rejected"); listener.message(ex.getMessage());
        } catch (Exception ex) {
            if (closed || !foreground || !connected || tickEpoch != sessionGeneration.get() || tickGeneration != safetyGeneration.get()) return;
            if ("arm".equals(attemptedOp) || "configure".equals(attemptedOp) || armHandshake.pending()) {
                pauseMotion("arm_configuration_failed"); listener.message("电机确认请求未完成，请重试使能；未重放运动指令");
            } else if (isArmed()) {
                // A lost HTTP response does not prove that the RK missed a
                // command. RK/FPGA lease enforcement and fresh authoritative
                // state decide motor authorization; keep bounded retries here.
                long now = SystemClock.elapsedRealtime();
                if (now - lastControlWarningAt >= 3000) {
                    lastControlWarningAt = now; listener.message("控制响应暂缓，运动暂停，使能待确认；正在重试");
                }
                pauseMotion("control_response_retry");
                checkHealth();
            } else if ("select".equals(attemptedOp)) {
                selection.clear(SystemClock.elapsedRealtime()); listener.message("选框请求未完成，请重新选择");
            }
        } finally { lastControlTickRunMs=Math.max(0,SystemClock.elapsedRealtime()-tickStarted); }
    }
    private void checkHealth() {
        if (!isConnected() || !foreground) return;
        long now = SystemClock.elapsedRealtime();
        // Only actual authenticated command arrival renews the RK authority.
        // Brief telemetry/HTTP delays pause intent and retry; they are not a disconnect.
        long transport = lastTransportAt > 0 ? lastTransportAt : connectedAt;
        if (transport > 0 && now >= transport && now - transport >= ConnectionPolicy.CONNECTION_LOSS_MS) {
            networkFailure(new IOException("连接持续30秒无响应，请重新连接")); return;
        }
        boolean stale = !ConnectionPolicy.telemetryFresh(now, latestStateAt > 0 ? latestStateAt : connectedAt);
        if (armed && stale && !telemetryPaused) {
            telemetryPaused = true;
            pauseMotion("telemetry_timeout"); listener.message("状态未更新，已暂停运动，使能待确认；正在重试");
        } else if (!stale) telemetryPaused = false;
    }
    /** A cached reply is an observation of its source time, never of receipt time.
     * Pending control replies may predate ARM/STOP, so cannot reconcile UI or intent. */
    private static long parseStateSourceAt(JSONObject state, long requestedAt) {
        if (requestedAt < 0) return -1;
        if (state.has("gateway_control_pending")) {
            Object pending = state.opt("gateway_control_pending");
            if (!(pending instanceof Boolean) || Boolean.TRUE.equals(pending)) return -1;
        }
        if (!state.has("gateway_state_age_ms")) return requestedAt;
        Object rawAge = state.opt("gateway_state_age_ms");
        if (!(rawAge instanceof Number)) return -1;
        double age = ((Number)rawAge).doubleValue();
        if (Double.isNaN(age) || Double.isInfinite(age) || age < 0 || age != Math.rint(age) ||
                age > 300 || age > requestedAt) return -1;
        return requestedAt - (long)age;
    }
    private void poll() {
        if (!isConnected() || !foreground) return;
        final long epoch = sessionGeneration.get(), generation = safetyGeneration.get();
        try {
            long started = SystemClock.elapsedRealtime();
            JSONObject state = request("GET", "/api/state", null, 600);
            if (!isConnected() || !foreground || epoch != sessionGeneration.get() || generation != safetyGeneration.get()) return;
            long sourceAt = parseStateSourceAt(state, started);
            if (sourceAt < 0 || sourceAt < latestStateAt || !ConnectionPolicy.telemetryFresh(SystemClock.elapsedRealtime(), sourceAt)) return;
            // ARM and an older GET can share the same millisecond. Equality
            // cannot prove that armed=false was sampled after ARM completed.
            // Wait for a strictly newer observation; a later true disarm still
            // revokes immediately and no observation can re-arm locally.
            long stateSequence = state.optLong("control_sequence", -1);
            if (stateSequence < 0) {
                JSONObject sourceControl = state.optJSONObject("state");
                if (sourceControl != null) stateSequence = sourceControl.optLong("last_control_seq", -1);
            }
            synchronized (navigationLock) {
            // Read ARM epoch and apply a false observation under the same lock
            // as successful ARM: a concurrent ACK cannot turn a comparison
            // against pre-ARM armedAt=0 into revocation of the new authority.
            boolean afterArm = sourceAt > armedAt && (stateSequence < 0 || stateSequence >= armedSequence);
            boolean observedArmed = state.has("logical_armed") ? state.optBoolean("logical_armed", false) : state.optBoolean("armed", false);
            if (armed && afterArm && !observedArmed) {
                cancelNavigationIntent();
                safetyGeneration.incrementAndGet(); actions.clear(); armHandshake.cancel();
                armed = false; throttle = steer = 0; mode = "manual";
                String reason = state.optString("reason", "请重新使能");
                listener.message("电机已停用：" + FollowPresentation.reason(reason));
            }
            }
            if (state.has("selected_id")) {
                long displayed = selection.reconcile(state.optLong("selected_id", 0), sourceAt);
                state.put("selected_id", displayed);
                if (!selection.pending() && selection.confirmed() <= 0 && "follow".equals(mode)) mode = "manual";
            }
            latestState = state;latestStateAt=sourceAt;
            if (resumeNeedsState && started >= foregroundResumedAt && sourceAt >= foregroundResumedAt) resumeNeedsState = false;
            synchronized(navigationLock){
            if("mapping".equals(mode)&&navigationAcceptedAt>0&&sourceAt>=navigationAcceptedAt) {
                JSONObject mapping=MappingPresentation.mapping(state);
                if(mapping!=null) {
                    String status=MappingPresentation.status(state);
                    if(!status.equals(navigationStatus)){navigationStatus=status;listener.navigation(status,null);}
                    String phase=mapping.optString("state","");
                    if("IDLE".equals(phase)||"ERROR".equals(phase)||MappingPresentation.confirmedLoaded(state)){
                        cancelNavigationIntent();mode="manual";
                    }
                }
            }
            if("preview".equals(mode)&&navigationAcceptedAt>0&&sourceAt>=navigationAcceptedAt) {
                JSONObject nav=state.optJSONObject("navigation");
                if(nav!=null) {
                    String phase=nav.optString("state","");
                    if("ERROR".equals(phase)||"CANCELLED".equals(phase)) {
                        String why=nav.optString("reason",phase);
                        cancelNavigationIntent();mode="manual";
                        listener.navigation(NavigationTarget.reason(why),null);
                    } else if(nav.optBoolean("can_start",false)&&!"preview_ready".equals(navigationStatus)) {
                        navigationStatus="preview_ready";
                        listener.navigation("路线预览已就绪，点击开始导航才行驶",navigationTarget);
                    }
                }
            }
            if("navigation".equals(mode)&&navigationAcceptedAt>0&&sourceAt>=navigationAcceptedAt) {
                JSONObject nav=state.optJSONObject("navigation");
                if(nav!=null&&!nav.optBoolean("active",false)) {
                    String reason=nav.optString("cancel_reason","");
                    if(reason.isEmpty())reason=nav.optString("reason",nav.optString("state","导航已停止"));
                    cancelNavigationIntent();mode="manual";
                    listener.navigation(NavigationTarget.reason(reason.isEmpty()?"导航已停止":reason),null);
                } else if(nav!=null&&nav.optBoolean("active",false)) {
                    String status=NavigationTarget.reason(nav.optString("state","planning"));
                    if(!status.equals(navigationStatus)) {
                        navigationStatus=status;listener.navigation(status,navigationTarget);
                    }
                }
            }
            }
            // Ignore telemetry begun before the explicit tap. Once the backend
            // has stopped a follow, only another tap may renew that intention.
            if ("follow".equals(mode) && followAcceptedAt > 0 && sourceAt >= followAcceptedAt && sourceAt >= followStartedAt) {
                String backendMode = state.optString("mode", "");
                String reason = state.optString("reason", "");
                boolean halted = FollowPresentation.stoppedReason(reason) ||
                        (followAcknowledged && !backendMode.isEmpty() && !"follow".equals(backendMode));
                if (halted) {
                    mode = "manual"; throttle = steer = 0;
                    safetyGeneration.incrementAndGet(); actions.clear();
                    listener.message(FollowPresentation.stoppedReason(reason) ? FollowPresentation.reason(reason) : "跟随已停止，再次跟随请重新点击按钮");
                } else if ("follow".equals(backendMode)) followAcknowledged = true;
            }
            if (!closed && connected && foreground && epoch == sessionGeneration.get()) listener.state(state);
        } catch (Exception ex) {
            if (!isConnected() || !foreground || epoch != sessionGeneration.get() || generation != safetyGeneration.get()) return;
            if (sessionFailure(ex)) { networkFailure(ex); return; }
            long now = SystemClock.elapsedRealtime();
            if (now - lastTelemetryWarningAt >= 3000) {
                lastTelemetryWarningAt = now; listener.message(armed && telemetryPaused ? "状态未更新，运动暂停，使能待确认；正在重试" : "状态暂未更新，正在重试");
            }
            checkHealth();
        }
    }
    private boolean waitVideo(long delay) {
        synchronized (videoWake) {
            if (closed || !connected) return false;
            try { videoWake.wait(delay); }
            catch (InterruptedException ex) { Thread.currentThread().interrupt(); return false; }
            return !closed && connected;
        }
    }
    private void stream() {
        long retryMs = ConnectionPolicy.VIDEO_RETRY_MIN_MS;
        boolean reported = false;
        while (isConnected() && !closed) {
            if (!foreground) { if (!waitVideo(500)) return; continue; }
            final long epoch = sessionGeneration.get(), videoEpoch = videoGeneration.get();
            HttpURLConnection connection = null;
            try {
                connection = open("GET", "/api/video.mjpg", 2000);
                videoConnection = connection;
                int code = connection.getResponseCode();
                if (code == 401 || code == 403) throw new ApiException("视频会话验证失效，请重新连接", code);
                if (code != 200) throw new IOException("视频连接失败（" + code + "）");
                try (InputStream input = new BufferedInputStream(connection.getInputStream(), 32768)) {
                    MultipartReader reader = new MultipartReader(input);
                    long previous = -1;
                    MultipartReader.LagTracker lag = new MultipartReader.LagTracker();
                    while (isConnected() && foreground && epoch == sessionGeneration.get() && videoEpoch == videoGeneration.get()) {
                        MultipartReader.Part part = reader.next();
                        JSONObject meta = new JSONObject(part.metadata);
                        if (meta.optLong("frame_id", -1) != part.frameId || part.frameId < previous)
                            throw new IOException("视频帧与检测框不同步");
                        final long received = SystemClock.elapsedRealtime();
                        if (!lag.fresh(meta.optLong("server_send_mono_ms", 0), received, meta.optLong("age_ms", 0)))
                            throw new IOException("视频积压，重新接收最新画面");
                        final boolean newerFrame = part.frameId > previous;
                        previous = part.frameId;
                        if (newerFrame) {
                            // Read the network independently of JPEG decode. A
                            // slow phone replaces one pending image; it never
                            // decodes a FIFO of historical camera frames.
                            lastTransportAt = received;
                            synchronized (decodeWake) {
                                pendingVideo = new PendingVideo(part, meta, epoch, videoEpoch, received);
                                decodeWake.notifyAll();
                            }
                            retryMs = ConnectionPolicy.VIDEO_RETRY_MIN_MS;
                            if (reported) { reported = false; listener.message("摄像头画面已恢复"); }
                        }
                    }
                }
            } catch (Exception ex) {
                if (closed || !connected || epoch != sessionGeneration.get()) return;
                if (videoEpoch != videoGeneration.get()) continue;
                if (sessionFailure(ex)) { networkFailure(ex); return; }
                // Video is not the motor lease. EOF/timeout/429 only retries this
                // connection and cannot cancel a healthy control session.
                if (foreground && !reported) {
                    reported = true; listener.message("摄像头画面中断，正在重连；控制连接保持");
                }
            } finally {
                if (connection != null) connection.disconnect();
                if (videoConnection == connection) videoConnection = null;
            }
            if (!waitVideo(retryMs)) return;
            retryMs = ConnectionPolicy.nextVideoRetry(retryMs);
        }
    }
    private void decodeVideo() {
        while (!closed && !Thread.currentThread().isInterrupted()) {
            PendingVideo item;
            synchronized (decodeWake) {
                while (!closed && pendingVideo == null) {
                    try { decodeWake.wait(200); }
                    catch (InterruptedException ex) { Thread.currentThread().interrupt(); return; }
                }
                if (closed) return;
                item = pendingVideo; pendingVideo = null;
            }
            if (item == null || !isConnected() || !foreground || item.epoch != sessionGeneration.get() ||
                    item.videoEpoch != videoGeneration.get() || SystemClock.elapsedRealtime() - item.received > 250) continue;
            Bitmap image = null;
            try {
                BitmapFactory.Options bounds = new BitmapFactory.Options(); bounds.inJustDecodeBounds = true;
                BitmapFactory.decodeByteArray(item.part.jpeg, 0, item.part.jpeg.length, bounds);
                if (bounds.outWidth < 1 || bounds.outWidth > 4096 || bounds.outHeight < 1 || bounds.outHeight > 4096 ||
                        (long)bounds.outWidth * bounds.outHeight > 5000000L) continue;
                image = BitmapFactory.decodeByteArray(item.part.jpeg, 0, item.part.jpeg.length);
                if (image == null) continue;
                if (item.meta.optInt("width", -1) != image.getWidth() || item.meta.optInt("height", -1) != image.getHeight()) continue;
                if (!isConnected() || !foreground || item.epoch != sessionGeneration.get() ||
                        item.videoEpoch != videoGeneration.get() || SystemClock.elapsedRealtime() - item.received > 250) continue;
                listener.frame(new Frame(image, item.meta, item.part.frameId, item.part.jpeg, item.received));
                image = null; // UI owns and recycles it after this handoff.
            } catch (RuntimeException ex) {
                // One invalid JPEG cannot terminate video reception or control.
            } finally { if (image != null) image.recycle(); }
        }
    }
    private void command(JSONObject body) throws Exception {
        final long epoch, generation, sentSequence, sentAt;
        final String op;
        synchronized (commandOrder) {
        // Serialize sequence assignment only. A delayed lower sequence cannot overtake STOP at the RK.
        op = body.optString("op");
        if (body.has("_motion_generation")) {
            long intentGeneration = body.optLong("_motion_generation", -1); body.remove("_motion_generation");
            if (intentGeneration != safetyGeneration.get()) return;
        }
        if ("follow".equals(op) && !"follow".equals(mode)) return;
        if ("rotate".equals(op) && !mode.equals("rotate_" + body.optString("direction"))) return;
        boolean safeZero = body.optBoolean("_safe_zero", false) && "drive".equals(op) && body.optDouble("throttle", 0) == 0 && body.optDouble("steer", 0) == 0;
        body.remove("_safe_zero");
        if (("drive".equals(op) || "follow".equals(op) || "idle".equals(op) || "rotate".equals(op)) && !isArmed() && !safeZero)
            throw new ApiException("电机未使能");
        if ("arm".equals(op) && (!foreground || !windowFocused || closed)) throw new ApiException("控制界面已离开");
        epoch = sessionGeneration.get(); generation = safetyGeneration.get();
        if (closed && !("stop".equals(op) || "disarm".equals(op))) return;
        body.put("session", session);sentSequence=sequence.incrementAndGet(); body.put("seq",sentSequence);
        sentAt = SystemClock.elapsedRealtime();
        lastControlSendAt=sentAt;
        } // Never hold sequence/intent lock during I/O: STOP may overtake a slow request.
        try {
            JSONObject accepted = request("POST", "/api/control", body, ConnectionPolicy.CONTROL_HTTP_TIMEOUT_MS);
            if (accepted.has("accepted_seq") && accepted.optLong("accepted_seq", -1) != sentSequence)
                throw new IOException("控制回复序号不匹配，等待 RK 最新状态");
        }
        finally { lastControlRttMs=Math.max(0,SystemClock.elapsedRealtime()-sentAt); }
        if (closed || !connected || epoch != sessionGeneration.get() || generation != safetyGeneration.get()) return;
        // Do not use a single very late reply as an ARM/start/selection barrier.
        // This bounded evidence check is per request, not a repeating ACK-age
        // timer; continuous190/195ms successful requests stay usable.
        if (!ConnectionPolicy.responseEvidenceFresh(SystemClock.elapsedRealtime(), sentAt))
            throw new IOException("控制回复过旧，等待 RK 最新状态");
        lastControlAckAt=SystemClock.elapsedRealtime();
        completedSequence.accumulateAndGet(sentSequence, Math::max);
        if ("arm".equals(op) || "drive".equals(op) || "idle".equals(op) || "rotate".equals(op) || "follow".equals(op))
            lastLeaseAt = sentAt; // Diagnostic request source only, never motor authority.
        if ("follow".equals(op) && "follow".equals(mode) && followAcceptedAt == 0)
            followAcceptedAt = SystemClock.elapsedRealtime();
    }
    private void bestEffortDisarm() {
        try { if (!session.isEmpty()) command(object("op", "disarm")); } catch (Exception ignored) { }
    }
    private void networkFailure(Exception ex) {
        if (closed || !failed.compareAndSet(false, true)) return;
        cancelNavigationIntent();
        safetyGeneration.incrementAndGet(); sessionGeneration.incrementAndGet();
        armed = false; connected = false; throttle = steer = 0; mode = "manual"; actions.clear();
        armHandshake.cancel();
        selection.clear(SystemClock.elapsedRealtime());
        immediateStop(true, ex instanceof ApiException ? "session_rejected" : "connection_failure");
        listener.connected(false); listener.message("连接中断，已停车：" + ex.getMessage());
        HttpURLConnection stream = videoConnection;
        if (stream != null) stream.disconnect();
        synchronized (videoWake) { videoWake.notifyAll(); }
    }
    private static boolean sessionFailure(Exception ex) {
        return ex instanceof ApiException && ((ApiException)ex).sessionFailure();
    }
    public void close() { close("connection_closed"); }
    public void close(String reason) {
        if (closed) return;
        background("activity_pause".equals(reason) ? "activity_pause" : "connection_closed");
        stop(true, "activity_pause".equals(reason) ? "activity_pause" : "connection_closed");
        closed = true; connected = false;
        HttpURLConnection stream = videoConnection;
        if (stream != null) stream.disconnect();
        sessionGeneration.incrementAndGet();
        synchronized (videoWake) { videoWake.notifyAll(); }
        synchronized (decodeWake) { pendingVideo = null; decodeWake.notifyAll(); }
        control.shutdownNow(); telemetry.shutdownNow(); video.shutdownNow();videoDecoder.shutdownNow();navigation.shutdownNow(); health.shutdownNow();
        emergency.shutdown(); // Allows the already queued best-effort STOP to finish.
    }
    private HttpURLConnection open(String method, String path, int timeout) throws IOException {
        HttpURLConnection connection = (HttpURLConnection)new URL(base + path).openConnection();
        connection.setRequestMethod(method); connection.setInstanceFollowRedirects(false);
        connection.setConnectTimeout(timeout); connection.setReadTimeout(timeout);
        connection.setRequestProperty("Authorization", "Bearer " + token);
        connection.setRequestProperty("Cache-Control", "no-store");
        return connection;
    }
    private JSONObject request(String method, String path, JSONObject data, int timeout) throws Exception {
        HttpURLConnection connection = open(method, path, timeout);
        try {
            if (data != null) {
                byte[] bytes = data.toString().getBytes(StandardCharsets.UTF_8);
                connection.setDoOutput(true); connection.setRequestProperty("Content-Type", "application/json");
                connection.setFixedLengthStreamingMode(bytes.length);
                try (OutputStream out = connection.getOutputStream()) { out.write(bytes); }
            }
            int code = connection.getResponseCode();
            if (code == 401 || code == 403)
                throw new ApiException(code == 401 ? "配对验证失败，请检查配对码后重新连接" : "控制会话失效，请重新连接", code);
            InputStream input = code >= 200 && code < 300 ? connection.getInputStream() : connection.getErrorStream();
            if (input == null) throw new IOException("HTTP " + code);
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            try (InputStream stream = input) {
                byte[] buffer = new byte[4096]; int n;
                while ((n = stream.read(buffer)) != -1) {
                    if (out.size() + n > 1048576) throw new IOException("响应内容过大");
                    out.write(buffer, 0, n);
                }
            }
            JSONObject json = new JSONObject(new String(out.toByteArray(), StandardCharsets.UTF_8));
            if (code < 200 || code >= 300 || !json.optBoolean("ok", false))
                throw new ApiException(json.optString("error", "请求失败 HTTP " + code), code);
            lastTransportAt = SystemClock.elapsedRealtime();
            return json;
        } finally { connection.disconnect(); }
    }
    public static JSONObject object(Object... pairs) {
        JSONObject result = new JSONObject();
        try { for (int i = 0; i < pairs.length; i += 2) result.put((String)pairs[i], pairs[i + 1]); }
        catch (Exception ex) { throw new IllegalArgumentException(ex); }
        return result;
    }
    private static final class ApiException extends IOException {
        final String code;
        final int httpStatus;
        ApiException(String text) { this(text, 0); }
        ApiException(String text, int httpStatus) { super(FollowPresentation.reason(text));code=text;this.httpStatus=httpStatus; }
        boolean sessionFailure() {
            return httpStatus == 401 || httpStatus == 403 || "not_control_owner".equals(code) ||
                    "invalid_session".equals(code) || "session_expired".equals(code);
        }
    }
}
