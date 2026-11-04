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
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

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
            this.image = image; this.metadata = metadata; this.id = id; this.jpeg = jpeg;
            this.received = SystemClock.elapsedRealtime();
        }
    }
    private final String base, token;
    private final Listener listener;
    private final ScheduledExecutorService control = Executors.newSingleThreadScheduledExecutor();
    private final ScheduledExecutorService telemetry = Executors.newSingleThreadScheduledExecutor();
    private final ExecutorService video = Executors.newSingleThreadExecutor();
    private final ExecutorService navigation = Executors.newSingleThreadExecutor();
    private final ExecutorService emergency = Executors.newSingleThreadExecutor();
    // This watchdog cannot be held up by an HTTP response or JPEG decoding.
    private final ScheduledExecutorService health = Executors.newSingleThreadScheduledExecutor();
    private final Object videoWake = new Object();
    private final AtomicBoolean started = new AtomicBoolean();
    private final AtomicLong sessionGeneration = new AtomicLong(), videoGeneration = new AtomicLong();
    private volatile long connectedAt, lastLeaseAt, lastTelemetryWarningAt, lastControlWarningAt, lastTransportAt;
    // Diagnostic clocks only. These values never authorize or renew movement.
    private volatile long lastControlSendAt, lastControlAckAt, lastControlRttMs;
    private volatile long lastControlTickAt, lastControlTickGapMs, lastControlTickRunMs, lastControlSchedulerDelayMs;
    private final ConcurrentLinkedQueue<JSONObject> actions = new ConcurrentLinkedQueue<>();
    private final AtomicLong sequence = new AtomicLong(), completedSequence = new AtomicLong(), navigationSequence = new AtomicLong();
    private volatile NavigationTarget navigationTarget;
    private volatile String navigationOperation="";
    private volatile long latestStateAt;
    private final AtomicLong diagnosticSequence = new AtomicLong();
    private volatile long navigationAcceptedAt;
    private volatile String navigationStatus="";
    private final AtomicLong safetyGeneration = new AtomicLong();
    private final Object navigationLock = new Object();
    private final AtomicBoolean failed = new AtomicBoolean();
    private final SelectionGate selection = new SelectionGate();
    private final ArmHandshake armHandshake = new ArmHandshake();
    private volatile boolean closed, connected, armed, foreground = true, windowFocused = true;
    private volatile String session = "", mode = "manual";
    private volatile float throttle, steer;
    private volatile long armedAt, followStartedAt, followAcceptedAt;
    private volatile boolean followAcknowledged;
    private volatile JSONObject latestState;
    private volatile HttpURLConnection videoConnection;
    public RobotClient(String base, String token, Listener listener) {
        this.base = LanAddress.normalize(base); this.token = token; this.listener = listener;
    }
    public boolean isConnected() { return connected && !closed; }
    public boolean isArmed() { return armed && isConnected() && foreground && windowFocused; }
    public String mode() { return mode; }
    public String followDiagnostics() { return FollowPresentation.diagnostics(latestState, selection.display()); }
    // Read at UI delivery time: a queued telemetry snapshot may predate a tap.
    public long selectedTarget() { return selection.display(); }
    public void start() {
        if (closed || !started.compareAndSet(false, true)) return;
        control.execute(() -> {
            try {
                JSONObject answer = request("POST", "/api/connect", object("client", "Android native 1.2.15"), 1500);
                session = answer.getString("session");
                if (session.length() < 8 || session.length() > 256) throw new IOException("会话无效");
                if (closed || !foreground) { bestEffortDisarm(); return; }
                sessionGeneration.incrementAndGet(); connectedAt = SystemClock.elapsedRealtime(); connected = true; armed = false;
                listener.connected(true); listener.message("已连接，电机未使能");
                telemetry.scheduleWithFixedDelay(this::poll, 0, 100, TimeUnit.MILLISECONDS);
                video.execute(this::stream);
                // Keep one serialized worker and read CURRENT intent each tick.
                // Fixed delay added50ms AFTER a160ms ACK, so the next ACK arrived
                // at370ms and falsely exhausted the unchanged350ms lease.
                // Fixed rate removes that extra idle; it cannot overlap requests
                // or extend authority from an old/late reply.
                control.scheduleAtFixedRate(this::tick, 0, 50, TimeUnit.MILLISECONDS);
                health.scheduleWithFixedDelay(this::checkHealth, 20, 20, TimeUnit.MILLISECONDS);
            } catch (Exception ex) { networkFailure(ex); }
        });
    }
    public void arm(int leftMask, int invertMask) {
        if (!isConnected() || !foreground || !windowFocused) return;
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
        if (!isArmed()) return;
        cancelNavigationIntent();
        this.throttle = Math.max(-1, Math.min(1, throttle));
        this.steer = Math.max(-1, Math.min(1, steer));
        this.mode = hold ? "hold" : "manual";
    }
    public void releaseStick(boolean cancel) {
        cancelNavigationIntent();
        throttle = steer = 0;
        if (cancel) { stop(true, "touch_cancel"); return; }
        mode = "manual";
        selection.clear(SystemClock.elapsedRealtime());
        safetyGeneration.incrementAndGet(); actions.clear();
        if (isArmed()) immediateStop(false, "joystick_release");
    }
    public void rotate(boolean clockwise) {
        cancelNavigationIntent();
        if (!isArmed()) { listener.message("请先使能电机"); return; }
        throttle = steer = 0;
        selection.clear(SystemClock.elapsedRealtime());
        mode = clockwise ? "rotate_cw" : "rotate_ccw";
    }
    public void releaseRotation(boolean cancel) {
        // Invalidate a queued rotate before STOP can be overtaken. A normal
        // release keeps the arm lease; cancellation explicitly disarms it.
        if (cancel) { stop(true, "rotation_cancel"); return; }
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
        selection.clear(SystemClock.elapsedRealtime());
        if (isArmed()) queue(object("op", "drive", "throttle", 0, "steer", 0, "hold", false));
    }
    private void cancelNavigationIntent() {
        synchronized(navigationLock){
        if(navigationTarget!=null||!navigationOperation.isEmpty()||"navigation".equals(mode)) {
            navigationTarget=null;navigationOperation="";navigationAcceptedAt=0;safetyGeneration.incrementAndGet();actions.clear();
        }
        }
    }
    public void startNavigation() {
        if(!isArmed()){listener.navigation("请先连接并使能电机，再开始导航",null);return;}
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
        beginNavigation("start",null);
    }
    public void startStraightAssist() {
        if(!isArmed()){listener.navigation("请先连接并使能电机，再保持直行",null);return;}
        if("navigation".equals(mode)){
            listener.navigation("straight".equals(navigationOperation)?"保持直行已在运行，可随时结束或使用摇杆接管":"请先结束当前导航，再启动保持直行",null);return;
        }
        long now=SystemClock.elapsedRealtime();
        if(latestState==null||now<latestStateAt||now-latestStateAt>1000){listener.navigation("状态未更新，请稍后再保持直行",null);return;}
        JSONObject nav=latestState.optJSONObject("navigation");
        if(nav==null||!nav.optBoolean("supports_straight_assist")){
            listener.navigation("请先安装 RK v27 更新，再使用保持直行避障",null);return;
        }
        // Readiness is checked by the RK straight-assist controller. Local-map
        // can_start belongs to map navigation and must not bypass or block it.
        beginNavigation("straight",null);
    }
    public void endNavigation() {
        // Invalidate locally before the independent emergency thread sends the
        // serialized STOP/disarm. A late start/goto cannot restore this intent.
        synchronized(navigationLock){stop(true, "navigation_end");listener.navigation("导航已结束，电机已停用",null);}
    }
    public void navigate(NavigationTarget target) {
        if(!isArmed()){listener.navigation("请先使能电机，再选择目的地",null);return;}
        if(target==null||!target.fresh(SystemClock.elapsedRealtime())){listener.navigation("地图已过期，请重新选择",null);return;}
        beginNavigation("goto",target);
    }
    private void beginNavigation(String operation,NavigationTarget target) {
        synchronized(navigationLock){
        if(!isArmed())return;
        long generation=safetyGeneration.incrementAndGet(),navSeq=navigationSequence.incrementAndGet();
        actions.clear();throttle=steer=0;mode="navigation";navigationTarget=target;navigationOperation=operation;navigationAcceptedAt=0;
        selection.clear(SystemClock.elapsedRealtime());
        // The successful zero DRIVE is the server-validated intent barrier.
        // Navigation uses an independent worker; 50ms idle leases keep flowing.
        queue(object("op","drive","throttle",0,"steer",0,"hold",false,"_navigation",navSeq,
                     "_navigation_op",operation,"_motion_generation",generation));
        listener.navigation("straight".equals(operation)?"正在启动保持直行（避障生效）…":"start".equals(operation)?"正在开始导航…":"正在提交目的地…",target);
        }
    }
    private boolean currentNavigation(long navSeq,long generation,String operation,NavigationTarget target) {
        return generation==safetyGeneration.get()&&navSeq==navigationSequence.get()&&isArmed()&&
                "navigation".equals(mode)&&operation.equals(navigationOperation)&&navigationTarget==target;
    }
    private void submitNavigation(long navSeq,long generation,long intentSeq,String operation,NavigationTarget target) {
        try { navigation.execute(() -> {
            if(!currentNavigation(navSeq,generation,operation,target))return;
            try {
                JSONObject requestBody=object("op",operation,"session",session,"intent_seq",intentSeq,"nav_seq",navSeq);
                if("goto".equals(operation)){
                    if(target==null||!target.fresh(SystemClock.elapsedRealtime()))throw new ApiException("map_snapshot_expired");
                    requestBody.put("x",target.x);requestBody.put("y",target.y);
                    requestBody.put("map_session",target.map.sessionId);requestBody.put("map_revision",target.map.revision);
                }
                request("POST","/api/navigation",requestBody,1200);
                synchronized(navigationLock){if(currentNavigation(navSeq,generation,operation,target)) {
                    navigationAcceptedAt=SystemClock.elapsedRealtime();navigationStatus="";
                    listener.navigation("straight".equals(operation)?"保持直行已开始，避障生效":"start".equals(operation)?"导航已开始，正在规划路线":"目的地已接受，正在规划路线",target);
                }}
            } catch(Exception ex) {
                if (sessionFailure(ex)) { networkFailure(ex); return; }
                synchronized(navigationLock){if(currentNavigation(navSeq,generation,operation,target)) {
                    stop(true, "navigation_rejected");listener.navigation(NavigationTarget.reason(ex instanceof ApiException ? ((ApiException)ex).code : ex.getMessage()),null);
                }}
            }
        }); } catch(java.util.concurrent.RejectedExecutionException ignored) { }
    }
    public void imuAction(boolean calibrate) {
        stop(true, calibrate ? "imu_calibrate" : "imu_zero");
        // This executor serializes the STOP/disarm and the calibration request.
        emergency.execute(() -> {
            if (!closed && isConnected()) {
                try { command(object("op", calibrate ? "calibrate" : "zero")); }
                catch (Exception ex) { if (sessionFailure(ex)) networkFailure(ex); else if (!closed) listener.message(ex.getMessage()); }
            }
        });
    }
    private void queue(JSONObject action) {
        if (actions.size() < 12 && !closed) {
            try { action.put("_generation", safetyGeneration.get()); } catch (Exception ignored) { }
            actions.offer(action);
        }
    }
    public void stop(boolean disarm) { stop(disarm, "explicit_stop"); }
    void stop(boolean disarm, String reason) {
        synchronized(navigationLock){
        navigationTarget=null;navigationOperation="";navigationAcceptedAt=0;
        throttle = steer = 0; mode = "manual";
        safetyGeneration.incrementAndGet();
        armHandshake.cancel();
        selection.clear(SystemClock.elapsedRealtime());
        actions.clear();
        if (disarm) armed = false;
        immediateStop(disarm, reason);
        }
    }
    private void immediateStop(boolean disarm) { immediateStop(disarm, "stop_request"); }
    private void immediateStop(boolean disarm, String reason) {
        if (session.isEmpty()) return;
        final long at=SystemClock.elapsedRealtime(), event=diagnosticSequence.incrementAndGet();
        final long stateAge=latestStateAt>0&&at>=latestStateAt?at-latestStateAt:-1;
        final long leaseAge=lastLeaseAt>0&&at>=lastLeaseAt?at-lastLeaseAt:-1;
        final JSONObject timing=object("client_control_send_ms",lastControlSendAt,
                "client_control_ack_ms",lastControlAckAt,"client_control_rtt_ms",lastControlRttMs,
                "client_tick_gap_ms",lastControlTickGapMs,"client_tick_run_ms",lastControlTickRunMs,
                "client_scheduler_delay_ms",lastControlSchedulerDelayMs);
        final String cause=reason;
        try {
            emergency.execute(() -> {
                try {
                    // Backend STOP also disarms. Zero DRIVE stops a released
                    // hold button without unexpectedly clearing its arm state.
                    JSONObject first=disarm ? object("op", "stop") : object("op", "drive", "throttle", 0, "steer", 0, "hold", false);
                    attachStopDiagnostics(first,cause,event,at,stateAge,leaseAge,timing);
                    command(first);
                    if (disarm) {
                        JSONObject second=object("op", "disarm");
                        attachStopDiagnostics(second,cause,event,at,stateAge,leaseAge,timing);
                        command(second);
                    }
                } catch (Exception ignored) { /* Server watchdog also expires the lease. */ }
            });
        } catch (java.util.concurrent.RejectedExecutionException ignored) { }
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
            stop(true, "window_focus_lost");
            listener.message("控制界面暂时失焦，已停车；连接保持，恢复后请重新使能");
        }
    }
    public void background() { background("activity_pause"); }
    private void background(String reason) {
        foreground = false; videoGeneration.incrementAndGet(); stop(true, reason);
        HttpURLConnection stream = videoConnection;
        if (stream != null) stream.disconnect();
        synchronized (videoWake) { videoWake.notifyAll(); }
    }
    public void foreground() {
        foreground = true; // Returning to the app never rearms it or replays old motion.
        synchronized (videoWake) { videoWake.notifyAll(); }
    }
    private void tick() {
        if (!isConnected() || !foreground || !windowFocused) return;
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
                    if (foreground && !closed && generation == safetyGeneration.get()) {
                        armedAt = SystemClock.elapsedRealtime(); armed = true;
                        listener.message("电机已使能，推动摇杆运动，松手即停");
                    } else immediateStop(true, "arm_intent_invalidated");
                }
                return;
            }
            if (!isArmed()) return;
            long intentGeneration = safetyGeneration.get();
            String intent = mode;
            JSONObject drive = intent.startsWith("rotate_") ? object("op", "rotate", "direction", "rotate_cw".equals(intent) ? "cw" : "ccw", "speed", ControlPolicy.ROTATION_SPEED) :
                    "follow".equals(intent) ? object("op", "follow") :
                    throttle == 0 && steer == 0 ? object("op", "idle") :
                    object("op", "drive", "throttle", throttle, "steer", steer, "hold", "hold".equals(mode));
            attemptedOp = drive.optString("op");
            drive.put("_motion_generation", intentGeneration);
            command(drive);
        } catch (ApiException ex) {
            if (closed || !foreground || !connected || tickEpoch != sessionGeneration.get() || tickGeneration != safetyGeneration.get()) return;
            if (sessionFailure(ex)) { networkFailure(ex); return; }
            cancelNavigationIntent();
            armHandshake.cancel(); actions.clear(); armed = false; throttle = steer = 0; mode = "manual";
            selection.clear(SystemClock.elapsedRealtime());
            immediateStop(true, "control_rejected"); listener.message(ex.getMessage());
        } catch (Exception ex) {
            if (closed || !foreground || !connected || tickEpoch != sessionGeneration.get() || tickGeneration != safetyGeneration.get()) return;
            if ("arm".equals(attemptedOp) || "configure".equals(attemptedOp) || armHandshake.pending()) {
                stop(true, "arm_configuration_failed"); listener.message("电机确认请求未完成，已停用，请重试使能");
            } else if (isArmed()) {
                // A single lost HTTP response does not destroy a healthy session.
                // A retry must succeed inside the original 350 ms motor lease.
                long now = SystemClock.elapsedRealtime();
                if (now - lastControlWarningAt >= 3000) {
                    lastControlWarningAt = now; listener.message("控制响应暂缓，正在重试");
                }
                checkHealth();
            } else if ("select".equals(attemptedOp)) {
                selection.clear(SystemClock.elapsedRealtime()); listener.message("选框请求未完成，请重新选择");
            }
        } finally { lastControlTickRunMs=Math.max(0,SystemClock.elapsedRealtime()-tickStarted); }
    }
    private void checkHealth() {
        if (!isConnected() || !foreground) return;
        long now = SystemClock.elapsedRealtime();
        // Motor authorization and transport health are separate clocks. Genuine
        // loss of every successful HTTP exchange still ends the session, even
        // after the shorter motor watchdog has already stopped the wheels.
        if (!ConnectionPolicy.telemetryFresh(now, lastTransportAt > 0 ? lastTransportAt : connectedAt)) {
            networkFailure(new IOException("HTTP 连接持续无响应，请重新连接")); return;
        }
        if (armed && !ConnectionPolicy.leaseFresh(now, lastLeaseAt)) {
            // Keep the 350 ms safety deadline without destroying healthy video,
            // status polling or authentication merely because a motor ACK was
            // lost. STOP revokes old intents; late replies cannot re-arm.
            stop(true, "control_ack_timeout");
            listener.message("控制确认超过 350 毫秒，已停用电机；连接保持，恢复后请重新使能");
            return;
        }
        if (armed && !ConnectionPolicy.telemetryFresh(now, latestStateAt > 0 ? latestStateAt : connectedAt)) {
            // State may resume without a new video/control session, but never
            // rearm or restore a previous throttle after this safety stop.
            stop(true, "telemetry_timeout"); listener.message("状态数据未更新，已停用电机；恢复后请重新使能");
        }
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
            if (armed && sourceAt > armedAt && !state.optBoolean("armed", false)) {
                cancelNavigationIntent();
                safetyGeneration.incrementAndGet(); actions.clear(); armHandshake.cancel();
                armed = false; throttle = steer = 0; mode = "manual";
                String reason = state.optString("reason", "请重新使能");
                listener.message("电机已停用：" + FollowPresentation.reason(reason));
            }
            if (state.has("selected_id")) {
                long displayed = selection.reconcile(state.optLong("selected_id", 0), sourceAt);
                state.put("selected_id", displayed);
                if (!selection.pending() && selection.confirmed() <= 0 && "follow".equals(mode)) mode = "manual";
            }
            latestState = state;latestStateAt=sourceAt;
            synchronized(navigationLock){if("navigation".equals(mode)&&navigationAcceptedAt>0&&sourceAt>=navigationAcceptedAt) {
                JSONObject nav=state.optJSONObject("navigation");
                if(nav!=null&&!nav.optBoolean("active",false)) {
                    String reason=nav.optString("cancel_reason","");
                    if(reason.isEmpty())reason=nav.optString("reason",nav.optString("state","导航已停止"));
                    cancelNavigationIntent();mode="manual";
                    listener.navigation(NavigationTarget.reason(reason.isEmpty()?"导航已停止":reason),null);
                } else if(nav!=null&&nav.optBoolean("active",false)) {
                    String status=NavigationTarget.reason(nav.optString("state","planning"));
                    if(nav.optBoolean("straight_assist_active")) {
                        String reason=nav.optString("reason","");
                        status="保持直行 · "+(reason.isEmpty()?status:NavigationTarget.reason(reason));
                    }
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
                lastTelemetryWarningAt = now; listener.message("状态暂未更新，正在重试");
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
                    while (isConnected() && foreground && epoch == sessionGeneration.get() && videoEpoch == videoGeneration.get()) {
                        MultipartReader.Part part = reader.next();
                        JSONObject meta = new JSONObject(part.metadata);
                        if (meta.optLong("frame_id", -1) != part.frameId || part.frameId < previous)
                            throw new IOException("视频帧与检测框不同步");
                        BitmapFactory.Options bounds = new BitmapFactory.Options(); bounds.inJustDecodeBounds = true;
                        BitmapFactory.decodeByteArray(part.jpeg, 0, part.jpeg.length, bounds);
                        if (bounds.outWidth < 1 || bounds.outWidth > 4096 || bounds.outHeight < 1 || bounds.outHeight > 4096 ||
                                (long)bounds.outWidth * bounds.outHeight > 5000000L) throw new IOException("视频尺寸异常");
                        Bitmap image = BitmapFactory.decodeByteArray(part.jpeg, 0, part.jpeg.length);
                        if (image == null) throw new IOException("JPEG 解码失败");
                        if (meta.optInt("width", -1) != image.getWidth() || meta.optInt("height", -1) != image.getHeight()) {
                            image.recycle(); throw new IOException("视频与检测坐标尺寸不一致");
                        }
                        final boolean newerFrame = part.frameId > previous;
                        previous = part.frameId;
                        if (!closed && connected && foreground && epoch == sessionGeneration.get() && videoEpoch == videoGeneration.get()) {
                            // Valid new video proves transport liveness only.
                            // It must never renew motor authorization, sensor
                            // freshness or a cancelled session/video generation.
                            if (newerFrame) lastTransportAt = SystemClock.elapsedRealtime();
                            retryMs = ConnectionPolicy.VIDEO_RETRY_MIN_MS;
                            if (reported) { reported = false; listener.message("摄像头画面已恢复"); }
                            listener.frame(new Frame(image, meta, part.frameId, part.jpeg));
                        } else image.recycle();
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
    private synchronized void command(JSONObject body) throws Exception {
        // Serialized sequence assignment + transport prevents a delayed drive overtaking STOP.
        String op = body.optString("op");
        if (body.has("_motion_generation")) {
            long generation = body.optLong("_motion_generation", -1); body.remove("_motion_generation");
            if (generation != safetyGeneration.get()) return;
        }
        if ("follow".equals(op) && !"follow".equals(mode)) return;
        if ("rotate".equals(op) && !mode.equals("rotate_" + body.optString("direction"))) return;
        if (("drive".equals(op) || "follow".equals(op) || "idle".equals(op) || "rotate".equals(op)) && !isArmed())
            throw new ApiException("电机未使能");
        if ("arm".equals(op) && (!foreground || !windowFocused || closed)) throw new ApiException("控制界面已离开");
        final long epoch = sessionGeneration.get(), generation = safetyGeneration.get();
        if (closed && !("stop".equals(op) || "disarm".equals(op))) return;
        body.put("session", session);long sentSequence=sequence.incrementAndGet(); body.put("seq",sentSequence);
        long sentAt = SystemClock.elapsedRealtime();
        lastControlSendAt=sentAt;
        try { request("POST", "/api/control", body, ConnectionPolicy.CONTROL_HTTP_TIMEOUT_MS); }
        finally { lastControlRttMs=Math.max(0,SystemClock.elapsedRealtime()-sentAt); }
        if (closed || !connected || epoch != sessionGeneration.get() || generation != safetyGeneration.get()) return;
        lastControlAckAt=SystemClock.elapsedRealtime();
        completedSequence.set(sentSequence);
        if ("arm".equals(op) || "drive".equals(op) || "idle".equals(op) || "rotate".equals(op) || "follow".equals(op))
            lastLeaseAt = sentAt; // Conservative request start, never the late response arrival.
        if (("arm".equals(op) || armed) && !ConnectionPolicy.leaseFresh(SystemClock.elapsedRealtime(), sentAt))
            throw new IOException("控制响应已过期");
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
        immediateStop(true, ex instanceof ApiException ? "session_rejected" : ex.getMessage()!=null&&ex.getMessage().contains("350") ? "control_ack_timeout" : "connection_failure");
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
        background("activity_pause".equals(reason) ? "activity_pause" : "connection_closed"); closed = true; connected = false;
        HttpURLConnection stream = videoConnection;
        if (stream != null) stream.disconnect();
        sessionGeneration.incrementAndGet();
        synchronized (videoWake) { videoWake.notifyAll(); }
        control.shutdownNow(); telemetry.shutdownNow(); video.shutdownNow();navigation.shutdownNow(); health.shutdownNow();
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
