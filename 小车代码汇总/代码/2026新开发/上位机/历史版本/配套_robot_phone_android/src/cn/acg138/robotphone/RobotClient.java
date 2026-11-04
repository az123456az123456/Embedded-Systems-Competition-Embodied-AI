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
    }
    public static final class Frame {
        public final Bitmap image;
        public final JSONObject metadata;
        public final long id, received;
        Frame(Bitmap image, JSONObject metadata, long id) {
            this.image = image; this.metadata = metadata; this.id = id;
            this.received = SystemClock.elapsedRealtime();
        }
    }
    private final String base, token;
    private final Listener listener;
    private final ScheduledExecutorService control = Executors.newSingleThreadScheduledExecutor();
    private final ScheduledExecutorService telemetry = Executors.newSingleThreadScheduledExecutor();
    private final ExecutorService video = Executors.newSingleThreadExecutor();
    private final ExecutorService emergency = Executors.newSingleThreadExecutor();
    private final ConcurrentLinkedQueue<JSONObject> actions = new ConcurrentLinkedQueue<>();
    private final AtomicLong sequence = new AtomicLong();
    private final AtomicLong safetyGeneration = new AtomicLong();
    private final AtomicBoolean failed = new AtomicBoolean();
    private final SelectionGate selection = new SelectionGate();
    private volatile boolean closed, connected, armed, foreground = true;
    private volatile String session = "", mode = "manual";
    private volatile float throttle, steer;
    private volatile long armedAt;
    private volatile HttpURLConnection videoConnection;
    public RobotClient(String base, String token, Listener listener) {
        this.base = LanAddress.normalize(base); this.token = token; this.listener = listener;
    }
    public boolean isConnected() { return connected && !closed; }
    public boolean isArmed() { return armed && isConnected() && foreground; }
    public String mode() { return mode; }
    public void start() {
        control.execute(() -> {
            try {
                JSONObject answer = request("POST", "/api/connect", object("client", "Android native 1.0"), 1500);
                session = answer.getString("session");
                if (session.length() < 8 || session.length() > 256) throw new IOException("会话无效");
                if (closed || !foreground) { bestEffortDisarm(); return; }
                connected = true; armed = false;
                listener.connected(true); listener.message("已连接，电机未使能");
                telemetry.scheduleWithFixedDelay(this::poll, 0, 100, TimeUnit.MILLISECONDS);
                video.execute(this::stream);
                control.scheduleWithFixedDelay(this::tick, 0, 50, TimeUnit.MILLISECONDS);
            } catch (Exception ex) { networkFailure(ex); }
        });
    }
    public void arm(int leftMask, int invertMask) {
        if (!isConnected() || !foreground) return;
        if (Integer.bitCount(leftMask & 15) != 2) { listener.message("请选择两个左侧电机"); return; }
        throttle = steer = 0; mode = "manual";
        selection.clear(SystemClock.elapsedRealtime());
        queue(object("op", "configure", "left_mask", leftMask, "invert_mask", invertMask));
        queue(object("op", "arm"));
    }
    public void manual(float throttle, float steer, boolean hold) {
        if (!isArmed()) return;
        this.throttle = Math.max(-1, Math.min(1, throttle));
        this.steer = Math.max(-1, Math.min(1, steer));
        this.mode = hold ? "hold" : "manual";
    }
    public void releaseStick(boolean cancel) {
        throttle = steer = 0;
        if (cancel) { stop(true); return; }
        mode = "manual";
        selection.clear(SystemClock.elapsedRealtime());
        if (isArmed()) queue(object("op", "drive", "throttle", 0, "steer", 0, "hold", false));
    }
    public void select(long frameId, long targetId) {
        if (!isConnected()) return;
        if (!isArmed()) { listener.message("请先使能电机，再选择目标框"); return; }
        // A fresh tap cancels all previous movement intent. Only the explicit
        // Follow button may start following the new target after its ACK.
        throttle = steer = 0; mode = "manual";
        long request = selection.request(targetId);
        queue(object("op", "select", "frame_id", frameId, "target_id", targetId, "_selection", request));
        listener.message("正在确认目标 #" + targetId + "…");
    }
    public void follow() {
        if (!isArmed()) { listener.message("请先使能电机"); return; }
        if (selection.pending()) { listener.message("正在确认选框，请稍后再按跟随"); return; }
        if (selection.confirmed() <= 0) { listener.message("请先点击视频中的目标框"); return; }
        throttle = steer = 0; mode = "follow";
    }
    public void setManualMode(boolean hold) {
        throttle = steer = 0; mode = hold ? "hold" : "manual";
        selection.clear(SystemClock.elapsedRealtime());
        if (isArmed()) queue(object("op", "drive", "throttle", 0, "steer", 0, "hold", false));
    }
    public void imuAction(boolean calibrate) {
        stop(true);
        // This executor serializes the STOP/disarm and the calibration request.
        emergency.execute(() -> {
            if (!closed && isConnected()) {
                try { command(object("op", calibrate ? "calibrate" : "zero")); }
                catch (Exception ex) { listener.message(ex.getMessage()); }
            }
        });
    }
    private void queue(JSONObject action) {
        if (actions.size() < 12 && !closed) {
            try { action.put("_generation", safetyGeneration.get()); } catch (Exception ignored) { }
            actions.offer(action);
        }
    }
    public void stop(boolean disarm) {
        throttle = steer = 0; mode = "manual";
        safetyGeneration.incrementAndGet();
        selection.clear(SystemClock.elapsedRealtime());
        actions.clear();
        if (disarm) armed = false;
        immediateStop(disarm);
    }
    private void immediateStop(boolean disarm) {
        if (session.isEmpty()) return;
        try {
            emergency.execute(() -> {
                try {
                    command(object("op", "stop"));
                    if (disarm) command(object("op", "disarm"));
                } catch (Exception ignored) { /* Server watchdog also expires the lease. */ }
            });
        } catch (java.util.concurrent.RejectedExecutionException ignored) { }
    }
    public void background() {
        foreground = false; stop(true);
    }
    public void foreground() {
        foreground = true; // Returning to the app never rearms it.
    }
    private void tick() {
        if (!isConnected() || !foreground) return;
        try {
            JSONObject action = actions.poll();
            if (action != null) {
                String op = action.optString("op");
                long generation = action.optLong("_generation", -1);
                long selectionRequest = action.optLong("_selection", -1);
                action.remove("_generation");
                action.remove("_selection");
                if (generation != safetyGeneration.get()) return;
                command(action);
                if ("select".equals(op) && selection.acknowledge(selectionRequest, SystemClock.elapsedRealtime()))
                    listener.message("目标 #" + selection.confirmed() + " 已确认，按“跟随目标”开始");
                if ("arm".equals(op)) {
                    if (foreground && !closed && generation == safetyGeneration.get()) {
                        armedAt = SystemClock.elapsedRealtime(); armed = true;
                        listener.message("电机已使能，松开摇杆即停车");
                    } else immediateStop(true);
                }
                return;
            }
            if (!isArmed()) return;
            JSONObject drive = "follow".equals(mode) ? object("op", "follow") :
                    throttle == 0 && steer == 0 ? object("op", "idle") :
                    object("op", "drive", "throttle", throttle, "steer", steer, "hold", "hold".equals(mode));
            command(drive);
        } catch (ApiException ex) {
            actions.clear(); armed = false; throttle = steer = 0; mode = "manual";
            selection.clear(SystemClock.elapsedRealtime());
            immediateStop(true); listener.message(ex.getMessage());
        } catch (Exception ex) { networkFailure(ex); }
    }
    private void poll() {
        if (!isConnected() || !foreground) return;
        try {
            long started = SystemClock.elapsedRealtime();
            JSONObject state = request("GET", "/api/state", null, 800);
            if (armed && started >= armedAt && !state.optBoolean("armed", false)) {
                armed = false; throttle = steer = 0; mode = "manual";
                listener.message("电机已停用，请重新使能");
            }
            if (state.has("selected_id")) {
                long displayed = selection.reconcile(state.optLong("selected_id", 0), started);
                state.put("selected_id", displayed);
                if (!selection.pending() && selection.confirmed() <= 0 && "follow".equals(mode)) mode = "manual";
            }
            listener.state(state);
        } catch (Exception ex) { networkFailure(ex); }
    }
    private void stream() {
        try {
            HttpURLConnection connection = open("GET", "/api/video.mjpg", 2000);
            videoConnection = connection;
            if (connection.getResponseCode() != 200) throw new IOException("视频连接失败（" + connection.getResponseCode() + "）");
            try (InputStream input = new BufferedInputStream(connection.getInputStream(), 32768)) {
                MultipartReader reader = new MultipartReader(input);
                long previous = -1;
                while (!closed && connected) {
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
                    previous = part.frameId;
                    if (!closed) listener.frame(new Frame(image, meta, part.frameId)); else image.recycle();
                }
            } finally { connection.disconnect(); }
        } catch (Exception ex) { if (!closed) networkFailure(ex); }
    }
    private synchronized void command(JSONObject body) throws Exception {
        // Serialized sequence assignment + transport prevents a delayed drive overtaking STOP.
        String op = body.optString("op");
        if (("drive".equals(op) || "follow".equals(op) || "idle".equals(op)) && !isArmed())
            throw new ApiException("电机未使能");
        if ("arm".equals(op) && (!foreground || closed)) throw new ApiException("控制界面已离开");
        body.put("session", session); body.put("seq", sequence.incrementAndGet());
        request("POST", "/api/control", body, 700);
    }
    private void bestEffortDisarm() {
        try { if (!session.isEmpty()) command(object("op", "disarm")); } catch (Exception ignored) { }
    }
    private void networkFailure(Exception ex) {
        if (closed || !failed.compareAndSet(false, true)) return;
        armed = false; connected = false; throttle = steer = 0; mode = "manual"; actions.clear();
        selection.clear(SystemClock.elapsedRealtime());
        immediateStop(true);
        listener.connected(false); listener.message("连接中断，已停车：" + ex.getMessage());
        HttpURLConnection stream = videoConnection;
        if (stream != null) stream.disconnect();
    }
    public void close() {
        if (closed) return;
        background(); closed = true; connected = false;
        HttpURLConnection stream = videoConnection;
        if (stream != null) stream.disconnect();
        control.shutdownNow(); telemetry.shutdownNow(); video.shutdownNow();
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
                throw new ApiException(json.optString("error", "请求失败 HTTP " + code));
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
        ApiException(String text) { super(text); }
    }
}
