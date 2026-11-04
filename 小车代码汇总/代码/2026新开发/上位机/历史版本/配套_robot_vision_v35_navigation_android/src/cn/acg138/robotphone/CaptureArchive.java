package cn.acg138.robotphone;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;
import java.util.LinkedHashSet;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import org.json.JSONObject;

/** A bounded disk worker independent of the control/UI and Bitmap lifetime. */
public final class CaptureArchive implements AutoCloseable {
    public interface Store {
        String write(Stamp stamp, byte[] jpeg) throws Exception;
        default void ensureDay(String day) throws Exception { }
    }
    public interface Clock { long wallMillis(); long elapsedMillis(); TimeZone timeZone(); }
    public interface Listener { void saved(String relativePath); void failed(String message); }
    public static final class Stamp {
        public final String day, filename;
        private Stamp(String day, String filename) { this.day = day; this.filename = filename; }
        public static Stamp at(long wallMs, TimeZone zone, long frameId) {
            SimpleDateFormat day = new SimpleDateFormat("yyyy-MM-dd", Locale.US);
            SimpleDateFormat time = new SimpleDateFormat("HH-mm-ss-SSS", Locale.US);
            day.setTimeZone(zone); time.setTimeZone(zone);
            Date moment = new Date(wallMs);
            return new Stamp(day.format(moment), time.format(moment) + "_frame" + Math.max(0, frameId) + ".jpg");
        }
    }
    private final NoVestCapturePolicy policy = new NoVestCapturePolicy();
    private final Clock clock;
    private final Listener listener;
    private final ThreadPoolExecutor worker;
    private Store store;
    private boolean enabled, closed;
    private long epoch, dayRetryAfter;
    private final LinkedHashSet<String> ensuredDays = new LinkedHashSet<>();
    private final LinkedHashSet<String> pendingDays = new LinkedHashSet<>();
    public CaptureArchive(Clock clock, Listener listener) {
        this.clock = clock; this.listener = listener;
        worker = new ThreadPoolExecutor(1, 1, 0, TimeUnit.MILLISECONDS, new ArrayBlockingQueue<Runnable>(2), new ThreadFactory() {
            @Override public Thread newThread(Runnable task) { Thread t = new Thread(task, "no-vest-storage"); t.setDaemon(true); t.setPriority(Thread.MIN_PRIORITY); return t; }
        }, new ThreadPoolExecutor.AbortPolicy());
    }
    public synchronized void configure(Store destination, boolean requested) {
        epoch++;
        if (store != destination) { ensuredDays.clear(); dayRetryAfter = 0; }
        store = destination; enabled = requested && destination != null && !closed;
        pendingDays.clear(); worker.getQueue().clear(); policy.reset();
    }
    /** Called by the foreground date tick. Only the first successful observation
     * of a day touches SAF; repeated ticks and same-directory toggle do no disk I/O. */
    public synchronized void ensureCurrentDay() {
        if (!enabled || closed || store == null || clock.elapsedMillis() < dayRetryAfter) return;
        final String day = Stamp.at(clock.wallMillis(), clock.timeZone(), 0).day;
        if (ensuredDays.contains(day) || pendingDays.contains(day)) return;
        final long acceptedEpoch = epoch;
        final Store acceptedStore = store;
        pendingDays.add(day);
        try {
            worker.execute(() -> {
                synchronized (CaptureArchive.this) { if (closed || epoch != acceptedEpoch) return; }
                String error = null;
                try { acceptedStore.ensureDay(day); }
                catch (Exception failure) { error = failure.getMessage(); if (error == null || error.length() == 0) error = failure.getClass().getSimpleName(); }
                synchronized (CaptureArchive.this) {
                    if (closed || epoch != acceptedEpoch) return;
                    pendingDays.remove(day);
                    if (error == null) {
                        ensuredDays.add(day);
                        // Bound memory if the phone clock is repeatedly changed.
                        while (ensuredDays.size() > 32) ensuredDays.remove(ensuredDays.iterator().next());
                        dayRetryAfter = 0;
                    } else {
                        dayRetryAfter = Math.max(0, clock.elapsedMillis()) + 3000;
                        listener.failed("日期目录创建失败：" + error);
                    }
                }
            });
        } catch (RejectedExecutionException full) {
            pendingDays.remove(day); dayRetryAfter = Math.max(0, clock.elapsedMillis()) + 3000;
            listener.failed("日期目录等待创建：存储队列已满");
        }
    }
    public synchronized void newSession() {
        // Transport interruption is not fresh evidence of target absence. Already
        // accepted photographs may finish; retain their success latch on reconnect.
        policy.interrupted();
    }
    public synchronized void frame(JSONObject metadata, byte[] jpeg, long frameId, long elapsedMs) {
        if (!enabled || closed || store == null) return;
        final NoVestCapturePolicy.Ticket ticket = policy.observe(metadata, elapsedMs);
        if (ticket == null) return;
        if (jpeg == null || jpeg.length < 4 || (jpeg[0] & 255) != 255 || (jpeg[1] & 255) != 216) {
            policy.complete(ticket, false, clock.elapsedMillis()); listener.failed("没有可保存的摄像头 JPEG 数据"); return;
        }
        final long acceptedEpoch = epoch;
        final Store acceptedStore = store;
        // Use the phone's clock/zone at capture, never the board's clock or the date
        // when a slow provider finishes. Re-evaluate for every photograph.
        final Stamp stamp = Stamp.at(clock.wallMillis(), clock.timeZone(), frameId);
        try {
            worker.execute(() -> {
                synchronized (CaptureArchive.this) { if (closed || epoch != acceptedEpoch) return; }
                String path = null, error = null;
                try { path = acceptedStore.write(stamp, jpeg); if (path == null || path.length() == 0) throw new IllegalStateException("存储服务未确认保存结果"); }
                catch (Exception failure) { error = failure.getMessage(); if (error == null || error.length() == 0) error = failure.getClass().getSimpleName(); }
                synchronized (CaptureArchive.this) {
                    if (closed || epoch != acceptedEpoch) return;
                    policy.complete(ticket, error == null, clock.elapsedMillis());
                    if (error == null) listener.saved(path); else listener.failed(error);
                }
            });
        } catch (RejectedExecutionException full) {
            policy.complete(ticket, false, clock.elapsedMillis()); listener.failed("存储队列已满，请检查所选目录或存储空间");
        }
    }
    public synchronized int queued() { return worker.getQueue().size(); }
    @Override public synchronized void close() {
        closed = true; enabled = false; epoch++; policy.reset(); pendingDays.clear(); worker.getQueue().clear(); worker.shutdownNow();
    }
}
