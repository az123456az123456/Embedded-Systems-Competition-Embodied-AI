package cn.acg138.robotphone;

/** Independent transport clocks. None can extend the RK/FPGA 350 ms control lease. */
public final class ConnectionPolicy {
    public static final int CONTROL_HTTP_TIMEOUT_MS = 200;
    public static final long CONTROL_LEASE_MS = 350, TELEMETRY_STALE_MS = 1500;
    public static final long VIDEO_RETRY_MIN_MS = 250, VIDEO_RETRY_MAX_MS = 3000;
    private ConnectionPolicy() { }
    public static boolean leaseFresh(long now, long lastSent) {
        return lastSent > 0 && now >= lastSent && now - lastSent < CONTROL_LEASE_MS;
    }
    public static boolean telemetryFresh(long now, long lastReceived) {
        return lastReceived > 0 && now >= lastReceived && now - lastReceived < TELEMETRY_STALE_MS;
    }
    public static long nextVideoRetry(long previous) {
        return Math.min(VIDEO_RETRY_MAX_MS, Math.max(VIDEO_RETRY_MIN_MS, previous * 2));
    }
}
