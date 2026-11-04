package cn.acg138.robotphone;

/** Transport/source clocks are observations. Motor authority expires at the RK
 * and FPGA, where actual accepted commands are known; the phone never renews it. */
public final class ConnectionPolicy {
    public static final int CONTROL_HTTP_TIMEOUT_MS = 800;
    // A single late HTTP reply is not evidence for an ARM/start barrier. This
    // is not a repeating phone ACK-age watchdog or a motor command lease.
    public static final long RESPONSE_EVIDENCE_MS = 1500, TELEMETRY_STALE_MS = 1500;
    public static final long CONNECTION_LOSS_MS = 30000;
    public static final long VIDEO_RETRY_MIN_MS = 250, VIDEO_RETRY_MAX_MS = 3000;
    private ConnectionPolicy() { }
    public static boolean responseEvidenceFresh(long now, long requestStarted) {
        return requestStarted > 0 && now >= requestStarted && now - requestStarted < RESPONSE_EVIDENCE_MS;
    }
    public static boolean telemetryFresh(long now, long lastReceived) {
        return lastReceived > 0 && now >= lastReceived && now - lastReceived < TELEMETRY_STALE_MS;
    }
    public static long nextVideoRetry(long previous) {
        return Math.min(VIDEO_RETRY_MAX_MS, Math.max(VIDEO_RETRY_MIN_MS, previous * 2));
    }
}
