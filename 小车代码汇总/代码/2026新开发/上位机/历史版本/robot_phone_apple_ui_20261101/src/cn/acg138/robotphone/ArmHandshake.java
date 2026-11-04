package cn.acg138.robotphone;

/** Configuration must reach the FPGA before ARM. The UI never retries ARM by itself. */
public final class ArmHandshake {
    public static final int WAITING = 0, READY = 1, TIMED_OUT = 2, CANCELED = 3;
    public static final long ACK_TIMEOUT_MS = 1500;
    private boolean pending, sent;
    private long generation, deadline;
    public synchronized boolean begin(long generation) {
        if (pending) return false;
        this.generation = generation; pending = true; sent = false; return true;
    }
    public synchronized void configurationSent(long generation, long now) {
        if (pending && this.generation == generation) { sent = true; deadline = now + ACK_TIMEOUT_MS; }
    }
    public synchronized boolean pending() { return pending; }
    public synchronized void cancel() { pending = sent = false; }
    public synchronized int observe(long generation, boolean linkReady, long now) {
        if (!pending || this.generation != generation) { cancel(); return CANCELED; }
        if (!sent) return WAITING;
        if (now >= deadline) { cancel(); return TIMED_OUT; }
        if (linkReady) { cancel(); return READY; }
        return WAITING;
    }
}
