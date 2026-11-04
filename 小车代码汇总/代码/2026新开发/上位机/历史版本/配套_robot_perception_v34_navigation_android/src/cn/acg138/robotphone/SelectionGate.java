package cn.acg138.robotphone;

/** Orders taps, their acknowledgements and concurrent telemetry responses. */
public final class SelectionGate {
    private long generation, requested, confirmed, acknowledgedAt;
    private boolean pending;
    public synchronized long request(long target) {
        generation++; requested = target; confirmed = 0; pending = true;
        return generation;
    }
    public synchronized boolean acknowledge(long requestGeneration, long time) {
        if (!pending || requestGeneration != generation) return false;
        confirmed = requested; pending = false; acknowledgedAt = time; return true;
    }
    public synchronized void clear(long time) {
        generation++; requested = confirmed = 0; pending = false; acknowledgedAt = time;
    }
    public synchronized long reconcile(long serverTarget, long pollStartedAt) {
        // An old GET must not overwrite a tap that has not reached POST yet, or
        // a response to a GET started before the corresponding POST completed.
        if (!pending && pollStartedAt > acknowledgedAt) confirmed = Math.max(0, serverTarget);
        return display();
    }
    public synchronized long display() { return pending ? requested : confirmed; }
    public synchronized long confirmed() { return pending ? 0 : confirmed; }
    public synchronized boolean pending() { return pending; }
}
