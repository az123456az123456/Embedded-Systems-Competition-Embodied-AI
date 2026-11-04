package cn.acg138.robotphone;

/** Visual time line only. Never queues, authorizes or delays a control action. */
public final class ButtonFeedback {
    public static final long PRESS_MS = 100, RELEASE_MS = 240;
    public String key;
    public float x, y;
    private long downAt, upAt = -1;
    private float releaseScale = 1;
    private boolean motion;

    public void down(String key, float x, float y, long now, boolean motion) {
        this.key = key; this.x = x; this.y = y; this.motion = motion;
        downAt = now; upAt = -1; releaseScale = 1;
    }
    public void up(long now) {
        if (key == null || upAt >= 0) return;
        releaseScale = scale(now); upAt = now;
    }
    public void clear() { key = null; upAt = -1; }
    private static float unit(long elapsed, long duration) {
        return Math.max(0, Math.min(1, (float)elapsed / duration));
    }
    public float scale(long now) {
        if (key == null || !motion) return 1;
        if (upAt < 0) {
            float t = unit(now - downAt, PRESS_MS);
            return 1 - .035f * (1 - (1-t)*(1-t));
        }
        float t = unit(now - upAt, RELEASE_MS), u = t - 1;
        // A small release spring; at most a fraction of a percent overshoot.
        float ease = 1 + 1.8f*u*u*u + .8f*u*u;
        return releaseScale + (1-releaseScale) * ease;
    }
    public float ripple(long now) { return unit(now-downAt, 360); }
    public float opacity(long now) {
        if (key == null || !motion) return 0;
        return upAt < 0 ? .14f : .14f * (1-unit(now-upAt, RELEASE_MS));
    }
    public boolean active(long now) { return key != null && (upAt < 0 || now-upAt < RELEASE_MS); }
    public boolean animating(long now) {
        return motion && active(now) && (upAt >= 0 || now-downAt < 360);
    }
}
