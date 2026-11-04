package cn.acg138.robotphone;
public final class VideoFreshnessTests {
    private static int checks;
    private static void check(boolean value) { checks++; if (!value) throw new AssertionError("video freshness"); }
    public static void main(String[] args) {
        MultipartReader.LagTracker tracker = new MultipartReader.LagTracker();
        check(tracker.fresh(1000, 900000, 10)); // independent monotonic clock epochs
        check(tracker.fresh(1033, 900034, 15));
        check(tracker.fresh(1066, 900080, 20));
        check(!tracker.fresh(1099, 903100, 12)); // three-second receive backlog
        check(!tracker.fresh(1132, 900132, 351)); // already old before sending
        check(tracker.fresh(1165, 900165, 22));
        check(!tracker.fresh(1198, 900198, -1));
        check(new MultipartReader.LagTracker().fresh(0, 10000, 12)); // old RK compatibility
        System.out.println("PASS Android independent-clock video freshness " + checks + " checks");
    }
}
