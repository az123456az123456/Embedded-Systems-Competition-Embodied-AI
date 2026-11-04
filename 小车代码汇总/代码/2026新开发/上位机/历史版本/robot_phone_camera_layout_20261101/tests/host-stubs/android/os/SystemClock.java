package android.os;
/** Test clock defaults to the host monotonic clock. Only the explicit
 * same-millisecond observation fixture freezes it; this is never in the APK. */
public final class SystemClock {
    private static volatile long fixedForTest=-1;
    public static long elapsedRealtime(){long fixed=fixedForTest;return fixed>=0?fixed:System.nanoTime()/1000000;}
    public static void setElapsedRealtimeForTest(long value){fixedForTest=value;}
}
