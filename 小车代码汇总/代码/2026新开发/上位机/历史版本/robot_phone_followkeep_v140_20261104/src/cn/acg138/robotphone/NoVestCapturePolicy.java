package cn.acg138.robotphone;

import org.json.JSONArray;
import org.json.JSONObject;

/** Each violation category independently requires >500ms of fresh observations.
 * Class name is retained so existing archive/settings retain their API and data.
 */
public final class NoVestCapturePolicy {
    public static final long APPEARANCE_MS=500,MAX_SAMPLE_GAP_MS=350,MAX_AGE_MS=350;
    private final SingleAppearance[] events={new SingleAppearance(0),new SingleAppearance(1),new SingleAppearance(2)};
    public static int category(String label) {
        if(label==null)return -1;
        String name=label.trim().toLowerCase(java.util.Locale.ROOT).replace('-','_').replace(' ','_');
        switch(name) {
            case "no_vest": case "no_safety_vest":return 0;
            case "no_mask":return 1;
            case "no_helmet":case "no_halmet":case "no_hardhat":return 2;
            default:return -1;
        }
    }
    public static final class Ticket {
        private final SingleAppearance.Ticket[] members;
        private Ticket(SingleAppearance.Ticket[] members){this.members=members;}
    }
    public synchronized void reset(){for(SingleAppearance event:events)event.reset();}
    public synchronized void interrupted(){for(SingleAppearance event:events)event.interrupted();}
    public synchronized Ticket observe(JSONObject metadata,long elapsedMs) {
        SingleAppearance.Ticket[] members=new SingleAppearance.Ticket[events.length];boolean any=false;
        for(int i=0;i<events.length;i++){members[i]=events[i].observe(metadata,elapsedMs);any|=members[i]!=null;}
        return any?new Ticket(members):null;
    }
    public synchronized void complete(Ticket ticket,boolean success,long elapsedMs) {
        if(ticket==null)return;
        for(int i=0;i<events.length;i++)events[i].complete(ticket.members[i],success,elapsedMs);
    }
    private static final class SingleAppearance {
    public static final long APPEARANCE_MS = 500;
    public static final long MAX_SAMPLE_GAP_MS = 350;
    public static final long MAX_AGE_MS = 350;
    private static final long RETRY_MS = 3000, ABSENCE_MS = 1000;
    private final int category;
    SingleAppearance(int category) { this.category=category; }
    private long lastResult = -1, lastSource = -1, lastObserved = -1, firstPositive = -1, firstPositiveSource = -1;
    private long firstNegative = -1, firstNegativeSource = -1, lastNegative = -1, lastNegativeSource = -1;
    private long lastPositiveSource = -1, generation = 1, pendingGeneration, retryAfter;
    private boolean saved, pending;

    public static final class Ticket {
        final long generation;
        private Ticket(long generation) { this.generation = generation; }
    }
    public synchronized void reset() {
        generation++; lastResult = lastSource = -1; breakTiming(); breakAbsence();
        pendingGeneration = retryAfter = 0; saved = pending = false;
    }
    /** A missing transport is not proof that the target disappeared. Keep the saved
     * latch (and any accepted disk job), while dropping every timing/sequence anchor. */
    public synchronized void interrupted() { lastResult = lastSource = -1; breakTiming(); breakAbsence(); }
    private void breakTiming() { firstPositive = firstPositiveSource = lastObserved = lastPositiveSource = -1; }
    private void breakAbsence() { firstNegative = firstNegativeSource = lastNegative = lastNegativeSource = -1; }
    private boolean continuous(long elapsed, long source, long previousElapsed, long previousSource, long firstElapsed, long firstSource) {
        long hostStep = elapsed - previousElapsed, sourceStep = source - previousSource;
        return previousElapsed >= 0 && previousSource >= 0 && hostStep >= 0 && hostStep <= MAX_SAMPLE_GAP_MS &&
                sourceStep > 0 && sourceStep <= MAX_SAMPLE_GAP_MS &&
                Math.abs((elapsed - firstElapsed) - (source - firstSource)) <= MAX_SAMPLE_GAP_MS;
    }
    public synchronized Ticket observe(JSONObject metadata, long elapsedMs) {
        if (metadata == null || elapsedMs < 0 || metadata.optBoolean("demo")) { breakTiming(); breakAbsence(); return null; }
        long result = metadata.optLong("detections_frame_id", -1);
        long source = metadata.optLong("detection_timestamp_ms", -1);
        long imageAge = metadata.optLong("age_ms", -1);
        long boxAge = metadata.optLong("box_age_ms", -1);
        long advertisedMax = metadata.optLong("box_max_age_ms", MAX_AGE_MS);
        JSONArray detections = metadata.optJSONArray("detections");
        boolean fresh = result > 0 && source > 0 && imageAge >= 0 && imageAge <= MAX_AGE_MS &&
                boxAge >= 0 && boxAge <= Math.min(MAX_AGE_MS, advertisedMax) && detections != null;
        if (!fresh) { breakTiming(); breakAbsence(); return null; }
        // Preview repetition and out-of-order inference results cannot advance time,
        // clear an appearance, or serve as negative detection evidence.
        if (result <= lastResult || source <= lastSource) return null;
        lastResult = result; lastSource = source;
        boolean present = false;
        for (int i = 0; i < Math.min(detections.length(), 128); i++) {
            JSONObject detection = detections.optJSONObject(i);
            if (detection != null && category(detection.optString("label")) == category) { present = true; break; }
        }
        if (!present) {
            // Confirm absence on both the phone elapsed clock and the real inference
            // source clock; paced playback of old cached metadata is not continuity.
            if (!continuous(elapsedMs, source, lastNegative, lastNegativeSource, firstNegative, firstNegativeSource)) {
                firstNegative = elapsedMs; firstNegativeSource = source;
            }
            lastNegative = elapsedMs; lastNegativeSource = source;
            if (elapsedMs - firstNegative >= ABSENCE_MS && source - firstNegativeSource >= ABSENCE_MS) {
                if (saved || pending) generation++;
                saved = pending = false; pendingGeneration = retryAfter = 0;
            }
            breakTiming(); return null;
        }
        breakAbsence();
        if (!continuous(elapsedMs, source, lastObserved, lastPositiveSource, firstPositive, firstPositiveSource)) {
            firstPositive = elapsedMs; firstPositiveSource = source;
        }
        lastObserved = elapsedMs; lastPositiveSource = source;
        if (saved || pending || elapsedMs < retryAfter || elapsedMs - firstPositive <= APPEARANCE_MS ||
                source - firstPositiveSource <= APPEARANCE_MS) return null;
        pending = true; pendingGeneration = generation;
        return new Ticket(generation);
    }
    public synchronized void complete(Ticket ticket, boolean success, long elapsedMs) {
        if (ticket == null || !pending || ticket.generation != generation || pendingGeneration != generation) return;
        pending = false;
        if (success) saved = true;
        else retryAfter = Math.max(0, elapsedMs) + RETRY_MS;
    }
    }
}
