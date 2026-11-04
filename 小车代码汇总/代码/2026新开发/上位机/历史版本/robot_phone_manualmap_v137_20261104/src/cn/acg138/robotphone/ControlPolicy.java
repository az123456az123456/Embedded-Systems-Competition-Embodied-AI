package cn.acg138.robotphone;

/** Android-independent limits shared by touch handling, settings and host tests. */
public final class ControlPolicy {
    private ControlPolicy() { }
    public static final int LEFT_MASK = 10; // M2 left rear, M4 left front.
    public static final int INVERT_MASK = 15; // Verified v15/v16 baseline: all four command directions inverted.
    public static final int SETTINGS_REVISION = 16;
    public static final float ROTATION_SPEED = .30f;
    public static final int BOX_MAX_AGE_MS = 350;

    public static boolean validPairing(String value) {
        if (value == null || value.indexOf('\n') >= 0 || value.indexOf('\r') >= 0) return false;
        if (value.matches("[0-9]{6}")) return true;
        return value.length() >= 8 && value.length() <= 128;
    }

    public static boolean selectable(long localAge, long frameAgeAtReceipt, long boxAgeAtReceipt, long maxBoxAge) {
        if (localAge < 0 || frameAgeAtReceipt < 0 || boxAgeAtReceipt < 0) return false;
        return localAge + frameAgeAtReceipt < 700 && localAge + boxAgeAtReceipt <= Math.min(BOX_MAX_AGE_MS, Math.max(0, maxBoxAge));
    }

    /** One finger owns motion; a second finger cancels the complete gesture. */
    public static final class Gesture {
        private int owner = -1;
        private boolean blocked;
        public boolean begin(int pointer, boolean armed) {
            if (!armed || blocked || owner >= 0 || pointer < 0) return false;
            owner = pointer; return true;
        }
        public int owner() { return owner; }
        public boolean isBlocked() { return blocked; }
        public void cancel() { owner = -1; blocked = true; }
        public void end() { owner = -1; blocked = false; }
    }
}
