package android.graphics;
/** Host network tests never decode the parked fake video connection. */
public final class BitmapFactory {
    public static class Options { public boolean inJustDecodeBounds; public int outWidth, outHeight; }
    public static Bitmap decodeByteArray(byte[] bytes,int start,int length){return null;}
    public static Bitmap decodeByteArray(byte[] bytes,int start,int length,Options options){return null;}
}
