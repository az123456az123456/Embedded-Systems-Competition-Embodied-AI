package android.graphics;
import java.io.ByteArrayInputStream;
import javax.imageio.ImageIO;
/** Host-only decoder; Android APK uses platform BitmapFactory. */
public final class BitmapFactory {
    public static volatile long testDecodeDelayMs;
    public static class Options { public boolean inJustDecodeBounds; public int outWidth, outHeight; }
    public static Bitmap decodeByteArray(byte[] b,int start,int length) { return decodeByteArray(b,start,length,null); }
    public static Bitmap decodeByteArray(byte[] b,int start,int length,Options o) {
        try {
            if(o==null&&testDecodeDelayMs>0)Thread.sleep(testDecodeDelayMs);
            java.awt.image.BufferedImage image=ImageIO.read(new ByteArrayInputStream(b,start,length));
            if(image==null)return null;
            if(o!=null){o.outWidth=image.getWidth();o.outHeight=image.getHeight();if(o.inJustDecodeBounds)return null;}
            return new Bitmap(image);
        } catch(Exception ex){return null;}
    }
}
