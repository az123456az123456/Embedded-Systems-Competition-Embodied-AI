import cn.acg138.robotphone.MultipartReader;
import cn.acg138.robotphone.LanAddress;
import cn.acg138.robotphone.SelectionGate;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Base64;

public final class HostTests {
    private static int count;
    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message); count++;
    }
    private static void invalidAddress(String address) {
        try { LanAddress.normalize(address); throw new AssertionError("accepted " + address); }
        catch (IllegalArgumentException expected) { count++; }
    }
    private static byte[] part(String length, String frame, String meta, byte[] image) throws Exception {
        String headers = "--robot\r\nContent-Type: image/jpeg\r\nContent-Length: " + length + "\r\nX-Frame-Id: " + frame + "\r\nX-Frame-Meta: " + meta + "\r\n\r\n";
        byte[] h = headers.getBytes(StandardCharsets.US_ASCII), out = new byte[h.length + image.length];
        System.arraycopy(h, 0, out, 0, h.length); System.arraycopy(image, 0, out, h.length, image.length); return out;
    }
    private static void invalidFrame(byte[] bytes) throws Exception {
        try { new MultipartReader(new ByteArrayInputStream(bytes)).next(); throw new AssertionError("invalid video accepted"); }
        catch (IOException expected) { count++; }
    }
    public static void main(String[] args) throws Exception {
        check(LanAddress.normalize("192.168.1.20").equals("http://192.168.1.20:8080"), "default port");
        check(LanAddress.normalize("http://10.0.2.2:8080/").equals("http://10.0.2.2:8080"), "emulator LAN");
        check(LanAddress.normalize("172.31.8.1:9000").equals("http://172.31.8.1:9000"), "private range");
        check(LanAddress.normalize("169.254.8.2").contains("169.254"), "link local");
        for (String address : new String[]{"https://192.168.1.2", "http://8.8.8.8", "http://172.32.1.2", "http://localhost", "http://192.168.1.2:0", "http://192.168.256.1", "http://192.168.1.2/api", "http://x@192.168.1.2", "http://192.168.1.2?x=1", "http://192.168.1.2/#x"}) invalidAddress(address);
        byte[] image = {(byte)255, (byte)216, (byte)255, (byte)217};
        String json = "{\"frame_id\":7,\"label\":\"瓶子\"}";
        String meta = Base64.getEncoder().encodeToString(json.getBytes(StandardCharsets.UTF_8));
        MultipartReader.Part parsed = new MultipartReader(new ByteArrayInputStream(part("4", "7", meta, image))).next();
        check(parsed.frameId == 7 && parsed.metadata.equals(json) && parsed.jpeg.length == 4, "paired JPEG metadata including UTF-8");
        invalidFrame(part("999999999", "7", meta, image));
        invalidFrame(part("-1", "7", meta, image));
        invalidFrame(part("4", "-1", meta, image));
        invalidFrame(part("4", "7", "not base64!", image));
        invalidFrame(part("10", "7", meta, image));
        invalidFrame(part("4", "7", meta, new byte[]{0, 0, 0, 0}));
        invalidFrame("garbage\r\n".getBytes(StandardCharsets.US_ASCII));
        byte[] p = part("4", "7", meta, image), separator = "\r\n".getBytes(StandardCharsets.US_ASCII);
        byte[] two = new byte[p.length * 2 + separator.length]; System.arraycopy(p,0,two,0,p.length); System.arraycopy(separator,0,two,p.length,2); System.arraycopy(p,0,two,p.length+2,p.length);
        MultipartReader reader = new MultipartReader(new ByteArrayInputStream(two));
        check(reader.next().frameId == 7 && reader.next().frameId == 7, "multipart boundaries and repeated frame");
        SelectionGate gate = new SelectionGate();
        long first = gate.request(8);
        check(gate.pending() && gate.confirmed() == 0 && gate.display() == 8, "tap cannot enable follow before ACK");
        check(gate.reconcile(0, 100) == 8, "poll while POST queued preserves tap");
        check(gate.acknowledge(first, 120) && gate.confirmed() == 8, "matching selection ACK enables follow");
        check(gate.reconcile(0, 119) == 8 && gate.reconcile(0, 120) == 8, "old or same-tick GET cannot erase successful POST");
        check(gate.reconcile(0, 121) == 0, "new target-lost state clears selection");
        first = gate.request(9); long second = gate.request(10);
        check(!gate.acknowledge(first, 130) && gate.pending() && gate.display() == 10, "old tap ACK cannot select another object");
        check(gate.acknowledge(second, 140) && gate.confirmed() == 10, "newest tap ACK accepted");
        long late = gate.request(11); gate.clear(150);
        check(!gate.acknowledge(late, 151) && gate.confirmed() == 0 && !gate.pending(), "STOP invalidates delayed selection ACK");
        System.out.println("PASS " + count + " Android host checks: private LAN, bounded synchronized video, selection ACK and telemetry races");
    }
}
