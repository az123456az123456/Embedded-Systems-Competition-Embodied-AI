import cn.acg138.robotphone.MultipartReader;
import cn.acg138.robotphone.LanAddress;
import cn.acg138.robotphone.SelectionGate;
import cn.acg138.robotphone.ControlPolicy;
import cn.acg138.robotphone.ArmHandshake;
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
        for (long poll = 121; poll <= 621; poll += 100)
            check(gate.reconcile(8, poll) == 8, "fresh server selection survives repeated idle telemetry polls");
        check(gate.reconcile(0, 622) == 0, "new target-lost state clears selection");
        first = gate.request(9); long second = gate.request(10);
        check(!gate.acknowledge(first, 130) && gate.pending() && gate.display() == 10, "old tap ACK cannot select another object");
        check(gate.acknowledge(second, 140) && gate.confirmed() == 10, "newest tap ACK accepted");
        long late = gate.request(11); gate.clear(150);
        check(!gate.acknowledge(late, 151) && gate.confirmed() == 0 && !gate.pending(), "STOP invalidates delayed selection ACK");
        check(ControlPolicy.LEFT_MASK == 10 && ControlPolicy.INVERT_MASK == 15, "verified v15/v16 M2/M4 left and all-four command inversion");
        check(ControlPolicy.validPairing("012345") && ControlPolicy.validPairing("existing-long-code"), "six-digit leading-zero and legacy pairing");
        for (String value : new String[]{"12345", "1234567", "abcdef", "12345\n", "12345678\r", new String(new char[129]).replace('\0','x')})
            check(!ControlPolicy.validPairing(value), "reject malformed pairing");
        check(ControlPolicy.selectable(10, 30, 300, 350), "current JPEG may carry a valid older detection frame");
        check(!ControlPolicy.selectable(60, 30, 300, 350), "network delay expires detection boxes");
        check(!ControlPolicy.selectable(20, 700, 0, 350), "stale JPEG is never selectable");
        check(!ControlPolicy.selectable(0, 0, -1, 350), "negative timestamps rejected");
        check(!ControlPolicy.selectable(50, 0, 200, 100), "server stricter age limit is respected");
        ControlPolicy.Gesture gesture = new ControlPolicy.Gesture();
        check(!gesture.begin(1, false) && gesture.owner() < 0, "cannot rotate or drive before arm");
        check(gesture.begin(7, true) && gesture.owner() == 7, "armed single finger owns hold");
        gesture.cancel();
        check(gesture.owner() < 0 && gesture.isBlocked() && !gesture.begin(8, true), "multitouch/CANCEL clears ownership and blocks restart");
        gesture.end();
        check(gesture.begin(9, true), "only a new gesture after all fingers release can restart");
        gesture.end(); check(gesture.owner() < 0 && !gesture.isBlocked(), "normal release removes motion ownership");
        ArmHandshake arm = new ArmHandshake();
        check(arm.begin(3) && !arm.begin(3), "one configure handshake pending at a time");
        check(arm.observe(3, true, 100) == ArmHandshake.WAITING, "an old link ACK cannot arm before new configure POST");
        arm.configurationSent(3, 200);
        check(arm.observe(3, false, 250) == ArmHandshake.WAITING, "50ms configure-to-arm race waits for FPGA ACK");
        check(arm.observe(3, true, 350) == ArmHandshake.READY && !arm.pending(), "only confirmed FPGA configuration releases ARM");
        check(arm.observe(3, true, 400) == ArmHandshake.CANCELED, "ACK cannot trigger ARM twice");
        check(arm.begin(4), "can start a fresh explicit ARM request"); arm.configurationSent(4, 500);
        check(arm.observe(4, false, 1999) == ArmHandshake.WAITING, "configuration has full 1500ms deadline");
        check(arm.observe(4, true, 2000) == ArmHandshake.TIMED_OUT && !arm.pending(), "late ACK never starts motors after timeout");
        arm.begin(5); arm.configurationSent(5, 3000);
        check(arm.observe(6, true, 3050) == ArmHandshake.CANCELED, "STOP generation invalidates pending ARM");
        arm.begin(7); arm.configurationSent(7, 4000); arm.cancel();
        check(!arm.pending() && arm.observe(7, true, 4050) == ArmHandshake.CANCELED, "background cancellation cannot auto-arm on a later ACK");
        System.out.println("PASS " + count + " Android host checks: LAN, pairing, synchronized video, selection ACK races, stale boxes and single-finger motion ownership");
    }
}
