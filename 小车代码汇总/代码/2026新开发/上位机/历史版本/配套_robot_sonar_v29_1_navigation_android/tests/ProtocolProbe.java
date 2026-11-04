import cn.acg138.robotphone.MultipartReader;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import javax.imageio.ImageIO;

/** Host-only integration test against the real RK server. Never included in the APK. */
public final class ProtocolProbe {
    public static void main(String[] args) throws Exception {
        String token = new String(Files.readAllBytes(Paths.get(args[1])), StandardCharsets.UTF_8).trim();
        HttpURLConnection connection = (HttpURLConnection)new URL(args[0] + "/api/video.mjpg").openConnection();
        connection.setRequestProperty("Authorization", "Bearer " + token); connection.setConnectTimeout(3000); connection.setReadTimeout(5000);
        if (connection.getResponseCode() != 200) throw new AssertionError("video HTTP " + connection.getResponseCode());
        MultipartReader reader = new MultipartReader(connection.getInputStream());
        long previous = -1;
        for (int i = 0; i < 3; i++) {
            MultipartReader.Part part = reader.next();
            Matcher matcher = Pattern.compile("\"frame_id\"\\s*:\\s*([0-9]+)").matcher(part.metadata);
            if (!matcher.find() || Long.parseLong(matcher.group(1)) != part.frameId || part.frameId < previous) throw new AssertionError("frame mismatch");
            BufferedImage image = ImageIO.read(new ByteArrayInputStream(part.jpeg));
            Matcher width = Pattern.compile("\"width\"\\s*:\\s*([0-9]+)").matcher(part.metadata);
            Matcher height = Pattern.compile("\"height\"\\s*:\\s*([0-9]+)").matcher(part.metadata);
            if (image == null || !width.find() || !height.find() || image.getWidth() != Integer.parseInt(width.group(1)) || image.getHeight() != Integer.parseInt(height.group(1))) throw new AssertionError("JPEG dimensions must match its metadata");
            if (!part.metadata.contains("age_ms") || !part.metadata.contains("detections") || !part.metadata.contains("box_age_ms") || !part.metadata.contains("detections_frame_id")) throw new AssertionError("missing async-preview metadata");
            previous = part.frameId;
        }
        connection.disconnect();
        System.out.println("PASS Android multipart reader: 3 actual server JPEG frames, coordinates, frame IDs, capture age");
    }
}
