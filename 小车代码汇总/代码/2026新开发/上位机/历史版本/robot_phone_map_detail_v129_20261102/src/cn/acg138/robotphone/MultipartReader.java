package cn.acg138.robotphone;

import java.io.ByteArrayOutputStream;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Base64;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/** Bounded multipart JPEG reader: the header and image always travel together. */
public final class MultipartReader {
    public static final int MAX_JPEG = 4 * 1024 * 1024;
    private final InputStream input;
    public MultipartReader(InputStream input) { this.input = input; }
    /** Compare clock offsets, never subtract unrelated phone/RK clocks. */
    public static final class LagTracker {
        private long lowerOffset = Long.MAX_VALUE;
        public boolean fresh(long serverSendMs, long localReceiveMs, long serverAgeMs) {
            if (serverAgeMs < 0 || serverAgeMs > 350) return false;
            if (serverSendMs <= 0) return true; // compatible with old RK metadata
            long offset = localReceiveMs - serverSendMs;
            lowerOffset = Math.min(lowerOffset, offset);
            return offset - lowerOffset <= 250;
        }
    }
    public static final class Part {
        public final byte[] jpeg;
        public final String metadata;
        public final long frameId;
        Part(byte[] jpeg, String metadata, long frameId) {
            this.jpeg = jpeg; this.metadata = metadata; this.frameId = frameId;
        }
    }
    private String line() throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        for (int n = 0; n < 32768; n++) {
            int b = input.read();
            if (b < 0) throw new EOFException("视频连接已关闭");
            if (b == '\n') return out.toString("US-ASCII").replace("\r", "");
            out.write(b);
        }
        throw new IOException("视频头过长");
    }
    public Part next() throws IOException {
        String boundary = "";
        for (int n = 0; n < 8; n++) {
            boundary = line();
            if (boundary.startsWith("--")) break;
            if (!boundary.isEmpty()) throw new IOException("无效视频边界");
        }
        if (!boundary.startsWith("--") || boundary.endsWith("--")) throw new EOFException("视频流结束");
        Map<String, String> headers = new HashMap<>();
        int total = 0;
        for (int n = 0; n < 16; n++) {
            String row = line(); total += row.length();
            if (total > 32768) throw new IOException("视频头过长");
            if (row.isEmpty()) break;
            int colon = row.indexOf(':');
            if (colon < 1) throw new IOException("无效视频头");
            headers.put(row.substring(0, colon).toLowerCase(Locale.ROOT), row.substring(colon + 1).trim());
            if (n == 15) throw new IOException("视频头过多");
        }
        try {
            if (!"image/jpeg".equalsIgnoreCase(headers.get("content-type"))) throw new IOException("视频应为 JPEG");
            int length = Integer.parseInt(headers.get("content-length"));
            if (length < 4 || length > MAX_JPEG) throw new IOException("视频帧长度异常");
            long frameId = Long.parseLong(headers.get("x-frame-id"));
            byte[] meta = Base64.getDecoder().decode(headers.get("x-frame-meta"));
            if (meta.length > 24576 || frameId < 0) throw new IOException("视频坐标元数据异常");
            byte[] bytes = new byte[length];
            int done = 0;
            while (done < length) {
                int n = input.read(bytes, done, length - done);
                if (n < 0) throw new EOFException("视频帧不完整");
                done += n;
            }
            if ((bytes[0] & 255) != 255 || (bytes[1] & 255) != 216) throw new IOException("不是 JPEG 视频帧");
            return new Part(bytes, new String(meta, StandardCharsets.UTF_8), frameId);
        } catch (IllegalArgumentException | NullPointerException ex) {
            throw new IOException("视频头缺失或无效", ex);
        }
    }
}
