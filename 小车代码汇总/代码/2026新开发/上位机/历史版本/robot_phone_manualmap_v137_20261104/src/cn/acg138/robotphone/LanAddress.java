package cn.acg138.robotphone;

import java.net.URI;

/** Deliberately accepts only direct private LAN IPv4 endpoints. No DNS or WAN. */
public final class LanAddress {
    public static String normalize(String value) {
        String text = value.trim();
        if (!text.contains("://")) text = "http://" + text;
        URI uri;
        try { uri = new URI(text); } catch (Exception ex) { throw new IllegalArgumentException("地址格式不正确"); }
        if (!"http".equals(uri.getScheme()) || uri.getUserInfo() != null || uri.getQuery() != null || uri.getFragment() != null)
            throw new IllegalArgumentException("请输入局域网 HTTP 地址");
        String host = uri.getHost();
        if (host == null) throw new IllegalArgumentException("请输入板子的 IPv4 地址");
        String[] parts = host.split("\\.");
        if (parts.length != 4) throw new IllegalArgumentException("请输入板子的 IPv4 地址");
        int[] n = new int[4];
        try {
            for (int i = 0; i < 4; i++) {
                if (!parts[i].matches("[0-9]{1,3}")) throw new NumberFormatException();
                n[i] = Integer.parseInt(parts[i]);
                if (n[i] > 255) throw new NumberFormatException();
            }
        } catch (NumberFormatException ex) { throw new IllegalArgumentException("IPv4 地址不正确"); }
        boolean local = n[0] == 10 || (n[0] == 192 && n[1] == 168) ||
                (n[0] == 172 && n[1] >= 16 && n[1] <= 31) || (n[0] == 169 && n[1] == 254);
        if (!local) throw new IllegalArgumentException("仅允许 10.x、172.16–31.x、192.168.x 或本地链路地址");
        int port = uri.getPort();
        if (port < -1 || port == 0 || port > 65535) throw new IllegalArgumentException("端口不正确");
        String path = uri.getPath();
        if (path != null && !path.isEmpty() && !"/".equals(path)) throw new IllegalArgumentException("地址后不要添加路径");
        return "http://" + n[0] + "." + n[1] + "." + n[2] + "." + n[3] + (port < 0 ? ":8080" : ":" + port);
    }
}
