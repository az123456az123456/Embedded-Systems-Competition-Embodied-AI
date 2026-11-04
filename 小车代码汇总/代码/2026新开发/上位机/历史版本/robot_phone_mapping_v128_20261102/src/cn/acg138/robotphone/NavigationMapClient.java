package cn.acg138.robotphone;

import android.os.SystemClock;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import org.json.JSONObject;

/** Read-only map polling on its own executor: cannot block control heartbeats, telemetry or video. */
public final class NavigationMapClient {
    public interface Listener { void map(NavigationMap map); void unavailable(String reason); }
    private final String base,token;private final Listener listener;
    private final ScheduledExecutorService worker=Executors.newSingleThreadScheduledExecutor();
    private final AtomicBoolean started=new AtomicBoolean();
    private volatile boolean closed;private volatile HttpURLConnection active;
    public NavigationMapClient(String base,String token,Listener listener) {
        this.base=LanAddress.normalize(base);this.token=token;this.listener=listener;
    }
    public void start(){if(!closed&&started.compareAndSet(false,true))worker.scheduleWithFixedDelay(this::poll,0,100,TimeUnit.MILLISECONDS);}
    private void poll(){
        if(closed)return;
        HttpURLConnection connection=null;
        try{
            long requestedAt=SystemClock.elapsedRealtime();
            connection=(HttpURLConnection)new URL(base+"/api/navigation/map").openConnection();active=connection;
            connection.setRequestMethod("GET");connection.setInstanceFollowRedirects(false);
            connection.setConnectTimeout(1000);connection.setReadTimeout(1000);
            connection.setRequestProperty("Authorization","Bearer "+token);connection.setRequestProperty("Cache-Control","no-store");
            int code=connection.getResponseCode();
            if(code!=200)throw new IllegalArgumentException(code==404?"导航服务未启动":code==401||code==403?"地图配对验证失败":"导航服务暂不可用（"+code+"）");
            ByteArrayOutputStream buffer=new ByteArrayOutputStream();
            try(InputStream input=connection.getInputStream()){
                byte[] part=new byte[4096];int count;
                while((count=input.read(part))!=-1){
                    if(buffer.size()+count>262144)throw new IllegalArgumentException("地图响应过大");
                    buffer.write(part,0,count);
                }
            }
            NavigationMap map=NavigationMap.parse(new JSONObject(new String(buffer.toByteArray(),StandardCharsets.UTF_8)),requestedAt);
            if(!closed)listener.map(map);
        }catch(Exception e){if(!closed)listener.unavailable(e instanceof IllegalArgumentException?e.getMessage():"地图连接中断，等待更新");}
        finally{if(connection!=null)connection.disconnect();active=null;}
    }
    public void close(){closed=true;worker.shutdownNow();HttpURLConnection connection=active;if(connection!=null)connection.disconnect();}
}
