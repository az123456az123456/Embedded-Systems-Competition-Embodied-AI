import cn.acg138.robotphone.*;
import java.io.*;
import java.net.*;
import javax.imageio.ImageIO;
/** Actual RobotClient threads/parser/decode path, in-memory HTTP and host JPEG. */
public final class ClientVideoLatestTests extends ClientNetworkTests {
    public static void main(String[] args)throws Exception{
        Thread.setDefaultUncaughtExceptionHandler((thread,error)->{error.printStackTrace();System.exit(1);});
        java.awt.image.BufferedImage image=new java.awt.image.BufferedImage(8,6,java.awt.image.BufferedImage.TYPE_INT_RGB);
        ByteArrayOutputStream jpeg=new ByteArrayOutputStream();ImageIO.write(image,"jpg",jpeg);jpg=jpeg.toByteArray();
        URL.setURLStreamHandlerFactory(protocol->protocol.equals("http")?new URLStreamHandler(){protected URLConnection openConnection(URL url){return new Connection(url);}}:null);
        android.graphics.BitmapFactory.testDecodeDelayMs=45;
        videoMode=8;long begin=now();RobotClient c=start();
        await(()->lastFrame!=null&&lastFrame.id==241,1200,"slow decoder reaches newest of 200 received frames without decoding FIFO");
        check(now()-begin<1200,"burst latest frame bounded");
        check(videoFrames.get()<15,"historical images are replaced before JPEG decoding");
        arm(c);check(c.isArmed()&&c.isConnected(),"decode overload never cancels motor authorization");
        check(lastFrame.received<=now()&&now()-lastFrame.received<250,"Frame received clock survives decode handoff");
        finish(c);System.out.println("PASS Android production receive/decode latest-slot "+checks+" checks; received=200 decoded="+videoFrames.get()+"; no RF hardware");
    }
}
