import cn.acg138.robotphone.MapDisplayFilter;
import java.nio.file.*;
import java.nio.charset.StandardCharsets;
public class MapEnhancerProbe {
 public static void main(String[] args)throws Exception {
  int w=Integer.parseInt(args[0]),h=Integer.parseInt(args[1]);
  String cells=new String(Files.readAllBytes(Paths.get(args[2])),StandardCharsets.US_ASCII);
  int[] pixels=MapDisplayFilter.pixels(cells,w,h);byte[] gray=new byte[pixels.length];
  for(int i=0;i<pixels.length;i++)gray[i]=(byte)pixels[i];
  Files.write(Paths.get(args[3]),gray);
 }
}
