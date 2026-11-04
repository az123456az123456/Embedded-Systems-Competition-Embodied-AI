import cn.acg138.robotphone.*;
import org.json.JSONObject;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.List;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.lang.reflect.Field;

/** Production RobotClient + ConnectionPolicy; real HTTP sockets to production Gateway.
 * Reflection changes only validated test host 192.168.1.1 -> loopback, never client logic. */
public class ProductionPhoneTiming {
 static List<String> messages = new CopyOnWriteArrayList<>();
 static volatile JSONObject state;
 static final java.util.concurrent.atomic.AtomicInteger delivered = new java.util.concurrent.atomic.AtomicInteger();
 static long now(){return System.nanoTime()/1000000;}
 static void await(java.util.function.BooleanSupplier condition,long timeout,String label)throws Exception{
  long end=now()+timeout;while(!condition.getAsBoolean()&&now()<end)Thread.sleep(5);
  if(!condition.getAsBoolean())throw new AssertionError(label+": "+messages);
 }
 public static void main(String[]args)throws Exception{
  int port=Integer.parseInt(new String(Files.readAllBytes(Paths.get(args[0],"port"))).trim());
  RobotClient c=new RobotClient("http://192.168.1.1:"+port,"012345",new RobotClient.Listener(){
   public void message(String m){messages.add(m);System.out.println("CLIENT "+m);}
   public void connected(boolean b){System.out.println("CLIENT connected="+b);}
   public void state(JSONObject s){state=s;delivered.incrementAndGet();}
   public void frame(RobotClient.Frame f){}
  });
  Field base=RobotClient.class.getDeclaredField("base");base.setAccessible(true);base.set(c,"http://127.0.0.1:"+port);
  try {
  c.start();await(()->c.isConnected()&&state!=null,2000,"connect/state");c.arm(10,15);await(c::isArmed,2000,"arm");
  await(()->state!=null&&state.optBoolean("armed")&&state.optJSONObject("navigation").optBoolean("can_start"),2000,"fresh armed state");c.startNavigation();await(()->messages.stream().anyMatch(m->m.contains("导航已开始")),2000,"start accepted");long started=now();
  if(args[1].equals("old")) {
   await(()->!c.isArmed(),2500,"actual production telemetry watchdog should reproduce STOP");
   await(()->messages.stream().anyMatch(m->m.contains("状态数据未更新")),200,"actual watchdog reason delivered");
   System.out.println("EXPECTED_OLD_FAILURE actual RobotClient: navigation accepted then disarmed after "+(now()-started)+"ms; connected="+c.isConnected()+"; reason=状态数据未更新; no simulated parse/health logic");
  } else {
   int before=delivered.get();Thread.sleep(3200);
   if(!c.isConnected()||!c.isArmed()||!c.mode().equals("navigation"))throw new AssertionError("new Gateway did not preserve healthy navigation: "+messages);
   if(delivered.get()-before<20)throw new AssertionError("production client did not accept fresh cache during repeated IO");
   java.lang.reflect.Method parse=RobotClient.class.getDeclaredMethod("parseStateSourceAt",JSONObject.class,long.class);parse.setAccessible(true);
   if(((Long)parse.invoke(null,new JSONObject().put("gateway_control_pending",true).put("gateway_state_age_ms",0),1000L))!=-1)throw new AssertionError("mutation barrier parser changed");
   if(((Long)parse.invoke(null,new JSONObject().put("gateway_control_pending",false).put("gateway_state_age_ms",301),1000L))!=-1)throw new AssertionError("source freshness bound changed");
   System.out.println("PASS_NEW actual RobotClient: BOOTSTRAP_WAIT healthy and armed for "+(now()-started)+"ms, accepted_states="+(delivered.get()-before)+", ARM/STOP pending and >300ms age still rejected");
   c.endNavigation();await(()->!c.isArmed(),500,"explicit navigation STOP");
  }
  Thread.sleep(200);
  } finally {c.close();Thread.sleep(100);}
 }
}
