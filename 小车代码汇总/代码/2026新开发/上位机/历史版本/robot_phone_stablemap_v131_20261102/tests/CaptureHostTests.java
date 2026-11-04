import cn.acg138.robotphone.CaptureArchive;
import cn.acg138.robotphone.NoVestCapturePolicy;
import org.json.JSONArray;
import org.json.JSONObject;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

/** Runs the production event policy/date naming/worker; no Android device claim. */
public final class CaptureHostTests {
    static int checks; static long id;
    static void check(boolean value,String message){if(!value)throw new AssertionError(message);checks++;}
    static JSONObject meta(boolean present,long time){long current=++id;return new JSONObject().put("detections_frame_id",current).put("detection_timestamp_ms",10000+time)
            .put("age_ms",0).put("box_age_ms",0).put("box_max_age_ms",350).put("detections",present?new JSONArray(new JSONObject().put("label","no_vest"),new JSONObject().put("label","no_vest")):new JSONArray());}
    static NoVestCapturePolicy.Ticket sample(NoVestCapturePolicy p,long time,boolean present){return p.observe(meta(present,time),time);}
    static JSONObject labels(long time,String... labels){
        Object[] items=new Object[labels.length];for(int i=0;i<labels.length;i++)items[i]=new JSONObject().put("label",labels[i]);JSONArray values=new JSONArray(items);
        return meta(false,time).put("detections",values);
    }
    static void policy(){
        String[] names={"no_vest","NO-Safety Vest","no_mask","NO-Mask","no_helmet","no_halmet","NO-Hardhat"};
        for(String name:names){
            NoVestCapturePolicy p=new NoVestCapturePolicy();
            for(long t:new long[]{0,200,400,500})check(p.observe(labels(t,name),t)==null,"strict >500ms boundary: "+name+" at "+t);
            NoVestCapturePolicy.Ticket ticket=p.observe(labels(501,name),501);
            check(ticket!=null,"501ms triggers: "+name);p.complete(ticket,true,501);
            for(long t=700;t<3000;t+=200)check(p.observe(labels(t,name),t)==null,"continuous same category saves once: "+name);
        }
        NoVestCapturePolicy p=new NoVestCapturePolicy();
        for(long t=0;t<2400;t+=200)check(p.observe(labels(t,(t/200)%2==0?"no_mask":"no_halmet"),t)==null,"different short categories cannot combine duration");
        p.reset();for(long t:new long[]{0,200,400,500})p.observe(labels(t,"no_vest"),t);
        NoVestCapturePolicy.Ticket ticket=p.observe(labels(501,"no_vest"),501);check(ticket!=null,"first category trigger");p.complete(ticket,true,501);
        for(long t:new long[]{600,800,1000,1100})check(p.observe(labels(t,"no_vest","NO-Mask"),t)==null,"new category starts its own500ms timer");
        ticket=p.observe(labels(1101,"no_vest","NO-Mask"),1101);check(ticket!=null,"mask triggers even while saved vest persists");p.complete(ticket,true,1101);
        p.interrupted();for(long t=2000;t<=2800;t+=200)check(p.observe(labels(t,"no_vest","NO-Mask"),t)==null,"reconnect does not clear captured category latches");
        for(long t=3000;t<=4000;t+=200)check(p.observe(labels(t,"NO-Mask"),t)==null,"absence unlocks vest only");
        for(long t:new long[]{4200,4400,4600,4700})check(p.observe(labels(t,"no_vest","NO-Mask"),t)==null,"vest reappearance waits500ms");
        ticket=p.observe(labels(4701,"no_vest","NO-Mask"),4701);check(ticket!=null,"confirmed reappearance can save again");p.complete(ticket,true,4701);
        p.reset();for(long t:new long[]{0,200,400,500})p.observe(labels(t,"NO-Mask","NO-Hardhat","NO-Safety Vest"),t);
        ticket=p.observe(labels(501,"NO-Mask","NO-Hardhat","NO-Safety Vest"),501);check(ticket!=null,"simultaneous categories share one frame ticket");p.complete(ticket,true,501);
        for(long t=700;t<=1500;t+=200)check(p.observe(labels(t,"NO-Mask","NO-Hardhat","NO-Safety Vest"),t)==null,"one completed photo marks all simultaneous categories");
        p.reset();JSONObject first=labels(0,"no_mask");for(int i=0;i<20;i++)check(p.observe(first,i*100)==null,"duplicate inference cannot accumulate time");
        p.reset();for(long t:new long[]{0,200,400})p.observe(labels(t,"no_mask"),t);
        check(p.observe(labels(500,"no_mask").put("box_age_ms",351),500)==null,"stale evidence breaks timing");
        for(long t:new long[]{600,800,1000,1100})check(p.observe(labels(t,"no_mask"),t)==null,"fresh continuity starts after stale sample");
        check(p.observe(labels(1101,"no_mask"),1101)!=null,"fresh independent interval qualifies");
        p.reset();for(long t=0;t<2000;t+=200)check(p.observe(labels(t/10,"no_halmet"),t)==null,"old source samples cannot fill current duration");
        p.reset();for(long t=0;t<2000;t+=200)check(p.observe(labels(t,"no_vest").put("demo",true),t)==null,"demo excluded");
        p.reset();for(long t=0;t<2000;t+=200)check(p.observe(labels(t,"Safety Vest","Mask","Hardhat","helmet","person"),t)==null,"positive/protective labels are not violations");
        p.reset();for(long t:new long[]{0,200,400,500})sample(p,t,true);ticket=sample(p,501,true);p.complete(ticket,false,501);
        for(long t=701;t<3501;t+=200)check(sample(p,t,true)==null,"failed storage retains3000ms retry backoff");
        check(sample(p,3501,true)!=null,"storage failure can retry same appearance");
        p.reset();for(long t:new long[]{0,200,400})sample(p,t,true);
        check(sample(p,1000,true)==null,"missing sample gap starts a new interval");
        for(long t:new long[]{1200,1400,1500})check(sample(p,t,true)==null,"gap does not count toward500ms");
        check(sample(p,1501,true)!=null,"post-gap new interval qualifies");
        p.reset();first=labels(0,"no_mask");p.observe(first,0);JSONObject older=labels(100,"no_vest").put("detections_frame_id",first.optLong("detections_frame_id")-1);p.observe(older,100);
        for(long t:new long[]{200,400,500})check(p.observe(labels(t,"no_mask"),t)==null,"out-of-order sample cannot reset or advance evidence");
        check(p.observe(labels(501,"no_mask"),501)!=null,"valid ordered mask sequence can qualify");
    }
    static void dates(){
        long midnight=java.time.Instant.parse("2026-10-26T16:00:00Z").toEpochMilli();
        CaptureArchive.Stamp a=CaptureArchive.Stamp.at(midnight-1,TimeZone.getTimeZone("Asia/Shanghai"),42);
        CaptureArchive.Stamp b=CaptureArchive.Stamp.at(midnight,TimeZone.getTimeZone("Asia/Shanghai"),43);
        check(a.day.equals("2026-10-26")&&a.filename.equals("23-59-59-999_frame42.jpg"),"before phone midnight naming");
        check(b.day.equals("2026-10-27")&&b.filename.equals("00-00-00-000_frame43.jpg"),"new phone date creates new day path");
        CaptureArchive.Stamp c=CaptureArchive.Stamp.at(midnight,TimeZone.getTimeZone("America/New_York"),44);
        check(c.day.equals("2026-10-26")&&c.filename.equals("12-00-00-000_frame44.jpg"),"phone timezone is re-evaluated each photograph");
        Locale old=Locale.getDefault();try{Locale.setDefault(new Locale("ar"));CaptureArchive.Stamp d=CaptureArchive.Stamp.at(midnight,TimeZone.getTimeZone("UTC"),45);check(d.filename.equals("16-00-00-000_frame45.jpg"),"portable ASCII path names independent of phone language");}finally{Locale.setDefault(old);}
    }
    static class Clock implements CaptureArchive.Clock {
        final AtomicLong time=new AtomicLong();final AtomicLong wall=new AtomicLong(java.time.Instant.parse("2026-10-26T15:59:59.999Z").toEpochMilli());
        volatile TimeZone zone=TimeZone.getTimeZone("Asia/Shanghai");
        public long wallMillis(){return wall.get();}public long elapsedMillis(){return time.get();}public TimeZone timeZone(){return zone;}
    }
    static final byte[] JPEG={(byte)255,(byte)216,12,45,(byte)255,(byte)217};
    static long appearance(CaptureArchive archive,Clock clock,long begin){for(int i=0;i<=6;i++){long time=begin+i*200;clock.time.set(time);archive.frame(meta(true,time),JPEG,id,time);}return begin+1200;}
    static long absence(CaptureArchive archive,Clock clock,long begin){for(int i=0;i<=5;i++){long time=begin+i*200;clock.time.set(time);archive.frame(meta(false,time),JPEG,id,time);}return begin+1000;}
    static void waitDayKnown(CaptureArchive archive,String day)throws Exception {
        java.lang.reflect.Field f=CaptureArchive.class.getDeclaredField("ensuredDays");f.setAccessible(true);
        long until=System.nanoTime()+TimeUnit.SECONDS.toNanos(2);
        while(System.nanoTime()<until){synchronized(archive){if(((Set<?>)f.get(archive)).contains(day))return;}Thread.sleep(2);}
        throw new AssertionError("directory task did not complete: "+day);
    }
    static void dateTick()throws Exception {
        Clock clock=new Clock();BlockingQueue<String> created=new LinkedBlockingQueue<>(),messages=new LinkedBlockingQueue<>();AtomicInteger creates=new AtomicInteger(),photos=new AtomicInteger();
        CaptureArchive archive=new CaptureArchive(clock,new CaptureArchive.Listener(){public void saved(String path){messages.add("saved");}public void failed(String error){messages.add(error);}});
        CaptureArchive.Store store=new CaptureArchive.Store(){public String write(CaptureArchive.Stamp stamp,byte[] jpeg){photos.incrementAndGet();return "unexpected";}
            public void ensureDay(String day){creates.incrementAndGet();created.add(day);}};
        archive.configure(store,false);for(int i=0;i<100;i++)archive.ensureCurrentDay();check(created.poll(40,TimeUnit.MILLISECONDS)==null,"disabled date ticks cannot create folders");
        archive.configure(null,true);archive.ensureCurrentDay();check(created.poll(40,TimeUnit.MILLISECONDS)==null,"unauthorized/no destination cannot create folders");
        archive.configure(store,true);for(int i=0;i<100;i++)archive.ensureCurrentDay();check("2026-10-26".equals(created.poll(2,TimeUnit.SECONDS)),"first enabled foreground tick creates today's directory without detections");waitDayKnown(archive,"2026-10-26");
        for(int i=0;i<1000;i++)archive.ensureCurrentDay();check(creates.get()==1,"repeated foreground tick never re-queries a completed day");
        archive.configure(store,false);archive.ensureCurrentDay();archive.configure(store,true);archive.ensureCurrentDay();check(created.poll(40,TimeUnit.MILLISECONDS)==null&&creates.get()==1,"same-directory off/on retains known day without repeated root query");
        clock.wall.incrementAndGet();archive.ensureCurrentDay();check("2026-10-27".equals(created.poll(2,TimeUnit.SECONDS)),"midnight tick creates new directory even without no_vest");waitDayKnown(archive,"2026-10-27");
        // A phone timezone change can produce a new local date even at a fixed
        // wall-clock instant; it must not wait for another detection or app restart.
        clock.wall.addAndGet(86400000L);clock.zone=TimeZone.getTimeZone("UTC");archive.ensureCurrentDay();
        check(created.poll(40,TimeUnit.MILLISECONDS)==null,"returning to an already ensured date does not query root again");
        clock.zone=TimeZone.getTimeZone("Asia/Shanghai");archive.ensureCurrentDay();
        check("2026-10-28".equals(created.poll(2,TimeUnit.SECONDS)),"in-session timezone change creates newly current local date");waitDayKnown(archive,"2026-10-28");
        check(photos.get()==0,"date housekeeping creates no placeholder images");archive.close();
        AtomicInteger attempts=new AtomicInteger();CaptureArchive failing=new CaptureArchive(clock,new CaptureArchive.Listener(){public void saved(String p){}public void failed(String e){messages.add(e);}});
        failing.configure(new CaptureArchive.Store(){public String write(CaptureArchive.Stamp s,byte[] b){throw new AssertionError("no photo");}public void ensureDay(String d)throws Exception{attempts.incrementAndGet();throw new java.io.IOException("directory denied");}},true);
        failing.ensureCurrentDay();String failure=messages.poll(2,TimeUnit.SECONDS);check(failure!=null&&failure.contains("directory denied"),"directory failure explicitly notified independently of images");
        clock.time.set(2999);for(int i=0;i<100;i++)failing.ensureCurrentDay();check(attempts.get()==1,"failed date creation backs off instead of querying every tick");
        clock.time.set(3000);failing.ensureCurrentDay();check(messages.poll(2,TimeUnit.SECONDS)!=null&&attempts.get()==2,"date task retries after three seconds");failing.close();
    }
    static void worker()throws Exception {
        Clock clock=new Clock();BlockingQueue<String> messages=new LinkedBlockingQueue<>();AtomicInteger writes=new AtomicInteger();CountDownLatch started=new CountDownLatch(1),proceed=new CountDownLatch(1);
        CaptureArchive archive=new CaptureArchive(clock,new CaptureArchive.Listener(){public void saved(String path){messages.add("saved:"+path);}public void failed(String error){messages.add("failed:"+error);}});
        archive.configure((stamp,jpeg)->{writes.incrementAndGet();check(jpeg==JPEG,"worker receives exact read-only JPEG bytes from the matched frame");started.countDown();if(!proceed.await(3,TimeUnit.SECONDS))throw new Exception("blocked");return stamp.day+"/"+stamp.filename;},true);
        long wall=clock.wall.get();appearance(archive,clock,0);check(started.await(2,TimeUnit.SECONDS),"disk write dispatched off frame caller");
        clock.wall.set(wall+100000);proceed.countDown();String saved=messages.poll(2,TimeUnit.SECONDS);
        check(saved!=null&&saved.startsWith("saved:2026-10-26/23-59-59-999_frame"),"slow writer crossing midnight keeps trigger's date and time");
        for(long t=1400;t<2800;t+=200){clock.time.set(t);archive.frame(meta(true,t),JPEG,id,t);}Thread.sleep(20);check(writes.get()==1,"only one physical write for continuous appearance");
        archive.newSession();appearance(archive,clock,3000);check(messages.poll(50,TimeUnit.MILLISECONDS)==null,"disconnect/reconnect is not target disappearance and cannot duplicate saved capture");
        absence(archive,clock,4400);appearance(archive,clock,5600);check(messages.poll(2,TimeUnit.SECONDS)!=null,"fresh confirmed absence still unlocks a new appearance after reconnect");
        archive.configure(null,true);appearance(archive,clock,7000);check(messages.poll(50,TimeUnit.MILLISECONDS)==null,"cannot save without explicit selected destination");
        AtomicInteger failures=new AtomicInteger();archive.configure((stamp,jpeg)->{if(failures.getAndIncrement()==0)throw new java.io.IOException("full");return stamp.day+"/"+stamp.filename;},true);
        long end=appearance(archive,clock,9000);String failed=messages.poll(2,TimeUnit.SECONDS);check("failed:full".equals(failed),"I/O failure explicitly notified without pretending success");
        for(long t=end+200;t<=end+3200;t+=200){clock.time.set(t);archive.frame(meta(true,t),JPEG,id,t);}
        check(messages.poll(2,TimeUnit.SECONDS).startsWith("saved:"),"same appearance retries after failure without UI/control blocking");
        archive.close();appearance(archive,clock,20000);check(messages.poll(50,TimeUnit.MILLISECONDS)==null,"closed archive cannot write");
        // Three jobs maximum: one writer and two queued. More appearances cannot
        // retain an unbounded amount of MJPEG data or block control's caller.
        CountDownLatch block=new CountDownLatch(1),release=new CountDownLatch(1);BlockingQueue<String> boundedMessages=new LinkedBlockingQueue<>();Clock qclock=new Clock();
        CaptureArchive bounded=new CaptureArchive(qclock,new CaptureArchive.Listener(){public void saved(String path){boundedMessages.add("saved");}public void failed(String error){boundedMessages.add(error);}});
        bounded.configure((stamp,jpeg)->{block.countDown();release.await();return stamp.day+"/"+stamp.filename;},true);
        appearance(bounded,qclock,0);check(block.await(2,TimeUnit.SECONDS),"bounded writer running");
        for(int i=0;i<3;i++){long start=2000+i*3000;absence(bounded,qclock,start);appearance(bounded,qclock,start+1200);}
        check(bounded.queued()==2,"queue limited to two pending photographs");String full=boundedMessages.poll(1,TimeUnit.SECONDS);check(full!=null&&full.contains("队列已满"),"full queue emits clear failure and never blocks frames");release.countDown();bounded.close();
    }
    public static void main(String[] args)throws Exception {policy();dates();dateTick();worker();System.out.println("PASS "+checks+" production violation policy/date/storage-worker checks; SAF provider/device verification separate");}
}
