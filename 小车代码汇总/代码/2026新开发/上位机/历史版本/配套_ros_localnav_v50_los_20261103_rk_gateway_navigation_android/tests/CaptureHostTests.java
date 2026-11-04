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
    static void policy(){
        NoVestCapturePolicy p=new NoVestCapturePolicy();
        for(long time:new long[]{0,200,400,600,800,999,1000})check(sample(p,time,true)==null,"not before strictly more than one second at "+time);
        NoVestCapturePolicy.Ticket ticket=sample(p,1001,true);check(ticket!=null,"1001ms fresh positive triggers exactly one scene snapshot");p.complete(ticket,true,1001);
        for(long time=1200;time<3000;time+=200)check(sample(p,time,true)==null,"continuous presence does not repeat photograph");
        JSONObject stale=meta(false,3000).put("box_age_ms",351);check(p.observe(stale,3000)==null,"stale negative cannot end an appearance");
        for(long time=4000;time<5200;time+=200)check(sample(p,time,true)==null,"video interruption does not unlock saved appearance");
        // One missed classification does not count as disappearance.
        sample(p,5400,false);for(long time=5600;time<=6800;time+=200)check(sample(p,time,true)==null,"single miss does not duplicate capture");
        for(long time=7000;time<=8000;time+=200)check(sample(p,time,false)==null,"new negative results confirm absence");
        for(long time=8200;time<=9200;time+=200)check(sample(p,time,true)==null,"reappearance must independently wait one second");
        check(sample(p,9201,true)!=null,"new confirmed appearance triggers after new full second");
        p.reset();JSONObject duplicate=meta(true,0);for(int i=0;i<30;i++)check(p.observe(duplicate,i*100)==null,"repeated inference ID cannot accumulate duration");
        check(sample(p,3000,true)==null,"new result after repeated old metadata starts fresh timing");
        p.reset();for(long time=0;time<=800;time+=200)sample(p,time,true);
        sample(p,900,false);for(long time=1000;time<=2000;time+=200)check(sample(p,time,true)==null,"uncaptured run resets on real negative");
        ticket=sample(p,2001,true);check(ticket!=null,"new positive continuous run eligible");p.complete(ticket,false,2001);
        for(long time=2200;time<=5000;time+=200)check(sample(p,time,true)==null,"failed write retries with three second backoff");
        check(sample(p,5001,true)!=null,"storage failure does not permanently suppress event");
        p.reset();for(long time=0;time<=800;time+=200)sample(p,time,true);
        p.observe(meta(true,900).put("box_age_ms",400),900);for(long time=1000;time<=2000;time+=200)check(sample(p,time,true)==null,"stale sample breaks pending continuity");
        check(sample(p,2001,true)!=null,"continuity can restart after stale frame");
        p.reset();for(long time=0;time<=2000;time+=200)check(p.observe(meta(true,time).put("demo",true),time)==null,"demo detections never saved as evidence");
        p.reset();for(long time=0;time<=2000;time+=200)check(p.observe(meta(true,time).put("detections",new JSONArray(new JSONObject().put("label","NO_VEST"))),time)==null,"label match is exact no_vest");
        p.reset();JSONObject first=meta(true,0);p.observe(first,0);JSONObject older=meta(false,100).put("detections_frame_id",first.optLong("detections_frame_id")-1);
        check(p.observe(older,100)==null,"out of order result cannot end appearance");
        for(long time=200;time<=1000;time+=200)check(sample(p,time,true)==null,"out of order does not add elapsed evidence");check(sample(p,1001,true)!=null,"valid real sequence continues");
        p.reset();for(long time=0;time<=800;time+=200)sample(p,time,true);
        check(sample(p,1400,true)==null,"missing video frames cannot fill a temporal hole");
        for(long time=1600;time<=2400;time+=200)check(sample(p,time,true)==null,"gap requires independent new full second");check(sample(p,2401,true)!=null,"post-gap positive run triggers after one second");
        p.reset();for(long time=0;time<=2000;time+=200)check(p.observe(meta(true,time/10),time)==null,"paced cached source100ms span cannot fill host1000ms positive duration");
        p.reset();for(long time=0;time<=2000;time+=200)check(p.observe(meta(true,time*3),time)==null,"source gaps over350ms cannot fill real continuity");
        p.reset();for(long time=0;time<=2000;time+=100)check(p.observe(meta(true,time*2),time)==null,"host/source progress mismatch cannot accumulate a fake appearance");
        p.reset();for(long time=0;time<=1000;time+=200)sample(p,time,true);ticket=sample(p,1001,true);check(ticket!=null,"captured event ready for absence replay test");p.complete(ticket,true,1001);
        for(long time=1200;time<=3200;time+=200)p.observe(meta(false,1100+(time-1200)/10),time);
        for(long time=3400;time<=4800;time+=200)check(sample(p,time,true)==null,"paced old negative frames cannot unlock captured appearance");
        p.interrupted();for(long time=5000;time<=6400;time+=200)check(sample(p,time,true)==null,"transport reset preserves already saved appearance latch");

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
    public static void main(String[] args)throws Exception {policy();dates();dateTick();worker();System.out.println("PASS "+checks+" production no_vest policy/date/storage-worker checks; SAF provider/device verification separate");}
}
