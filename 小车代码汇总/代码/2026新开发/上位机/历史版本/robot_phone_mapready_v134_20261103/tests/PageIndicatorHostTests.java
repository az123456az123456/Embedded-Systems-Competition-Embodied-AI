import android.content.Context;
import android.os.SystemClock;
import cn.acg138.robotphone.DashboardView;
import cn.acg138.robotphone.PageIndicatorMotion;
import java.io.File;

public final class PageIndicatorHostTests {
    static int checks;
    static void check(boolean ok,String label){if(!ok)throw new AssertionError(label);checks++;}
    public static void main(String[] args)throws Exception {
        PageIndicatorMotion m=new PageIndicatorMotion();
        for(int page=0;page<2;page++) {
            PageIndicatorMotion.Shape idle=m.frame(page,false,0,0,true);
            check(idle.head==page&&idle.tail==page&&idle.radius==4.5f,"selected dot persists at idle");
            PageIndicatorMotion.Shape stretched=m.frame(page,true,page==0?-.31f:.31f,0,true);
            check(Math.abs(stretched.head-stretched.tail)>.49f,"one point stretches toward other point");
            check(stretched.tail==page,"tail stays anchored before contact");
            PageIndicatorMotion.Shape contact=m.frame(page,true,page==0?-.62f:.62f,0,true);
            check(contact.head==1-page&&contact.tail==page,"capsule touches destination before tail retracts");
            PageIndicatorMotion.Shape merging=m.frame(page,true,page==0?-.81f:.81f,0,true);
            check(merging.head==1-page&&Math.abs(merging.head-merging.tail)>.49&&Math.abs(merging.head-merging.tail)<.51,"after contact tail retracts toward destination");
            m.settle(page,1-page,.31f,1000,true);
            PageIndicatorMotion.Shape fading=m.frame(1-page,false,0,1320,true);
            check(fading.head==1-page&&fading.tail==1-page&&fading.ghostAlpha>0,"merged dot retains fading connected trail");
            PageIndicatorMotion.Shape finished=m.frame(1-page,false,0,1580,true);
            check(finished.head==1-page&&finished.tail==1-page&&finished.ghostAlpha==0&&!finished.animate,"trail fully disappears after settle and fade");
            m.settle(page,page,.2f,2000,true);
            PageIndicatorMotion.Shape cancelled=m.frame(page,false,0,2580,true);
            check(cancelled.head==page&&cancelled.tail==page,"cancelled swipe returns to original page");
        }
        m.settle(0,1,.3f,3000,true);
        check(m.frame(1,false,0,3010,false).head==1,"system animations disabled snaps without motion");
        m.reset();check(!m.frame(0,false,0,3010,true).animate,"tab switch clears prior page animation");
        android.animation.ValueAnimator.enabled=true;
        GuiHostTests.Actions a=new GuiHostTests.Actions();DashboardView v=new DashboardView(new Context(),a);
        v.connection(true);v.flags(true,"manual");v.status("界面演示 · 功能页滑动");
        String output=args[0];new File(output).mkdirs();
        SystemClock.setElapsedRealtimeForTest(10000);GuiHostTests.draw(v,1000,600,output+"/page-00.png");
        float right=GuiHostTests.f(v,"rightX"), width=GuiHostTests.f(v,"rightWidth");
        float start=right+width-30;
        GuiHostTests.touch(v,0,start,114);
        for(int i=1;i<=12;i++) {
            SystemClock.setElapsedRealtimeForTest(10000+i*20);
            GuiHostTests.touch(v,2,start-width*(i/12f)*.82f,114);
            GuiHostTests.draw(v,1000,600,output+String.format("/page-%02d.png",i));
        }
        check(a.arms==0&&a.rotates==0&&a.starts==0,"page stretch never activates a control");
        GuiHostTests.touch(v,1,start-width*.82f,114);
        for(int i=13;i<=42;i++) {
            SystemClock.setElapsedRealtimeForTest(10240+(i-12)*20);
            GuiHostTests.draw(v,1000,600,output+String.format("/page-%02d.png",i));
        }
        check(((Number)GuiHostTests.field(v,"controlPage")).intValue()==1,"page commits to requested second page");
        check(GuiHostTests.button(v,"follow")!=null&&GuiHostTests.button(v,"cw")==null,"visible actions match settled page");
        check(!((Boolean)GuiHostTests.field(v,"pageIndicatorAnimating")),"settled indicator stops animation redraws");
        android.animation.ValueAnimator.enabled=false;SystemClock.setElapsedRealtimeForTest(-1);
        System.out.println("PASS "+checks+" persistent liquid-page indicator checks; production drawing and synthetic clock");
    }
}
