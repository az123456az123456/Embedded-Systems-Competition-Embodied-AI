import android.content.Context;
import android.graphics.RectF;
import android.os.SystemClock;
import cn.acg138.robotphone.ButtonFeedback;
import cn.acg138.robotphone.DashboardView;
import java.io.File;

public final class ButtonFeedbackHostTests {
    static int checks;
    static void check(boolean result,String label) { if(!result)throw new AssertionError(label);checks++; }
    public static void main(String[] args) throws Exception {
        ButtonFeedback f = new ButtonFeedback(); f.down("arm",20,30,1000,true);
        check(f.scale(1000)==1,"no jump on DOWN");
        check(Math.abs(f.scale(1100)-.965f)<.00001,"3.5 percent press depth");
        check(f.scale(1100)==f.scale(1500),"hold remains compressed");
        f.up(1500);float first=f.scale(1500);
        check(Math.abs(first-.965f)<.00001,"release starts at current scale");
        for(int ms=1500;ms<=1740;ms+=10)
            check(f.scale(ms)>=.9649&&f.scale(ms)<=1.005,"bounded release spring");
        check(f.scale(1740)==1&&!f.active(1740),"release settles completely");
        check(f.opacity(1740)==0,"ripple disappears completely");
        f.down("stop",10,10,2000,true); f.up(2020);
        check(f.scale(2020)>.965&&f.scale(2020)<1,"quick tap releases continuously");
        f.down("arm",10,10,2021,true);
        check("arm".equals(f.key)&&f.scale(2021)==1,"new press replaces old animation without queuing actions");
        f.down("arm",10,10,3000,false);
        check(f.scale(3100)==1&&f.opacity(3100)==0&&!f.animating(3100),"system animations disabled is respected");
        android.animation.ValueAnimator.enabled=true;
        String output=args[0];new File(output).mkdirs();
        GuiHostTests.Actions a=new GuiHostTests.Actions();DashboardView view=new DashboardView(new Context(),a);
        view.connection(true); view.flags(true,"manual");
        SystemClock.setElapsedRealtimeForTest(10000);GuiHostTests.draw(view,1000,600,null);
        GuiHostTests.down(view,"arm");check(a.arms==0,"press feedback cannot enable on DOWN");
        RectF arm=GuiHostTests.button(view,"arm");
        for(int i=0;i<=22;i++) {
            SystemClock.setElapsedRealtimeForTest(10000+i*20);
            if(i==10)GuiHostTests.touch(view,1,arm.centerX(),arm.centerY());
            GuiHostTests.draw(view,1000,600,output+String.format("/press-%02d.png",i));
        }
        check(a.arms==1,"release dispatches existing ARM exactly once");
        GuiHostTests.down(view,"stop");check(a.stops==1,"STOP is immediate while feedback is running");
        GuiHostTests.touch(view,1,0,0);view.flags(true,"manual");
        GuiHostTests.draw(view,1000,600,null);GuiHostTests.down(view,"cw");
        check(a.rotates==0,"water ripple never shortcuts rotation hold");
        view.runPostedTasks(179);check(a.rotates==0,"hold threshold unchanged");
        view.runPostedTasks(1);check(a.rotates==1,"rotation starts on original 180ms threshold");
        GuiHostTests.touch(view,3,0,0);check(a.canceled==1,"ACTION_CANCEL remains immediate");
        android.animation.ValueAnimator.enabled=false;SystemClock.setElapsedRealtimeForTest(-1);
        System.out.println("PASS "+checks+" feedback checks; real View, synthetic clock, no device timing claim");
    }
}
