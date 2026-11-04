import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.RectF;
import android.view.MotionEvent;
import cn.acg138.robotphone.DashboardView;
import cn.acg138.robotphone.RobotClient;
import cn.acg138.robotphone.SelectionGate;
import cn.acg138.robotphone.NavigationMap;
import cn.acg138.robotphone.NavigationTarget;
import org.json.JSONArray;
import org.json.JSONObject;
import java.awt.image.BufferedImage;
import java.io.File;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import javax.imageio.ImageIO;

/** Executes the production View with Java2D adapters. No Android device is simulated. */
public final class GuiHostTests {
    private static int checks; private static Canvas lastCanvas; private static BufferedImage lastImage;
    static void check(boolean ok,String label){if(!ok)throw new AssertionError(label);checks++;}
    static class Actions implements DashboardView.Actions {
        int navigations,starts,ends,mappings;NavigationTarget target;
        public void startNavigation(){starts++;}public void startMapping(){mappings++;}public void endNavigation(){ends++;}public void endMapping(){ends++;}
        public void navigate(NavigationTarget t){navigations++;target=t;}
        int rotates,rotationReleases,canceled,stops,selections,drives,releases,arms,follows,zeros,calibrations,modes;
        boolean clockwise,hold;float throttle,steer;long selectedTarget,followedTarget;
        public void settings(){} public void connect(){} public void arm(){arms++;} public void stop(){stops++;selectedTarget=0;}
        public void follow(){follows++;followedTarget=selectedTarget;} public void mode(boolean h){modes++;hold=h;selectedTarget=0;}
        public void zero(){zeros++;selectedTarget=0;} public void calibrate(){calibrations++;selectedTarget=0;}
        public void stick(float t,float s,boolean h){drives++;throttle=t;steer=s;hold=h;}
        public void release(boolean cancel){releases++;selectedTarget=0;if(cancel)canceled++;}
        public void rotate(boolean cw){rotates++;clockwise=cw;selectedTarget=0;}
        public void releaseRotation(boolean cancel){rotationReleases++;selectedTarget=0;if(cancel)canceled++;}
        public void select(long frame,long target){selections++;selectedTarget=target;}
    }
    static Object field(Object object,String name)throws Exception{Field f=object.getClass().getDeclaredField(name);f.setAccessible(true);return f.get(object);}
    static float f(DashboardView view,String name)throws Exception{return ((Number)field(view,name)).floatValue();}
    static RectF r(DashboardView view,String name)throws Exception{return (RectF)field(view,name);}
    static RectF button(DashboardView view,String key)throws Exception{
        for(Object b:(List<?>)field(view,"buttons"))if(key.equals(field(b,"key")))return (RectF)field(b,"rect");return null;
    }
    static void touch(DashboardView view,int action,float x,float y)throws Exception{float s=f(view,"scale");view.onTouchEvent(new MotionEvent(action,x*s,y*s));}
    static void tap(DashboardView view,String key)throws Exception{RectF b=button(view,key);if(b==null)throw new AssertionError("missing button "+key);float x=b.centerX(),y=b.centerY();touch(view,0,x,y);touch(view,1,x,y);}
    static void down(DashboardView view,String key)throws Exception{RectF b=button(view,key);if(b==null)throw new AssertionError("missing button "+key);touch(view,0,b.centerX(),b.centerY());}
    static void hold(DashboardView view,String key)throws Exception{down(view,key);view.runPostedTasks(180);}
    static void swipe(DashboardView view,boolean next)throws Exception{
        RectF panel=r(view,"controlRect");float x=next?panel.right-30:panel.left+30,xx=next?panel.left+30:panel.right-30,y=114;
        touch(view,0,x,y);touch(view,2,xx,y);touch(view,1,xx,y);
    }
    static int dots(Canvas c){int count=0;for(float[] circle:c.drawnCircles)if(circle[1]==260&&Math.abs(circle[2]-3.2f)<.01)count++;return count;}
    static void draw(DashboardView view,int w,int h,String name)throws Exception{
        view.layout(w,h);lastImage=new BufferedImage(w,h,BufferedImage.TYPE_INT_RGB);lastCanvas=new Canvas(lastImage);view.draw(lastCanvas);
        if(name!=null)ImageIO.write(lastImage,"png",new File(name));
    }
    static JSONObject detection(long id,int left,int top,int right,int bottom){return new JSONObject().put("id",id).put("label","目标").put("score",.9).put("box",new JSONArray(left,top,right,bottom));}
    static JSONObject meta(JSONObject... boxes){return new JSONObject().put("age_ms",0).put("box_age_ms",0).put("box_max_age_ms",350).put("detections",new JSONArray((Object[])boxes));}
    static JSONObject imu(boolean fresh){return new JSONObject().put("valid",true).put("fresh",fresh).put("initialized",true).put("roll",1.2).put("pitch",-2.3).put("yaw",45.6).put("accel",new JSONArray(.1,-.2,9.8));}
    static JSONObject state(boolean fresh){return new JSONObject().put("imu",imu(fresh)).put("vision",new JSONObject().put("capture_fps",30).put("result_fps",24).put("model","yolo11s-int8-rk3588.rknn")).put("motor",new JSONObject().put("link_ok",true).put("left",0).put("right",0).put("ack_left",0).put("ack_right",0));}
    static BufferedImage source(int w,int h){
        BufferedImage image=new BufferedImage(w,h,BufferedImage.TYPE_INT_RGB);java.awt.Graphics2D g=image.createGraphics();
        g.setColor(new java.awt.Color(206,224,241));g.fillRect(0,0,w,h);g.setColor(new java.awt.Color(186,207,225));
        for(int x=0;x<w;x+=120)g.drawLine(x,0,x,h);for(int y=0;y<h;y+=120)g.drawLine(0,y,w,y);
        int d=Math.min(w,h)/7;int[] colors={0xffe98e8e,0xff8bd4ae,0xff8cabdc,0xffe9cc82};
        for(int i=0;i<4;i++){g.setColor(new java.awt.Color(colors[i]));g.fillRect(i%2==0?0:w-d,i<2?0:h-d,d,d);}g.dispose();return image;
    }
    static void frame(DashboardView view,BufferedImage source,JSONObject meta){view.frame(new RobotClient.Frame(new Bitmap(source),meta,81));}
    static void buttonSet(DashboardView view,String... wanted)throws Exception{
        List<String> keys=new ArrayList<>();for(Object b:(List<?>)field(view,"buttons")){
            String key=(String)field(b,"key");if(!key.equals("settings")&&!key.equals("connect")&&!key.equals("control_page")&&!key.equals("stop")&&!key.equals("perception")&&!key.equals("navigation"))keys.add(key);
        }
        check(keys.size()==4&&keys.containsAll(java.util.Arrays.asList(wanted)),"exactly four actions on each function page");
    }
    static void motionAndPaging(String output)throws Exception{
        Actions a=new Actions();DashboardView view=new DashboardView(new Context(),a);draw(view,1000,600,output+"/perception-host.png");
        buttonSet(view,"arm","nav_start","ccw","cw");check(button(view,"control_page")==null,"page arrow button removed");check(dots(lastCanvas)==2,"idle page always shows both base dots");
        down(view,"arm");check(a.arms==0,"ARM deferred until confirmed tap UP");RectF arm=button(view,"arm");touch(view,1,arm.centerX(),arm.centerY());check(a.arms==1,"ARM tap UP dispatches once");
        hold(view,"cw");check(a.rotates==0,"rotation refused before arm");touch(view,1,0,0);view.flags(true,"manual");
        down(view,"cw");check(a.rotates==0,"rotation waits for stationary hold intent");view.runPostedTasks(179);check(a.rotates==0,"rotation cannot begin before hold threshold");view.runPostedTasks(1);check(a.rotates==1&&a.clockwise,"CW stationary hold begins rotation");touch(view,1,0,0);check(a.rotationReleases==1,"rotation UP sends zero");
        hold(view,"ccw");check(a.rotates==2&&!a.clockwise,"CCW direction retained");RectF ccw=button(view,"ccw");touch(view,2,ccw.centerX(),65);check(a.rotationReleases==2,"sliding off rotation releases it");touch(view,1,ccw.centerX(),65);
        hold(view,"cw");touch(view,5,0,0);check(a.canceled==0,"second finger preserves rotation owner and enable");touch(view,1,0,0);view.flags(true,"manual");
        down(view,"cw");touch(view,3,0,0);int before=a.rotates;view.runPostedTasks(1000);check(a.canceled==1&&a.rotates==before,"CANCEL removes pending rotation hold");touch(view,1,0,0);
        float jx=f(view,"joystickX"),jy=f(view,"joystickY");view.flags(false,"manual");touch(view,0,jx,jy-42);check(a.drives==0,"unarmed joystick cannot drive");touch(view,1,jx,jy-42);
        view.flags(true,"manual");touch(view,0,jx,jy-42);check(a.throttle>0&&a.steer==0,"forward keeps original stick mapping");int releases=a.releases;touch(view,1,jx,jy-42);check(a.releases==releases+1,"stick release sends immediate zero");
        touch(view,0,jx,jy+49);check(a.throttle<0&&a.steer==0,"backwards mapping retained");touch(view,1,jx,jy+49);
        touch(view,0,jx-46,jy);check(a.throttle==0&&a.steer<0,"left steer retained");touch(view,2,jx+200,jy);check(a.steer==1&&a.throttle==0,"outside drag clamps to original full scale");touch(view,1,jx+200,jy);
        touch(view,0,jx+46,jy);check(a.steer>0,"right steer retained");touch(view,1,jx+46,jy);
        touch(view,0,jx,jy-42);touch(view,5,0,0);check(a.canceled==1,"multitouch preserves joystick and enable");touch(view,1,0,0);view.flags(true,"manual");
        RectF cw=button(view,"cw");float oldX=cw.centerX(),oldY=cw.centerY();hold(view,"cw");int rotationReleases=a.rotationReleases;
        touch(view,2,cw.centerX()-f(view,"rightWidth")*.7f,cw.centerY());check(a.rotationReleases==rotationReleases+1,"horizontal swipe immediately releases active rotation");touch(view,1,cw.centerX()-f(view,"rightWidth")*.7f,cw.centerY());
        before=a.rotates;touch(view,0,oldX,oldY);touch(view,1,oldX,oldY);check(a.rotates==before,"old page buttons cannot fire before redraw");
        draw(view,1000,600,null);buttonSet(view,"mapping","follow","zero","calibrate");check(button(view,"stop")!=null,"STOP always visible on second page");check(button(view,"mapping")!=null,"exploration mapping replaces separate end-navigation auxiliary action");
        check(button(view,"hold")==null&&button(view,"manual")==null,"heading hold and manual mode buttons removed");view.state(state(true));
        tap(view,"zero");tap(view,"calibrate");check(a.zeros==1&&a.calibrations==1,"second page keeps zero and calibration actions");
        swipe(view,false);draw(view,1000,600,null);view.flags(true,"manual");touch(view,0,jx,jy-42);releases=a.releases;swipe(view,true);check(a.releases==releases+1,"function swipe stops an active stick gesture");
        draw(view,1000,600,null);swipe(view,false);draw(view,1000,600,null);
        BufferedImage source=source(1920,1080);JSONObject metadata=meta(detection(11,600,300,1000,700));view.state(state(true));view.flags(true,"follow");frame(view,source,metadata);draw(view,1000,600,null);
        RectF image=r(view,"imageRect");float px=image.left+800f/1920*image.width(),py=image.top+500f/1080*image.height();touch(view,0,px,py);touch(view,1,px,py);
        check(a.selectedTarget==11,"first page selects full-frame coordinates");releases=a.releases;rotationReleases=a.rotationReleases;int stops=a.stops,modes=a.modes;swipe(view,true);
        check(a.releases==releases&&a.rotationReleases==rotationReleases&&a.stops==stops&&a.modes==modes,"stationary swipe preserves selection without idle DRIVE or mode changes");
        draw(view,1000,600,null);tap(view,"follow");check(a.followedTarget==11,"page-one selection reaches page-two Follow unchanged");
        releases=a.releases;swipe(view,false);check(a.releases==releases+1,"Follow then immediate swipe before flags tick stops motion");draw(view,1000,600,null);
        stops=a.stops;tap(view,"navigation");check(a.stops==stops,"navigation tab switch pauses without disarm");draw(view,1000,600,output+"/navigation-host.png");
        stops=a.stops;tap(view,"perception");check(a.stops==stops,"returning perception tab pauses without disarm");draw(view,1000,600,null);view.flags(true,"manual");down(view,"stop");check(a.stops==stops+1,"fixed STOP dispatches immediately on DOWN");touch(view,1,0,0);
        view.flags(true,"manual");metadata.put("box_age_ms",351);frame(view,source,metadata);draw(view,1000,600,null);int selections=a.selections;touch(view,0,px,py);touch(view,1,px,py);check(a.selections==selections,"stale detection cannot be selected");
    }
    static void swipeIntentAndDots(String output)throws Exception{
        Actions a=new Actions();DashboardView view=new DashboardView(new Context(),a);view.flags(true,"manual");draw(view,1000,600,null);
        RectF arm=button(view,"arm");touch(view,0,arm.centerX(),arm.centerY());touch(view,2,arm.centerX()-100,arm.centerY());view.runPostedTasks(1000);draw(view,1000,600,output+"/swipe-indicators-host.png");
        check(dots(lastCanvas)==2,"two gray page dots visible during horizontal swipe");
        float dcx=f(view,"rightX")+f(view,"rightWidth")/2;check(lastImage.getRGB((int)(dcx-10),260)==0xff7d7d86&&lastImage.getRGB((int)(dcx+10),260)==0xffd1d1d6,"page indicators use gray active/inactive fills");check(a.arms==0,"swipe beginning on ARM never enables motors");touch(view,1,arm.centerX()-100,arm.centerY());draw(view,1000,600,null);check(dots(lastCanvas)==2,"base dots remain visible on UP");buttonSet(view,"mapping","follow","zero","calibrate");
        RectF zero=button(view,"zero");touch(view,0,zero.centerX(),zero.centerY());touch(view,2,zero.centerX()+120,zero.centerY());touch(view,1,zero.centerX()+120,zero.centerY());draw(view,1000,600,null);check(a.zeros==0&&a.calibrations==0,"swipe cannot invoke calibration or zero");buttonSet(view,"arm","nav_start","ccw","cw");
        RectF cw=button(view,"cw");touch(view,0,cw.centerX(),cw.centerY());touch(view,2,cw.centerX()-120,cw.centerY());view.runPostedTasks(500);check(a.rotates==0,"swipe beginning on rotation never starts it");draw(view,1000,600,null);check(dots(lastCanvas)==2,"swipe dots also visible from motion button");touch(view,3,0,0);view.runPostedTasks(500);draw(view,1000,600,null);check(dots(lastCanvas)==2&&a.rotates==0,"CANCEL retains dots and cancels delayed rotation");touch(view,1,0,0);
        view.flags(true,"manual");draw(view,1000,600,null);RectF panel=r(view,"controlRect");touch(view,0,panel.right-40,114);touch(view,2,panel.right-65,114);draw(view,1000,600,null);check(dots(lastCanvas)==2,"short horizontal drag is indicated");touch(view,1,panel.right-65,114);draw(view,1000,600,null);buttonSet(view,"arm","nav_start","ccw","cw");check(dots(lastCanvas)==2,"short swipe settles without flipping and hides dots");
        int armPixel=lastImage.getRGB((int)(button(view,"arm").left+4),(int)(button(view,"arm").top+24));check(armPixel==0xff111111,"function button fill is black");
        RectF stop=button(view,"stop");check(lastImage.getRGB((int)stop.centerX(),(int)(stop.top+4))==0xffdf3544,"STOP uses brighter red fill");
        check(lastCanvas.drawnTexts.stream().noneMatch(t->t.contains("下一页")||t.contains("上一页")),"page arrow labels removed");
    }
    static int buttonFill(DashboardView view,String key)throws Exception {
        RectF b=button(view,key);float scale=f(view,"scale");
        return lastImage.getRGB((int)((b.left+12)*scale),(int)((b.top+b.height()/2)*scale));
    }
    static int buttonTextColor(DashboardView view,String key)throws Exception {
        RectF b=button(view,key);
        for(Canvas.TextDraw t:lastCanvas.textDraws)if(b.contains(t.x,t.y))return t.color;
        throw new AssertionError("button text missing "+key);
    }
    static void selectedButtonsAndHeader(String output)throws Exception {
        Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);draw(v,1000,600,null);
        for(String key:new String[]{"arm","cw","ccw","settings","connect","navigation"}) {
            check(buttonFill(v,key)==((key.equals("settings")||key.equals("connect"))?0xffdcdcdc:0xfff2f2f2),"inactive "+key+" uses released v12.6 gray");
            check(buttonTextColor(v,key)==0xff141414,"inactive "+key+" uses black text");
        }
        for(String key:new String[]{"perception"}) {
            check(buttonFill(v,key)==0xff111111,"selected "+key+" uses black fill");
            check(buttonTextColor(v,key)==0xffffffff,"selected "+key+" uses white text");
        }
        check(lastCanvas.drawnTexts.stream().noneMatch(t->t.contains("ACG138")||t.contains("小车控制台")),"top-left brand and title removed");
        float rx=f(v,"rightX");
        check(lastCanvas.textDraws.stream().noneMatch(t->t.x>=rx&&t.y>=354),"joystick card contains no status, mode or duty text");
        v.status("已连接");draw(v,1000,600,null);RectF status=r(v,"statusRect");float shortWidth=status.width();
        check(Math.abs(status.centerX()-r(v,"contentRect").centerX())<.01,"short status is centered over left display");
        check(lastImage.getRGB((int)(status.left+4),(int)status.centerY())==0xff707070,"status uses medium-gray pill fill");
        Canvas.TextDraw statusText=lastCanvas.textDraws.get(0);
        check(statusText.size==18&&statusText.color==0xffffffff,"status pill uses larger18px white text");
        check(lastCanvas.textDraws.stream().anyMatch(t->t.text.equals("设置")&&t.size==statusText.size),"status and Settings use matching larger fonts");
        v.status("已连接，小车准备就绪");draw(v,1000,600,null);check(r(v,"statusRect").width()>shortWidth,"pill width follows actual text length");
        String longStatus="很长的连接状态，";for(int i=0;i<7;i++)longStatus+=longStatus;
        v.status(longStatus);draw(v,1000,600,null);
        check(r(v,"statusRect").right<button(v,"settings").left-10,"long status does not overlap Settings");
        check(Math.abs(r(v,"statusRect").centerX()-r(v,"contentRect").centerX())<.01,"long status remains centered over left display");
        check(r(v,"statusRect").left>=r(v,"contentRect").left&&r(v,"statusRect").right<=r(v,"contentRect").right,"long status stays inside left display width");
        check(lastCanvas.drawnTexts.get(0).endsWith("…"),"long status truncates at available header width");
        v.connection(true);v.flags(true,"manual");v.status("已连接 · 手动差速");draw(v,1000,600,null);
        check(buttonFill(v,"arm")==0xff111111&&buttonFill(v,"connect")==0xff111111,"armed/connected active controls highlighted");
        down(v,"settings");draw(v,1000,600,null);check(buttonFill(v,"settings")==0xff111111,"Settings uses black while pressed");
        RectF settings=button(v,"settings");touch(v,1,settings.centerX(),settings.centerY());draw(v,1000,600,null);
        check(buttonFill(v,"settings")==0xffdcdcdc,"Settings returns to darker v12.6 gray on release");
        down(v,"cw");draw(v,1000,600,null);check(buttonFill(v,"cw")==0xfff2f2f2,"rotation stays unselected until actual hold intent");
        v.runPostedTasks(180);draw(v,1000,600,null);
        check(buttonFill(v,"cw")==0xff111111&&buttonFill(v,"ccw")==0xfff2f2f2&&buttonFill(v,"nav_start")==0xfff2f2f2,"only actual held rotation is selected");
        RectF cw=button(v,"cw");touch(v,1,cw.centerX(),cw.centerY());draw(v,1000,600,null);
        v.state(state(true));v.flags(true,"navigation");draw(v,1000,600,null);
        check(buttonFill(v,"nav_start")==0xff111111,"navigation mode highlights start button even without selected destination");
        swipe(v,true);draw(v,1000,600,null);
        check(buttonFill(v,"mapping")==0xfff2f2f2&&buttonFill(v,"follow")==0xfff2f2f2,"exploration mapping stays inactive before actual server confirmation");
        v.flags(true,"follow");draw(v,1000,600,null);
        check(buttonFill(v,"follow")==0xff111111&&buttonFill(v,"mapping")==0xfff2f2f2,"follow mode changes selected button");
        down(v,"zero");draw(v,1000,600,null);check(buttonFill(v,"zero")==0xff111111,"momentary zero press selected before UP");
        RectF zero=button(v,"zero");touch(v,1,zero.centerX(),zero.centerY());draw(v,1000,600,null);
        check(buttonFill(v,"zero")==0xfff2f2f2,"zero button clears momentary selection after release");
        RectF panel=r(v,"controlRect");touch(v,0,panel.left+30,114);touch(v,2,panel.left+130,114);draw(v,1000,600,null);
        for(float[] circle:lastCanvas.drawnCircles)if(Math.abs(circle[2]-3.2f)<.01) {
            check(panel.contains(circle[0]-3,circle[1]-3)&&panel.contains(circle[0]+3,circle[1]+3),"page dot lies fully inside action card bottom");
            check(circle[1]>246,"page dot does not overlap lowest button row");
        }
        touch(v,3,0,0);touch(v,1,0,0);v.flags(true,"manual");v.status("已连接 · 完整摄像头画面");
        draw(v,1000,600,output+"/selection-states-host.png");
    }
    static void layoutAndFullFrame(String output)throws Exception{
        for(int[] size:new int[][]{{1000,600},{2340,1080},{1280,800},{960,540}})for(int[] sensor:new int[][]{{1920,1080},{1280,960}}){
            int w=size[0],h=size[1],sw=sensor[0],sh=sensor[1];Actions a=new Actions();DashboardView view=new DashboardView(new Context(),a);view.state(state(true));view.flags(true,"manual");view.connection(true);view.status("完整摄像头画面 · 右侧功能分页 · 松手即停");BufferedImage source=source(sw,sh);JSONObject metadata=meta();frame(view,source,metadata);
            draw(view,w,h,null);float ui=f(view,"scale"),height=f(view,"height"),jx=f(view,"joystickX"),jy=f(view,"joystickY"),rx=f(view,"rightX");RectF image=r(view,"imageRect"),container=r(view,"videoRect"),content=r(view,"contentRect");
            RectF nav=r(view,"navigationRect"),status=r(view,"statusRect");
            check(Math.abs(image.top-nav.top)<.01&&Math.abs(image.bottom-nav.bottom)<.01,"camera fills navigation height at "+w+"x"+h+" sensor "+sw+"x"+sh);
            check(Math.abs(status.centerX()-image.centerX())<.01&&status.bottom<image.top,"status centered above camera without obscuring it at "+w+"x"+h);
            check(Math.abs(image.width()/sw-image.height()/sh)<.001,"one camera scale for X/Y; no stretching");
            check(Math.abs(image.centerX()-container.centerX())<.01&&Math.abs(image.top-container.top)<.01&&Math.abs(image.bottom-container.bottom)<.01,"height-fit image stays centered in clipped viewport");
            int[] colors={0xffe98e8e,0xff8bd4ae,0xff8cabdc,0xffe9cc82};for(int corner=0;corner<4;corner++){
                float x=corner%2==0?image.left+3:image.right-3,y=corner<2?image.top+3:image.bottom-3;
                if(container.contains(x,y))check(lastImage.getRGB((int)(x*ui),(int)(y*ui))==colors[corner],"visible source corner "+corner+" keeps its original color");
            }
            check(container.left>=content.left-.01&&container.right<=content.right+.01&&container.top>=content.top-.01&&container.bottom<=content.bottom+.01&&container.right<rx,"visible camera stays in left region away from controls");
            if(image.left<content.left){
                check(lastImage.getRGB((int)((content.left-4)*ui),(int)(image.centerY()*ui))==0xfff6f6f6,"cropped image cannot paint outside left viewport");
                check(lastImage.getRGB((int)((content.right+7)*ui),(int)(image.centerY()*ui))==0xfff6f6f6,"cropped image cannot paint into control gutter");
            }
            check(jx-82>=rx&&jx+82<=f(view,"width")-48&&jy-82>=354&&jy+82<=height-20,"large joystick touch circle stays inside right panel with screen-edge clearance");
            boolean overlap=false;for(Object b:(List<?>)field(view,"buttons")){RectF rect=(RectF)field(b,"rect");if(rect.left<jx+82&&rect.right>jx-82&&rect.top<jy+82&&rect.bottom>jy-82)overlap=true;}
            check(!overlap,"no button overlaps right joystick hit region");check(button(view,"arm").width()>=150,"first page actions remain wide");
            check(lastCanvas.drawnTexts.stream().noneMatch(s->s.contains("横滚")||s.contains("俯仰")||s.contains("加速度")||s.contains("Yaw")),"phone has no IMU information card");
            metadata.put("detections",new JSONArray(detection(41,sw/3,sh/3,sw*2/3,sh*2/3)));frame(view,source,metadata);draw(view,w,h,null);touch(view,0,image.centerX(),image.centerY());touch(view,1,image.centerX(),image.centerY());check(a.selectedTarget==41,"box tap shares complete-image mapping");
            metadata.put("detections",new JSONArray(detection(42,0,0,sw,sh)));frame(view,source,metadata);draw(view,w,h,null);int selected=a.selections;
            float blankX=image.centerX(),blankY=content.top+1;if(image.top-content.top<3){blankX=content.left+1;blankY=image.centerY();if(image.left-content.left<3)blankX=content.right+7;}
            touch(view,0,blankX,blankY);touch(view,1,blankX,blankY);check(a.selections==selected,"bright area outside actual camera rejects target selection");
            frame(view,source,metadata);draw(view,w,h,null);selected=a.selections;touch(view,0,container.left+3,container.top+3);touch(view,1,container.left+3,container.top+3);check(a.selectedTarget==42&&a.selections==selected+1,"visible crop edge shares correct detection coordinates");
            metadata.put("box_age_ms",351);frame(view,source,metadata);draw(view,w,h,null);selected=a.selections;touch(view,0,image.centerX(),image.centerY());touch(view,1,image.centerX(),image.centerY());check(a.selections==selected,"full-frame layout still rejects stale boxes");
            metadata.put("box_age_ms",0);frame(view,source,metadata);draw(view,w,h,null);
            if(sw==1920){String suffix=w==1000?"": "-"+w+"x"+h;draw(view,w,h,output+"/perception-connected"+suffix+"-host.png");swipe(view,true);draw(view,w,h,output+"/functions-page2"+suffix+"-host.png");buttonSet(view,"mapping","follow","zero","calibrate");check(button(view,"stop")!=null,"fixed STOP stays visible on page two at every aspect");tap(view,"navigation");draw(view,w,h,output+"/navigation"+suffix+"-host.png");}
        }
    }
    static int orangePixels(){int n=0;for(int y=0;y<lastImage.getHeight();y++)for(int x=0;x<lastImage.getWidth();x++)if(lastImage.getRGB(x,y)==0xffff9500)n++;return n;}
    static int boxEdge(DashboardView v,int sourceLeft,int sourceTop,int sourceBottom)throws Exception{
        RectF image=r(v,"imageRect");float s=f(v,"scale");
        int x=Math.round((image.left+sourceLeft/1920f*image.width())*s),y=Math.round((image.top+(sourceTop+sourceBottom)/2160f*image.height())*s);
        return lastImage.getRGB(x,y);
    }
    static void persistentTargetHighlight(String output)throws Exception{
        Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);v.connection(true);v.flags(true,"manual");
        BufferedImage camera=source(1920,1080);JSONObject first=detection(51,300,220,700,700),other=detection(52,1100,300,1500,800);
        JSONObject pendingUiState=state(true).put("selected_id",0);SelectionGate gate=new SelectionGate();
        // A poll has already queued this state before the first tap reaches the UI.
        gate.reconcile(0,10);frame(v,camera,meta(first,other));draw(v,1000,600,null);RectF image=r(v,"imageRect");
        float firstX=image.left+500/1920f*image.width(),firstY=image.top+450/1080f*image.height();
        touch(v,0,firstX,firstY);touch(v,1,firstX,firstY);long requested=gate.request(a.selectedTarget);
        v.state(pendingUiState,gate.display());draw(v,1000,600,null);
        check(boxEdge(v,300,220,700)==0xffff9500,"queued pre-tap state cannot erase orange selection at UI delivery");
        check(gate.acknowledge(requested,20),"highlight sequence receives matching selection ACK");
        v.status("目标 #51 已选中");
        for(int i=0;i<6;i++){
            JSONObject moved=detection(51,300+i*18,220,700+i*18,700);
            frame(v,camera,meta(moved,other));v.state(state(true).put("selected_id",51),gate.reconcile(51,21+i*100));draw(v,1000,600,null);
            check(boxEdge(v,300+i*18,220,700)==0xffff9500,"selected tracked target stays orange across new frames and status polls");
            check(boxEdge(v,1100,300,800)!=0xffff9500,"unselected target stays neutral across new frames");
        }
        draw(v,1000,600,output+"/persistent-target-highlight-host.png");
        draw(v,2340,1080,output+"/persistent-target-highlight-2340x1080-host.png");draw(v,1000,600,null);
        // Old selection state is queued, but another tap wins before its UI callback.
        JSONObject queuedOldTarget=state(true).put("selected_id",51);
        float otherX=image.left+1300/1920f*image.width(),otherY=image.top+550/1080f*image.height();
        touch(v,0,otherX,otherY);touch(v,1,otherX,otherY);requested=gate.request(a.selectedTarget);
        v.state(queuedOldTarget,gate.display());draw(v,1000,600,null);
        check(boxEdge(v,1100,300,800)==0xffff9500&&boxEdge(v,390,220,700)!=0xffff9500,"queued previous-target state cannot overwrite newer target tap");
        check(gate.acknowledge(requested,700),"replacement target ACK confirmed");
        // Fresh metadata without the selected ID must never draw a frozen box.
        frame(v,camera,meta(first));v.state(state(true).put("selected_id",52),gate.display());draw(v,1000,600,null);
        check(orangePixels()==0,"missing target draws no orange ghost even before loss telemetry arrives");
        // A stale queued selected state must not restore a truly cleared target.
        JSONObject queuedSelected=state(true).put("selected_id",52);gate.reconcile(0,701);
        frame(v,camera,meta(first,other));v.state(queuedSelected,gate.display());draw(v,1000,600,null);
        check(orangePixels()==0,"true target loss wins over an older queued selected state");
        check(((Number)field(v,"selected")).longValue()==0,"real server target loss clears selected ID");
        v.state(state(true).put("selected_id",51));JSONObject expired=meta(first,other).put("box_age_ms",351);frame(v,camera,expired);draw(v,1000,600,null);
        check(orangePixels()==0,"350ms validity limit still removes even a selected detection");int taps=a.selections;
        touch(v,0,firstX,firstY);touch(v,1,firstX,firstY);check(a.selections==taps,"expired selected target remains unselectable");
        v.state(state(true).put("selected_id",0));frame(v,camera,meta(first,other));draw(v,1000,600,null);
        check(orangePixels()==0,"returning detection does not silently regain selection after true loss");
    }
    static JSONObject mapData(String session,long revision) {
        char[] cells=new char[4096];java.util.Arrays.fill(cells,'?');
        for(int y=8;y<57;y++)for(int x=8;x<57;x++)cells[y*64+x]='.';
        for(int y=18;y<44;y++)cells[y*64+48]='#';cells[0]='#';cells[63*64]='.';
        return new JSONObject().put("schema",1).put("available",true).put("pose_valid",true).put("width",64).put("height",64).put("resolution_m",.1)
                .put("origin",new JSONArray(-3.2,-3.2)).put("cells",new String(cells)).put("pose",new JSONArray(0,0,Math.PI/2))
                .put("path",new JSONArray(new JSONArray(0,0),new JSONArray(1,0),new JSONArray(1,2)))
                .put("trajectory",new JSONArray(new JSONArray(-1.8,-1),new JSONArray(-1,-1),new JSONArray(0,0)))
                .put("goal",new JSONArray(1,2)).put("state","tracking").put("reason","").put("map_revision",revision).put("session_id",session)
                .put("snapshot_age_ms",0).put("sensor_age_ms",0);
    }
    static void realNavigationMap(String output)throws Exception {
        Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);draw(v,1000,600,null);tap(v,"navigation");v.status("仿真验证 · 实际地图来自 RK3588");
        v.navigation(NavigationMap.parse(mapData("first",10),android.os.SystemClock.elapsedRealtime()));
        draw(v,1000,600,output+"/navigation-map-live-host.png");RectF r=r(v,"mapRect");
        check(Math.abs(r.width()-r.height())<.001,"map maintains square metric grid without stretching");
        check(lastImage.getRGB((int)(r.left+r.width()/128),(int)(r.bottom-r.height()/128))==0xffffffff,"cell(0,0) appears at bottom-left");
        check(lastImage.getRGB((int)(r.left+r.width()/128),(int)(r.top+r.height()/128))==0xff585858,"highest +y row appears at top-left");
        int pathX=Math.round(r.left+4.2f/6.4f*r.width()),pathY=Math.round(r.bottom-4.2f/6.4f*r.height());
        check(lastImage.getRGB(pathX,pathY)==0xff46beff,"positive-y route renders upward in sky blue");
        int px=Math.round(r.centerX()),py=Math.round(r.centerY());
        check(lastImage.getRGB(px,py+8)==0xffff2d2d,"user-confirmed forward correction: +pi/2 icon points down");
        check(lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("轨迹"))&&lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("6.4")),"map legend and meter extent shown");
        int drives=a.drives,arms=a.arms;touch(v,0,r.centerX(),r.centerY());touch(v,1,r.centerX(),r.centerY());
        check(a.drives==drives&&a.arms==arms,"read-only map touch never arms or moves vehicle");
        v.navigation(NavigationMap.parse(mapData("first",9),android.os.SystemClock.elapsedRealtime()));
        check(((NavigationMap)field(v,"navigationMap")).revision==10,"older revision in same coordinate session ignored");
        v.navigation(NavigationMap.parse(mapData("reset",1),android.os.SystemClock.elapsedRealtime()));
        check(((NavigationMap)field(v,"navigationMap")).revision==1,"new coordinate session accepts restarted revision");
        v.navigation(NavigationMap.parse(mapData("reset",2).put("pose_valid",false),android.os.SystemClock.elapsedRealtime()));draw(v,1000,600,null);
        check(lastCanvas.drawnTexts.contains("定位未就绪"),"invalid pose is clearly labelled");
        check(lastImage.getRGB(px,py-8)!=0xffff2d2d,"invalid pose does not draw stale vehicle");
        v.navigation(NavigationMap.parse(mapData("reset",3),android.os.SystemClock.elapsedRealtime()-2100));draw(v,1000,600,output+"/navigation-map-stale-host.png");
        check(lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("地图已过期")),"stale map explicitly unavailable");
        v.navigationUnavailable("导航服务未启动");draw(v,1000,600,null);
        check(field(v,"navigationMap")==null&&field(v,"navigationBitmap")==null&&lastCanvas.drawnTexts.contains("导航服务未启动"),"unavailable clears map and cached pixels");
        v.navigation(NavigationMap.parse(mapData("wide",1),android.os.SystemClock.elapsedRealtime()));draw(v,2340,1080,output+"/navigation-map-live-2340x1080-host.png");
        r=r(v,"mapRect");check(Math.abs(r.width()-r.height())<.001,"wide handset keeps map metric aspect");
    }
    static void navigationDestinations(String output)throws Exception {
        Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);draw(v,1000,600,null);tap(v,"navigation");
        v.navigation(NavigationMap.parse(mapData("tap",1),android.os.SystemClock.elapsedRealtime()));draw(v,1000,600,null);
        RectF map=r(v,"mapRect");check(map.width()>=400&&map.width()>302*1.3,"navigation map enlarged by at least30percent per edge");
        check(button(v,"follow")==null&&button(v,"cw")==null&&button(v,"arm")!=null&&button(v,"stop")!=null,"navigation hides camera actions but keeps arm and emergency stop");
        float x=map.left+20.5f/64*map.width(),y=map.bottom-40.5f/64*map.height();
        touch(v,0,x,y);touch(v,1,x,y);check(a.navigations==0&&a.arms==0,"map never implicitly arms");
        v.flags(true,"manual");touch(v,0,x,y);check(a.navigations==0,"destination not sent on DOWN");touch(v,1,x,y);
        check(a.navigations==1&&a.target.cellX==20&&a.target.cellY==40,"tap-UP maps exact world cell without vertical inversion");
        check(Math.abs(a.target.x+1.15)<1e-6&&Math.abs(a.target.y-.85)<1e-6,"cell-center destination in metres");
        draw(v,1000,600,output+"/navigation-destination-1000x600-host.png");
        touch(v,0,x,y);touch(v,2,x+30,y);touch(v,1,x+30,y);check(a.navigations==1,"dragging never sends destination");
        touch(v,0,x,y);touch(v,3,x,y);touch(v,1,x,y);check(a.navigations==1&&a.canceled>0,"cancel pauses without destination");v.flags(true,"manual");
        touch(v,0,x,y);touch(v,5,x,y);touch(v,1,x,y);check(a.navigations==1,"second finger cannot select destination");v.flags(true,"manual");
        touch(v,0,x,y);v.navigation(NavigationMap.parse(mapData("tap",2).put("origin",new JSONArray(10,10)),android.os.SystemClock.elapsedRealtime()));touch(v,1,x,y);
        check(a.navigations==2&&a.target.map.revision==1&&Math.abs(a.target.x+1.15)<1e-6,"map refresh mid-touch preserves displayed DOWN coordinate snapshot");
        touch(v,0,x,y);v.navigation(NavigationMap.parse(mapData("reset",1),android.os.SystemClock.elapsedRealtime()));touch(v,1,x,y);check(a.navigations==2,"coordinate-session reset during tap rejected");
        draw(v,1000,600,null);map=r(v,"mapRect");
        float wallX=map.left+48.5f/64*map.width(),wallY=map.bottom-30.5f/64*map.height();touch(v,0,wallX,wallY);touch(v,1,wallX,wallY);check(a.navigations==2,"obstacle cannot be navigation goal");
        v.navigation(NavigationMap.parse(mapData("reset",2).put("pose_valid",false),android.os.SystemClock.elapsedRealtime()));draw(v,1000,600,null);touch(v,0,x,y);touch(v,1,x,y);check(a.navigations==2,"invalid pose blocks tap-go");
        v.navigation(NavigationMap.parse(mapData("reset",3),android.os.SystemClock.elapsedRealtime()-2001));draw(v,1000,600,null);touch(v,0,x,y);touch(v,1,x,y);check(a.navigations==2&&r(v,"mapRect").isEmpty(),"stale map has no active destination hit region");
        v.navigation(NavigationMap.parse(mapData("reset",4),android.os.SystemClock.elapsedRealtime()));v.navigationResult("目的地已接受，正在规划路线",a.target);draw(v,2340,1080,output+"/navigation-destination-2340x1080-host.png");
        check(lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("目的地已接受")),"destination acceptance status visible");
        v.navigationResult("请先在 RK 端完成编码器和车速标定",null);draw(v,1000,600,null);check(lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("编码器")),"navigation rejection is visible");
    }
    static void followGeometry(String output)throws Exception {
        Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);v.connection(true);
        frame(v,source(1920,1080),meta(detection(41,200,300,600,700)));draw(v,1000,600,null);
        RectF image=r(v,"imageRect");float x=image.left+400f/1920*image.width(),y=image.top+500f/1080*image.height();
        touch(v,0,x,y);touch(v,1,x,y);check(a.selectedTarget==41&&a.drives==0,"unarmed target tap supports calibration without driving");
        JSONObject info=state(true).put("selected_target",new JSONObject().put("id",41).put("eye",0).put("u_px",411.5).put("v_px",532).put("angle_valid",true).put("turn_angle_deg",-12.3));
        v.state(info,41);draw(v,1000,600,output+"/follow-bearing-host.png");
        check(lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("左目 u=411.5")&&t.contains("左转 12.3°")),"selected camera pixel and RK left angle appear in existing footer");
        info.optJSONObject("selected_target").put("turn_angle_deg",8.2).put("eye",1);v.state(info,41);draw(v,1000,600,null);
        check(lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("右目")&&t.contains("右转 8.2°")),"positive RK angle labels clockwise/right");
        info.optJSONObject("selected_target").put("angle_valid",false).put("geometry_reason","camera_uncalibrated");v.state(info,41);draw(v,1000,600,null);
        check(lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("镜头尚未标定")),"invalid angle never presented as a zero-degree command");
        info.optJSONObject("selected_target").put("id",40);v.state(info,41);draw(v,1000,600,null);
        check(lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("等待角度"))&&lastCanvas.drawnTexts.stream().noneMatch(t->t.contains("u=411.5")),"stale different-target geometry never displayed for current selection");
    }
    static void navigationStartEnd(String output)throws Exception {
        Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);draw(v,1000,600,null);
        check(button(v,"manual")==null&&button(v,"hold")==null,"perception page removes old mode buttons");
        tap(v,"nav_start");check(a.starts==0,"disconnected navigation start cannot dispatch");
        v.connection(true);tap(v,"nav_start");check(a.starts==0&&a.arms==0,"start never implicitly arms");
        v.flags(true,"manual");down(v,"nav_start");check(a.starts==0,"start waits for confirmed tap UP");
        RectF b=button(v,"nav_start");touch(v,1,b.centerX(),b.centerY());check(a.starts==1,"armed explicit start dispatches once");
        tap(v,"navigation");draw(v,1000,600,null);
        check(button(v,"nav_start")!=null&&button(v,"nav_end")!=null&&button(v,"manual")==null,"navigation tab has visible start/end without redundant manual button");
        tap(v,"nav_start");check(a.starts==2,"tab switch retains authorization for explicit new start");
        v.flags(true,"manual");tap(v,"nav_start");check(a.starts==3,"navigation tab start dispatches after explicit tap");
        v.flags(true,"navigation");v.navigation(NavigationMap.parse(mapData("start-no-target",1),android.os.SystemClock.elapsedRealtime()));
        v.state(state(true).put("navigation",new JSONObject().put("active",true).put("state","SCAN_CLOCKWISE")));v.navigationResult("正在顺时针扫描可通行方向",null);
        draw(v,1000,600,output+"/navigation-start-active-host.png");
        check(buttonFill(v,"nav_start")==0xff111111&&lastCanvas.drawnTexts.stream().anyMatch(t->t.contains("扫描")),"navigation without a map target displays active mode and live state");
        tap(v,"nav_end");check(a.ends==1&&!((Boolean)field(v,"armed")),"end works immediately while navigation active");
        draw(v,1000,600,output+"/navigation-ended-host.png");
        tap(v,"nav_end");check(a.ends==2,"end remains available while unarmed");
        float jx=f(v,"joystickX"),jy=f(v,"joystickY");v.flags(true,"navigation");touch(v,0,jx,jy-42);check(a.drives>0&&a.throttle>0,"joystick retains automatic manual takeover");touch(v,1,jx,jy-42);
        draw(v,2340,1080,output+"/navigation-controls-2340x1080-host.png");
        check(lastCanvas.drawnTexts.stream().noneMatch(t->t.equals("手动差速")||t.equals("手动接管")||t.equals("航向保持")||t.equals("保持航向")),"retired controls absent from rendered UI");
    }
    static void mappingUi(String output)throws Exception {
        Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);draw(v,1000,600,null);
        check(a.mappings==0,"opening app cannot start mapping");
        tap(v,"navigation");draw(v,1000,600,null);
        check(a.mappings==0,"opening map tab cannot start mapping");
        check(button(v,"straight")==null&&button(v,"mapping")!=null,"mapping replaces straight on navigation page");
        check(lastCanvas.drawnTexts.contains("探索建图")&&!lastCanvas.drawnTexts.contains("保持直行"),"correct button labels");
        tap(v,"mapping");check(a.mappings==0,"disconnected mapping tap cannot dispatch");
        v.connection(true);tap(v,"mapping");check(a.mappings==0&&a.arms==0,"mapping tap never implicitly arms");
        v.flags(true,"manual");down(v,"mapping");check(a.mappings==0,"mapping waits for confirmed UP");
        RectF b=button(v,"mapping");touch(v,1,b.centerX(),b.centerY());check(a.mappings==1,"explicit mapping tap dispatches once");
        v.flags(true,"mapping");v.state(state(true).put("navigation",new JSONObject().put("supports_exploration_mapping",true).put("exploration_protocol_version",1)
            .put("mapping",new JSONObject().put("active",true).put("state","NAVIGATING"))));
        draw(v,2340,1080,output+"/exploration-active-host.png");
        check(buttonFill(v,"mapping")==0xff111111&&lastCanvas.drawnTexts.contains("结束建图"),"running exploration has black selected state and stop label");
        int releases=a.releases;tap(v,"perception");draw(v,1000,600,null);
        v.state(state(true).put("motor",new JSONObject().put("left",.2).put("right",.2)));
        swipe(v,true);draw(v,1000,600,null);tap(v,"navigation");draw(v,2340,1080,null);
        check(a.releases==releases&&"mapping".equals(field(v,"mode")),"map/camera and function-page browsing do not cancel exploration");
        v.navigation(NavigationMap.parse(mapData("exploring",1),android.os.SystemClock.elapsedRealtime()));draw(v,2340,1080,null);
        RectF mapArea=r(v,"mapRect");touch(v,0,mapArea.centerX(),mapArea.centerY());touch(v,1,mapArea.centerX(),mapArea.centerY());
        check(a.navigations==0,"map taps cannot replace exploring target");
        int starts=a.starts;tap(v,"nav_start");check(a.starts==starts,"navigation start cannot replace mapping");
        tap(v,"mapping");check(a.ends==1,"active mapping button sends ending action");
        check(lastCanvas.drawnTexts.stream().noneMatch(t->t.equals("地图已载入 FPGA")),"active map is never displayed as final loaded map");
        v.flags(false,"manual");v.state(state(true));tap(v,"perception");draw(v,1000,600,null);swipe(v,true);draw(v,1000,600,output+"/exploration-button-host.png");
        buttonSet(v,"mapping","follow","zero","calibrate");check(button(v,"straight")==null,"straight removed from auxiliary page too");
        tap(v,"navigation");
        NavigationMap map=NavigationMap.parse(mapData("final-map",9),android.os.SystemClock.elapsedRealtime());v.navigation(map);
        JSONObject mapping=new JSONObject().put("state","LOADED").put("active",false).put("loaded_map_session","final-map").put("loaded_map_revision",9);
        JSONObject nav=new JSONObject().put("supports_exploration_mapping",true).put("exploration_protocol_version",1).put("mapping",mapping);
        v.state(state(true).put("navigation",nav));draw(v,1000,600,null);
        check(!lastCanvas.drawnTexts.contains("地图已载入 FPGA"),"LOADED without commit confirmation cannot show success");
        mapping.put("map_commit_confirmed",true);v.state(state(true).put("navigation",nav));draw(v,1000,600,output+"/exploration-map-loaded-host.png");
        check(lastCanvas.drawnTexts.get(0).equals("地图已载入 FPGA"),"confirmed matching map shown centered above navigation");
        mapping.put("loaded_map_revision",8);v.state(state(true).put("navigation",nav));draw(v,1000,600,null);
        check(!lastCanvas.drawnTexts.contains("地图已载入 FPGA"),"wrong revision cannot show success");
        mapping.put("loaded_map_revision",9).put("loaded_map_session","old-session");v.state(state(true).put("navigation",nav));draw(v,1000,600,null);
        check(!lastCanvas.drawnTexts.contains("地图已载入 FPGA"),"wrong map session cannot show success");
        mapping.put("loaded_map_session","final-map");v.state(state(true).put("navigation",nav));
        Field received=DashboardView.class.getDeclaredField("stateReceivedAt");received.setAccessible(true);received.setLong(v,android.os.SystemClock.elapsedRealtime()-1001);draw(v,1000,600,null);
        check(!lastCanvas.drawnTexts.contains("地图已载入 FPGA"),"stale telemetry cannot show success");
        v.connection(false);draw(v,1000,600,null);check(buttonFill(v,"mapping")==0xfff2f2f2,"disconnection clears active mapping highlight");
    }
    static final java.util.Map<String,String> nativeSonarFixtures=new java.util.HashMap<>();
    static JSONObject sonarState(String name){return new JSONObject(nativeSonarFixtures.get(name));}
    static Canvas.TextDraw sonarRow(){for(Canvas.TextDraw t:lastCanvas.textDraws)if(t.text.startsWith("L:"))return t;throw new AssertionError("missing single sonar footer row");}
    static void sonarFooter(String output)throws Exception {
        check(nativeSonarFixtures.size()==10,"View uses actual native RK sonar-health fixtures");
        for(int[] size:new int[][]{{1000,600},{2340,1080},{3000,1000},{960,540},{1280,800}}) {
            Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);v.connection(true);v.flags(true,"manual");
            frame(v,source(1920,1080),meta());v.state(sonarState("normal"));draw(v,size[0],size[1],null);
            Canvas.TextDraw row=sonarRow();RectF image=r(v,"imageRect"),content=r(v,"contentRect");float height=f(v,"height");
            android.graphics.Paint measure=new android.graphics.Paint(0);measure.setTextSize(row.size);
            check(row.text.equals("L:0.82m  F:1.23m  R:>4m")&&row.size==12,"perception shows only one small L/F/R range line");
            check(row.y-row.size>=image.bottom+1&&row.y+4<height-59,"range line stays below full camera and above bottom tabs at every aspect");
            check(row.x>=content.left&&row.x+measure.measureText(row.text)<=content.right,"range line stays within left camera footer width");
            check(lastCanvas.textDraws.stream().filter(t->t.text.startsWith("L:")).count()==1,"no duplicate sonar rows or extra range card");
            check(lastCanvas.textDraws.stream().anyMatch(t->t.text.contains("FPS")&&t.y==row.y&&t.x>row.x+measure.measureText(row.text)),"existing FPS shares same footer row without covering sonar text");
            check(image.width()>0&&image.height()>0&&Math.abs(image.top-r(v,"navigationRect").top)<.01&&Math.abs(image.bottom-r(v,"navigationRect").bottom)<.01,"camera fills navigation height without overlapping sonar footer");
            if(size[0]==1000)draw(v,size[0],size[1],output+"/sonar-footer-perception-host.png");
            tap(v,"navigation");v.navigation(NavigationMap.parse(mapData("sonar",1),android.os.SystemClock.elapsedRealtime()));v.state(sonarState("normal"));draw(v,size[0],size[1],null);
            Canvas.TextDraw navRow=sonarRow();check(navRow.text.equals("L:0.82m  F:1.23m  R:>4m"),"navigation tab shares same simple range row");
            check(navRow.y-navRow.size>=r(v,"navigationRect").bottom+1&&navRow.y+4<height-59,"navigation map and range row do not overlap tabs or each other");
            if(size[0]==1000)draw(v,size[0],size[1],output+"/sonar-footer-navigation-host.png");
        }
        Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);v.connection(true);v.state(sonarState("fpga_invalid"));draw(v,1000,600,null);
        check(sonarRow().text.equals("L:0.82m  F:无效  R:>4m"),"View hides retained last front range when native FPGA invalid");
        v.state(sonarState("source_expired"));draw(v,1000,600,null);check(sonarRow().text.contains("F:过期"),"View presents native source expiry");
        v.state(sonarState("normal"));Field received=DashboardView.class.getDeclaredField("stateReceivedAt");received.setAccessible(true);received.setLong(v,android.os.SystemClock.elapsedRealtime()-1001);draw(v,1000,600,null);
        check(sonarRow().text.equals("L:过期  F:过期  R:过期"),"View does not retain normal ranges after phone telemetry stops");
        v.state(sonarState("normal"));v.connection(false);draw(v,1000,600,null);check(sonarRow().text.equals("L:—  F:—  R:—"),"View clears normal range on disconnect");
    }
    static void multi(DashboardView v,int action,int index,int[] ids,float[] x,float[] y)throws Exception {
        float scale=f(v,"scale");float[] sx=x.clone(),sy=y.clone();
        for(int i=0;i<x.length;i++){sx[i]*=scale;sy[i]*=scale;}
        v.onTouchEvent(new MotionEvent(action,index,ids,sx,sy));
    }
    static void multiTouchOwnership()throws Exception {
        Actions a=new Actions();DashboardView v=new DashboardView(new Context(),a);v.connection(true);v.flags(true,"manual");draw(v,1000,600,null);
        float x=f(v,"joystickX"),y=f(v,"joystickY");
        touch(v,0,x,y-40);int drives=a.drives;
        multi(v,5,1,new int[]{7,19},new float[]{x,5},new float[]{y-40,5});
        check(a.canceled==0&&a.releases==0&&a.stops==0&&a.drives==drives,"second finger does not cancel or reissue joystick motion");
        multi(v,2,0,new int[]{19,7},new float[]{x+70,x-40},new float[]{y+70,y});
        check(a.steer<0&&a.throttle==0,"joystick follows owner ID when pointer indexes reorder");drives=a.drives;
        multi(v,6,0,new int[]{19,7},new float[]{5,x-40},new float[]{5,y});
        check(a.releases==0&&a.drives==drives,"secondary UP cannot release owner or drive");
        multi(v,5,1,new int[]{7,22},new float[]{x-40,x},new float[]{y,y-50});
        multi(v,6,0,new int[]{7,22},new float[]{x-40,x},new float[]{y,y-50});
        check(a.releases==1&&a.canceled==0,"owner POINTER_UP sends zero and preserves enable");drives=a.drives;
        multi(v,2,0,new int[]{22},new float[]{x},new float[]{y-50});multi(v,1,0,new int[]{22},new float[]{x},new float[]{y-50});
        check(a.drives==drives&&a.releases==1&&a.canceled==0,"remaining secondary pointer never inherits throttle");
        hold(v,"cw");RectF cw=button(v,"cw");int rotations=a.rotates;
        multi(v,5,1,new int[]{7,19},new float[]{cw.centerX(),1},new float[]{cw.centerY(),1});
        multi(v,2,0,new int[]{19,7},new float[]{1,cw.centerX()},new float[]{1,cw.centerY()});
        multi(v,6,0,new int[]{19,7},new float[]{1,cw.centerX()},new float[]{1,cw.centerY()});
        check(a.rotates==rotations&&a.rotationReleases==0&&a.canceled==0,"secondary touch cannot cancel held rotation");
        touch(v,1,cw.centerX(),cw.centerY());check(a.rotationReleases==1&&a.canceled==0,"rotation owner releases normally after multi-touch");
        RectF arm=button(v,"arm");touch(v,0,arm.centerX(),arm.centerY());
        multi(v,5,1,new int[]{7,19},new float[]{arm.centerX(),x},new float[]{arm.centerY(),y});
        multi(v,6,0,new int[]{7,19},new float[]{arm.centerX(),x},new float[]{arm.centerY(),y});
        multi(v,1,0,new int[]{19},new float[]{x},new float[]{y});
        check(a.arms==0&&a.canceled==0&&a.stops==0,"multi-finger panel touch neither toggles arm nor takes joystick");
        touch(v,0,x,y-40);multi(v,2,0,new int[]{19},new float[]{x},new float[]{y-50});
        check(a.canceled==1&&Boolean.TRUE.equals(field(v,"armed")),"genuinely lost driving owner pauses safely without disarming");touch(v,1,0,0);v.flags(true,"manual");
        touch(v,0,x,y-40);touch(v,3,x,y-40);check(a.canceled==2&&Boolean.TRUE.equals(field(v,"armed")),"real ACTION_CANCEL pauses current gesture without revoking authority");touch(v,1,0,0);
        draw(v,1000,600,null);tap(v,"navigation");v.flags(true,"manual");v.navigation(NavigationMap.parse(mapData("multi",1),android.os.SystemClock.elapsedRealtime()));draw(v,1000,600,null);
        RectF map=r(v,"mapRect");int targets=a.navigations, canceledBeforeMap=a.canceled;
        touch(v,0,map.centerX(),map.centerY());
        multi(v,5,1,new int[]{7,19},new float[]{map.centerX(),map.centerX()+50},new float[]{map.centerY(),map.centerY()+50});
        multi(v,6,0,new int[]{7,19},new float[]{map.centerX(),map.centerX()+50},new float[]{map.centerY(),map.centerY()+50});
        multi(v,1,0,new int[]{19},new float[]{map.centerX()+50},new float[]{map.centerY()+50});
        check(a.navigations==targets&&a.canceled==canceledBeforeMap,"two-finger map gesture cannot dispatch a navigation destination or disarm");
        v.flags(true,"manual");touch(v,0,x,y-40);RectF stop=button(v,"stop");int stops=a.stops;
        multi(v,5,1,new int[]{7,19},new float[]{x,stop.centerX()},new float[]{y-40,stop.centerY()});
        check(a.stops==stops+1&&!((Boolean)field(v,"armed")),"explicit red STOP works with second finger while owner holds joystick");
        multi(v,6,0,new int[]{7,19},new float[]{x,stop.centerX()},new float[]{y-40,stop.centerY()});
        multi(v,1,0,new int[]{19},new float[]{stop.centerX()},new float[]{stop.centerY()});
    }
    public static void main(String[] args)throws Exception{String output=args.length==0?"build/ui-preview":args[0];new File(output).mkdirs();if(args.length<2)throw new AssertionError("native sonar fixture file is required");for(String row:java.nio.file.Files.readAllLines(java.nio.file.Paths.get(args[1]),java.nio.charset.StandardCharsets.UTF_8)){JSONObject fixture=new JSONObject(row);nativeSonarFixtures.put(fixture.optString("case"),fixture.optJSONObject("state").toString());}motionAndPaging(output);swipeIntentAndDots(output);selectedButtonsAndHeader(output);layoutAndFullFrame(output);persistentTargetHighlight(output);realNavigationMap(output);navigationDestinations(output);followGeometry(output);navigationStartEnd(output);mappingUi(output);sonarFooter(output);multiTouchOwnership();System.out.println("PASS "+checks+" production View host checks; Java2D preview only, Android device validation pending");}
}
