package android.view;
import android.content.Context;import android.graphics.Canvas;import java.util.ArrayList;
public class View {
 private static final class Pending {final Runnable task;long delay;Pending(Runnable r,long d){task=r;delay=d;}}
 private final ArrayList<Pending> pending=new ArrayList<>();
 public View(Context c){}public void setFocusable(boolean b){}public void setKeepScreenOn(boolean b){}public void setContentDescription(String s){}
 public void invalidate(){}public void postInvalidateDelayed(long ms){}
 public boolean postDelayed(Runnable task,long ms){pending.add(new Pending(task,ms));return true;}
 public boolean removeCallbacks(Runnable task){return pending.removeIf(p->p.task==task);}
 public void runPostedTasks(long elapsed){ArrayList<Runnable> ready=new ArrayList<>();for(Pending p:pending){p.delay-=elapsed;if(p.delay<=0)ready.add(p.task);}pending.removeIf(p->p.delay<=0);for(Runnable r:ready)r.run();}
 protected void onSizeChanged(int w,int h,int ow,int oh){}protected void onDraw(Canvas c){}
 public void layout(int w,int h){onSizeChanged(w,h,0,0);}public void draw(Canvas c){onDraw(c);}
 public boolean onTouchEvent(MotionEvent e){return true;}public boolean performClick(){return true;}
}
