package android.view;
/** Host event adapter with Android-style pointer IDs and action indexes. */
public class MotionEvent {
 public static final int ACTION_DOWN=0,ACTION_UP=1,ACTION_MOVE=2,ACTION_CANCEL=3,ACTION_POINTER_DOWN=5,ACTION_POINTER_UP=6;
 private final int action,actionIndex;private final int[] ids;private final float[] x,y;
 public MotionEvent(int action,float x,float y){this(action,0,new int[]{7},new float[]{x},new float[]{y});}
 public MotionEvent(int action,int actionIndex,int[] ids,float[] x,float[] y){
  if(ids.length==0||ids.length!=x.length||ids.length!=y.length||actionIndex<0||actionIndex>=ids.length)throw new IllegalArgumentException("pointers");
  this.action=action;this.actionIndex=actionIndex;this.ids=ids.clone();this.x=x.clone();this.y=y.clone();
 }
 public int getActionMasked(){return action;}public int getActionIndex(){return actionIndex;}public int getPointerCount(){return ids.length;}
 public float getX(){return x[0];}public float getY(){return y[0];}public float getX(int i){return x[i];}public float getY(int i){return y[i];}
 public int getPointerId(int i){return ids[i];}public int findPointerIndex(int p){for(int i=0;i<ids.length;i++)if(ids[i]==p)return i;return -1;}
}
