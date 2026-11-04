package android.graphics;
public class Matrix {
 public float[] values;
 public void setValues(float[] v){values=v.clone();}
 public boolean setPolyToPoly(float[] s,int si,float[] d,int di,int n){
  if(n!=3)throw new IllegalArgumentException("three point affine adapter only");
  float sx=s[si+2]-s[si],sy=s[si+3]-s[si+1],tx=s[si+4]-s[si],ty=s[si+5]-s[si+1],det=sx*ty-tx*sy;
  if(det==0)return false;
  float dx=d[di+2]-d[di],dy=d[di+3]-d[di+1],ex=d[di+4]-d[di],ey=d[di+5]-d[di+1];
  float a=(dx*ty-ex*sy)/det,b=(ex*sx-dx*tx)/det,c=(dy*ty-ey*sy)/det,e=(ey*sx-dy*tx)/det;
  values=new float[]{a,b,d[di]-a*s[si]-b*s[si+1],c,e,d[di+1]-c*s[si]-e*s[si+1],0,0,1};return true;
 }
}
