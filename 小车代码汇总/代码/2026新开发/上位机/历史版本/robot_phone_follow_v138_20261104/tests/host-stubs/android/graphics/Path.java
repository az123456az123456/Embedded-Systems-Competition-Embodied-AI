package android.graphics;
import java.awt.Shape;
import java.awt.geom.RoundRectangle2D;
public class Path {
    public enum Direction { CW, CCW }
    Shape shape;
    private final java.awt.geom.Path2D.Float polygon=new java.awt.geom.Path2D.Float();
    public void moveTo(float x,float y){polygon.moveTo(x,y);shape=polygon;}
    public void lineTo(float x,float y){polygon.lineTo(x,y);shape=polygon;}
    public void close(){polygon.closePath();shape=polygon;}
    public void addRoundRect(RectF rect,float rx,float ry,Direction direction) {
        shape=new RoundRectangle2D.Float(rect.left,rect.top,rect.width(),rect.height(),rx*2,ry*2);
    }
}
