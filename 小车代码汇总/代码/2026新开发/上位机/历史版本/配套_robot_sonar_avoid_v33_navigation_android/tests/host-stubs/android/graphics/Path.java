package android.graphics;
import java.awt.Shape;
import java.awt.geom.RoundRectangle2D;
public class Path {
    public enum Direction { CW, CCW }
    Shape shape;
    public void addRoundRect(RectF rect,float rx,float ry,Direction direction) {
        shape=new RoundRectangle2D.Float(rect.left,rect.top,rect.width(),rect.height(),rx*2,ry*2);
    }
}
