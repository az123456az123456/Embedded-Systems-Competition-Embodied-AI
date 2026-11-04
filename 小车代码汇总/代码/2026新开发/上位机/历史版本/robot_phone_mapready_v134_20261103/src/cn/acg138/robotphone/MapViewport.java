package cn.acg138.robotphone;

/** Display-only world bounds. Refreshes never pan/zoom; only explicit fit or a new commit does. */
public final class MapViewport {
    public double x,y,span;
    private double sx,sy,ss,tx,ty,ts;
    private String session="";
    private long since;
    private boolean localFocus;
    private String committedSession="";
    private long committedRevision=-1;
    private double committedX,committedY;
    private void moveTo(double nx,double ny,double ns,long now){
        sx=x;sy=y;ss=span;tx=nx;ty=ny;ts=ns;since=now;
    }
    private void rememberCommit(NavigationMap m){
        committedSession=m.sessionId;committedRevision=m.revision;
        committedX=m.originX;committedY=m.originY;
    }
    public void update(NavigationMap m,long now) {
        if(m==null||!m.available)return;
        double nx=m.viewOriginX,ny=m.viewOriginY,ns=m.viewSpan;
        String identity=m.raster!=null&&!m.raster.session.isEmpty()?m.raster.session:m.sessionId;
        if(span<=0||!session.equals(identity)) {
            localFocus=m.localWindowConfirmed;
            committedSession="";committedRevision=-1;
            if(localFocus){nx=m.originX;ny=m.originY;ns=6.4;rememberCommit(m);}
            session=identity;x=sx=tx=nx;y=sy=ty=ny;span=ss=ts=ns;since=now;return;
        }
        double u=now>=since?Math.min(1,(now-since)/300.):0;u=u*u*(3-2*u);
        if(u>=1){x=tx;y=ty;span=ts;}
        else {x=sx+(tx-sx)*u;y=sy+(ty-sy)*u;span=ss+(ts-ss)*u;}
        // The server confirms the exact FPGA bitmap, not just a completed save.
        // Fit its world rectangle while retaining every native ROS pixel.
        if(m.localWindowConfirmed&&(!committedSession.equals(m.sessionId)||committedRevision!=m.revision
                ||committedX!=m.originX||committedY!=m.originY)){
            moveTo(m.originX,m.originY,6.4,now);rememberCommit(m);localFocus=true;
        }
        // Extent/origin updates and transient metadata gaps do not move the
        // camera. Raster origins still place each pixel in its real world cell.
    }
    public boolean locallyFocused(){return localFocus;}
    public void toggleOverview(NavigationMap m,long now){
        if(m==null||!m.available)return;
        update(m,now);
        if(m.localWindowConfirmed&&!localFocus){
            moveTo(m.originX,m.originY,6.4,now);localFocus=true;
        } else {
            moveTo(m.viewOriginX,m.viewOriginY,m.viewSpan,now);localFocus=false;
        }
    }
    public boolean animating(long now){return span>0&&now>=since&&now-since<300&&(x!=tx||y!=ty||span!=ts);}
}
