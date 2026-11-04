package cn.acg138.robotphone;

/** Display-only world bounds: keep the frame, ease expansion, reset per session. */
public final class MapViewport {
    public double x,y,span;
    private double sx,sy,ss,tx,ty,ts;
    private String session="";
    private long since;
    public void update(NavigationMap m,long now) {
        if(m==null||!m.available)return;
        double nx=m.viewOriginX,ny=m.viewOriginY,ns=m.viewSpan;
        String identity=m.raster!=null&&!m.raster.session.isEmpty()?m.raster.session:m.sessionId;
        if(span<=0||!session.equals(identity)) {
            session=identity;x=sx=tx=nx;y=sy=ty=ny;span=ss=ts=ns;since=now;return;
        }
        double u=now>=since?Math.min(1,(now-since)/300.):0;u=u*u*(3-2*u);
        x=sx+(tx-sx)*u;y=sy+(ty-sy)*u;span=ss+(ts-ss)*u;
        if(nx>=tx-1e-6&&ny>=ty-1e-6&&nx+ns<=tx+ts+1e-6&&ny+ns<=ty+ts+1e-6)return;
        double loX=Math.min(tx,nx),loY=Math.min(ty,ny),hiX=Math.max(tx+ts,nx+ns),hiY=Math.max(ty+ts,ny+ns);
        double size=Math.max(hiX-loX,hiY-loY),pad=Math.max(.25,size*.125);
        sx=x;sy=y;ss=span;since=now;ts=size+2*pad;tx=(loX+hiX-ts)/2;ty=(loY+hiY-ts)/2;
    }
    public boolean animating(long now){return span>0&&now-since<300;}
}
