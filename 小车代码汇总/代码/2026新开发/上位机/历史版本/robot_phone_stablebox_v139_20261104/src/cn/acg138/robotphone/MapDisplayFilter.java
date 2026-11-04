package cn.acg138.robotphone;

/** Map Enhancer Wizard V2 Gaussian stage: GaussianBlur(5x5, sigma=0),
 * BORDER_REFLECT_101. Separable integer kernel [1,4,6,4,1]/16, no downsampling.
 * Only phone display pixels change. Raw obstacle/unknown samples are preserved;
 * the Wizard's threshold/morphology never changes the navigation grid. */
public final class MapDisplayFilter {
    private static final int[] K={1,4,6,4,1};
    private static int reflect(int x,int n){
        if(n<=1)return 0;
        while(x<0||x>=n)x=x<0?-x:2*n-2-x;
        return x;
    }
    private static int gray(char c){return c=='#'?255:c=='.'?88:24;}
    public static int[] pixels(String cells,int w,int h){
        if(w<=0||h<=0||(long)w*h>1000000||cells.length()!=w*h)throw new IllegalArgumentException("invalid map pixels");
        int[] out=new int[w*h],horizontal=new int[w*h];
        for(int y=0;y<h;y++)for(int x=0;x<w;x++){
            int sum=0;for(int dx=-2;dx<=2;dx++)sum+=K[dx+2]*gray(cells.charAt(y*w+reflect(x+dx,w)));
            horizontal[y*w+x]=sum;
        }
        for(int y=0;y<h;y++)for(int x=0;x<w;x++){
            char cell=cells.charAt(y*w+x);int g=gray(cell);
            if(cell=='.'){
                int total=0;
                for(int dy=-2;dy<=2;dy++)total+=K[dy+2]*horizontal[reflect(y+dy,h)*w+x];
                g=Math.max(88,(total+128)/256);
            }
            out[(h-1-y)*w+x]=0xff000000|(g<<16)|(g<<8)|g;
        }
        return out;
    }
    /** Supersampled circular splats round adjacent obstacle cells without joining
     * separate components. Radius covers the occupied square; no obstacle is
     * removed. Gaussian halo remains display-only. Output capped at 4M pixels. */
    public static int scale(int w,int h){
        long n=(long)w*h;return n>0&&n<=250000?4:n<=1000000?2:1;
    }
    public static int[] roundedPixels(String cells,int w,int h){
        int[] base=pixels(cells,w,h);int s=scale(w,h),ow=w*s,oh=h*s;
        int[] out=new int[ow*oh];
        for(int y=0;y<oh;y++)for(int x=0;x<ow;x++){
            double xx=(x+.5)/s-.5,yy=(y+.5)/s-.5;
            int x0=(int)Math.floor(xx),y0=(int)Math.floor(yy);
            double fx=xx-x0,fy=yy-y0;
            int a=base[Math.max(0,Math.min(h-1,y0))*w+Math.max(0,Math.min(w-1,x0))]&255;
            int b=base[Math.max(0,Math.min(h-1,y0))*w+Math.max(0,Math.min(w-1,x0+1))]&255;
            int c=base[Math.max(0,Math.min(h-1,y0+1))*w+Math.max(0,Math.min(w-1,x0))]&255;
            int d=base[Math.max(0,Math.min(h-1,y0+1))*w+Math.max(0,Math.min(w-1,x0+1))]&255;
            int g=(int)Math.round((a+(b-a)*fx)*(1-fy)+(c+(d-c)*fx)*fy);
            out[y*ow+x]=0xff000000|(g<<16)|(g<<8)|g;
        }
        // Rounded union: radius sqrt(1/2) cells retains all original occupied
        // corners. A one-cell doorway is never bridged by these circles.
        double r=Math.sqrt(.5)*s,edge=.65;
        for(int y=0;y<h;y++)for(int x=0;x<w;x++)if(cells.charAt(y*w+x)=='#'){
            double cx=(x+.5)*s,cy=(h-y-.5)*s;
            int loX=Math.max(0,(int)Math.floor(cx-r-edge)),hiX=Math.min(ow-1,(int)Math.ceil(cx+r+edge));
            int loY=Math.max(0,(int)Math.floor(cy-r-edge)),hiY=Math.min(oh-1,(int)Math.ceil(cy+r+edge));
            for(int py=loY;py<=hiY;py++)for(int px=loX;px<=hiX;px++){
                double dx=px+.5-cx,dy=py+.5-cy;
                double alpha=Math.max(0,Math.min(1,(r+edge-Math.sqrt(dx*dx+dy*dy))/(2*edge)));
                int i=py*ow+px,old=out[i]&255,g=Math.max(old,(int)Math.round(88+167*alpha));
                if(alpha>0)out[i]=0xff000000|(g<<16)|(g<<8)|g;
            }
        }
        // Join actual 4-neighbour occupied cells with constant-width capsules:
        // a wall becomes a smooth stroke rather than a string of scallops.
        for(int y=0;y<h;y++)for(int x=0;x<w;x++)if(cells.charAt(y*w+x)=='#'){
            double cx=(x+.5)*s,cy=(h-y-.5)*s;
            if(x+1<w&&cells.charAt(y*w+x+1)=='#')stroke(out,ow,oh,cx,cy,cx+s,cy,r,edge);
            if(y+1<h&&cells.charAt((y+1)*w+x)=='#')stroke(out,ow,oh,cx,cy,cx,cy-s,r,edge);
        }
        return out;
    }
    private static void stroke(int[] out,int w,int h,double ax,double ay,double bx,double by,double r,double edge){
        int x0=Math.max(0,(int)Math.floor(Math.min(ax,bx)-r-edge)),x1=Math.min(w-1,(int)Math.ceil(Math.max(ax,bx)+r+edge));
        int y0=Math.max(0,(int)Math.floor(Math.min(ay,by)-r-edge)),y1=Math.min(h-1,(int)Math.ceil(Math.max(ay,by)+r+edge));
        for(int y=y0;y<=y1;y++)for(int x=x0;x<=x1;x++){
            double dx=x+.5-Math.max(Math.min(ax,bx),Math.min(Math.max(ax,bx),x+.5));
            double dy=y+.5-Math.max(Math.min(ay,by),Math.min(Math.max(ay,by),y+.5));
            double alpha=Math.max(0,Math.min(1,(r+edge-Math.sqrt(dx*dx+dy*dy))/(2*edge)));
            int i=y*w+x,g=Math.max(out[i]&255,(int)Math.round(88+167*alpha));
            if(alpha>0)out[i]=0xff000000|(g<<16)|(g<<8)|g;
        }
    }

}
