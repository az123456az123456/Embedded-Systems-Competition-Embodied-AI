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
}
