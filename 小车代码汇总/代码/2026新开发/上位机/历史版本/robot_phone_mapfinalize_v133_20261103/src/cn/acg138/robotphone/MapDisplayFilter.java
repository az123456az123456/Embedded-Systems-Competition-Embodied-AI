package cn.acg138.robotphone;

/** GaussianBlur(3x3, sigma=0), as in Map Enhancer Wizard V2, for pixels only.
 * Preserve obstacle and unknown samples. No threshold/erosion in control maps. */
public final class MapDisplayFilter {
    private static int reflect(int x,int n){return n<=1?0:x<0?-x:x>=n?2*n-2-x:x;}
    private static int gray(char c){return c=='#'?255:c=='.'?88:24;}
    public static int[] pixels(String cells,int w,int h){
        if(w<=0||h<=0||(long)w*h>1000000||cells.length()!=w*h)throw new IllegalArgumentException("invalid map pixels");
        int[] out=new int[w*h];
        for(int y=0;y<h;y++)for(int x=0;x<w;x++){
            char cell=cells.charAt(y*w+x);int g=gray(cell);
            if(cell=='.'){
                int total=0;
                for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++)
                    total+=(dx==0?2:1)*(dy==0?2:1)*gray(cells.charAt(reflect(y+dy,h)*w+reflect(x+dx,w)));
                g=Math.max(88,(total+8)/16);
            }
            out[(h-1-y)*w+x]=0xff000000|(g<<16)|(g<<8)|g;
        }
        return out;
    }
}
