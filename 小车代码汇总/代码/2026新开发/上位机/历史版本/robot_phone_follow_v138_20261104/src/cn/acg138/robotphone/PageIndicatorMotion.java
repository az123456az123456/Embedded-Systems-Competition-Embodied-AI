package cn.acg138.robotphone;

/** Two-page liquid indicator; presentation only, never owns a touch gesture. */
public final class PageIndicatorMotion {
    public static final long SETTLE_MS = 320, FADE_MS = 260;
    private int from, to;
    private float initialProgress, finalProgress;
    private long releasedAt = -1;
    private boolean enabled;

    public void settle(int fromPage, int selectedPage, float distance, long now, boolean motion) {
        from = fromPage; to = 1-fromPage;
        initialProgress = Math.max(0,Math.min(1,distance));
        finalProgress = selectedPage==fromPage ? 0 : 1;
        releasedAt=now; enabled=motion;
    }
    public void reset() { releasedAt=-1; enabled=false; }
    public static final class Shape {
        public float tail, head, radius=4.5f, ghostAlpha;
        public boolean animate;
    }
    public Shape frame(int page, boolean dragging, float displacement, long now, boolean motion) {
        Shape shape=new Shape();
        int start=page, destination=1-page;
        float p=0;
        if (dragging && motion) {
            float directed=page==0 ? -displacement : displacement;
            p=Math.max(0,Math.min(1,directed));
        } else if (enabled && motion && releasedAt>=0 && now>=releasedAt && now-releasedAt<SETTLE_MS+FADE_MS) {
            start=from; destination=to;
            float t=Math.max(0,Math.min(1,(float)(now-releasedAt)/SETTLE_MS));
            float u=1-t;
            float ease=1-u*u*u;
            p=initialProgress+(finalProgress-initialProgress)*ease;
            // Only the faint connected tail fades after the destination is reached.
            float fade=Math.max(0,1-(float)(now-releasedAt)/ (SETTLE_MS+FADE_MS));
            shape.ghostAlpha=.16f*fade*Math.max(initialProgress, Math.abs(finalProgress-initialProgress));
            shape.radius=4.5f+(float)(.28*Math.sin(Math.PI*t));
            shape.animate=true;
        } else {
            shape.head=shape.tail=page; return shape;
        }
        // First stretch one continuous capsule until it meets the other dot;
        // only after contact does its trailing edge catch up and become a dot.
        float leading=Math.min(1,p/.62f);
        float trailing=Math.max(0,(p-.62f)/.38f);
        shape.head=start+(destination-start)*leading;
        shape.tail=start+(destination-start)*trailing;
        shape.radius*=1-.12f*(float)Math.sin(Math.PI*p);
        return shape;
    }
}
