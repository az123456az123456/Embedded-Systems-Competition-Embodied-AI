package cn.acg138.robotphone;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.os.SystemClock;
import android.view.MotionEvent;
import android.view.View;
import org.json.JSONArray;
import org.json.JSONObject;
import java.util.ArrayList;
import java.util.Locale;

/** Apple-inspired presentation; existing driving, ownership and network actions are unchanged. */
public final class DashboardView extends View {
    public interface Actions {
        void settings(); void connect(); void arm(); void stop(); void follow();
        void mode(boolean hold); void zero(); void calibrate();
        void stick(float throttle, float steer, boolean hold); void release(boolean cancel);
        void rotate(boolean clockwise); void releaseRotation(boolean cancel);
        void select(long frameId, long targetId);
        void navigate(NavigationTarget target);
        void semanticInput(); void startSemantic();
        void startNavigation(); void startManualMapping(); void startMapping(); void endMapping(); void endNavigation();
    }
    private final PageIndicatorMotion pageIndicator = new PageIndicatorMotion();
    private boolean pageIndicatorAnimating;
    private final ButtonFeedback feedback = new ButtonFeedback();
    private int feedbackPointer = -1;
    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG | Paint.FILTER_BITMAP_FLAG);
    private final Actions actions;
    private final ArrayList<Button> buttons = new ArrayList<>();
    private final ControlPolicy.Gesture gesture = new ControlPolicy.Gesture();
    private final int background = Color.rgb(246, 246, 246), card = Color.WHITE;
    private final int ink = Color.rgb(20, 20, 20), muted = Color.rgb(118, 118, 118);
    private final int black = Color.rgb(17, 17, 17), inactive = Color.rgb(242, 242, 242);
    private final int red = Color.rgb(223, 53, 68), statusBackground = Color.rgb(112, 112, 112);
    private final int separator = Color.rgb(224, 224, 224);
    private RobotClient.Frame frame;
    private NavigationMap navigationMap;
    private android.graphics.Bitmap navigationBitmap;
    private String navigationError = "等待导航服务";
    private final RectF mapRect = new RectF(), navigationRect = new RectF(), mapTouchRect = new RectF();
    private final MapViewport mapViewport=new MapViewport();
    private double touchViewX,touchViewY,touchViewSpan;
    private NavigationMap touchedMap;
    private int mapPointer = -1;
    private float mapDownX, mapDownY;
    private long mapDownAt;
    private boolean mapDragged;
    private NavigationTarget pendingDestination;
    private String destinationStatus = "";
    private String semanticText="";
    public void semanticText(String text){semanticText=text==null?"":text;invalidate();}
    private JSONObject state = new JSONObject();
    private long stateReceivedAt;
    private final RectF contentRect = new RectF(), videoRect = new RectF(), imageRect = new RectF(), rotationRect = new RectF();
    private final RectF controlRect = new RectF(), pendingPanelRect = new RectF(), statusRect = new RectF();
    private float scale = 1, width = 1000, height = 600, rightX, rightWidth, joystickX, joystickY, stickX, stickY;
    private boolean connected, armed, hold, rotating;
    private int tab, controlPage;
    private int panelPointer = -1;
    private float panelDownX, panelDownY, panelDelta;
    private boolean panelSwiping, panelMoved;
    private String pendingPanelKey, immediatePressedKey;
    private static final int ROTATION_HOLD_MS = 180;
    private final Runnable rotationHold = new Runnable() {
        @Override public void run() {
            if (panelPointer < 0 || panelMoved || panelSwiping ||
                    !("cw".equals(pendingPanelKey) || "ccw".equals(pendingPanelKey))) return;
            if (!gesture.begin(panelPointer, armed)) { status("请先使能电机"); return; }
            rotating = true; rotationRect.set(pendingPanelRect.left, pendingPanelRect.top, pendingPanelRect.right, pendingPanelRect.bottom);
            hold = false; actions.rotate("cw".equals(pendingPanelKey)); invalidate();
        }
    };
    private long selected = -1;
    private String status = "填写板子地址和配对码，即可连接小车", mode = "manual";
    private static final class Button {
        final RectF rect; final String key;
        Button(RectF rect, String key) { this.rect = rect; this.key = key; }
    }
    public DashboardView(Context context, Actions actions) {
        super(context); this.actions = actions; setFocusable(true); setKeepScreenOn(true);
        setContentDescription("ACG138 小车：摄像头等比放大、分页功能、右侧大摇杆与按住旋转");
    }
    public void connection(boolean connected) {
        this.connected = connected;
        if (!connected) { armed = false; clearGesture(); feedback.clear(); }
        invalidate();
    }
    public void flags(boolean armed, String mode) {
        if (this.armed && !armed) clearGesture();
        this.armed = armed; this.mode = mode; this.hold = "hold".equals(mode); invalidate();
    }
    public void status(String text) { status = text == null ? "" : text; invalidate(); }
    public void state(JSONObject value) {
        state(value, value.optLong("selected_id", selected));
    }
    public void state(JSONObject value, long selectedTarget) {
        state = value; stateReceivedAt=SystemClock.elapsedRealtime(); selected = selectedTarget;
        if (hold && !imuFresh()) {
            hold = false; clearGesture(); actions.release(false); actions.mode(false);
            status = "IMU 数据未更新，航向保持已退出；可重新推动摇杆手动驾驶";
        }
        invalidate();
    }
    private boolean imuFresh() {
        JSONObject imu = state.optJSONObject("imu");
        return imu != null && imu.optBoolean("valid") && imu.optBoolean("initialized", true) && !imu.optBoolean("calibrating") &&
                imu.optBoolean("fresh", imu.optLong("sample_age_ms", 0) <= 300);
    }
    public void frame(RobotClient.Frame value) {
        RobotClient.Frame previous = frame; frame = value;
        if (previous != null && previous.image != value.image) previous.image.recycle();
        invalidate();
    }
    public void clearVideo() {
        if (frame != null) { frame.image.recycle(); frame = null; }
        imageRect.setEmpty(); selected = -1; invalidate();
    }
    private void clearPanelGesture() {
        if (panelSwiping) {
            pageIndicator.settle(controlPage, controlPage, Math.abs(panelDelta)/rightWidth,
                    SystemClock.elapsedRealtime(), android.animation.ValueAnimator.areAnimatorsEnabled());
            buttons.clear();
        }
        removeCallbacks(rotationHold); panelPointer = -1; pendingPanelKey = null;
        panelMoved = panelSwiping = false; panelDelta = 0;
    }
    private void clearGesture() { mapPointer=-1; touchedMap=null; clearPanelGesture(); gesture.end(); rotating = false; stickX = stickY = 0; }
    @Override protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        scale = Math.min(w / 1000f, h / 600f); width = w / scale; height = h / scale;
        rightWidth = Math.max(310, Math.min(370, width * .34f)); rightX = width - rightWidth - 20;
        contentRect.set(20, 64, rightX - 18, height - 78);
        navigationRect.set(20, 64, rightX - 18, height - 78);
        mapPointer=-1;touchedMap=null;
        controlRect.set(rightX, 90, width - 20, 270);
        joystickX = rightX + rightWidth / 2; joystickY = (354 + height - 20) / 2;
    }
    private void text(Canvas c, String value, float x, float y, float size, int color) {
        paint.setStyle(Paint.Style.FILL); paint.setColor(color); paint.setTextSize(size);
        paint.setTypeface(android.graphics.Typeface.create("sans-serif", android.graphics.Typeface.NORMAL)); c.drawText(value, x, y, paint);
    }
    private String fit(String value, float maxWidth, float size) {
        paint.setTextSize(size); if (paint.measureText(value) <= maxWidth) return value;
        int n = value.length(); while (n > 0 && paint.measureText(value.substring(0, n) + "…") > maxWidth) n--;
        return value.substring(0, n) + "…";
    }
    private void rounded(Canvas c, RectF rect, int color, float radius) {
        paint.setStyle(Paint.Style.FILL); paint.setColor(color); c.drawRoundRect(rect, radius, radius, paint);
    }
    private void heading(Canvas c, String value, float x, float y, float size, int color) {
        paint.setStyle(Paint.Style.FILL); paint.setColor(color); paint.setTextSize(size);
        paint.setTypeface(android.graphics.Typeface.create("sans-serif-medium", android.graphics.Typeface.NORMAL));
        c.drawText(value, x, y, paint);
    }
    private void panel(Canvas c, RectF rect) {
        rounded(c, new RectF(rect.left, rect.top + 2, rect.right, rect.bottom + 2), Color.rgb(232, 232, 232), 18);
        rounded(c, rect, card, 18);
    }

    private void button(Canvas c, float x, float y, float w, float h, String key, String label, int bg, int fg) {
        RectF rect = new RectF(x, y, x + w, y + h); buttons.add(new Button(rect, key));
        long now = SystemClock.elapsedRealtime();
        boolean responding = key.equals(feedback.key) && feedback.active(now);
        c.save();
        if (responding) {
            float factor = feedback.scale(now);
            c.translate(rect.centerX(), rect.centerY()); c.scale(factor, factor);
            c.translate(-rect.centerX(), -rect.centerY());
        }
        rounded(c, rect, bg, 13);
        if (responding && feedback.opacity(now) > 0) {
            c.save(); android.graphics.Path mask = new android.graphics.Path();
            mask.addRoundRect(rect, 13, 13, android.graphics.Path.Direction.CW); c.clipPath(mask);
            paint.setStyle(Paint.Style.FILL);
            // All ripple channels are black; STOP retains its red base fill.
            paint.setColor(Color.argb((int)(255*feedback.opacity(now)),0,0,0));
            float fx = Math.max(rect.left, Math.min(rect.right, feedback.x));
            float fy = Math.max(rect.top, Math.min(rect.bottom, feedback.y));
            c.drawCircle(fx, fy, (float)Math.hypot(w,h) * feedback.ripple(now), paint); c.restore();
        }
        boolean semanticButton=key.startsWith("semantic_");
        if(semanticButton)label=fit(label,w-12,14);
        paint.setTextSize(semanticButton?14:18); paint.setColor(fg); paint.setStyle(Paint.Style.FILL);
        paint.setTypeface(android.graphics.Typeface.create("sans-serif-medium", android.graphics.Typeface.NORMAL));
        c.drawText(label, x + (w - paint.measureText(label)) / 2, y + h / 2 + 6, paint);
        c.restore();
    }
    private boolean selectedButton(String key) {
        switch (key) {
            case "arm": return armed;
            case "manual": return !hold && !rotating && !"navigation".equals(mode) && !"mapping".equals(mode) && !"semantic".equals(mode) && !"follow".equals(mode) && !mode.startsWith("rotate_");
            case "hold": return hold;
            case "nav_start": return "navigation".equals(mode);
            case "mapping": return mappingActive()&&!manualMappingActive();
            case "manual_mapping": return manualMappingActive();
            case "semantic_start": return semanticActive();
            case "nav_end": return key.equals(pendingPanelKey) && !panelMoved && !panelSwiping;
            case "follow": return "follow".equals(mode);
            case "cw": return (rotating && "cw".equals(pendingPanelKey)) || "rotate_cw".equals(mode);
            case "ccw": return (rotating && "ccw".equals(pendingPanelKey)) || "rotate_ccw".equals(mode);
            case "connect": return connected || key.equals(immediatePressedKey);
            case "settings": return key.equals(immediatePressedKey);
            case "zero": case "calibrate":
                return key.equals(pendingPanelKey) && !panelMoved && !panelSwiping;
            case "perception": return tab == 0;
            case "navigation": return tab == 1;
            default: return false;
        }
    }
    private boolean mappingActive() {
        return !semanticActive() && ("manual_mapping".equals(mode) || "mapping".equals(mode) || (connected && MappingPresentation.busy(state)));
    }
    private boolean manualMappingActive() { return !semanticActive() && ("manual_mapping".equals(mode) || (connected&&MappingPresentation.manualBusy(state))); }
    private boolean semanticActive() { return "semantic".equals(mode) || (connected && SemanticPresentation.active(state)); }
    private String navigationButtonLabel() { return "navigation".equals(mode)?"结束导航":"开始导航"; }
    private String mappingButtonLabel() { return mappingActive()&&!manualMappingActive()?"结束建图":MappingPresentation.retryLoad(state)?"重试载入":"探索建图"; }
    private void choiceButton(Canvas c, float x, float y, float w, float h, String key, String label) {
        boolean selected = selectedButton(key);
        int resting = "settings".equals(key) || "connect".equals(key) ? Color.rgb(220,220,220) : inactive;
        button(c, x, y, w, h, key, label, selected ? black : resting, selected ? Color.WHITE : ink);
    }
    private void drawStatus(Canvas c) {
        long now = SystemClock.elapsedRealtime();
        boolean mappingFresh = connected && now >= stateReceivedAt && now-stateReceivedAt <= 1000;
        String semantic=mappingFresh?SemanticPresentation.status(state):"";
        String display = !semantic.isEmpty()?semantic:tab == 1 && mappingFresh && MappingPresentation.loaded(state,navigationMap)
                ? "地图已载入 FPGA" : tab == 1 && mappingFresh && (MappingPresentation.busy(state)||MappingPresentation.terminal(state))
                ? MappingPresentation.status(state) : status;
        String label = (state.optBoolean("demo") ? "演示数据 · " : "") + display;
        if (label.length() == 0) label = connected ? "已连接" : "未连接";
        paint.setTypeface(android.graphics.Typeface.create("sans-serif", android.graphics.Typeface.NORMAL));
        float maxWidth = contentRect.width() - 8;
        label = fit(label, maxWidth - 24, 18);
        paint.setTextSize(18);
        float pillWidth = Math.min(maxWidth, paint.measureText(label) + 24);
        float left = contentRect.centerX() - pillWidth / 2;
        statusRect.set(left, 12, left + pillWidth, 52);
        rounded(c, statusRect, statusBackground, 12);
        text(c, label, statusRect.left + 12, statusRect.top + 26, 18, Color.WHITE);
    }

    @Override protected void onDraw(Canvas canvas) {
        super.onDraw(canvas); canvas.drawColor(background); canvas.save(); canvas.scale(scale, scale); buttons.clear();
        drawStatus(canvas);
        choiceButton(canvas, width - 174, 12, 72, 40, "settings", "设置");
        choiceButton(canvas, width - 92, 12, 72, 40, "connect", connected ? "断开" : "连接");
        if (tab == 0) drawPerception(canvas); else { drawNavigation(canvas); drawSonarFooter(canvas,null); }
        drawControls(canvas); drawTabs(canvas); canvas.restore(); postInvalidateDelayed(feedback.animating(SystemClock.elapsedRealtime()) || pageIndicatorAnimating ? 16 : 100);
    }
    private void drawPerception(Canvas c) {
        boolean haveFrame = frame != null && !frame.image.isRecycled();
        float iw = haveFrame ? frame.image.getWidth() : 1920, ih = haveFrame ? frame.image.getHeight() : 1080;
        float factor = contentRect.height() / ih;
        float imageWidth = iw * factor, imageLeft = contentRect.centerX() - imageWidth / 2;
        // Match navigation height with one uniform scale. Wide sources are clipped
        // at the sides; narrower sources remain centered without stretching.
        videoRect.set(Math.max(contentRect.left, imageLeft), contentRect.top,
                Math.min(contentRect.right, imageLeft + imageWidth), contentRect.bottom);
        panel(c, videoRect);
        if (!haveFrame) {
            imageRect.setEmpty(); text(c, "等待实时画面", videoRect.left + 25, videoRect.centerY() - 8, 25, ink);
            text(c, "双目 YOLO · 实时画面", videoRect.left + 25, videoRect.centerY() + 23, 16, muted);
        } else {
            // Keep the full projected image rect for detection coordinates; drawing
            // and hit testing are limited to the visible viewport, videoRect.
            imageRect.set(imageLeft, contentRect.top, imageLeft + imageWidth, contentRect.bottom);
            c.save(); c.clipRect(videoRect); paint.setStyle(Paint.Style.FILL); paint.setColor(Color.WHITE);
            c.drawBitmap(frame.image, null, imageRect, paint);
            JSONArray detections = frame.metadata.optJSONArray("display_detections");
            boolean stableLayer=detections!=null;
            if(!stableLayer)detections=frame.metadata.optJSONArray("detections");
            if ((stableLayer||selectable()) && detections != null) for (int i = 0; i < Math.min(128, detections.length()); i++) {
                JSONObject detection = detections.optJSONObject(i);
                if(stableLayer&&(detection==null||!ControlPolicy.displayable(SystemClock.elapsedRealtime()-frame.received,
                        frame.metadata.optLong("age_ms",0),displayAge(detection),
                        frame.metadata.optLong("display_hold_ms",0))))continue;
                RectF box = box(detection); if (box == null) continue;
                boolean chosen = detection.optLong("id", -2) == selected;
                int accent = chosen ? Color.rgb(255, 149, 0) : Color.rgb(215, 215, 215);
                paint.setColor(accent); paint.setStrokeWidth(chosen ? 3 : 2); paint.setStyle(Paint.Style.STROKE); c.drawRoundRect(box, 5, 5, paint);
                String label = detection.optString("label", "目标") + String.format(Locale.US, " %.0f%%", detection.optDouble("score") * 100);
                paint.setTextSize(13); float tw = paint.measureText(label), tagY = Math.max(videoRect.top, box.top - 22);
                rounded(c, new RectF(box.left, tagY, Math.min(videoRect.right, box.left + tw + 12), tagY + 21), chosen ? accent : Color.argb(220, 17, 17, 17), 4);
                text(c, label, box.left + 5, tagY + 15, 13, chosen ? Color.WHITE : accent);
            }
            c.restore();
        }
        JSONObject vision = state.optJSONObject("vision"); if (vision == null) vision = new JSONObject();
        String model = vision.optString("model", "等待模型信息").replace("-rk3588.rknn", "").replace(".rknn", "");
        String info = model + " · 采集 " + number(vision, "capture_fps") + " / 识别 " + number(vision, "result_fps") + " FPS";
        if (selected > 0) info = FollowPresentation.target(state, selected);
        else info += selectable() ? " · 点击目标框" : haveFrame ? " · 检测框暂不可选" : "";
        drawSonarFooter(c,info);
    }
    private void drawSonarFooter(Canvas c,String info) {
        String sonar=SonarPresentation.line(state,connected,stateReceivedAt,SystemClock.elapsedRealtime());
        float baseline=height-64;
        text(c,sonar,contentRect.left,baseline,12,ink);
        paint.setTextSize(12);float infoX=contentRect.left+paint.measureText(sonar)+16;
        if(info!=null&&contentRect.right-infoX>30)text(c,fit(info,contentRect.right-infoX,12),infoX,baseline,12,muted);
    }
    private String number(JSONObject object, String key) { return object.has(key) ? String.format(Locale.US, "%.1f", object.optDouble(key)) : "—"; }
    public void navigation(NavigationMap map) {
        if (map.available && navigationMap != null && navigationMap.sessionId.equals(map.sessionId) && map.revision < navigationMap.revision) return;
        if(pendingDestination!=null&&!pendingDestination.map.sessionId.equals(map.sessionId)) {
            pendingDestination=null;destinationStatus="地图已重置，请重新选择";
        }
        // Normally already prepared by NavigationMapClient. This fallback also
        // supports direct/local map delivery; odom-only updates share pixels.
        map.prepareDisplay(navigationMap);
        boolean rasterChanged=navigationMap==null||map.preparedDisplayPixels()!=navigationMap.preparedDisplayPixels();
        navigationMap = map; navigationError = map.available ? "" : "等待有效导航地图";
        if (rasterChanged && navigationBitmap != null) { navigationBitmap.recycle(); navigationBitmap = null; }
        if (map.available && (rasterChanged || navigationBitmap==null)) {
            int rw=map.bitmapWidth(),rh=map.bitmapHeight();
            int[] pixels = map.preparedDisplayPixels();
            navigationBitmap=android.graphics.Bitmap.createBitmap(pixels,rw,rh,android.graphics.Bitmap.Config.ARGB_8888);
        }
        invalidate();
    }
    public void navigationUnavailable(String reason) {
        mapPointer=-1;touchedMap=null;pendingDestination=null;destinationStatus="";navigationMap=null; navigationError=reason==null?"等待导航服务":reason;
        if(navigationBitmap!=null){navigationBitmap.recycle();navigationBitmap=null;} invalidate();
    }
    private float mapX(double x) { return mapRect.left+(float)((x-mapViewport.x)/mapViewport.span*mapRect.width()); }
    private float mapY(double y) { return mapRect.bottom-(float)((y-mapViewport.y)/mapViewport.span*mapRect.height()); }
    private void mapLine(Canvas c,double[][] points,int color,float stroke) {
        paint.setColor(color);paint.setStyle(Paint.Style.STROKE);paint.setStrokeWidth(stroke);
        for(int i=1;i<points.length;i++)c.drawLine(mapX(points[i-1][0]),mapY(points[i-1][1]),mapX(points[i][0]),mapY(points[i][1]),paint);
    }
    private void mapGoal(Canvas c,double x,double y) {
        float gx=mapX(x),gy=mapY(y);
        paint.setStyle(Paint.Style.FILL);paint.setColor(Color.rgb(70,190,255));
        c.drawCircle(gx,gy,11,paint);
        paint.setColor(Color.WHITE);c.drawCircle(gx,gy,6,paint);
    }
    private void mapCar(Canvas c,double[] pose) {
        float x=mapX(pose[0]),y=mapY(pose[1]);
        float length=16,width=14;
        // User-requested 180-degree icon correction relative to v1.2.32.
        // This is display-only: pose, TF, goals and motor commands are untouched.
        float fx=-(float)Math.cos(pose[2]),fy=(float)Math.sin(pose[2]),lx=-fy,ly=fx;
        android.graphics.Path arrow=new android.graphics.Path();
        arrow.moveTo(x+fx*length,y+fy*length);
        arrow.lineTo(x-fx*length*.7f+lx*width,y-fy*length*.7f+ly*width);
        arrow.lineTo(x-fx*length*.25f,y-fy*length*.25f);
        arrow.lineTo(x-fx*length*.7f-lx*width,y-fy*length*.7f-ly*width);arrow.close();
        paint.setStyle(Paint.Style.FILL);paint.setColor(Color.rgb(255,45,45));c.drawPath(arrow,paint);
        paint.setStyle(Paint.Style.STROKE);paint.setStrokeWidth(1.5f);paint.setColor(Color.rgb(220,36,36));c.drawPath(arrow,paint);
    }

    public void navigationResult(String result, NavigationTarget target) {
        pendingDestination=target;destinationStatus=result==null?"":result;status(destinationStatus);
    }
    private void drawNavigation(Canvas c) {
        imageRect.setEmpty();mapRect.setEmpty();panel(c,navigationRect);
        NavigationMap m=navigationMap;long now=SystemClock.elapsedRealtime();
        // Freeze the transform while a point is being selected; use its exact
        // displayed world transform on release, even if a new map arrives.
        if(mapPointer<0)mapViewport.update(m,now);
        if(mapPointer<0&&mapViewport.animating(now))postInvalidateOnAnimation();
        String label=m!=null&&m.fresh(now)?"导航 · "+String.format(Locale.US,"%.1f × %.1f m",mapViewport.span,mapViewport.span)
                +(m.localWindowConfirmed?" · 地图已载入 FPGA":""):"导航地图";
        text(c,fit(label,navigationRect.width()-224,18),navigationRect.left+14,navigationRect.top+25,18,ink);
        choiceButton(c,navigationRect.right-194,navigationRect.top+3,104,28,"manual_mapping",manualMappingActive()?"结束建图":"手动建图");
        if(m!=null&&m.fresh(now))choiceButton(c,navigationRect.right-80,navigationRect.top+3,66,28,"map_view",
                m.localWindowConfirmed&&!mapViewport.locallyFocused()?"局部":"全图");
        if(m==null||!m.fresh(now)||navigationBitmap==null){
            String message=m!=null&&!m.fresh(now)?"地图已过期，等待更新":navigationError;
            text(c,fit(message,navigationRect.width()-32,20),navigationRect.left+16,navigationRect.centerY(),20,muted);
            text(c,"启动 RK 导航程序后显示真实地图",navigationRect.left+16,navigationRect.centerY()+30,15,muted);
            return;
        }
        boolean poseFresh=m.poseFresh(now);
        float side=Math.max(1,Math.min(navigationRect.width()-16,navigationRect.height()-54));
        mapRect.set(navigationRect.centerX()-side/2,navigationRect.top+34,navigationRect.centerX()+side/2,navigationRect.top+34+side);
        c.save();c.clipRect(mapRect);paint.setStyle(Paint.Style.FILL);paint.setColor(Color.WHITE);paint.setFilterBitmap(true);
        c.drawColor(Color.rgb(24,24,24));
        if(m.raster==null){
            RectF dst=new RectF(mapX(m.originX),mapY(m.originY+m.height*m.resolution),mapX(m.originX+m.width*m.resolution),mapY(m.originY));
            c.drawBitmap(navigationBitmap,null,dst,paint);
        }
        else {
            NavigationMap.Raster r=m.raster;double co=Math.cos(r.yaw),si=Math.sin(r.yaw);
            double wx=r.width*r.resolution,hy=r.height*r.resolution;
            float[] src={0,0,navigationBitmap.getWidth(),0,0,navigationBitmap.getHeight()};
            float[] dst={mapX(r.x-si*hy),mapY(r.y+co*hy),
                mapX(r.x+co*wx-si*hy),mapY(r.y+si*wx+co*hy),mapX(r.x),mapY(r.y)};
            android.graphics.Matrix transform=new android.graphics.Matrix();
            if(transform.setPolyToPoly(src,0,dst,0,3))c.drawBitmap(navigationBitmap,transform,paint);
        }
        mapLine(c,m.trajectory,Color.rgb(142,142,142),2);mapLine(c,m.path,Color.rgb(70,190,255),5);
        if(m.goal!=null)mapGoal(c,m.goal[0],m.goal[1]);
        if(pendingDestination!=null&&m.sessionId.equals(pendingDestination.map.sessionId)) {
            mapGoal(c,pendingDestination.x,pendingDestination.y);
        }
        if(poseFresh) {
            JSONObject follow=state.optJSONObject("follow");
            if(connected&&now>=stateReceivedAt&&now-stateReceivedAt<=500&&follow!=null&&
                    follow.optBoolean("active",false)&&follow.optBoolean("distance_valid",false)) {
                double bx=follow.optDouble("target_x_body_m",Double.NaN),by=follow.optDouble("target_y_body_m",Double.NaN);
                double heading=m.pose[2]+Math.PI,co=Math.cos(heading),si=Math.sin(heading);
                double tx=m.pose[0]+co*bx-si*by,ty=m.pose[1]+si*bx+co*by;
                if(!Double.isNaN(tx)&&!Double.isInfinite(tx)&&!Double.isNaN(ty)&&!Double.isInfinite(ty)) {
                    paint.setStyle(Paint.Style.STROKE);paint.setStrokeWidth(2);paint.setColor(Color.rgb(245,154,35));
                    c.drawLine(mapX(m.pose[0]),mapY(m.pose[1]),mapX(tx),mapY(ty),paint);
                    c.drawCircle(mapX(tx),mapY(ty),10,paint);
                }
            }
            mapCar(c,m.pose);
        }
        c.restore();
        JSONObject navState=state.optJSONObject("navigation");
        String liveStatus=navState!=null&&navState.optBoolean("active")?NavigationTarget.reason(navState.optString("state","planning")):"";
        boolean mappingFresh=connected&&now>=stateReceivedAt&&now-stateReceivedAt<=1000;
        String completion=mappingFresh&&(MappingPresentation.finalizing(state)||MappingPresentation.terminal(state))
                &&(MappingPresentation.finalizing(state)||!MappingPresentation.loaded(state,m)||(pendingDestination==null&&liveStatus.isEmpty()))
                ?(MappingPresentation.loaded(state,m)?(armed?"点击地图选终点，再点“开始导航”":"地图已载入 FPGA · 请使能电机后选点"):MappingPresentation.status(state)):"";
        String hint=!completion.isEmpty()?completion:!poseFresh?"定位未就绪":!destinationStatus.isEmpty()?destinationStatus:!liveStatus.isEmpty()?liveStatus:
                (armed?(m.localWindowConfirmed?"点击地图选终点，再点“开始导航”":"探索建图请点击“探索建图”"):"先使能电机，再开始导航或点选目的地");
        text(c,fit(hint,navigationRect.width()-28,13),navigationRect.left+14,navigationRect.bottom-6,13,muted);
    }
    private void drawNavigationControls(Canvas c) {
        panel(c,controlRect);
        text(c,"导航控制",rightX+16,117,17,ink);
        float x=rightX+14,bw=(rightWidth-38)/2;
        choiceButton(c,x,132,bw,43,"arm",armed?"停用电机":"使能电机");
        choiceButton(c,x+bw+10,132,bw,43,"nav_start",navigationButtonLabel());
        choiceButton(c,x,183,bw,43,"mapping",mappingButtonLabel());
        choiceButton(c,x+bw+10,183,bw,43,"nav_end","结束导航");
        float inputWidth=rightWidth-134;
        choiceButton(c,x,235,inputWidth,27,"semantic_input",fit(semanticText.isEmpty()?"输入搜索目标…":semanticText,inputWidth-12,13));
        choiceButton(c,x+inputWidth+8,235,98,27,"semantic_start",semanticActive()?"结束搜索":"开始搜索");
    }
    private void drawControlPage(Canvas c, int page, float offset) {
        float x = rightX + 14 + offset, bw = (rightWidth - 38) / 2, x2 = x + bw + 10;
        text(c, page == 0 ? "驾驶控制" : "辅助功能", rightX + 16 + offset, 117, 17, ink);
        if (page == 0) {
            choiceButton(c, x, 138, bw, 49, "arm", armed ? "停用电机" : "使能电机");
            choiceButton(c, x2, 138, bw, 49, "nav_start", navigationButtonLabel());
            choiceButton(c, x, 197, bw, 49, "ccw", "按住逆时针");
            choiceButton(c, x2, 197, bw, 49, "cw", "按住顺时针");
        } else {
            choiceButton(c, x, 138, bw, 49, "mapping", mappingButtonLabel());
            choiceButton(c, x2, 138, bw, 49, "follow", "follow".equals(mode)?"取消跟随":"跟随目标");
            choiceButton(c, x, 197, bw, 49, "zero", "姿态归零");
            choiceButton(c, x2, 197, bw, 49, "calibrate", "静止校准");
        }
    }
    private void drawControls(Canvas c) {
        pageIndicatorAnimating=false;
        if(tab==1)drawNavigationControls(c);
        else { panel(c, controlRect);
        c.save(); c.clipRect(controlRect);
        drawControlPage(c, controlPage, panelDelta);
        if (panelSwiping && controlPage == 0 && panelDelta < 0) drawControlPage(c, 1, panelDelta + rightWidth);
        else if (panelSwiping && controlPage == 1 && panelDelta > 0) drawControlPage(c, 0, panelDelta - rightWidth);
        c.restore();
        drawPageIndicator(c);

        }
        button(c, rightX, 286, rightWidth, 54, "stop", "■   停车", red, Color.WHITE);
        panel(c, new RectF(rightX, 354, width - 20, height - 20));
        paint.setStyle(Paint.Style.FILL); paint.setColor(background); c.drawCircle(joystickX, joystickY, 70, paint);
        paint.setColor(separator); paint.setStyle(Paint.Style.STROKE); paint.setStrokeWidth(1.5f); c.drawCircle(joystickX, joystickY, 69, paint);
        c.drawLine(joystickX - 48, joystickY, joystickX + 48, joystickY, paint); c.drawLine(joystickX, joystickY - 48, joystickX, joystickY + 48, paint);
        paint.setStyle(Paint.Style.FILL); paint.setColor(armed ? black : Color.rgb(172, 172, 172)); c.drawCircle(joystickX + stickX * 42, joystickY + stickY * 42, 28, paint);
    }
    private void drawPageIndicator(Canvas c) {
        final float left = rightX + rightWidth / 2 - 10, y = controlRect.bottom - 10;
        // Both base dots remain visible. The active shape replaces the selected dot.
        paint.setStyle(Paint.Style.FILL); paint.setColor(Color.rgb(209,209,214));
        c.drawCircle(left, y, 3.2f, paint); c.drawCircle(left + 20, y, 3.2f, paint);
        PageIndicatorMotion.Shape shape = pageIndicator.frame(controlPage, panelSwiping,
                panelDelta/rightWidth, SystemClock.elapsedRealtime(), android.animation.ValueAnimator.areAnimatorsEnabled());
        pageIndicatorAnimating = shape.animate;
        if (shape.ghostAlpha > 0) {
            rounded(c,new RectF(left-2,y-2,left+22,y+2),Color.argb((int)(255*shape.ghostAlpha),125,125,134),2);
        }
        float tail=left+20*shape.tail, head=left+20*shape.head, radius=shape.radius;
        int active=Color.rgb(125,125,134);
        if (Math.abs(tail-head)<.01f) {
            paint.setStyle(Paint.Style.FILL);paint.setColor(active);c.drawCircle(head,y,radius,paint);
        } else rounded(c,new RectF(Math.min(tail,head)-radius,y-radius,
                Math.max(tail,head)+radius,y+radius),active,radius);
    }
    private void drawTabs(Canvas c) {
        rounded(c, new RectF(contentRect.left, height - 59, contentRect.right, height - 9), card, 18);
        float tw = Math.min(170, (contentRect.width() - 30) / 2), left = contentRect.centerX() - tw - 5;
        choiceButton(c, left, height - 53, tw, 38, "perception", "感知");
        choiceButton(c, left + tw + 10, height - 53, tw, 38, "navigation", "导航");
    }
    private boolean selectable() {
        if (frame == null || frame.image.isRecycled()) return false;
        long imageAge = frame.metadata.optLong("age_ms", 0);
        return ControlPolicy.selectable(SystemClock.elapsedRealtime() - frame.received, imageAge,
                frame.metadata.optLong("box_age_ms", imageAge), frame.metadata.optLong("box_max_age_ms", ControlPolicy.BOX_MAX_AGE_MS));
    }
    private long displayAge(JSONObject detection) {
        long age=detection.optLong("age_ms",Long.MAX_VALUE);
        long source=detection.optLong("source_mono_ms",0),send=frame.metadata.optLong("server_send_mono_ms",0);
        if(source>0&&send>=source)age=Math.max(age,send-source);
        return age;
    }
    private RectF box(JSONObject detection) {
        if (detection == null || frame == null || imageRect.isEmpty()) return null;
        JSONArray b = detection.optJSONArray("box"); if (b == null || b.length() != 4) return null;
        float x1 = (float)b.optDouble(0), y1 = (float)b.optDouble(1), x2 = (float)b.optDouble(2), y2 = (float)b.optDouble(3);
        if (!Float.isFinite(x1) || !Float.isFinite(y1) || !Float.isFinite(x2) || !Float.isFinite(y2) || x2 <= x1 || y2 <= y1) return null;
        float sx = imageRect.width() / frame.image.getWidth(), sy = imageRect.height() / frame.image.getHeight();
        float left = Math.max(videoRect.left, imageRect.left + x1 * sx), top = Math.max(videoRect.top, imageRect.top + y1 * sy);
        float right = Math.min(videoRect.right, imageRect.left + x2 * sx), bottom = Math.min(videoRect.bottom, imageRect.top + y2 * sy);
        return right > left && bottom > top ? new RectF(left, top, right, bottom) : null;
    }
    private void stopMotionForPanelSwipe() {
        // Browsing controls is not manual takeover of an autonomous mapping task.
        if(("mapping".equals(mode)||"semantic".equals(mode)||"follow".equals(mode)) && gesture.owner()<0 && !rotating)return;
        JSONObject motor = state.optJSONObject("motor");
        boolean active = gesture.owner() >= 0 || rotating || "follow".equals(mode) || mode.startsWith("rotate_") ||
                (selected <= 0 && motor != null && (Math.abs(motor.optDouble("left")) > .0001 || Math.abs(motor.optDouble("right")) > .0001));
        if (active) {
            boolean wasRotation = rotating; gesture.end(); rotating = false; stickX = stickY = 0;
            if (wasRotation) actions.releaseRotation(false); else actions.release(false);
            hold = false; mode = "manual"; status = "功能页滑动，运动已停止";
        }
    }
    private void beginPanelGesture(MotionEvent event, float x, float y) {
        clearPanelGesture(); panelPointer = event.getPointerId(0); panelDownX = x; panelDownY = y;
        for (Button button : buttons) if (button.rect.contains(x, y)) {
            pendingPanelKey = button.key;
            pendingPanelRect.set(button.rect.left, button.rect.top, button.rect.right, button.rect.bottom);
            break;
        }
        // Motion buttons wait for a genuine stationary hold. A page swipe
        // beginning over CW/CCW must never send a short rotation command.
        if ("cw".equals(pendingPanelKey) || "ccw".equals(pendingPanelKey)) {
            if (armed) postDelayed(rotationHold, ROTATION_HOLD_MS);
            else status("请先使能电机");
        }
    }
    private void panelTouch(MotionEvent event, int action) {
        int pointer = event.findPointerIndex(panelPointer);
        boolean release = action == MotionEvent.ACTION_UP ||
                (action == MotionEvent.ACTION_POINTER_UP && event.getPointerId(event.getActionIndex()) == panelPointer);
        if (pointer < 0) {
            boolean wasRotation = rotating; clearGesture();
            if (wasRotation) actions.releaseRotation(true); else actions.release(true);
            invalidate(); return;
        }
        float x = event.getX(pointer) / scale, y = event.getY(pointer) / scale;
        float dx = x - panelDownX, dy = y - panelDownY;
        if (action == MotionEvent.ACTION_MOVE) {
            if (Math.hypot(dx, dy) > 12) { feedback.up(SystemClock.elapsedRealtime()); panelMoved = true; removeCallbacks(rotationHold); }
            if (tab==0 && !panelSwiping && Math.abs(dx) > 18 && Math.abs(dx) > Math.abs(dy) * 1.25f) {
                panelSwiping = true; pendingPanelKey = null; removeCallbacks(rotationHold);
                stopMotionForPanelSwipe();
            }
            if (panelSwiping) {
                boolean towardOtherPage = controlPage == 0 ? dx < 0 : dx > 0;
                panelDelta = Math.max(-rightWidth, Math.min(rightWidth, towardOtherPage ? dx : dx * .12f));
            } else if (rotating && !rotationRect.contains(x, y)) {
                gesture.end(); rotating = false; actions.releaseRotation(false);
            }
            invalidate(); return;
        }
        if (release) {
            boolean swiped = panelSwiping, wasRotation = rotating;
            int next = controlPage;
            if (swiped && Math.abs(dx) >= rightWidth * .25f) {
                if (controlPage == 0 && dx < 0) next = 1;
                if (controlPage == 1 && dx > 0) next = 0;
            }
            String key = !panelMoved && pendingPanelKey != null && pendingPanelRect.contains(x, y) ? pendingPanelKey : null;
            float indicatorDistance = Math.abs(panelDelta) / rightWidth;
            clearPanelGesture();
            if (swiped) pageIndicator.settle(controlPage, next, indicatorDistance, SystemClock.elapsedRealtime(),
                    android.animation.ValueAnimator.areAnimatorsEnabled());
            if (wasRotation) { gesture.end(); rotating = false; actions.releaseRotation(false); }
            if (swiped) {
                controlPage = next; buttons.clear(); performClick(); invalidate();
            } else if (key != null && !"cw".equals(key) && !"ccw".equals(key)) click(key);
            else invalidate();
        }
    }
    private void feedbackDown(String key, float x, float y, int pointer) {
        feedback.down(key, x, y, SystemClock.elapsedRealtime(), android.animation.ValueAnimator.areAnimatorsEnabled());
        feedbackPointer = pointer; invalidate();
    }
    @Override public boolean onTouchEvent(MotionEvent event) {
        int action = event.getActionMasked(); float x = event.getX() / scale, y = event.getY() / scale;
        if (action == MotionEvent.ACTION_DOWN) {
            for (Button button : buttons) if (button.rect.contains(x,y)) {
                feedbackDown(button.key,x,y,event.getPointerId(0)); break;
            }
        }
        if (action == MotionEvent.ACTION_UP || action == MotionEvent.ACTION_CANCEL ||
                (action == MotionEvent.ACTION_POINTER_UP && event.getPointerId(event.getActionIndex()) == feedbackPointer)) {
            feedback.up(SystemClock.elapsedRealtime()); feedbackPointer = -1;
        }
        if (action == MotionEvent.ACTION_CANCEL) {
            mapPointer=-1;touchedMap=null;immediatePressedKey = null; clearPanelGesture(); gesture.cancel(); rotating = false; stickX = stickY = 0; actions.release(true);
            status("触控已取消，运动已暂停，使能保持"); invalidate(); return true;
        }
        // Secondary fingers never own a driving gesture or implicitly disarm it.
        // Explicitly pressing the red STOP remains available with either finger.
        // A multi-finger map/panel gesture is not a tap: suppress pending actions, but
        // let the original pointer keep an already active stick/rotation until it lifts.
        if (action == MotionEvent.ACTION_POINTER_DOWN) {
            int added = event.getActionIndex();
            float addedX = event.getX(added) / scale, addedY = event.getY(added) / scale;
            for (Button button : buttons) if ("stop".equals(button.key) && button.rect.contains(addedX, addedY)) {
                feedbackDown("stop",addedX,addedY,event.getPointerId(added));
                click("stop"); return true;
            }
            if (mapPointer >= 0) mapDragged = true;
            if (panelPointer >= 0 && !rotating) {
                panelMoved = true; pendingPanelKey = null; removeCallbacks(rotationHold);
            }
            return true;
        }
        if (gesture.isBlocked()) { if (action == MotionEvent.ACTION_UP) gesture.end(); return true; }
        if (panelPointer >= 0) { panelTouch(event, action); return true; }
        if(mapPointer>=0) {
            int pointer=event.findPointerIndex(mapPointer);
            if(pointer<0){mapPointer=-1;touchedMap=null;return true;}
            float mx=event.getX(pointer)/scale,my=event.getY(pointer)/scale;
            if(Math.hypot(mx-mapDownX,my-mapDownY)>10)mapDragged=true;
            if(action==MotionEvent.ACTION_UP || (action==MotionEvent.ACTION_POINTER_UP && event.getPointerId(event.getActionIndex())==mapPointer)) {
                NavigationMap snapshot=touchedMap;mapPointer=-1;touchedMap=null;
                long now=SystemClock.elapsedRealtime();
                if(!mapDragged&&now-mapDownAt<=500) {
                    if(!armed){status("请先使能电机，再选择目的地");return true;}
                    try {
                        if(navigationMap==null||!navigationMap.sessionId.equals(snapshot.sessionId))throw new IllegalArgumentException("地图已重置，请重新选择");
                        NavigationTarget target=NavigationTarget.fromScreen(snapshot,now,mx,my,mapTouchRect.left,mapTouchRect.top,mapTouchRect.right,mapTouchRect.bottom,touchViewX,touchViewY,touchViewSpan);
                        pendingDestination=target;destinationStatus="正在提交目的地…";hold=false;mode="navigation";
                        actions.navigate(target);status(destinationStatus);performClick();
                    } catch(IllegalArgumentException error){status(error.getMessage());}
                }
            }
            return true;
        }
        if (action == MotionEvent.ACTION_DOWN) {
            if (controlRect.contains(x, y)) { beginPanelGesture(event, x, y); return true; }
            for (Button button : buttons) if (button.rect.contains(x, y)) { immediatePressedKey = button.key; click(button.key); return true; }
            if (Math.hypot(x - joystickX, y - joystickY) <= 82) {
                if (!gesture.begin(event.getPointerId(0), armed)) { status("请先使能电机"); return true; }
                rotating = false; updateStick(x, y); return true;
            }
            if(tab==1&&mapRect.contains(x,y)&&navigationMap!=null) {
                if(mappingActive()){status("请先结束建图，再选择导航目的地");return true;}
                mapPointer=event.getPointerId(0);mapDownX=x;mapDownY=y;mapDownAt=SystemClock.elapsedRealtime();mapDragged=false;
                touchedMap=navigationMap;touchViewX=mapViewport.x;touchViewY=mapViewport.y;touchViewSpan=mapViewport.span;
                mapTouchRect.set(mapRect.left,mapRect.top,mapRect.right,mapRect.bottom);return true;
            }
            if (tab == 0 && selectable() && videoRect.contains(x, y)) {
                JSONArray detections = frame.metadata.optJSONArray("display_detections");
                if(detections==null)detections=frame.metadata.optJSONArray("detections");
                JSONObject best = null; float area = Float.MAX_VALUE;
                if (detections != null) for (int i = 0; i < Math.min(detections.length(), 128); i++) {
                    JSONObject detection = detections.optJSONObject(i);
                    if(detection==null||!detection.optBoolean("selectable",true))continue;
                    RectF rect = box(detection);
                    if (rect != null && rect.contains(x, y) && rect.width() * rect.height() < area) { best = detection; area = rect.width() * rect.height(); }
                }
                if (best != null && best.optLong("id", -1) > 0) {
                    // select() stops previous movement and makes the client manual
                    // immediately; mirror that before the next 100ms status tick.
                    selected = best.optLong("id"); hold = false; mode = "manual";
                    actions.select(frame.id, selected); status("正在确认目标 #" + selected);
                }
            }
            return true;
        }
        if (gesture.owner() >= 0) {
            int pointer = event.findPointerIndex(gesture.owner());
            boolean release = action == MotionEvent.ACTION_UP || (action == MotionEvent.ACTION_POINTER_UP && event.getPointerId(event.getActionIndex()) == gesture.owner());
            if (pointer < 0 || release) {
                boolean wasRotation = rotating; clearGesture();
                if (wasRotation) actions.releaseRotation(pointer < 0); else actions.release(pointer < 0);
                if (pointer < 0) { status = "控制手指已丢失，运动已暂停，使能保持"; }
                invalidate(); performClick();
            } else if (action == MotionEvent.ACTION_MOVE) updateStick(event.getX(pointer) / scale, event.getY(pointer) / scale);
        }
        if (action == MotionEvent.ACTION_UP) { immediatePressedKey = null; invalidate(); }
        return true;
    }
    private void updateStick(float x, float y) {
        stickX = (x - joystickX) / 52; stickY = (y - joystickY) / 52;
        float size = (float)Math.hypot(stickX, stickY); if (size > 1) { stickX /= size; stickY /= size; }
        pendingDestination=null;destinationStatus="";
        actions.stick(-(Math.abs(stickY) < .06 ? 0 : stickY), Math.abs(stickX) < .06 ? 0 : stickX, hold && imuFresh()); invalidate();
    }
    private void click(String key) {
        switch (key) {
            case "map_view":
                if(navigationMap!=null&&navigationMap.fresh(SystemClock.elapsedRealtime()))
                    mapViewport.toggleOverview(navigationMap,SystemClock.elapsedRealtime());
                break;
            case "semantic_input": clearGesture(); actions.semanticInput(); break;
            case "semantic_start": clearGesture(); if(semanticActive())actions.stop();else actions.startSemantic(); break;
            case "settings": clearGesture(); actions.settings(); break;
            case "connect": clearGesture(); actions.connect(); break;
            case "arm": clearGesture(); actions.arm(); break;
            case "stop": pendingDestination=null;destinationStatus="已停车";hold = false; clearGesture(); armed = false; actions.stop(); break;
            case "nav_start":
                if(mappingActive()){status("请先结束建图，再开始导航");break;}
                if("navigation".equals(mode)){click("nav_end");return;}
                if(!connected){status("请先连接 RK3588");break;}
                if(!armed){status("请先使能电机，再开始导航");break;}
                hold=false;clearGesture();pendingDestination=null;destinationStatus="正在开始导航…";
                actions.startNavigation();break;
            case "manual_mapping":
                if(semanticActive()){status("请先结束语义搜索，再手动建图");break;}
                if(manualMappingActive()){hold=false;clearGesture();pendingDestination=null;actions.endMapping();break;}
                if(mappingActive()){status("请先结束探索建图，再手动建图");break;}
                if(!connected){status("请先连接 RK3588");break;}
                if(!armed){status("请先使能电机，再手动建图");break;}
                hold=false;clearGesture();pendingDestination=null;actions.startManualMapping();break;
            case "mapping":
                if(manualMappingActive()){status("请先结束手动建图，再探索建图");break;}
                if(semanticActive()){status("请先结束语义搜索，再探索建图");break;}
                if(connected&&MappingPresentation.retryLoad(state)){hold=false;clearGesture();pendingDestination=null;
                    destinationStatus="重试载入已保存地图…";actions.endMapping();break;}
                if(mappingActive()){hold=false;clearGesture();pendingDestination=null;
                    destinationStatus="正在结束建图，等待 RK 确认…";actions.endMapping();break;}
                if(!connected){status("请先连接 RK3588");break;}
                if(!armed){status("请先使能电机，再探索建图");break;}
                hold=false;clearGesture();pendingDestination=null;
                actions.startMapping();break;
            case "nav_end":
                if(mappingActive()){actions.endMapping();break;}
                pendingDestination=null;destinationStatus="导航已结束，电机已停用";hold=false;mode="manual";
                clearGesture();armed=false;actions.endNavigation();break;
            case "manual": pendingDestination=null;destinationStatus="手动接管";hold = false; clearGesture(); actions.mode(false); break;
            case "hold":
                if (!imuFresh()) { hold = false; actions.mode(false); status = "IMU 尚未就绪，摇杆可继续手动控制"; }
                else { hold = !hold; actions.mode(hold); }
                break;
            case "follow":
                hold = false;
                mode="follow".equals(mode)?"manual":armed&&selected>0?"follow":mode;
                actions.follow(); break;
            case "zero": clearGesture(); actions.zero(); break;
            case "calibrate": clearGesture(); actions.calibrate(); break;
            case "perception": case "navigation":
                int next = "navigation".equals(key) ? 1 : 0;
                if (tab != next) {
                    boolean keepMapping = "mapping".equals(mode)||"semantic".equals(mode)||"follow".equals(mode);
                    clearGesture();
                    if(!keepMapping){actions.release(true);mode="manual";}
                    tab = next;pageIndicator.reset();pendingDestination=null;destinationStatus="";controlPage=0;buttons.clear();
                    status(keepMapping?("follow".equals(mode)?"目标跟随继续，保持 0.4 米":"semantic".equals(mode)?"语义搜索继续":"探索建图继续，可在导航页查看地图"):"页面已切换，运动已暂停，使能保持");
                }
                break;
        }
        performClick(); invalidate();
    }
    @Override public boolean performClick() { return super.performClick(); }
}
