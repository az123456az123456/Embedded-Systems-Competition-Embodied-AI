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

/** One native, touch-friendly landscape screen. All drawing is on Android Canvas. */
public final class DashboardView extends View {
    public interface Actions {
        void settings(); void connect(); void arm(); void stop(); void follow();
        void mode(boolean hold); void zero(); void calibrate();
        void stick(float throttle, float steer, boolean hold); void release(boolean cancel);
        void select(long frameId, long targetId);
    }
    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG | Paint.FILTER_BITMAP_FLAG);
    private final Actions actions;
    private final ArrayList<Button> buttons = new ArrayList<>();
    private final int background = Color.rgb(12, 19, 29), card = Color.rgb(23, 34, 47);
    private final int white = Color.rgb(234, 242, 248), muted = Color.rgb(142, 164, 183);
    private final int cyan = Color.rgb(71, 211, 202), red = Color.rgb(246, 94, 107);
    private RobotClient.Frame frame;
    private JSONObject state = new JSONObject();
    private final RectF videoRect = new RectF(), imageRect = new RectF();
    private float scale = 1, width = 1000, height = 540, joystickX, joystickY, stickX, stickY;
    private int pointer = -1;
    private boolean connected, armed, hold;
    private long selected = -1;
    private String status = "连接板子的局域网后，填写板子地址和配对码", mode = "manual";
    private static final class Button {
        final RectF rect; final String key, text; final int color;
        Button(RectF rect, String key, String text, int color) { this.rect = rect; this.key = key; this.text = text; this.color = color; }
    }
    public DashboardView(Context context, Actions actions) {
        super(context); this.actions = actions; setFocusable(true); setKeepScreenOn(true);
        setContentDescription("ACG138 小车遥控：视频目标、IMU姿态角、虚拟摇杆、停车与校准");
    }
    public void connection(boolean connected) { this.connected = connected; if (!connected) { armed = false; stickX = stickY = 0; } invalidate(); }
    public void flags(boolean armed, String mode) { this.armed = armed; this.mode = mode; this.hold = "hold".equals(mode); invalidate(); }
    public void status(String text) { status = text == null ? "" : text; invalidate(); }
    public void state(JSONObject value) { state = value; selected = state.optLong("selected_id", selected); invalidate(); }
    public void frame(RobotClient.Frame value) {
        RobotClient.Frame previous = frame; frame = value;
        if (previous != null && previous.image != value.image) previous.image.recycle();
        invalidate();
    }
    public void clearVideo() {
        if (frame != null) { frame.image.recycle(); frame = null; }
        selected = -1; invalidate();
    }
    @Override protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        scale = Math.min(w / 1000f, h / 540f);
        width = w / scale; height = h / scale;
        joystickX = 90; joystickY = height - 90;
        videoRect.set(18, 79, width * .68f - 8, height - 181);
    }
    private void text(Canvas c, String text, float x, float y, float size, int color) {
        paint.setStyle(Paint.Style.FILL); paint.setColor(color); paint.setTextSize(size);
        c.drawText(text, x, y, paint);
    }
    private void rounded(Canvas c, RectF rect, int color) {
        paint.setStyle(Paint.Style.FILL); paint.setColor(color); c.drawRoundRect(rect, 15, 15, paint);
    }
    private void button(Canvas c, float x, float y, float w, float h, String key, String label, int color) {
        RectF rect = new RectF(x, y, x + w, y + h); buttons.add(new Button(rect, key, label, color));
        rounded(c, rect, color); paint.setTextSize(17); paint.setColor(white);
        c.drawText(label, x + (w - paint.measureText(label)) / 2, y + h / 2 + 6, paint);
    }
    @Override protected void onDraw(Canvas canvas) {
        super.onDraw(canvas); canvas.drawColor(background); canvas.save(); canvas.scale(scale, scale); buttons.clear();
        text(canvas, "ACG138", 20, 35, 25, white);
        text(canvas, "小车控制台", 148, 34, 18, muted);
        paint.setColor(connected ? cyan : red); canvas.drawCircle(width - 250, 27, 5, paint);
        text(canvas, connected ? "已连接" : "未连接", width - 238, 33, 16, connected ? cyan : muted);
        button(canvas, width - 155, 10, 64, 35, "settings", "设置", card);
        button(canvas, width - 82, 10, 64, 35, "connect", connected ? "断开" : "连接", Color.rgb(29, 88, 103));
        String line = (state.optBoolean("demo") ? "演示数据 · " : "") + status;
        if (line.length() > 80) line = line.substring(0, 80);
        text(canvas, line, 20, 63, 15, muted);
        rounded(canvas, videoRect, Color.rgb(7, 13, 21));
        if (frame == null || frame.image.isRecycled()) {
            text(canvas, "等待实时画面", videoRect.left + 24, videoRect.centerY() - 5, 25, white);
            text(canvas, "点击识别框选择目标，再按“跟随目标”", videoRect.left + 24, videoRect.centerY() + 27, 16, muted);
            imageRect.setEmpty();
        } else {
            float s = Math.min(videoRect.width() / frame.image.getWidth(), videoRect.height() / frame.image.getHeight());
            float iw = frame.image.getWidth() * s, ih = frame.image.getHeight() * s;
            imageRect.set(videoRect.centerX() - iw / 2, videoRect.centerY() - ih / 2, videoRect.centerX() + iw / 2, videoRect.centerY() + ih / 2);
            canvas.save(); canvas.clipRect(videoRect); canvas.drawBitmap(frame.image, null, imageRect, paint);
            JSONArray detections = frame.metadata.optJSONArray("detections");
            if (detections != null) for (int i = 0; i < Math.min(128, detections.length()); i++) {
                JSONObject d = detections.optJSONObject(i); RectF box = box(d);
                if (box == null) continue;
                boolean chosen = d.optLong("id", -2) == selected;
                paint.setColor(chosen ? Color.rgb(255, 211, 109) : cyan); paint.setStrokeWidth(chosen ? 3 : 2); paint.setStyle(Paint.Style.STROKE);
                canvas.drawRoundRect(box, 5, 5, paint);
                String label = d.optString("label", "target") + String.format(Locale.US, " %.0f%%", d.optDouble("score") * 100);
                paint.setStyle(Paint.Style.FILL); paint.setTextSize(14); float tw = paint.measureText(label);
                RectF tag = new RectF(box.left, Math.max(imageRect.top, box.top - 23), Math.min(imageRect.right, box.left + tw + 12), Math.max(imageRect.top, box.top - 23) + 22);
                paint.setColor(Color.argb(220, 8, 21, 31)); canvas.drawRoundRect(tag, 4, 4, paint);
                text(canvas, label, tag.left + 5, tag.top + 16, 14, chosen ? Color.rgb(255, 211, 109) : white);
            }
            canvas.restore();
            long age = SystemClock.elapsedRealtime() - frame.received + Math.max(0, frame.metadata.optLong("age_ms", 0));
            String quality = age < 1000 ? "LIVE  ·  点击目标框" : "画面已过期 · 暂停选框";
            text(canvas, quality, videoRect.left + 13, videoRect.bottom - 13, 14, age < 1000 ? cyan : red);
        }
        float rx = width * .68f + 8, rw = width - rx - 18;
        RectF imuRect = new RectF(rx, 79, width - 18, height - 181); rounded(canvas, imuRect, card);
        JSONObject imu = state.optJSONObject("imu"); if (imu == null) imu = new JSONObject();
        text(canvas, "IMU 姿态", rx + 18, 108, 19, white);
        text(canvas, imu.optBoolean("valid") ? "实时" : "等待数据", rx + rw - 85, 108, 13, imu.optBoolean("valid") ? cyan : muted);
        float available = Math.max(180, imuRect.height());
        float rowY = 143, row = Math.min(45, (available - 90) / 4);
        String[] names = {"横滚 Roll", "俯仰 Pitch", "航向 Yaw"}; String[] keys = {"roll", "pitch", "yaw"};
        for (int i = 0; i < 3; i++) {
            text(canvas, names[i], rx + 18, rowY + row * i, 15, muted);
            text(canvas, imu.optBoolean("valid") ? String.format(Locale.US, "%+.1f°", imu.optDouble(keys[i])) : "—", rx + rw - 108, rowY + row * i + 2, 26, white);
        }
        JSONArray acceleration = imu.optJSONArray("accel");
        text(canvas, "加速度 (m/s²)", rx + 18, imuRect.bottom - 56, 13, muted);
        String a = acceleration == null ? "—  /  —  /  —" : String.format(Locale.US, "%+.2f   %+.2f   %+.2f", acceleration.optDouble(0), acceleration.optDouble(1), acceleration.optDouble(2));
        text(canvas, a, rx + 18, imuRect.bottom - 32, 17, white);
        JSONObject motor = state.optJSONObject("motor"); if (motor == null) motor = new JSONObject();
        text(canvas, (armed ? "电机已使能" : "电机已停用") + (motor.optBoolean("link_ok") ? "  ·  FPGA在线" : "  ·  FPGA待连接"), rx + 18, imuRect.bottom - 11, 13, armed ? cyan : muted);
        paint.setColor(card); paint.setStyle(Paint.Style.FILL); canvas.drawCircle(joystickX, joystickY, 65, paint);
        paint.setColor(Color.rgb(48, 69, 87)); paint.setStyle(Paint.Style.STROKE); paint.setStrokeWidth(1.5f); canvas.drawCircle(joystickX, joystickY, 63, paint);
        canvas.drawLine(joystickX - 48, joystickY, joystickX + 48, joystickY, paint); canvas.drawLine(joystickX, joystickY - 48, joystickX, joystickY + 48, paint);
        paint.setStyle(Paint.Style.FILL); paint.setColor(armed ? cyan : Color.rgb(92, 113, 130)); canvas.drawCircle(joystickX + stickX * 45, joystickY + stickY * 45, 25, paint);
        text(canvas, "松手即停", joystickX - 32, height - 12, 13, muted);
        float bx = 182, by = height - 153, bw = Math.min(124, (width - 402) / 4);
        button(canvas, bx, by, bw, 45, "arm", armed ? "停用电机" : "使能电机", armed ? Color.rgb(130, 82, 43) : Color.rgb(27, 103, 101));
        button(canvas, bx + bw + 10, by, bw, 45, "manual", "手动", "manual".equals(mode) ? Color.rgb(34, 79, 98) : card);
        button(canvas, bx + 2 * (bw + 10), by, bw, 45, "hold", "航向保持", hold ? Color.rgb(34, 79, 98) : card);
        button(canvas, bx + 3 * (bw + 10), by, bw, 45, "follow", "跟随目标", "follow".equals(mode) ? Color.rgb(34, 79, 98) : card);
        button(canvas, bx, by + 57, bw, 42, "zero", "姿态归零", card);
        button(canvas, bx + bw + 10, by + 57, bw, 42, "calibrate", "静止校准", card);
        text(canvas, selected <= 0 ? "未选择目标" : "选中目标 #" + selected, bx + 2 * (bw + 10), by + 83, 16, muted);
        button(canvas, width - 185, height - 152, 167, 104, "stop", "■  停车", Color.rgb(170, 48, 63));
        text(canvas, "控制模式：" + ("follow".equals(mode) ? "目标跟随" : hold ? "航向保持" : "手动差速") + "  ·  切到后台自动停用", bx, height - 22, 14, muted);
        canvas.restore();
        postInvalidateDelayed(100);
    }
    private RectF box(JSONObject d) {
        if (d == null || frame == null || imageRect.isEmpty()) return null;
        JSONArray b = d.optJSONArray("box"); if (b == null || b.length() != 4) return null;
        float x1 = (float)b.optDouble(0), y1 = (float)b.optDouble(1), x2 = (float)b.optDouble(2), y2 = (float)b.optDouble(3);
        if (!Float.isFinite(x1) || !Float.isFinite(y1) || !Float.isFinite(x2) || !Float.isFinite(y2) || x2 <= x1 || y2 <= y1) return null;
        float sx = imageRect.width() / frame.image.getWidth(), sy = imageRect.height() / frame.image.getHeight();
        return new RectF(imageRect.left + Math.max(0, x1) * sx, imageRect.top + Math.max(0, y1) * sy,
                imageRect.left + Math.min(frame.image.getWidth(), x2) * sx, imageRect.top + Math.min(frame.image.getHeight(), y2) * sy);
    }
    @Override public boolean onTouchEvent(MotionEvent event) {
        int action = event.getActionMasked();
        float x = event.getX() / scale, y = event.getY() / scale;
        if (action == MotionEvent.ACTION_CANCEL) {
            pointer = -1; stickX = stickY = 0; actions.release(true); invalidate(); return true;
        }
        if (action == MotionEvent.ACTION_POINTER_DOWN) {
            int index = event.getActionIndex();
            float px = event.getX(index) / scale, py = event.getY(index) / scale;
            // STOP remains usable with a second finger while holding the joystick.
            for (Button b : buttons) if ("stop".equals(b.key) && b.rect.contains(px, py)) { click("stop"); return true; }
        }
        if (action == MotionEvent.ACTION_DOWN) {
            if (Math.hypot(x - joystickX, y - joystickY) <= 83) {
                if (!armed) { status("请先使能电机"); return true; }
                pointer = event.getPointerId(0); updateStick(x, y); return true;
            }
            for (Button b : buttons) if (b.rect.contains(x, y)) { click(b.key); return true; }
            if (frame != null && SystemClock.elapsedRealtime() - frame.received + Math.max(0, frame.metadata.optLong("age_ms", 0)) < 700 && imageRect.contains(x, y)) {
                JSONArray ds = frame.metadata.optJSONArray("detections"); JSONObject best = null; float area = Float.MAX_VALUE;
                if (ds != null) for (int i = 0; i < Math.min(ds.length(), 128); i++) {
                    JSONObject d = ds.optJSONObject(i); RectF b = box(d);
                    if (b != null && b.contains(x, y) && b.width() * b.height() < area) { best = d; area = b.width() * b.height(); }
                }
                if (best != null) {
                    selected = best.optLong("id", -1);
                    if (selected >= 0) { actions.select(frame.id, selected); status("已选择目标 #" + selected + "，按“跟随目标”开始"); }
                }
            }
            return true;
        }
        if (pointer >= 0) {
            int i = event.findPointerIndex(pointer);
            if (i < 0) {
                pointer = -1; stickX = stickY = 0; actions.release(true); invalidate();
            } else if (action == MotionEvent.ACTION_UP || (action == MotionEvent.ACTION_POINTER_UP && event.getPointerId(event.getActionIndex()) == pointer)) {
                pointer = -1; stickX = stickY = 0; actions.release(false); invalidate(); performClick();
            } else if (action == MotionEvent.ACTION_MOVE) updateStick(event.getX(i) / scale, event.getY(i) / scale);
        }
        return true;
    }
    private void updateStick(float x, float y) {
        stickX = (x - joystickX) / 60; stickY = (y - joystickY) / 60;
        float size = (float)Math.hypot(stickX, stickY); if (size > 1) { stickX /= size; stickY /= size; }
        float tx = Math.abs(stickX) < .06 ? 0 : stickX, ty = Math.abs(stickY) < .06 ? 0 : stickY;
        actions.stick(-ty, tx, hold); invalidate();
    }
    private void click(String key) {
        switch (key) {
            case "settings": actions.settings(); break;
            case "connect": actions.connect(); break;
            case "arm": actions.arm(); break;
            case "stop": hold = false; pointer = -1; stickX = stickY = 0; actions.stop(); break;
            case "follow": hold = false; actions.follow(); break;
            case "manual": hold = false; actions.mode(false); break;
            case "hold": hold = !hold; actions.mode(hold); break;
            case "zero": actions.zero(); break;
            case "calibrate": actions.calibrate(); break;
        }
        performClick(); invalidate();
    }
    @Override public boolean performClick() { return super.performClick(); }
}
