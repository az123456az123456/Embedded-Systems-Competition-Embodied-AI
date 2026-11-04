package cn.acg138.robotphone;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.view.View;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import org.json.JSONObject;

public final class MainActivity extends Activity implements DashboardView.Actions {
    private final Handler ui = new Handler(Looper.getMainLooper());
    private DashboardView dashboard;
    private RobotClient client;
    private SharedPreferences settings;
    private int epoch;
    private boolean resumed;
    private final Object frameLock = new Object();
    private RobotClient.Frame pendingFrame;
    private boolean framePosted;
    private final Runnable statusTick = new Runnable() {
        @Override public void run() {
            if (dashboard != null) dashboard.flags(client != null && client.isArmed(), client == null ? "manual" : client.mode());
            ui.postDelayed(this, 100);
        }
    };
    @Override public void onCreate(Bundle saved) {
        super.onCreate(saved);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION |
                View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY | View.SYSTEM_UI_FLAG_LAYOUT_STABLE);
        settings = getSharedPreferences("robot_phone", MODE_PRIVATE);
        dashboard = new DashboardView(this, this); setContentView(dashboard);
        ui.post(statusTick);
    }
    @Override protected void onResume() { super.onResume(); resumed = true; }
    @Override protected void onPause() {
        resumed = false;
        disconnect();
        dashboard.status("已离开控制界面，电机停用；回来后请重新连接和使能");
        super.onPause();
    }
    @Override public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (client != null) {
            if (hasFocus && resumed) client.foreground();
            else { client.background(); dashboard.flags(false, "manual"); }
        }
    }
    @Override protected void onDestroy() {
        disconnect(); ui.removeCallbacksAndMessages(null); super.onDestroy();
    }
    private void disconnect() {
        epoch++;
        if (client != null) { client.close(); client = null; }
        synchronized (frameLock) {
            if (pendingFrame != null) { pendingFrame.image.recycle(); pendingFrame = null; }
        }
        if (dashboard != null) dashboard.connection(false);
    }
    @Override public void connect() {
        if (client != null && client.isConnected()) { disconnect(); dashboard.status("连接已断开，电机已停用"); return; }
        String token = settings.getString("token", "");
        if (token.length() < 8) { settings(); return; }
        disconnect(); dashboard.clearVideo();
        final int current = epoch;
        try {
            RobotClient created = new RobotClient(settings.getString("address", "http://192.168.1.100:8080"), token, new RobotClient.Listener() {
                private void post(Runnable action) { ui.post(() -> { if (epoch == current) action.run(); }); }
                @Override public void message(String text) { post(() -> dashboard.status(text)); }
                @Override public void connected(boolean value) { post(() -> dashboard.connection(value)); }
                @Override public void state(JSONObject state) { post(() -> dashboard.state(state)); }
                @Override public void frame(RobotClient.Frame frame) {
                    synchronized (frameLock) {
                        if (epoch != current) { frame.image.recycle(); return; }
                        if (pendingFrame != null) pendingFrame.image.recycle();
                        pendingFrame = frame;
                        if (framePosted) return;
                        framePosted = true;
                    }
                    ui.post(() -> {
                        RobotClient.Frame latest;
                        synchronized (frameLock) { latest = pendingFrame; pendingFrame = null; framePosted = false; }
                        if (latest != null) { if (epoch == current) dashboard.frame(latest); else latest.image.recycle(); }
                    });
                }
            });
            client = created; dashboard.status("正在连接板子…"); created.start();
        } catch (IllegalArgumentException ex) { dashboard.status(ex.getMessage()); }
    }
    @Override public void settings() {
        if (client != null) client.stop(true);
        LinearLayout form = new LinearLayout(this); form.setOrientation(LinearLayout.VERTICAL);
        int padding = (int)(20 * getResources().getDisplayMetrics().density); form.setPadding(padding, 6, padding, 6);
        addLabel(form, "RK3588 局域网地址（手机和板子连接同一路由器）");
        EditText address = new EditText(this); address.setSingleLine(); address.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        address.setText(settings.getString("address", "http://192.168.1.100:8080")); form.addView(address);
        addLabel(form, "配对码（板端启动时设置，至少 8 个字符）");
        EditText token = new EditText(this); token.setSingleLine(); token.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_PASSWORD);
        token.setText(settings.getString("token", "")); form.addView(token);
        addLabel(form, "左侧电机：默认 M1 左前、M2 左后；M3 右前、M4 右后");
        LinearLayout leftRow = new LinearLayout(this); form.addView(leftRow);
        CheckBox[] left = new CheckBox[4], invert = new CheckBox[4];
        int leftMask = settings.getInt("left_mask", 3), invertMask = settings.getInt("invert_mask", 12);
        for (int i = 0; i < 4; i++) { left[i] = new CheckBox(this); left[i].setText("M" + (i + 1)); left[i].setChecked((leftMask & (1 << i)) != 0); leftRow.addView(left[i]); }
        addLabel(form, "电机方向翻转（默认翻转 M3、M4）");
        LinearLayout invertRow = new LinearLayout(this); form.addView(invertRow);
        for (int i = 0; i < 4; i++) { invert[i] = new CheckBox(this); invert[i].setText("M" + (i + 1)); invert[i].setChecked((invertMask & (1 << i)) != 0); invertRow.addView(invert[i]); }
        addLabel(form, "仅用于可信局域网；设置、切后台或失焦都会停用电机。归零是角度参考归零，静止校准用于传感器偏置。首次运行须烧录配套遥控 FPGA 固件。");
        ScrollView scroll = new ScrollView(this); scroll.addView(form);
        AlertDialog dialog = new AlertDialog.Builder(this).setTitle("连接与电机设置").setView(scroll).setNegativeButton("取消", null).setPositiveButton("保存", null).create();
        dialog.setOnShowListener(ignored -> dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(v -> {
            try {
                String normalized = LanAddress.normalize(address.getText().toString());
                String pairing = token.getText().toString().trim();
                if (pairing.length() < 8 || pairing.length() > 128 || pairing.indexOf('\n') >= 0 || pairing.indexOf('\r') >= 0) throw new IllegalArgumentException("配对码需为 8–128 个字符");
                int lm = 0, im = 0;
                for (int i = 0; i < 4; i++) { if (left[i].isChecked()) lm |= 1 << i; if (invert[i].isChecked()) im |= 1 << i; }
                if (Integer.bitCount(lm) != 2) throw new IllegalArgumentException("请选择两个左侧电机");
                settings.edit().putString("address", normalized).putString("token", pairing).putInt("left_mask", lm).putInt("invert_mask", im).apply();
                disconnect(); dashboard.status("设置已保存，按“连接”开始"); dialog.dismiss();
            } catch (IllegalArgumentException ex) { token.setError(ex.getMessage()); }
        }));
        dialog.show();
    }
    private void addLabel(LinearLayout layout, String text) {
        TextView label = new TextView(this); label.setText(text); label.setTextSize(14); label.setPadding(0, 12, 0, 4); layout.addView(label);
    }
    @Override public void arm() {
        if (client == null || !client.isConnected()) { dashboard.status("请先连接 RK3588"); return; }
        if (client.isArmed()) { client.stop(true); dashboard.status("电机已停用"); }
        else client.arm(settings.getInt("left_mask", 3), settings.getInt("invert_mask", 12));
    }
    @Override public void stop() { if (client != null) client.stop(true); dashboard.status("停车，电机已停用；再次运动需重新使能"); }
    @Override public void follow() { if (client != null) client.follow(); }
    @Override public void mode(boolean hold) { if (client != null) client.setManualMode(hold); }
    @Override public void zero() { if (client != null) client.imuAction(false); dashboard.status("已停车，正在设置姿态零点"); }
    @Override public void calibrate() {
        if (client != null) client.stop(true);
        new AlertDialog.Builder(this).setTitle("静止校准 IMU").setMessage("将小车停在平稳位置并保持静止，校准期间不要移动。校准结束后需重新使能电机。")
                .setNegativeButton("取消", null).setPositiveButton("开始校准", (d, w) -> { if (client != null) { client.foreground(); client.imuAction(true); } }).show();
    }
    @Override public void stick(float throttle, float steer, boolean hold) { if (client != null) client.manual(throttle, steer, hold); }
    @Override public void release(boolean cancel) { if (client != null) client.releaseStick(cancel); }
    @Override public void select(long frameId, long targetId) { if (client != null) client.select(frameId, targetId); }
}
