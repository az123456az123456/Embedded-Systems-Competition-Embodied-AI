package cn.acg138.robotphone;

import android.app.Activity;
import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.view.Gravity;
import android.view.WindowInsets;
import android.view.WindowInsetsController;
import android.content.res.Configuration;
import android.app.AlertDialog;
import android.content.SharedPreferences;
import android.content.Intent;
import android.net.Uri;
import android.os.SystemClock;
import android.provider.DocumentsContract;
import android.widget.Button;
import java.util.TimeZone;
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
    private NavigationMapClient navigationClient;
    private SharedPreferences settings;
    private CaptureArchive captureArchive;
    private SafCaptureStore captureStore;
    private Uri captureStoreTree;
    private final Runnable captureDayTick = new Runnable() {
        @Override public void run() {
            if (!resumed) return;
            if (captureArchive != null) captureArchive.ensureCurrentDay();
            ui.postDelayed(this, 1000);
        }
    };
    private TextView captureDirectoryLabel;
    private CheckBox captureEnabledBox;
    private static final int CHOOSE_CAPTURE_DIRECTORY = 13826;
    private volatile int epoch;
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
        settings = getSharedPreferences("robot_phone", MODE_PRIVATE);
        dashboard = new DashboardView(this, this);
        // Match the original app: the dashboard owns the whole content window.
        // Insets must not pad/offset a second parent around its canvas and touch coordinates.
        setContentView(dashboard);
        applyFullscreen();
        captureArchive = new CaptureArchive(new CaptureArchive.Clock() {
            public long wallMillis() { return System.currentTimeMillis(); }
            public long elapsedMillis() { return SystemClock.elapsedRealtime(); }
            public TimeZone timeZone() { return TimeZone.getDefault(); }
        }, new CaptureArchive.Listener() {
            public void saved(String path) { ui.post(() -> dashboard.status("no_vest 已保存：" + path)); }
            public void failed(String error) { ui.post(() -> dashboard.status("截图未保存：" + error)); }
        });
        configureCaptureArchive();
        // A UI/map upgrade must never replace an already tested user's wheel settings.
        if (!settings.contains("left_mask") || !settings.contains("invert_mask")) {
            SharedPreferences.Editor edit = settings.edit();
            if (!settings.contains("left_mask")) edit.putInt("left_mask", ControlPolicy.LEFT_MASK);
            if (!settings.contains("invert_mask")) edit.putInt("invert_mask", ControlPolicy.INVERT_MASK);
            edit.putInt("motor_layout_revision", ControlPolicy.SETTINGS_REVISION).apply();
            dashboard.status("v16 默认：M1右前、M2左后、M3右后、M4左前；请核对轮组设置");
        }
        ui.post(statusTick);
    }
    @Override protected void onResume() {
        super.onResume(); resumed = true;
        applyFullscreen();
        if (client != null) client.foreground();
        ui.removeCallbacks(captureDayTick); ui.post(captureDayTick);
    }
    @Override protected void onPause() {
        resumed = false;
        ui.removeCallbacks(captureDayTick);
        if (client != null) client.background();
        dashboard.status("控制界面暂离，运动已暂停；30秒无指令后电机使能失效");
        super.onPause();
    }
    @Override public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) applyFullscreen();
        if (client != null) {
            client.windowFocusChanged(hasFocus && resumed);
            if (!hasFocus) dashboard.flags(client.isArmed(), "manual");
        }
    }
    @Override public void onConfigurationChanged(Configuration configuration) {
        super.onConfigurationChanged(configuration);
        applyFullscreen();
    }
    private void applyFullscreen() {
        // PhoneWindow.getInsetsController() dereferences its lazily created decor.
        // Initialize it first, including calls made before attachment on Android 11+.
        View decor = getWindow().getDecorView();
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        if (android.os.Build.VERSION.SDK_INT >= 28) {
            WindowManager.LayoutParams attrs = getWindow().getAttributes();
            attrs.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES;
            getWindow().setAttributes(attrs);
        }
        if (android.os.Build.VERSION.SDK_INT >= 30) {
            getWindow().setDecorFitsSystemWindows(false);
            WindowInsetsController controller = getWindow().getInsetsController();
            if (controller != null) {
                controller.setSystemBarsBehavior(WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
                controller.hide(WindowInsets.Type.systemBars());
            }
        } else {
            decor.setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN |
                    View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY |
                    View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN |
                    View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION);
        }
    }
    @Override protected void onDestroy() {
        disconnect();
        if (captureArchive != null) captureArchive.close();
        ui.removeCallbacksAndMessages(null); super.onDestroy();
    }
    private void disconnect() { disconnect("connection_closed"); }
    private void disconnect(String reason) {
        getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        epoch++;
        if (captureArchive != null) captureArchive.newSession();
        if (navigationClient != null) { navigationClient.close(); navigationClient = null; }
        if (dashboard != null) dashboard.navigationUnavailable("未连接导航服务");
        if (client != null) { client.close(reason); client = null; }
        synchronized (frameLock) {
            if (pendingFrame != null) { pendingFrame.image.recycle(); pendingFrame = null; }
        }
        if (dashboard != null) dashboard.connection(false);
    }
    @Override public void connect() {
        if (client != null && client.isConnected()) { disconnect(); dashboard.status("连接已断开，电机已停用"); return; }
        String token = settings.getString("token", "");
        if (!ControlPolicy.validPairing(token)) { settings(); return; }
        disconnect(); dashboard.clearVideo();
        final int current = epoch;
        try {
            RobotClient created = new RobotClient(settings.getString("address", "http://192.168.1.100:8080"), token, new RobotClient.Listener() {
                private void post(Runnable action) { ui.post(() -> { if (epoch == current) action.run(); }); }
                @Override public void message(String text) { post(() -> dashboard.status(text)); }
                @Override public void navigation(String text,NavigationTarget target) { post(() -> dashboard.navigationResult(text,target)); }
                @Override public void connected(boolean value) { post(() -> {
                    if (!value && captureArchive != null) captureArchive.newSession();
                    dashboard.connection(value);
                    if (value && resumed) getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
                    else getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
                    if (navigationClient != null) {
                        if (value) navigationClient.start();
                        else { navigationClient.close(); dashboard.navigationUnavailable("连接中断，等待更新"); }
                    }
                }); }
                @Override public void state(JSONObject state) {
                    post(() -> dashboard.state(state, client == null ? 0 : client.selectedTarget()));
                }
                @Override public void frame(RobotClient.Frame frame) {
                    synchronized (frameLock) {
                        if (epoch != current) { frame.image.recycle(); return; }
                        if (captureArchive != null) captureArchive.frame(frame.metadata, frame.jpeg, frame.id, frame.received);
                        if (pendingFrame != null) pendingFrame.image.recycle();
                        pendingFrame = frame;
                        if (framePosted) return;
                        framePosted = true;
                    }
                    ui.post(() -> {
                        RobotClient.Frame latest;
                        synchronized (frameLock) { latest = pendingFrame; pendingFrame = null; framePosted = false; }
                        if (latest != null) { if (epoch == current && android.os.SystemClock.elapsedRealtime()-latest.received <= 250) dashboard.frame(latest); else latest.image.recycle(); }
                    });
                }
            });
            navigationClient = new NavigationMapClient(settings.getString("address", "http://192.168.1.100:8080"), token,
                    new NavigationMapClient.Listener() {
                        public void map(NavigationMap map) { ui.post(() -> { if (epoch == current) dashboard.navigation(map); }); }
                        public void unavailable(String reason) { ui.post(() -> { if (epoch == current) dashboard.navigationUnavailable(reason); }); }
                    });
            client = created; dashboard.status("正在连接板子…"); created.start();
        } catch (IllegalArgumentException ex) { dashboard.status(ex.getMessage()); }
    }
    @Override public void settings() {
        if (client != null) client.pauseMotion("settings_open");
        LinearLayout form = new LinearLayout(this); form.setOrientation(LinearLayout.VERTICAL);
        form.setBackgroundColor(Color.rgb(242,242,247));
        int padding = (int)(20 * getResources().getDisplayMetrics().density); form.setPadding(padding, 6, padding, 6);
        addSection(form, "连接小车", "手机与小车连接同一 Wi-Fi 或热点。");
        addLabel(form, "小车地址");
        EditText address = new EditText(this); address.setSingleLine(); address.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        address.setText(settings.getString("address", "http://192.168.1.100:8080")); styleInput(address); form.addView(address);
        addLabel(form, "配对码 · 6 位数字或已有配对码");
        EditText token = new EditText(this); token.setSingleLine(); token.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_PASSWORD);
        token.setText(settings.getString("token", "")); styleInput(token); form.addView(token);
        addSection(form, "电机配置", "升级保留当前配置，请仅在核对轮组后修改。");
        addLabel(form, "左侧电机 · 选择两项");
        LinearLayout leftRow = new LinearLayout(this); form.addView(leftRow);
        CheckBox[] left = new CheckBox[4], invert = new CheckBox[4];
        int leftMask = settings.getInt("left_mask", ControlPolicy.LEFT_MASK), invertMask = settings.getInt("invert_mask", ControlPolicy.INVERT_MASK);
        for (int i = 0; i < 4; i++) { left[i] = new CheckBox(this); left[i].setText("M" + (i + 1)); left[i].setMinHeight(dp(48)); left[i].setChecked((leftMask & (1 << i)) != 0); leftRow.addView(left[i]); }
        addLabel(form, "方向翻转 · 保留已验证的设置");
        LinearLayout invertRow = new LinearLayout(this); form.addView(invertRow);
        for (int i = 0; i < 4; i++) { invert[i] = new CheckBox(this); invert[i].setText("M" + (i + 1)); invert[i].setMinHeight(dp(48)); invert[i].setChecked((invertMask & (1 << i)) != 0); invertRow.addView(invert[i]); }
        
        addSection(form, "自动截图", "no_vest 持续出现超过 1 秒保存一张；按手机日期归档。");
        captureDirectoryLabel = new TextView(this); captureDirectoryLabel.setTextSize(14); captureDirectoryLabel.setTextColor(Color.rgb(99,99,108)); captureDirectoryLabel.setPadding(0,dp(8),0,dp(8)); form.addView(captureDirectoryLabel);
        Button chooseDirectory = new Button(this); chooseDirectory.setText("选择保存位置"); chooseDirectory.setMinHeight(dp(48)); form.addView(chooseDirectory);
        chooseDirectory.setOnClickListener(v -> chooseCaptureDirectory());
        captureEnabledBox = new CheckBox(this); captureEnabledBox.setText("启用自动截图"); captureEnabledBox.setMinHeight(dp(48)); form.addView(captureEnabledBox);
        refreshCaptureDirectory();
        addSection(form, "关于控制", "打开设置会暂停运动。请在完成设置后返回控制台操作。");
        addLabel(form, client == null ? "目标跟随由小车计算；手机负责选择目标与遥控。" : client.followDiagnostics());
        ScrollView scroll = new ScrollView(this); scroll.addView(form);
        AlertDialog dialog = new AlertDialog.Builder(this).setTitle("设置").setView(scroll).setNegativeButton("取消", null).setPositiveButton("保存", null).create();
        dialog.setOnShowListener(ignored -> dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(v -> {
            try {
                String normalized = LanAddress.normalize(address.getText().toString());
                String pairing = token.getText().toString().trim();
                if (!ControlPolicy.validPairing(pairing)) throw new IllegalArgumentException("请输入 6 位数字，或原有的 8–128 位配对码");
                int lm = 0, im = 0;
                for (int i = 0; i < 4; i++) { if (left[i].isChecked()) lm |= 1 << i; if (invert[i].isChecked()) im |= 1 << i; }
                if (Integer.bitCount(lm) != 2) throw new IllegalArgumentException("请选择两个左侧电机");
                settings.edit().putString("address", normalized).putString("token", pairing).putInt("left_mask", lm).putInt("invert_mask", im)
                        .putBoolean("capture_no_vest", captureEnabledBox.isChecked() && hasCaptureGrant()).apply();
                configureCaptureArchive();
                disconnect(); dashboard.status("设置已保存，按“连接”开始"); dialog.dismiss();
            } catch (IllegalArgumentException ex) { token.setError(ex.getMessage()); }
        }));
        dialog.show();
        if (dialog.getWindow() != null) {
            dialog.getWindow().setGravity(Gravity.BOTTOM);
            GradientDrawable sheet = new GradientDrawable(); sheet.setColor(Color.rgb(242,242,247)); sheet.setCornerRadius(dp(24));
            dialog.getWindow().setBackgroundDrawable(sheet);
            dialog.getWindow().setDimAmount(.28f);
            dialog.getWindow().setLayout(Math.min(getResources().getDisplayMetrics().widthPixels - dp(32), dp(680)),
                    (int)(getResources().getDisplayMetrics().heightPixels * .88f));
        }
    }
    private Uri captureTree() {
        String value = settings.getString("capture_tree", "");
        try { return value.length() == 0 ? null : Uri.parse(value); } catch (RuntimeException invalid) { return null; }
    }
    private boolean hasCaptureGrant() {
        try { return SafCaptureStore.hasGrant(getContentResolver(), captureTree()); }
        catch (RuntimeException inaccessible) { return false; }
    }
    private void configureCaptureArchive() {
        CaptureArchive.Store destination = null;
        if (hasCaptureGrant()) {
            Uri selected = captureTree();
            try {
                if (captureStore == null || !selected.equals(captureStoreTree)) {
                    captureStore = new SafCaptureStore(this, selected); captureStoreTree = selected;
                }
                destination = captureStore;
            } catch (IllegalArgumentException ignored) { captureStore = null; captureStoreTree = null; }
        } else { captureStore = null; captureStoreTree = null; }
        captureArchive.configure(destination, settings.getBoolean("capture_no_vest", false));
        if (resumed) captureArchive.ensureCurrentDay();
    }
    private String captureLocation() {
        // A tree's document ID is part of the URI, so presenting a local path never
        // queries a possibly slow DocumentsProvider on the UI/control thread.
        Uri tree = captureTree();
        try {
            String id = DocumentsContract.getTreeDocumentId(tree);
            return id.startsWith("primary:") ? "手机存储/" + id.substring(8) : id;
        } catch (RuntimeException invalid) { return tree == null ? "未选择目录" : tree.toString(); }
    }
    private void refreshCaptureDirectory() {
        boolean granted = hasCaptureGrant();
        if (captureDirectoryLabel != null) captureDirectoryLabel.setText(granted ?
                "保存目录：" + captureLocation() + "（已授权）" : "未选择目录或授权已失效，请先选择文件夹");
        if (captureEnabledBox != null) {
            captureEnabledBox.setEnabled(granted);
            captureEnabledBox.setChecked(granted && settings.getBoolean("capture_no_vest", false));
        }
    }
    private void chooseCaptureDirectory() {
        if (client != null) client.pauseMotion("capture_folder_picker");
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT_TREE);
        intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION |
                Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION | Intent.FLAG_GRANT_PREFIX_URI_PERMISSION);
        Uri previous = captureTree(); if (previous != null) intent.putExtra(DocumentsContract.EXTRA_INITIAL_URI, previous);
        try { startActivityForResult(intent, CHOOSE_CAPTURE_DIRECTORY); }
        catch (RuntimeException unavailable) { dashboard.status("无法打开系统文件夹选择器"); }
    }
    @Override protected void onActivityResult(int request, int result, Intent data) {
        super.onActivityResult(request, result, data);
        if (request != CHOOSE_CAPTURE_DIRECTORY || result != RESULT_OK || data == null || data.getData() == null) return;
        Uri tree = data.getData();
        try {
            int flags = data.getFlags() & (Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION);
            if ((flags & (Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION)) !=
                    (Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION))
                throw new IllegalArgumentException("所选目录没有读写授权");
            getContentResolver().takePersistableUriPermission(tree, flags);
            if (!SafCaptureStore.hasGrant(getContentResolver(), tree)) throw new IllegalArgumentException("目录授权未能保留");
            settings.edit().putString("capture_tree", tree.toString()).apply();
            configureCaptureArchive(); refreshCaptureDirectory();
            dashboard.status("截图目录已授权；在设置中勾选自动截图后保存");
        } catch (RuntimeException failure) { dashboard.status("截图目录选择失败：" + failure.getMessage()); }
    }
    private int dp(int value) { return Math.round(value * getResources().getDisplayMetrics().density); }
    private void styleInput(EditText input) {
        input.setTextSize(17); input.setTextColor(Color.rgb(20,20,20)); input.setMinHeight(dp(48));
        input.setPadding(dp(12),dp(10),dp(12),dp(10));
        GradientDrawable background = new GradientDrawable(); background.setColor(Color.WHITE); background.setCornerRadius(dp(12));
        input.setBackground(background);
    }
    private void addSection(LinearLayout layout, String title, String detail) {
        TextView heading = new TextView(this); heading.setText(title); heading.setTextSize(20);
        heading.setTypeface(android.graphics.Typeface.create("sans-serif-medium", android.graphics.Typeface.NORMAL));
        heading.setTextColor(Color.rgb(20,20,20)); heading.setPadding(0,dp(24),0,dp(4)); layout.addView(heading);
        addLabel(layout, detail);
    }
    private void addLabel(LinearLayout layout, String text) {
        TextView label = new TextView(this); label.setText(text); label.setTextSize(14); label.setTextColor(Color.rgb(99,99,108)); label.setPadding(0, dp(8), 0, dp(6)); layout.addView(label);
    }
    @Override public void arm() {
        if (client == null || !client.isConnected()) { dashboard.status("请先连接 RK3588"); return; }
        if (client.isArmed()) { client.stop(true, "arm_toggle_off"); dashboard.status("电机已停用"); }
        else client.arm(settings.getInt("left_mask", ControlPolicy.LEFT_MASK), settings.getInt("invert_mask", ControlPolicy.INVERT_MASK));
    }
    @Override public void stop() { if (client != null) client.stop(true); dashboard.status("停车，电机已停用；再次运动需重新使能"); }
    @Override public void follow() { if (client != null) client.follow(); }
    @Override public void mode(boolean hold) { if (client != null) client.setManualMode(hold); }
    @Override public void zero() { if (client != null) client.imuAction(false); dashboard.status("已停车，正在设置姿态零点"); }
    @Override public void calibrate() {
        if (client != null) client.pauseMotion("imu_calibration_dialog");
        new AlertDialog.Builder(this).setTitle("静止校准 IMU").setMessage("将小车停在平稳位置并保持静止，校准期间不要移动。校准结束后需重新使能电机。")
                .setNegativeButton("取消", null).setPositiveButton("开始校准", (d, w) -> { if (client != null) { client.foreground(); client.imuAction(true); } }).show();
    }
    @Override public void stick(float throttle, float steer, boolean hold) { if (client != null) client.manual(throttle, steer, hold); }
    @Override public void release(boolean cancel) { if (client != null) client.releaseStick(cancel); }
    @Override public void rotate(boolean clockwise) { if (client != null) client.rotate(clockwise); }
    @Override public void releaseRotation(boolean cancel) { if (client != null) client.releaseRotation(cancel); }
    @Override public void startNavigation() { if(client!=null)client.startNavigation();else dashboard.navigationResult("请先连接 RK3588",null); }
    @Override public void startMapping() {
        if(client!=null){client.startMapping();dashboard.flags(client.isArmed(),client.mode());}
        else dashboard.navigationResult("请先连接 RK3588",null);
    }
    @Override public void endMapping() {
        if(client!=null){client.endMapping();dashboard.flags(client.isArmed(),client.mode());}
        else dashboard.navigationResult("请先连接 RK3588",null);
    }
    @Override public void endNavigation() { if(client!=null)client.endNavigation();dashboard.navigationResult("导航已结束，电机已停用",null); }
    @Override public void navigate(NavigationTarget target) { if(client!=null)client.navigate(target);else dashboard.navigationResult("请先连接 RK3588",null); }
    @Override public void select(long frameId, long targetId) { if (client != null) client.select(frameId, targetId); }
}
