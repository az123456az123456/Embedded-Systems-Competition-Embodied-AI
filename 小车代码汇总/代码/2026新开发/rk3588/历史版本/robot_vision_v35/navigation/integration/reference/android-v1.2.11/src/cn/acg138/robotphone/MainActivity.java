package cn.acg138.robotphone;

import android.app.Activity;
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
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION |
                View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY | View.SYSTEM_UI_FLAG_LAYOUT_STABLE);
        settings = getSharedPreferences("robot_phone", MODE_PRIVATE);
        dashboard = new DashboardView(this, this); setContentView(dashboard);
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
        ui.removeCallbacks(captureDayTick); ui.post(captureDayTick);
    }
    @Override protected void onPause() {
        resumed = false;
        ui.removeCallbacks(captureDayTick);
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
        disconnect();
        if (captureArchive != null) captureArchive.close();
        ui.removeCallbacksAndMessages(null); super.onDestroy();
    }
    private void disconnect() {
        epoch++;
        if (captureArchive != null) captureArchive.newSession();
        if (navigationClient != null) { navigationClient.close(); navigationClient = null; }
        if (dashboard != null) dashboard.navigationUnavailable("未连接导航服务");
        if (client != null) { client.close(); client = null; }
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
                        if (latest != null) { if (epoch == current) dashboard.frame(latest); else latest.image.recycle(); }
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
        if (client != null) client.stop(true);
        LinearLayout form = new LinearLayout(this); form.setOrientation(LinearLayout.VERTICAL);
        int padding = (int)(20 * getResources().getDisplayMetrics().density); form.setPadding(padding, 6, padding, 6);
        addLabel(form, "RK3588 地址（与手机同一局域网或手机热点）");
        EditText address = new EditText(this); address.setSingleLine(); address.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        address.setText(settings.getString("address", "http://192.168.1.100:8080")); form.addView(address);
        addLabel(form, "配对码（新版本 6 位数字，也兼容原长配对码）");
        EditText token = new EditText(this); token.setSingleLine(); token.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_PASSWORD);
        token.setText(settings.getString("token", "")); form.addView(token);
        addLabel(form, "v15/v16 轮组：M1右前、M2左后、M3右后、M4左前。左侧 M2、M4");
        LinearLayout leftRow = new LinearLayout(this); form.addView(leftRow);
        CheckBox[] left = new CheckBox[4], invert = new CheckBox[4];
        int leftMask = settings.getInt("left_mask", ControlPolicy.LEFT_MASK), invertMask = settings.getInt("invert_mask", ControlPolicy.INVERT_MASK);
        for (int i = 0; i < 4; i++) { left[i] = new CheckBox(this); left[i].setText("M" + (i + 1)); left[i].setChecked((leftMask & (1 << i)) != 0); leftRow.addView(left[i]); }
        addLabel(form, "额外方向翻转（v15/v16 验证配置为四个全勾选；升级保留现有设置）");
        LinearLayout invertRow = new LinearLayout(this); form.addView(invertRow);
        for (int i = 0; i < 4; i++) { invert[i] = new CheckBox(this); invert[i].setText("M" + (i + 1)); invert[i].setChecked((invertMask & (1 << i)) != 0); invertRow.addView(invert[i]); }
        addLabel(form, client == null ? "目标角度由 RK3588 计算，手机只发送选框与跟随操作。" : client.followDiagnostics());
        addLabel(form, "no_vest 自动截图：持续识别超过 1 秒，每次出现保存 1 张；按手机日期建立文件夹。");
        captureDirectoryLabel = new TextView(this); captureDirectoryLabel.setTextSize(14); form.addView(captureDirectoryLabel);
        Button chooseDirectory = new Button(this); chooseDirectory.setText("选择截图保存文件夹"); form.addView(chooseDirectory);
        chooseDirectory.setOnClickListener(v -> chooseCaptureDirectory());
        captureEnabledBox = new CheckBox(this); captureEnabledBox.setText("启用 no_vest 自动截图"); form.addView(captureEnabledBox);
        refreshCaptureDirectory();
        addLabel(form, "仅用于可信局域网；设置、切后台或失焦都会停用电机。归零是角度参考归零，静止校准用于传感器偏置。首次运行须烧录配套遥控 FPGA 固件。");
        ScrollView scroll = new ScrollView(this); scroll.addView(form);
        AlertDialog dialog = new AlertDialog.Builder(this).setTitle("连接与电机设置").setView(scroll).setNegativeButton("取消", null).setPositiveButton("保存", null).create();
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
        if (client != null) client.stop(true);
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
    private void addLabel(LinearLayout layout, String text) {
        TextView label = new TextView(this); label.setText(text); label.setTextSize(14); label.setPadding(0, 12, 0, 4); layout.addView(label);
    }
    @Override public void arm() {
        if (client == null || !client.isConnected()) { dashboard.status("请先连接 RK3588"); return; }
        if (client.isArmed()) { client.stop(true); dashboard.status("电机已停用"); }
        else client.arm(settings.getInt("left_mask", ControlPolicy.LEFT_MASK), settings.getInt("invert_mask", ControlPolicy.INVERT_MASK));
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
    @Override public void rotate(boolean clockwise) { if (client != null) client.rotate(clockwise); }
    @Override public void releaseRotation(boolean cancel) { if (client != null) client.releaseRotation(cancel); }
    @Override public void startNavigation() { if(client!=null)client.startNavigation();else dashboard.navigationResult("请先连接 RK3588",null); }
    @Override public void endNavigation() { if(client!=null)client.endNavigation();dashboard.navigationResult("导航已结束，电机已停用",null); }
    @Override public void navigate(NavigationTarget target) { if(client!=null)client.navigate(target);else dashboard.navigationResult("请先连接 RK3588",null); }
    @Override public void select(long frameId, long targetId) { if (client != null) client.select(frameId, targetId); }
}
