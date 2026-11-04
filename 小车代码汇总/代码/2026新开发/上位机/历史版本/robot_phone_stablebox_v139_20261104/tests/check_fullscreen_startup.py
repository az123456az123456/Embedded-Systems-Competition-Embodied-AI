#!/usr/bin/env python3
"""Run the actual startup prefix/fullscreen method against a lazy-window model.

This is a bounded regression test, NOT an Android emulator. The failure contract
comes from AOSP PhoneWindow: getDecorView() installs decor, getInsetsController()
dereferences existing decor. The old source is retained as a negative control.
"""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build/fullscreen-startup"
BUILD.mkdir(parents=True, exist_ok=True)
REPORT = ROOT / "reports"
REPORT.mkdir(exist_ok=True)
source = (ROOT / "src/cn/acg138/robotphone/MainActivity.java").read_text()
old = json.loads((ROOT / "tests/fixtures/fullscreen-v1.2.21.json").read_text())
prefix = source[source.index("        super.onCreate(saved);"):source.index("        captureArchive = new CaptureArchive")]
method = source[source.index("    private void applyFullscreen() {"):source.index("    @Override protected void onDestroy()")]
assert "setContentView(dashboard);" in prefix
assert "safeContent" not in prefix

model = r'''
import java.util.*;
class Bundle {}
class DashboardView { DashboardView(Object context, Object actions) {} }
class WindowManager {
    static class LayoutParams {
        static final int FLAG_FULLSCREEN=1024;
        static final int LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES=1;
        int layoutInDisplayCutoutMode;
    }
}
class WindowInsets { static class Type { static int systemBars(){return 7;} } }
class WindowInsetsController {
    static final int BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE=2;
    int behavior, hidden, calls;
    void setSystemBarsBehavior(int b){behavior=b;}
    void hide(int mask){hidden=mask;calls++;}
}
class View {
    static final int SYSTEM_UI_FLAG_FULLSCREEN=4, SYSTEM_UI_FLAG_HIDE_NAVIGATION=2,
        SYSTEM_UI_FLAG_IMMERSIVE_STICKY=4096, SYSTEM_UI_FLAG_LAYOUT_STABLE=256,
        SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN=1024, SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION=512;
    int flags;
    WindowInsetsController controller;
    WindowInsetsController getWindowInsetsController(){return controller;}
    void setSystemUiVisibility(int flags){this.flags=flags;}
}
class Window {
    View decor; boolean controllerReady=true, fits=true; int flags, modernCalls;
    final WindowManager.LayoutParams attrs=new WindowManager.LayoutParams();
    final WindowInsetsController controller=new WindowInsetsController();
    final List<String> events=new ArrayList<>();
    View getDecorView(){
        if(decor==null){decor=new View();events.add("decor_created");}
        decor.controller=controllerReady?controller:null;return decor;
    }
    void addFlags(int f){flags|=f;}
    WindowManager.LayoutParams getAttributes(){return attrs;}
    void setAttributes(WindowManager.LayoutParams a){}
    void setDecorFitsSystemWindows(boolean f){modernCalls++;fits=f;}
    WindowInsetsController getInsetsController(){
        modernCalls++;events.add("controller_requested");
        // Same null-dereference behavior as AOSP PhoneWindow, not a safe fake.
        return decor.getWindowInsetsController();
    }
}
class ActivityModel {
    static final int MODE_PRIVATE=0;
    final Window window=new Window(); Object settings; DashboardView dashboard;
    void onCreate(Bundle saved){}
    Window getWindow(){return window;}
    Object getSharedPreferences(String n,int m){return new Object();}
    void setContentView(DashboardView d){window.getDecorView();window.events.add("content_installed");}
}
'''

def activity(name, startup, fullscreen):
    return ("class " + name + " extends ActivityModel {\n"
            "void start(){Bundle saved=new Bundle();\n" + startup + "}\n" + fullscreen +
            "void reapply(){applyFullscreen();}\n}\n")

runner = r'''
public class FullscreenStartupRegression {
    static int checks;
    static void check(boolean ok,String message){checks++;if(!ok)throw new AssertionError(message);}
    static void policy(Window w,int api){
        check(w.decor!=null,"decor exists");
        check((w.flags&1024)!=0,"fullscreen retained");
        if(api>=30){
            check(!w.fits,"edge-to-edge retained");
            check(w.controller.hidden==7,"system bars hidden");
            check(w.controller.behavior==2,"transient bars by swipe");
        }else{
            check(w.modernCalls==0,"no API30 call on legacy platform");
            check((w.decor.flags&5894)==5894,"legacy immersive/layout flags");
        }
        if(api>=28)check(w.attrs.layoutInDisplayCutoutMode==1,"cutout policy retained");
    }
    public static void main(String[] args){
        for(int api:new int[]{26,28,29,30,31,32,33,34,35,36}){
            android.os.Build.VERSION.SDK_INT=api;
            OldActivity old=new OldActivity();String oldResult="NO_EXCEPTION";
            try{old.start();check(api<30,"negative control must fail on cold API30+");}
            catch(NullPointerException expected){
                check(api>=30,"legacy negative control must not crash");
                check(old.window.decor==null,"failure precedes decor creation");
                oldResult="EXPECTED_NULL_DECOR_NPE";
            }
            FixedActivity fixed=new FixedActivity();fixed.start();policy(fixed.window,api);
            check(fixed.window.events.contains("content_installed"),"startup reaches content");
            if(api>=30)check(fixed.window.events.indexOf("content_installed")<
                fixed.window.events.indexOf("controller_requested"),"content before controller");
            for(int i=0;i<3;i++)fixed.reapply();policy(fixed.window,api);
            FixedActivity early=new FixedActivity();early.reapply();policy(early.window,api);
            if(api>=30){
                FixedActivity pending=new FixedActivity();pending.window.controllerReady=false;
                pending.start();check(pending.window.controller.calls==0,"null controller is tolerated");
                pending.window.controllerReady=true;pending.reapply();policy(pending.window,api);
            }
            System.out.println("{\"api_model\":"+api+",\"negative_control\":\""+oldResult+
                "\",\"cold_start\":\"PASS\",\"repeat_apply\":\"PASS\",\"early_apply\":\"PASS\","+
                "\"missing_controller\":\""+(api>=30?"PASS":"NOT_APPLICABLE")+"\"}");
        }
        System.out.println("{\"assertions\":"+checks+",\"status\":\"PASS\"}");
    }
}
'''
(BUILD / "android/os").mkdir(parents=True, exist_ok=True)
(BUILD / "android/os/Build.java").write_text("package android.os; public class Build {public static class VERSION {public static int SDK_INT;}}\n")
(BUILD / "FullscreenStartupRegression.java").write_text(model + activity("OldActivity", old["startup_prefix"], old["fullscreen_method"]) + activity("FixedActivity", prefix, method) + runner)
subprocess.run(["javac", "--release", "8", "-d", str(BUILD), str(BUILD / "android/os/Build.java"), str(BUILD / "FullscreenStartupRegression.java")], check=True)
result = subprocess.run(["java", "-cp", str(BUILD), "FullscreenStartupRegression"], text=True, capture_output=True)
(REPORT / "fullscreen-startup.log").write_text(result.stdout + result.stderr)
if result.returncode:
    raise SystemExit(result.returncode)
rows = [json.loads(line) for line in result.stdout.splitlines()]
report = {
    "scope": "Production startup prefix and fullscreen method; lazy-window host model, not Android runtime",
    "source_sha256": hashlib.sha256(source.encode()).hexdigest(),
    "negative_control_source_sha256": old["activity_sha256"],
    "framework_reference": "https://android.googlesource.com/platform/frameworks/base/+/2a9d43ccd30cf6c590f23f0c8169353cd0121d24/core/java/com/android/internal/policy/PhoneWindow.java",
    "cases": rows[:-1], "result": rows[-1],
    "not_run": ["Actual Android Activity startup", "OEM window manager", "Device logcat confirmation"],
}
(REPORT / "fullscreen-startup.json").write_text(json.dumps(report, indent=2) + "\n")
print(result.stdout, end="")
