# ACG138 原生 Android 小车控制器

> 日期：2026-10-15；最低 Android 8.0（API 26），编译/目标 API 35。

## 📱 安装与连接

将 `dist/ACG138-Robot-Control-debug.apk` 复制到安卓手机，点击 APK 安装；系统询问时允许当前文件管理器安装此应用。手机和 RK3588 要在能互相访问的同一局域网中。板子通过系统已识别的 USB 无线网卡接入路由器，手机接该路由器 Wi-Fi。

启动配套 RK3588 服务端，在本应用右上角“设置”填写板子地址（例如 `http://192.168.1.100:8080`）和板端设置的配对码，再点击“连接”。配对码至少 8 个字符。手机不需要浏览器、Python、RKNN 模型或摄像头权限。

这是原生 Java Activity + Android Canvas 应用，不使用 WebView，也不从手机执行模型推理。模型推理和 IMU 姿态计算仍在 RK3588，FPGA 负责电机 PWM 和通信超时停车。**旧自动电机测试固件不接收遥控指令，必须配合本次提供的遥控固件。**

## 🎮 操作

| 操作 | 行为 |
|---|---|
| 使能电机 | 只使能，不自动行走；默认 M1/M2 为左侧，M3/M4 为右侧，方向翻转 M3/M4 |
| 虚拟摇杆 | 上下控制前后、左右按差速转向；正常松手立即发送零指令并保留使能 |
| 航向保持 | 打开后推动摇杆前后，以 IMU 航向作修正；松手仍停车 |
| 点击框 → 跟随目标 | 必须先使能，再点击当前画面的框，再点击跟随；选中本身不触发运动 |
| 停车 | 发送停止并停用电机；之后必须重新使能 |
| 姿态归零 | 停车并设置当前显示 R/P/Y 参考零点；不消除重力 |
| 静止校准 | 停车，按提示保持小车静止，采集陀螺仪偏置；结束后重新使能 |
| 切后台、失焦、触控取消、网络故障 | 停止发运动租约并尽力发送停止/停用；不会恢复自动运动 |

检测框和 JPEG 在同一个 multipart part 中携带，点击时发送该图像的 `frame_id` 及稳定目标 `id`。显示使用视频原坐标缩放到实际图像矩形，保留黑边，避免点击位置与框错位。过期帧禁止选择。加速度单位为 m/s²，包含重力。

## 🔗 连接约定

- `POST /api/connect`：令牌认证，获得新会话；连接不会使能电机。
- `GET /api/state`：独立线程读取 IMU、状态与模式。
- `GET /api/video.mjpg`：独立线程解码 JPEG，part 头含 `Content-Length`、`X-Frame-Id`、Base64 UTF-8 JSON `X-Frame-Meta`。
- `POST /api/control`：独立串行指令线程，20 Hz 运动租约，使用会话及递增序号。已使能但没有运动请求时使用 `idle` 维持控制者会话，保留已选择的目标，且不延长任何旧运动租约。
- 全部请求使用 `Authorization: Bearer 配对码`；仅允许私有 IPv4 局域网 HTTP 地址，不跟随重定向。HTTP 配对码用于可信局域网，不等同加密传输。

输入端点、配对码和轮组设置只保存在应用私有配置中，关闭 Android 备份。首次使用不会默认连接、使能或行驶。若旧控制会话尚未过期，服务端会拒绝抢占。

## 🛠️ 从源码构建

使用 JDK 17 和 [Google 官方 Android SDK](https://developer.android.com/studio) 的 `platforms;android-35`、`build-tools;35.0.0`，设置：

```sh
export ANDROID_SDK_ROOT=/实际的/Android/Sdk
./test-host.sh
./build-apk.sh
```

也可单独指定 `BUILD_TOOLS` 和 `ANDROID_JAR`。构建不需要 Gradle 或第三方 Java 依赖，依次使用官方 aapt2、javac、d8、zipalign、apksigner。可安装 APK、签名校验和清单在 `dist/`。

首次构建自动创建本地开发签名 `.local/debug.keystore`（标准开发密码），请保留以便以后覆盖升级。源码归档不分发私钥；换机器重新签名后，需卸载之前的开发版再安装。不要用开发签名发布应用商店。

命令行构建与 APK 签名依据：[Android 构建说明](https://developer.android.com/build/building-cmdline)、[apksigner](https://developer.android.com/tools/apksigner)。API 35 的局域网访问使用 INTERNET 权限；参见 [Android 本地网络权限说明](https://developer.android.com/privacy-and-security/local-network-permission)。

## ✅ 验证范围

已编译原生 APK 并验证 APK v2/v3 签名和 zipalign。主机测试覆盖局域网地址限制，multipart 长度、UTF-8 元数据、截断、非法帧和连续帧，以及选框确认和并发状态轮询竞态。选框必须等待服务端确认后才能跟随；跟随中重新选框先取消旧运动意图，必须再按一次跟随。具体结果见 `dist/verification.json`。

本环境没有连接安卓手机或 Android 模拟器，尚未进行手机安装、触屏操作及 USB 网卡/电机实物联调；编译成功不代表实物功能已验收。
