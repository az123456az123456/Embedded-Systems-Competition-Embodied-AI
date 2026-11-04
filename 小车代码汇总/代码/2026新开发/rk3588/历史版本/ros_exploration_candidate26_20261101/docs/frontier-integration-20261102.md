# Frontier 探索执行链更新（2026-11-02）

本轮将现有 Frontier 选点与停车传图、FPGA 执行及手机状态串联。修改已部署到 RK 独立目录，两个 ROS 包编译成功；按用户“改好后不测试”要求，未启动节点、未发送运动指令。此文说明实现与编译结果，不代表自主探索实机验收通过。

## 探索流程

1. 手机“探索建图”经现有 HTTP/Gateway 和 MAPPING_START 开始会话；不凭 SLAM 启动就认定车辆已探索。
2. SLAM 产生首张局部可用地图。确认停车并取消旧任务后，下发保守膨胀的 64×64、0.1m/格地图，等待 FPGA COMMIT 与状态对账。
3. `acg138_frontier/acg138_frontier/core.py:65` 的 `select_frontier()` 找四邻接的自由/未知边界，聚类后选择可达目标。自由阈值为 0～25；最小簇 3 格；最小进展 0.25m；局部接近目标默认距边界不超过 0.8m。评分结合局部连通距离、离边界距离和簇规模，不把质心直接放进障碍。
4. 目标限制在已确认局部图的可达自由格内。全局边界在窗口外时仅选择明确标识的窗口内中间目标，不裁剪坐标；FPGA 仍负责 A*、跟踪、超声避障及到达。
5. 到达后取消旧任务，确认停车，再加载下一张局部图并选择下一点。没有 Frontier 为 NO_FRONTIER；有 Frontier 但不可达是错误，不虚报探索完成。用户结束后停车、保存最终地图，再加载固定图。

未引入 ROS 速度控制、RK 路径执行器或无图随机行驶。未引入 roadmap_explorer 依赖；未声称任何 25%～45% 提速。

## 本次修复

- `acg138_fpga_map_bridge/src/nav3_wire.cpp:375`：仅对首次 MAP_BEGIN 的 BUSY 拒绝作同一完整事务重试，间隔 50ms、上限 1500ms；等待中继续 CANCEL/POSE 服务。不重试 BUSY CHUNK/COMMIT 或 START。超时失败后不继续该地图事务。
- FPGA 只读依据：`小车代码汇总/代码/2026新开发/fpga/candidate-26-live-factor/source/softcore/firmware/nav_exec.c:43` 中 `map_writable()` 要求 IDLE/CANCELLED、无 owner/resume、external_quiet、map_quiet、A* 空闲；第 48 行先拒 BUSY，再修改地图暂存数据。此逻辑未修改。
- `acg138_frontier/acg138_frontier/node.py`：按原始 generation 和源单调时间戳输出真实探索状态；传图失败显式 ERROR；等待停车传图有 15s 阶段上限，其他 SELECTING 阶段 5s。手动接管、过期和取消不自动续行。
- `acg138_frontier/acg138_frontier/mapping_node.py`：只投影匹配本轮 generation、源年龄不超过 250ms 的探索状态到现有 `/mapping/state`，保留建图生命周期字段。手机现有状态渲染无需更换 APK。
- `deploy/run-stack.sh:51`：三个原 `/tmp` 日志路径链接到 `/acg138-ros2/data/logs/integration`，保留重启前证据；启动时对超过 8MiB 的日志轮换。保留此前相机准备完成再建立 ROS 套接字的顺序。

## 已确认与未确认

历史日志可确认 MAP_BEGIN 返回 status=10/BUSY，之后 RK 的 snapshot 日志重复了失败状态，不能据此推断 FPGA 被反复写图。每次 BUSY 的具体物理成因仍未确认。证据：`reports/board-current/exploration-persisted-evidence.console`。此前用户日志 free=7，不能据此保证存在可达前沿；没有放宽膨胀或强制标记自由格。

本次编译：RK 上 colcon 两包构建，exit=0，耗时 39.0s；见 `reports/board-current/frontier-execution-chain-build-status.console`。安装后的两个 Python 文件与源码逐字节一致；见 `reports/board-current/frontier-compile-only-export.console`。编译不等于功能测试。

FPGA RTL/固件/位流、500ms/30s/250ms/60ms 保护、CPR 60527.7 与方向 (-1,1,-1,1) 均未改。YOLO 相机输入通路、每眼 640×640、三 NPU worker、完整同帧双目才供跟随等要求保留。

## 启动与回滚

当前板上独立目录已更新。退出旧栈后运行：

```sh
sh /root/robot-fpga-integration-20261102/deploy/run-stack.sh --phone-fps auto --motor-interface eth0 --imu-interface eth0
```

启动默认空闲，不恢复旧运动；手机沿用原配对和当前 RK Wi-Fi 地址。通过既有使能链路取得授权后，点击“探索建图”。结束建图后只有 COMMIT 对账成功才显示“地图已载入 FPGA”。

本次备份位于 `/root/model-validation-last-20261102/before-frontier-execution-chain-fix/`：`overlay-before.tar.gz` 与旧 `run-stack.sh`。退出本套进程后方可回滚，同一时刻只运行一套；无需重烧 FPGA。

## 剩余验收

- 新代码自主探索移动：NOT_RUN（用户要求不测试）。
- 新代码 MAP_BEGIN BUSY 恢复、状态投影、阶段超时：NOT_RUN，仅编译通过。
- 真实选点开始/到达/结束：历史静态地图起点阻塞，尚未闭环通过。
- 手机 Android 11+ 真机、视频延迟、落地跟踪避障：未验收。历史 PASS 仅适用于其原始版本和测试条件。
