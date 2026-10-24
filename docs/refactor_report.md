> 本文为上一轮重构记录；当前布局、HOLD 行为与检查结果见[最新报告](layout_tracking_review.md)。

# 系统重构与验证报告（2026-10-22）

> 本报告及其 JSON 保存重构完成时的验证快照。随后已修复本机 x86_64 原生构建并实际执行原先跳过的测试；最新原生加载/离线结果见[后续修复记录](../rokae/docs/native_build.md)，不改写下文当时的测试结果。

代码重构、目录迁移、文档整理及可执行的功能验证已完成。**完整 pytest 结果为 122 通过、10 跳过、1 失败，另有 9 个子测试通过；不能宣称全部通过。**唯一失败在重构前备份中也已复现，原因与处理见下文。

## 目录、职责与兼容

完整目录和依赖图见[架构说明](architecture.md)。主要边界如下：

| 位置 | 职责及依赖方向 |
| --- | --- |
| `dexretarget/app/` | 入口、任务解析、键盘会话、资源组装；连接控制观察接口与记录入口 |
| `dexretarget/teleop/` | 输入重定向、目标发布、初始化/home、设备适配器；不导入相机、记录或应用层 |
| `dexretarget/camera/` | 每相机一个 SDK 进程、编码、时间/帧号、进程生命周期；不导入机器人或数据模块 |
| `dexretarget/dataset/` | Recorder、HDF5、对齐/质量、回收/恢复、导出；核心实现不初始化设备 |
| `anydexretarget/`、`rokae/` | 原有算法、公共 geometry、运动学、控制器与原生伺服 |
| `config/tasks/`、`realsense/config/`、`data/config/` | 任务、相机、记录配置分别维护，通过引用组合 |
| `tools/`、`integrations/`、`archive/` | 正式维护工具、仍支持的示例/兼容入口、历史资料 |

重构前先建立自有源码/文档/配置的哈希清单及备份（`/tmp/dex-refactor-audit/`），随后追踪入口、导入、配置、线程和关闭路径。模型、SDK、第三方依赖、真实数据及用途未确认的示例均未盲目删除。

主要迁移与清理：

- `integrations/teleop_arm_hand.py` 的实现拆到应用组装与独立控制循环；旧文件成为兼容 CLI。增加 `python -m dexretarget` 和同名 console 命令；原参数/默认值用重构前快照验证。
- Quest、Inspire、ROKAE 与 HandOutput 的规范实现移到 `teleop/devices/`，旧路径转发同一个类。AST 对照确认这些驱动及公共几何的函数/类定义未改变。
- 原 Collector 拆为纯 Recorder 与应用层 CaptureSources；后者借用已有设备读反馈并拥有相机源。旧 `Collector(config_path, context)`、`start_sources/close` 继续由兼容适配层提供。
- 相机进程和 worker 移到 `camera/`；相机获得父进程已校验配置的快照，避免重复读取配置文件造成记录与执行不一致。
- session、task、keyboard 从通用控制算法目录移到应用层；旧导入仍有效。采集不再通过 teleop 导入几何函数。
- 相机/Quest/ROKAE 诊断、独立关节工具和实时仿真移到 `tools/{diagnostics,robot,simulation}/`；旧脚本保持参数、导入及退出码。
- 任务配置以 `config/tasks/cup.yaml` 为唯一源，旧路径为符号链接。手型配置矩阵、可视化/标定示例、其他输入设备、双 IIWA 后端仍有实际用途，已分类说明并保留原资源路径。
- 根 README 精简为导航；当前流程合并到 usage/configuration/architecture。旧长手册与带日期审查记录移入 `archive/docs/`，原路径保留短链接。第三方手册与许可证保留；临时缓存单独清理。
- 包元数据统一到 pyproject，setup.py 只保留构建兼容入口；已用 `--no-deps --no-build-isolation -e .` 更新现有 dexteleop 安装，未新增环境或依赖。

## 实际控制流与数据流

`键盘 → Session 统一门控 → TeleopLoop/主循环 → 已有臂执行器和手串口`。
相机与遥操作并列，由应用创建；遥操作不启动、停止或指挥相机。

`控制观察接口 → CaptureObserver → Recorder.emit`；`相机进程 → 带时间的图像 → Recorder.emit`；
`CaptureSources → 借用原臂/手实例读取实测 → Recorder.emit`。
随后进入 `有界队列 → 单 HDF5 写线程 → 停止边界/排空 → 20 Hz 因果对齐 → 质量标记/连续片段 → 原子提交 → LeRobot 导出`。

无采集模式不创建相机、反馈读取线程或 writer。采集模式下源提前预热，r 从当前位置接入记录；s/f 只切断当前记录并后台保存。p 停止新增臂手目标，原始观察继续记录；h 要求已暂停且没有录制/保存工作，完成后仍暂停。

保留原始主机/设备时间、独立序号、读取起止、请求目标、接受目标、写确认和实测反馈。相机 30 FPS、臂反馈目标 50 Hz、手反馈目标 20 Hz、训练网格 20 Hz；12 维 state/action 和 π0.5 的 32 维模型投影不变。软件因果对齐不等于曝光硬同步。

## Review 发现与修复

| 问题 | 修复与验证 |
| --- | --- |
| 初始化中途失败可能遗留已启动资源 | 共享资源注册/清理；优先关门控，逆序关闭；单个 close 失败仍继续清理其余资源 |
| qpos 与关键点时间分开发布可能配错 | 状态锁内原子发布，保留 float32 与防御复制；并发读取与复制隔离测试 |
| writer 已 detach 后退出可能失去所有权 | Recorder 跟踪 pending writer，退出排空；未完成不允许下一段启动 |
| 元数据更新失败或保存线程无法启动 | 标记 invalid、排空线程；后台线程启动失败时先暂停再同步保存，保留成功/失败标签 |
| 满队列的停止哨兵可能无限等待 | 同一截止时间覆盖哨兵入队和 join，超时报错保留 partial；注入满队列验证 |
| 相机部分启动失败、重复启动、退出失败 | 清理已有子进程，拒绝重复启动；限时等待/kill 与读线程退出，避免在阻塞 readline 下关闭缓冲锁 |
| 反馈线程仍读设备时释放句柄 | source 先停止并 join，再由应用断开借用设备；读写继续使用原串口锁 |
| 键盘线程启动异常后终端未恢复 | 关闭路径用 finally 恢复 termios；已有 PTY 键盘测试保留 |
| 新模块入口遇 Conda 重执行丢失 `-m` | 保留原解释器 argv；新旧入口和安装的 console 命令均执行检查 |
| 兼容诊断脚本可能丢失失败退出码 | 保留 SystemExit(main())；注入非零返回码验证 |
| 独立关节工具的最大位移选项未真正约束运动 | 下发前检查 target-current；拒绝非有限限制与反馈。模拟驱动验证越界/NaN 时从不调用运动接口；未调整正常运动参数 |
| 集成测试从共享目录选“最新两段”会串扰 | 按本次子进程的 Saved 路径检查，重新运行两种路线通过 |

对原生 C++ 伺服、设备协议、坐标映射、滤波、控制频率和默认物理参数未作大规模改写。配置哈希确认 ROKAE YAML、采集 JSON、相机 JSON、任务内容与 requirements 均未改变。

## 实际执行结果

统一解释器为 `/data/miniconda3/envs/dexteleop/bin/python`。汇总原始指标见[验证证据 JSON](validation/run_history.md)。

| 检查 | 结果与范围 |
| --- | --- |
| 完整 `python -m pytest -q -r s` | **122 passed、10 skipped、1 failed、9 subtests passed**；1 项既有配置断言失败，未隐藏 |
| 显式排除该既有断言的可用测试集 | 122 passed、10 skipped、1 deselected、9 subtests passed；排除项不算通过 |
| 模块边界与文档检查 | 自有 Python AST/依赖循环、本地 Markdown 文件/锚点通过；不检查外部 URL、第三方源码和生成数据链接 |
| CLI/导入兼容 | 新旧帮助、data CLI、安装 console 入口、类身份、原参数和控制配置快照通过 |
| 数据格式对照 | 正常/暂停/反馈过期各 74 个 HDF5 dataset 的值、dtype、shape、单位和质量相同；加入重构前指纹回归测试 |
| 无采集完整入口 | 模拟 Quest + PTY 手 + mock 臂：暂停启动、使能、暂停、home、退出通过；77 次写、0 次反馈读，无录制 |
| cartesian / impedance 模拟入口 | 各成功段 90 窗口/80 有效，暂停两侧分别 40 帧；失败段 24/24 有效但 success=false；门控、回位、再次录制通过 |
| pinocchio + 真实双相机入口 | 相机真实，其余模拟：同样 90/80 与失败段 24/24；相机帧号无缺口、无缺失流，暂停无臂手动作泄漏 |
| 双相机独立 RGB 带宽 | 640×480 RGB8/30 FPS，预热 5 秒后测 10 秒；各 300 帧，D435i 29.978 Hz、D455 29.987 Hz，无帧号缺口/超时/错误 |
| 官方 LeRobot 导出读回 | 上述真实相机成功段导出 v2.1：2 个 episode、80 帧；20×12 动作窗口正确 padding，不跨暂停，任务来源保留，失败段默认拒绝 |
| Shell 语法 | SDK 安装/机器人构建脚本 bash -n 通过；本轮未执行系统 SDK 重装或机器人原生编译 |

### 性能范围

相同 32 行 HDF5 fixture，两个独立进程交替执行各 20 次，另先预热 2 次：含 fsync/写盘/对齐的中位耗时，重构前 93.59 ms、重构后 94.34 ms（+0.80%）；范围分别 79.01–106.67 ms、80.04–112.77 ms。差异处于本次运行波动范围，不能据此承诺无性能回退或证明 1 kHz 真机性能。

真实双相机短测和采集/导出链路通过；尚未执行长时间录像、GPU 推理并发负载、磁盘慢写/故障硬件测试或实体臂手实时抖动测量。解码 RGB 数组数据率 55.296 MB/s，不是 USB 线缆实际流量。

### 保留的失败与未验证项

唯一失败是 `test_init_nrt_speed_is_independent_and_conservative`：公共配置 joint_speed=0.25 对应 SDK 250，测试要求小于 100。**在重构前源码备份中运行同一测试也失败**。任务样例 home 另用 0.05；公共默认值和原断言均未改。降低真实运动速度或放宽测试需要明确物理依据，本轮按“核心行为不变”的优先级保留并报告，不能把它算通过。

本机是 x86_64，目录中的已编译扩展为 aarch64；没有匹配 ABI/平台的 `_rokae_cpp`。10 项原生测试被明确跳过，纯 Python 控制测试正常收集；不能用 mock 结果替代原生验证。

实体 Quest 跟踪、Inspire 串口/到位和反馈延迟、真实 ROKAE 位置/关节流/阻抗、上电与 home/取消的硬件响应、相机曝光同步/深度/IMU、真实任务可用频率和完整 π0.5 微调/部署都未在本轮验证。官方 LeRobot 能读回不代表已经启动 OpenPI 训练；本机 Python 3.10 仍不满足官方训练环境的 Python ≥3.11 要求。下一步硬件操作按[验收流程](../rokae/docs/commissioning.md)执行。
