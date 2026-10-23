# 配置、数据集布局与跟踪保持重构

本轮按操作者要求改变目录接口和跟踪丢失行为；不再保留根 `example/` 或 `data` Python 包。

## 配置与入口

- 唯一 ROKAE 默认：`config/robots/rokae.yaml`。CLI、任务、05/06 调试工具统一读取。
- `config/tasks/cup.yaml` 管理任务 home；任务覆盖通用 init，显式 CLI 优先于任务。相对配置路径统一基于仓库根目录。
- Cup 新值为 `[0, .35, 0, 1.85, 0, -.62920367, 1.57079633]` rad；原精度版本和上一版本地试验姿态均保留注释。新旧原姿态仅有约 3.2e-9 rad 舍入差。
- 相机、采集、手型、模型和工具配置全部迁入 `config/` 对应子目录。旧 ROKAE 配置完整保存在 `archive/config/rokae_teleop.yaml.disabled`，不参与加载。
- `integrations/config` 仅指向唯一的 `config/retargeting`；为了保留可选手型工具原相对路径，不保存参数副本。
- `dexretarget` 是应用 Python 包：app 组装，teleop 控制，camera 图像，dataset 存储，policy 模型适配。
- `data/collection` → `dexretarget/dataset`；`data/openpi` → `dexretarget/policy`；安装脚本迁至 `tools/install_collection.sh`，依赖清单迁至 `requirements/`。
- `example/` → `integrations/`，保留可选厂商输入/输出、双 IIWA、标定 GUI 和资源。主链路不导入 integrations。
- 删除 6 个仅转发的重复入口（teleop_arm_hand、两个 check、joint tool、两个仿真包装），调用方改用正式 CLI/tools；删除空配置目录；未删除数据、SDK、模型、未经确认的硬件逻辑。根 README 重写，中文入口去重，操作/配置/架构与数据文档更新。

新命令：

```bash
python -m dexretarget --task-config config/tasks/cup.yaml --no-capture
python -m dexretarget --task-config config/tasks/cup.yaml
python -m dexretarget.dataset --help
python -m dexretarget.policy.run --help
```

旧 `python example/...`、`python -m data`、`data.collection` 导入和旧配置路径需要更新。
这属于本轮明确的目录迁移，不宣称旧路径仍兼容。原 HDF5/schema/时间戳与 action/state 表示保持。

## 控制与数据流

Quest → TeleopLoop 重定向 → 臂控制器/手发送。应用组装层旁路观察控制目标，另借用设备读取真实状态；双相机独立进程输出帧，以上记录进入 Recorder → HDF5 → 20 Hz 因果对齐 → 有效片段/导出。

使能意图与输入状态分开：

- `e/p` 管理 Session.enabled；启动/手动暂停仍出现 hold_disabled。
- 输入过期进入 tracking_held，臂调用驱动 hold，手不新增目标；不关闭 enabled。
- 连续新鲜 0.20 s 后，在同一会话锁内重新锚定 TCP、重置 IK 和最后处理的腕部时间戳，再打开输出门。
- 手目标从最后发送值用 0.5 s 过渡到当前目标；记录实际发送并确认的值，不把预期值冒充反馈。
- HOLD 时 control=false，继续记录观察；恢复记录独立事件，训练片段不跨过无效段。
- `p` 不自动解除。失联期间不允许运行 r；驱动 callback_error/safe_stop 仍退出。
- hold 表示保持最后命令目标，不能保证物理瞬时静止；手可能完成先前已接受的运动。没有加入断电或释放物体动作。

## Review 与修复

- 移除默认入口与可选 preset 的歧义，检查任务/CLI/调试工具实际加载路径。
- 数据包搬迁后，更新导入、安装元数据、子进程入口和文档链接；修复配置路径改变导致的指纹比较差异，仅把相机路径标签规范为历史值，未刷新历史数据哈希来掩盖回归。
- 恢复时参考和消费时间戳一起更新；旧/逆序腕部帧在发布前拒绝；输入线程也检查手数据新鲜度，避免主线程两次轮询之间继续发布臂目标。
- 恢复回调失败时输出门保持关闭；手动暂停优先。模块边界检查改用新 dataset 名称，避免迁移后检查失效。
- 保留 RT watchdog、速度/加速度/jerk、软限位、串口锁、writer 生命周期与回收站删除语义。

## 验证范围

在现有 dexteleop 环境执行测试与纯模拟采集，无新增依赖，无上电/运动请求。全量结果记录于下文。

未验证：真机跟踪丢失/恢复手感、Inspire 实际运动过程、实时线程调度与网络抖动、双相机实机采集、所有可选厂商硬件/GUI。离线通过不证明物理安全、无性能回退或不存在所有问题；自动恢复机制应在现场空载小幅试验后用于正式数据采集。

## 已执行结果

- `python -m pytest -q`：213 passed、9 subtests passed（7.92 s）。覆盖新 HOLD 状态机、手目标过渡、实际输入线程门控、恢复顺序、逆序/过期输入拒绝、手动暂停、恢复失败关门，以及既有控制、相机、记录、schema 指纹与生命周期测试。
- `tools/check_project.py`：183 Python 文件、42 核心模块、131 本地链接，无错误。
- 正式遥操作、数据处理、policy、05/06 工具 `--help` 检查通过。
- `python -m dexretarget.dataset simulate --seconds 2 --output-root /tmp/dex-layout-validation` + validate：40/40 有效帧、0 丢记录、无 writer 错误，所有流存在。
- 未安装依赖、未重编译原生桥（本轮无原生代码修改），全部使用 dexteleop。

- 完整采集入口（伪 Quest UDP + Inspire PTY + mock Pinocchio 臂 + 合成双相机）通过：主动断流 → HOLD → 无 e 自动恢复 → 键盘暂停 → 保存 → home → 第二段失败保存 → 退出。断流/手动暂停区间均无新增臂手目标；第一段 144 帧中 119 帧有效并分成 3 段，第二段 24/24 有效，success 标签分别 true/false。原始报告：`docs/validation/layout_capture_20261022.json`。
- 端到端测试最初暴露旧断言将多个暂停区间合并的问题；改为按每个连续 control=false 区间检查，同时增加主动断流，重跑通过。未把预期无效帧计为有效训练数据。
- 额外检查 CLI 默认、Cup 引用与 SDK 工具默认解析为同一份 ROKAE 配置，且旧 example 目录不存在。

- 完整 `--no-capture` 入口通过：启动暂停 → e → p → home → q；伪串口 84 次目标写入、0 次采集反馈读取。报告：`docs/validation/layout_teleop_20261022.json`。
