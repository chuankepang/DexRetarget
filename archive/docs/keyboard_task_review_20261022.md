> 历史记录：只对应当时的机器与版本，不代表当前验证结果。当前状态见[重构与验证报告](../../docs/refactor_report.md)。

# 任务配置、键盘控制与数据流 review（2026-10-22）

本次按用户确认的协议实现：不要求每段从固定 home 开始；允许先使能预调整，再按 r
录制。纯遥操作与采集共用键盘及臂手门控，home 是独立运动操作。
Python 与全部验证使用 `/data/miniconda3/envs/dexteleop`，本次没有新增环境或额外依赖。

## 功能与职责检查

| 功能 | 实现与检查结果 |
| --- | --- |
| 任务配置 | `anydexretarget/teleop/task.py`；校验 schema、必要字段、设备选项、有限数值及 home；显式 CLI > 任务配置 > 默认值；文件引用以项目根目录为基准 |
| 启动 home | 示例任务复用原正装七轴 init；按任务速度、分段和路点调用原有 MoveAbsJ；检查最终关节反馈；默认保持暂停，不自动打开手 |
| 键盘 | `keyboard.py` 单独拥有 stdin；交互 TTY 单键，非 TTY 按行；验证过终端设置恢复；EOF/输入失效触发退出 |
| 使能/暂停 | `session.py` 的共享锁保护门控、臂目标发布和手写入；e 切换、p 明确暂停；使能失败回滚；不再使用 Quest 按钮或 SIGUSR 信号 |
| 非固定起点 | e 后可预调整；r 仅检查源就绪和 active，不检查 home；Pinocchio mock 联测确认第一段起始位置与回 home 后下一段起点不同 |
| 暂停续录 | p 后继续保存相机、臂手实测、Quest 和控制事件，停止新增臂手目标；e 用新鲜 wrist 与当前命令 TCP 同步重建参考 |
| 重新定参考 | c 请求重设机械臂参考，记录无效边界；不改变灵巧手目标 |
| s/f 保存 | 立即捕获停止边界并摘下 writer，后台排空、对齐、原子提交；不自动暂停；保存中仍能 p，但不能开新录制或回 home |
| 时长/空间限制 | 使用同一个后台保存路径，失败原因保留；低空间仍触发控制退出；独立离线 Collector 保留同步结束接口 |
| h 返回 home | 必须暂停且录制/保存结束；独占运动，按路点/目标返回，检查关节误差；可选手寄存器目标在臂回位后发送；完成后仍暂停 |
| home 中止/失败 | p/q 发出取消与既有 SDK stop 请求，不发送后续路点；故障锁定使能；错误路径尽力停止；成功重试 home 或重启后恢复 |
| d 与恢复 | 保留 dataset 内回收站及路径校验；不新增永久删除；沿用既有 trash/restore 测试 |
| 退出/部分初始化失败 | 先关闭统一门控，等待持有设备的工作线程，关闭相机/写盘、串口和机器人；单个关闭错误不会跳过其余清理；新增初始化失败资源回收测试 |
| LeRobot/OpenPI | 保留 12 维实际表示及 32 维模型接口；按连续有效段导出；记录任务 ID/配置哈希；兼容性检查包含 CLI 解析后的实际控制参数 |

## 数据流检查

```text
任务 YAML → 设备/控制参数 + 任务 home + collection 配置引用
键盘 → 会话状态/统一门控
Quest → 关键点重定向、相对 TCP 映射
       → 门控与控制器 → 接受的 TCP 参考 / 手 ANGLE_SET + 匹配 ACK
独立读状态 → 实测 TCP/q/dq、ANGLE_ACT/可选力
双相机进程 → JPEG、帧号、设备/主机时间、内参
以上各流 → 有界队列 → 单写入线程 HDF5 原始流
s/f/限制/退出 → 明确停止时间 → 20 Hz 因果对齐 + 有效性 mask
连续有效片段 → 独立 LeRobot episode → OpenPI 12 维输入/动作，模型补维
```

观察保持“接收时间不晚于 t”；动作仍为下一时间槽内接受/确认的指令，实测值不冒充
动作。暂停期间不补旧指令；无效处切断窗口，不拼接两侧。home 要求 writer 已摘下，
复位轨迹不会混入训练 episode。`metadata.json` 保存任务快照、哈希、原 YAML、解析后
实际控制参数及 CLI；导出 provenance 保留 task_id、task_profile_sha256 和原始段范围。

## 验证证据

- Python 回归：86 项通过、2 项 deselect；两个依赖未编译原生绑定的测试文件未纳入。
  原有初始化速度配置不一致仍作为已知问题保留，没有修改公共运动配置来让测试变绿。
- 新增测试覆盖：任务覆盖优先级/错误配置、未使能禁止录制、统一暂停、后台保存响应、
  home 排他/取消/失败、使能回滚、TTY 单键与恢复、初始化失败清理、限制触发保存、
  手原生寄存器写入，以及导出控制契约比较。既有同步、串口校验、删除恢复和适配器测试继续通过。
- 完整入口已分别联测 Cartesian、Pinocchio 和 impedance 的 mock 路线；Quest UDP 和
  Inspire PTY 均为模拟；相机既测过合成源，也测过真实 D435i/D455。
- 最终真实双相机 + Pinocchio mock 臂 + 虚拟串口手：
  `episode_20261022_142853_e63690aa93` 共 90 个时间槽，80 个有效，
  有效段 `[0,40)`、`[50,90)`；暂停附近 10 槽按预期排除，两个相机帧号缺口均为零。
  暂停期间串口目标写入不增长，原始流也无暂停中间区的臂/手目标；home 后保持暂停，
  可再次使能、录制并用 f 保存失败段。
- 同次联测第二段 `episode_20261022_142859_04c05c9de1`：24 个时间槽，24 个有效，
  `success=false`，默认导出被拒绝。
- 官方 LeRobot v2.1 回读：第一段导出 80 帧、2 个 episode；核对 `[20,12]` 动作窗口，
  第一段末尾只做该段末帧 padding，不跨到下一段；任务 ID/哈希溯源检查通过。
  汇总位于 `data/camera_tests/keyboard_export_review.json`。
- 并发运行导出和短采集时曾出现一个额外 50 ms 槽缺动作，质量检查将其排除；
  短于 20 帧的连续片段不会导出。联测断言按实际暂停边界检查，避免把正确的短段过滤误判为跨段。

复现（始终为模拟臂手，不连接真实机器人）：

```bash
python tests/check_capture_entrypoint.py --synthetic-cameras --arm-control pinocchio --seconds 2
python tests/check_capture_entrypoint.py --synthetic-cameras --arm-control impedance --seconds 3
python tests/check_capture_entrypoint.py --arm-control pinocchio --seconds 2
```

最后一项读取真实相机。复用的例行回归命令见[原验证记录](data_collection_validation_20261022.md)。

## 尚未验证的真机部分

本机缺少匹配平台的 `_rokae_cpp`，没有向真实机械臂或灵巧手发运动目标；真实机器人
home 路径、模式切换/停止延迟、碰撞与现场接触退出策略，以及真实串口响应仍需现场验收。
现有 SDK stop 用于请求停止，不能当作独立急停保证；退出会等待 home 工作线程结束后
才断开设备。home 规划仅做关节分段，不具备环境避障。`hand_angles: null` 保留目标，
写 ACK 不表示手指已到位。

多小时压力采集、真实接触任务和完整 π0.5 微调/推理未在此次验证范围内；当前 Python
3.10 用于采集导出，官方 OpenPI 训练依赖仍要求 Python ≥3.11。
