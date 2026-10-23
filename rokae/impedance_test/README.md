# ROKAE 固定参考阻抗独立试验

面向当前 xMatePro3 / xCore 2.3.2 / SDK 0.3.4。代码直接链接本地官方 SDK，不运行 Quest、Inspire、相机、主机 IK 或项目遥操作参考发生器。**此前真机曾在无输入时快速运动；本目录是隔离诊断，不是已经通过真机验证的修复。**

## 文件与参数来源

- `run.py`：读取统一配置、默认只打印计划；执行时创建独立结果目录并启动原生程序。
- `main.cpp`：直接 SDK 回位/固定参考阻抗、1 ms 回调采样、停止与资源回收。
- `guard.hpp` / `guard_test.cpp`：离线验证状态有效性和漂移/速度/位移约束。
- `analyze.py`：离线分析 CSV，不连接机器人。

质量/重心和六轴刚度来自 [机器人配置](../../config/robots/rokae.yaml)；home 来自 [Cup 配置](../../config/tasks/cup.yaml)，不复制一份 YAML 默认值：

```text
home rad = [0, 0.13323130, 0, 1.17304764, 0, 1.2835298642, 0]
mass kg  = 0.93938639463
cog m    = [-0.05271043598571, 0.01217669684527, 0.00860984186271]
K        = [300,300,300,15,15,15] N/m, Nm/rad
```

已标定质量/重心只写入一次 SDK Toolset 并读回，阻抗阶段将同一负载显式传给 RT setLoad。这是赋值，不将质量相加。惯量保留当前控制器值，并记录为未验证；不由质量/质心猜算惯量。此试验采用当前法兰中心 TCP，检测到非零 TCP/旋转会拒绝运动，不自动覆盖工具坐标。力控参考为工具/法兰轴，世界方向不直接作为 K 的轴定义。当前平移和转动增益各向同性。

## 编译及离线验证

在仓库根目录，全部使用 dexteleop：

```bash
conda activate dexteleop
cmake -S rokae/impedance_test -B rokae/build/impedance_test -DCMAKE_BUILD_TYPE=Release
cmake --build rokae/build/impedance_test --parallel
ctest --test-dir rokae/build/impedance_test --output-on-failure
python rokae/impedance_test/run.py --stage hold
```

最后一条默认 DRY RUN，不连接、不上电、不写数据目录。只有加 `--execute` 才连接；运动阶段还需输入指定确认词，没有跳过确认选项。

## 按阶段运行

**1. 只读检查**：不设置负载、不切模式、不上电、不运动。

```bash
python rokae/impedance_test/run.py --stage inspect --execute
```

**2. 单独回 home**：确认运动空间、完整装配与物理停止装置可用后执行。输入 `MOVE_HOME`，程序使用官方 RT `MoveJ`，速度倍率 0.05；不是阻抗运动。不修改软限位、不恢复报警，不自动避障。回位完成后停止并请求下电，核对下电结果。

```bash
python rokae/impedance_test/run.py --stage home --execute
```

**3. 在 home 进行 5 秒固定参考观察**：输入 `START_IMPEDANCE`。第一次全程不触碰，观察是否自行漂移。该阶段要求关节已在 home 附近（各轴误差 ≤0.02 rad），不会自动回 home。

```bash
python rokae/impedance_test/run.py --stage hold --duration 5 --execute
```

只有这一步无异常运动、无保护且日志正常时，才考虑下一步；软件完成计时不代表完整验收。出现自行运动或保护应停止本轮流程并保留结果，不能反复重启。

**4. 后续轻推观察**：在前一步稳定的条件下运行，前两秒不要触碰，之后轻推手掌/刚性装配处，避免手指和细长支架。缓慢偏离后松手，观察是否向初始平衡位姿恢复。阻抗始终已激活，不需要按 e。

```bash
python rokae/impedance_test/run.py --stage hold --duration 20 --execute
```

推得越远，理想弹簧恢复力越大；K=300 N/m 时 1 cm 误差对应约 3 N 的弹簧项。实际还包含重力补偿、阻尼与关节动力学，不能将它当作总接触力。官方原生接口没有独立 Cartesian damping 设置，本程序不声称能单独调阻尼或精确验证阻尼系数。七轴肘角行为沿用官方示例，不强制发送 hasElbow，不保证冗余方向刚性保持。

## 停止、保护与日志

Ctrl+C 请求 SDK 停止，不代替物理急停，也无法承诺在通信或 SDK 阻塞时立即生效。正常/异常结束按 `stopMove → stopLoop → 停接收 → 请求下电并读回` 清理；只有线程停止及下电确认后才恢复 NrtCommand。退出后不自动回 home，不恢复报警。回位监控超时为 60 秒。

默认附加软件护栏（独立于控制器安全保护，实际停止效果尚待硬件验证）：

| 条件 | 阈值 |
| --- | --- |
| 前两秒单关节相对初始漂移 | 0.02 rad |
| 整段单关节相对初始偏移 | 0.25 rad |
| 单关节速度绝对值 | 0.15 rad/s |
| TCP 相对固定平衡点位移 | 0.03 m |
| TCP 相对固定平衡点转角 | 0.15 rad |

这些护栏会限制可推动范围；触发代表本试验终止条件成立，不自动等同于控制器故障。未放宽任何控制器保护阈值。若反馈字段不支持、状态无效、软限位关闭或当前关节越界，会拒绝执行/停止。

结果保存至 `data/diagnostics/impedance_test/<stage>_<唯一后缀>/`：

- `manifest.json`、`parameters.txt`：实际配置和输入。
- `console.txt`、`summary.txt`：运行状态、停止原因、最近最多 10 条告警/错误。初始化失败详情在 console；不冒充完整历史日志。
- `trace.csv`：仅 hold 阶段。每次回调记录时间、实际周期、固定参考、实测 TCP、q/dq、实测/指令关节力矩和控制器指令关节位置。缓冲区预分配，回调中不打印、不写文件；线程停止后导出。故障断电时 CSV 可能不完整；进程被强制杀死无法保证内存数据落盘。

```bash
python rokae/impedance_test/analyze.py data/diagnostics/impedance_test/实际hold目录/trace.csv
```

分析可判断固定参考是否变化、初始漂移、最大位移/关节速度及回调间隔。没有独立标定外力数据，不能从位移独自计算真实刚度/阻尼。CSV 的 q_c/tau_c 是控制器提供的指令反馈，不代表最终伺服端全部信号；回调间隔不是控制器伺服时序。

## 官方依据与数据完整性

ROKAE [本地官方 v0.3.4 笛卡尔阻抗示例](../third_party/official_sdk_audit/xCoreSDK-CPP-77d2fd7546e657d4e66a82b29c2a5f21d01db3e3/example/rt/cartesian_impedance_control.cpp) 的顺序为 automatic → RtCommand → power on，并先配置力控坐标再设置刚度、期望力。本试验采用该接口路径，移除了示例的大幅正弦运动与非零期望力；没有复制未经标定的工具旋转。

Franka [官方示例](https://frankarobotics.github.io/libfranka/0.16.1/cartesian_impedance_control_8cpp-example.html) 同样以初始实测位姿作为弹簧平衡点，但应用层力矩控制和 ROKAE 原生位姿接口不同，不能直接复制公式或增益。

RobotAssist 惯量问题：官方 [V3.2 手册](https://static.rokae.com/Downloads/Manual/xCore%E6%8E%A7%E5%88%B6%E7%B3%BB%E7%BB%9F%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8CV3.2_A.pdf) 8.12.2.4 提到负载惯量计算，并给出一步/两步辨识流程；这不证明 xCore 2.3.2 当前向导会自动输出惯量。用户现有截图及 SDK 活动参数均为零惯量，尚未验证其来源/是否支持该辨识项。可确认的界面入口为“工具列表 → 对应工具 → 负载辨识 → 选择一步/两步 → 运行并保存”；用户已经完成两步流程，没有已证实的另一个“自动算惯量”按钮，不应为了惯量盲目重复拆装。主轴方向、单位和参考点须按对应版本定义，不能仅把 m·r² 当作完整惯量输入。

## 最新现场结果：hold_1jjub90t

用户已执行 hold，但固定参考下约 0.110 秒内 TCP 上升 5.36 mm，J2 达到 0.15097 rad/s，触发软件 joint_speed 护栏；程序随后主动停止并下电，summary 确认下电。没有完成 5 秒，也未达到轻推试验前提。**暂停继续运行 hold 或提高护栏阈值**。详情见[离线分析](../../docs/validation/hold_1jjub90t_analysis.json)。q_c/tau_c 在本记录中的零值尚未确认语义，不能据此认定是实际全零伺服目标。

## 验证边界

已执行编译、离线 CTest/pytest、计划打印和一次真实只读 inspect。开发期间没有自动执行 home/hold/推动。随后用户的 hold 触发软件护栏并确认下电，记录如上；这只是一次护栏触发结果，未通过静止稳定性测试或完整停止验证。根因仍未解决，不能宣称正常阻抗。

## 查询上位机保存的 tool1

### 读取当前法兰与工具坐标系

`read_frames` 不调用模式、工具、负载、上电设置接口，读取当前世界/基座、法兰、TCP、参考坐标系与关节角。应在没有其他控制程序运行时使用。

```bash
cmake -S rokae/impedance_test -B rokae/build/impedance_test -DCMAKE_BUILD_TYPE=Release
cmake --build rokae/build/impedance_test --target read_frames --parallel
LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
rokae/build/impedance_test/read_frames 192.168.0.160 192.168.0.100
```

输出单位为米/弧度，矩阵按行排列。关系为 `ref_T_end = inverse(world_T_ref) * world_T_base * base_T_flange * flange_T_end`。读取不是同一时刻的原子快照，因此同时输出查询前后关节角判断是否移动。

2026-10-23 实读与 `tcp-axis.png` 的位置/关节角相符，RPY 在显示精度附近相符；当前 TCP、参考与基座变换均为单位矩阵，工具系与法兰系重合。当前活动 SDK 负载仍为 0 kg，坐标核对成功不能视为负载或阻抗验证通过。数值见[坐标核对记录](../../docs/validation/frame_readback_20261023.json)。

`read_tools.cpp` 查询当前活动 Toolset 和 `toolsInfo()` 列表，打印质量、重心、惯量、TCP，不选择工具或写入负载。SDK 0.3.4 的保存工具查询属于 NrtRLTask 接口；直接查询可能返回模式错误。

```bash
conda activate dexteleop
cmake --build rokae/build/impedance_test --target read_tools --parallel
LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
rokae/build/impedance_test/read_tools 192.168.0.160 192.168.0.100 tool1 --query-project-mode
```

`--query-project-mode` 要求下电、idle，临时选择 NrtRLTask，仅查询，不 loadProject/runProject，完成后恢复 SDK 常规 NrtCommand 模式（旧 SDK 没有查询原运动控制模式的接口，因此不声称恢复任意未知原模式）。去掉该参数则不切模式，可能无法读取工程工具列表。该程序不用于正在运行其他控制客户端时。

本次实际查询找到 tool1，但质量/重心/惯量/TCP 均为零；另有 g_tool_0，质量为 2 kg、CoM=[0,0,0.1] m。两者均不匹配此前 0.939386 kg 的标定结果，不能擅自选择来运行阻抗；需确认上位机保存所在工程及完整工具名。原始结果保存在 `data/diagnostics/tool1_project_readback_kor2lsid/result.json`。

### 按工程复查结果

用户指定分别检查 `test` 与 `xstack_default` 后，新增显式参数 `--project 工程名`，仅在下电 idle 下加载工程定义；`--parse-project` 可追加 ppToMain 解析（不调用 runProject）。加载/解析会改变已加载工程和程序指针，因此不将它称为完全无状态修改的只读操作。完成后恢复 NrtCommand，工程仍保留为显式加载的工程。

本次实际结果：

| 工程 | 结果 |
| --- | --- |
| test / task0 | 加载成功；加载后、ppToMain 解析后两次 toolsInfo 都找到 tool1，但质量、重心、惯量、TCP 均为零 |
| xstack_default | projectsInfo 任务列表为空；尝试空任务加载后 SDK 返回“参数错误,参数类型或个数错误”，无法取得此工程的工具列表，不能推断 tool1 不存在 |

最终保持 `test` 已加载但未运行，电机下电；无上电、无运动、无工具选择、无负载写入。最终 test 原始查询：`data/diagnostics/tools_test_parsed_wf8oodqc/result.json`；xstack_default 失败原始查询：`data/diagnostics/tools_xstack_default_kuk_qrfb/result.json`。该结果只能说明 SDK 查询的 test/tool1 没有返回标定数据，不能直接断言上位机保存失败或辨识结果错误。
