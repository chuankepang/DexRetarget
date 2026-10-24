# ROKAE 固定参考阻抗独立试验

最新轻推行程调整：`hold_8dr6_xzi` 在 19.395 s 因 J3 偏移 0.500062 rad 触发主机 joint_excursion，停止并确认下电；当时 TCP 偏移 8.16 mm，记录最大 38.48 mm、最大关节速度 0.3833 rad/s，未触发速度检查。轻推阶段（2 s 后）总关节偏移上限现为 **1.0 rad**；baseline 仍 0.50 rad，启动仍 0.04 rad，轻推速度 0.70 rad/s、TCP 100 mm。J1/J3 大幅变化提示冗余姿态运动，放宽行程不是零空间控制修复；若松手后继续自行运动，应停止并核查。新增停止详情会打印具体关节、偏移及阈值。未修改控制器保护、SDK 限速/滤波或负载。


当前独立测试的软件护栏已按要求翻倍：启动关节偏移 0.04 rad、总关节偏移 0.50 rad、TCP 转角 0.30 rad；baseline/启动速度 0.30 rad/s、TCP 位移 60 mm；light-push 在第 2 秒后速度 0.70 rad/s、TCP 位移 100 mm。前 2 秒仍不触碰。回 home 的速度与到位容差、刚度、负载、控制器底层保护未改变。下方旧运行记录中的阈值属于当时配置。


**当前状态：独立阻抗尚未验收通过。** RT setLoad 的 skip 对照已经实测，仍在约 0.283 秒上漂约 8.22 mm；仅跳过调用未修复问题。本次日志已确认传感器零点标定成功。现启用 `apply_on_impedance_start: true`，用最新截图值直接设置 SDK 活动负载并读回；无需 RL 工具名。保留 TCP/ref/惯量，先独立无触碰观察，详见[最新复核](../docs/impedance_resolution.md)。默认 explicit 和正式遥操作保持原行为；下方旧 hold 命令保留历史追溯，暂不重复。

负载开关已修复：独立测试现在读取 `robot.payload.apply_on_impedance_start`。true 覆盖 SDK Toolset 的质量/重心；false 跳过写入并核验活动负载。两者进入 RT 后均显式调用 RT `setLoad`，不是关闭所有负载配置。协议更新为 v5，旧 v1–v4 保留原行为；summary.txt 分别记录 Toolset 与 RT 写入。详见[负载重复补偿审查](../docs/payload_review.md)。

独立阻抗测试的回 home 速度已调整：默认 NRT MoveAbsJ 保留 SDK speed=100，在线倍率由 **0.05 提至 0.50（10 倍）**；legacy-rt 的 MoveJ 速度比例同步为 0.50。原启动命令不变，home/hold 两个阶段均生效。实际耗时仍受行程、加减速和控制器限制影响，不承诺恰好缩短至十分之一。到位后仍连续静止确认 0.5 秒，阻抗增益、负载、接触护栏未改。

最新方向性试验：0.0642 kg 仅为软件实验值，不能认定为完整组件质量。八轮新记录确认发送目标与 SDK 指令反馈保持固定（最大平移差 <0.001 mm），仍有方向性回弹问题；当前证据更支持补偿/摩擦偏差，根因未闭环。先核对相同参数下的 RL/SDK 与真实负载，不继续盲目减质量。详见[新增证据与修正顺序](../docs/impedance_startup_audit.md)的“方向不对称与软件质量试验”小节。

## 松手不回弹：模式与数据流复核（2026-10-24）

用户确认松手后未回到平衡点，并感到上抬省力、下压费力。当前不能把阻抗性能判为通过；扩大位移边界不能修复恢复作用。

源码调用 `setFcCoor → setLoad → setCartesianImpedance → setCartesianImpedanceDesiredTorque(全零) → startMove(cartesianImpedance)`；回调每周期返回启动时固定的 `CartesianPosition`。没有自由拖动调用，没有把实测位姿更新成目标，矩阵行优先/构造函数已通过本地 SDK 离线测试。与 [ROKAE 官方 RT 示例](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/example/rt/cartesian_impedance_control.cpp) 的接口类型一致，但 API 成功不是内部力控生效的独立证明。不要复制官方示例里的零刚度轴和非零期望力。

`robotassist` 为平移 100 N/m、转动 100 Nm/rad。理想平移弹簧项在 1/2/5 cm 分别为 1/2/5 N；转动 5° 的弹簧项约 8.73 Nm。单位不同，六个 100 不代表六个方向手感一样。低平移刚度下，摩擦或补偿残差可能掩盖回弹；转动较硬、手掌施力相对法兰存在力臂，也可能导致“不丝滑”。这些是待验证解释，不是已定位根因。

[Franka 官方示例](https://github.com/frankarobotics/libfranka/blob/main/examples/cartesian_impedance_control.cpp) 使用固定平衡点以及弹簧和速度阻尼项；本项目将位姿交给 ROKAE 内部阻抗器，不能再将 Franka 力矩公式重复叠加。旧 SDK 未暴露独立笛卡尔阻尼参数，30 Hz 是指令滤波频率，不是阻尼比。阻尼项在静止时为零，不能单独解释永久静态偏差。

最新记录 `hold_2vxrak3b`：软件质量 0.3642 kg，运行满 30.000661 s；最大 TCP 偏移 38.589 mm，结束仍距原参考约 38.478 mm。前约 18 s 接近静止，之后有明显运动；没有精确松手时间或外力测量，不能由轨迹独自估算回弹刚度。`hold_tqbwgo5x` 用 0.9642 kg 在 0.271 s 启动漂移，Z 上移 8.220 mm。软件质量与完整组件辨识值不一致，不能把“某个姿态稳定”当成负载辨识完成。简单模型中质量少填通常欠补偿向下，所以上抬更轻不能仅归因于少填质量，仍需查模型/零偏、坐标和内部力控。

已补充独立 trace 的 `sent_tcp_*`（本周期实际返回 SDK 的目标）、`tcp_c_*`（SDK pos_c 指令反馈）和 `tcp_c_valid`。旧记录仅含固定参考，不能独立证明 SDK 实际执行目标。无效/全零 pos_c 不当成零目标或模式失败；分析报告显示有效覆盖率和相对参考偏移。新增字段订阅失败会按原有失败清理路径退出，不能忽略错误继续试验。原 trace 列保留、追加新列，正式采集数据格式不变。

准备了 `spring-check` 对照：**仅平移刚度从 100 改为 300 N/m**，转动仍 100 Nm/rad、world 力控轴、当前 YAML 负载和既有护栏都保留。不自动改写现场负载，不同时放开到 10 cm。此为鉴别试验，尚未验证真机效果。

```bash
conda activate dexteleop
python rokae/impedance_test/run.py --stage hold --profile spring-check --startup latest-compatible --interaction light-push --duration 30 --execute
```

先检查打印的负载；输入 y 后回 home，前 2 秒不碰，确认无自行漂移后轻推约 10 mm，一次只沿一个方向，不扭转手掌。尽量在第 10 秒前完全松手，此后保持不接触直至结束，记录松手时刻。如果已有自行运动/振荡，停止该轮。30 秒会主动下电。若提高刚度后恢复作用仍不明显，优先查看 `sent_tcp` 与 `tcp_c`；不靠继续加刚度/扩大范围掩盖问题。

离线分析（不连接机器人）：

```bash
python rokae/impedance_test/analyze.py data/diagnostics/impedance_test/hold_实际目录/trace.csv
```

## 轻推回弹测试与最终接触控制目标

最终目标是 Quest 遥操作、采集和策略部署共用原生笛卡尔阻抗执行路径：Quest/策略给出移动的平衡位姿，机器人与物体接触时允许实测位姿偏离，并产生恢复作用。独立试验固定平衡点只是底层验证，不是自由拖拽示教，也不等于关闭过载、异常速度或通信保护。

`hold_izsi2veb` 已进入阻抗，在 9.299 秒轻推时 J7 达 **0.150535 rad/s（8.625°/s）**，越过独立脚本的 0.15 rad/s 护栏，随后程序主动下电且清理成功。TCP 偏移约 18.69 mm；前后最近十条控制器告警相同。这个 0.15 阈值来自独立试验，**没有直接套用到正式遥操作控制器**。

新增显式轻推试验：

```bash
conda activate dexteleop
python rokae/impedance_test/run.py --stage hold --profile robotassist --startup latest-compatible --interaction light-push --duration 30 --execute
```

输入 `y` 后先回 home。**前 2 秒不接触，等“轻推阶段”提示，先缓慢偏移约 10–20 mm 后松手观察恢复。** 显式 light-push 在第 2 秒后使用关节速度 **0.35 rad/s**、TCP 位移 **50 mm** 边界；首 2 秒和默认 baseline 仍为 0.15 rad/s、30 mm。位移是相对固定平衡点的三维距离，不是累计行程或每个轴分别允许 50 mm。0.15 rad 姿态、0.25 rad 关节偏移、启动漂移护栏和控制器自身保护不变；不重定平衡点，也不会自动重启。这些是尚未完成现场验证的实验边界，不是额定安全范围。到时、越界或 Ctrl+C 均停止并下电。

`hold_529f8y0a` 在 10.446 秒达到 **30.020 mm**，触发当时的 30 mm 软件边界；最大关节速度约 0.112 rad/s，没有触发速度护栏。程序下电及清理成功，前后保存的最近告警相同。本次仅将后续轻推位移边界改为 50 mm，负载和刚度未改；下一次位移终止会打印实际偏移和阈值。

`hold_enrw1sbx` 在 7.103 秒因 J6=0.251327 rad/s 超过当时的 0.25 rad/s 主机护栏停止，TCP 偏移 26.62 mm，清理及下电成功。现仅将轻推速度阈值提高到 0.35 rad/s（约 20.05°/s），其余边界保留。独立脚本没有加速度终止阈值；SDK 的 `setFilterLimit(true,30)` 与 30 Hz 指令滤波继续开启，正式遥操作中的目标加速度参数不控制这条固定参考测试。尚未验证新阈值下的完整推移/松手回弹。

此轻推选项不会改写正式遥操作的刚度/保护，不能将一次推力测试当作采集/部署验收。`robotassist` 测试刚度为 100×6，正式配置仍为 300×3、15×3；后续需用同一经核实负载、TCP、刚度验证移动平衡点与接触过程。记录时必须区分请求目标、接受目标与实测位姿，不能把接触下的位姿误差当作跟踪失败，或把软件接受目标当作实测状态。

本次没有改写负载。请核对打印的质量与重心是否为预期试验值：完整组件未拆除时，0.5642 / 0.2642 kg 的软件试验值不等于真实质量。当前仍未解决辨识值与稳定试验值不一致的原因，不能仅靠减质量或放宽护栏验收柔顺控制。

## 最新结果：5 秒后下电是正常计时结束

`hold_o0p5ys90` 使用 **0.2642 kg、[0,0,-52] mm**，成功运行阻抗约 **5.000 秒 / 5061 帧**。最大 TCP 偏移 **1.028 mm**，末帧相对首帧 Z 变化 **-0.487 mm**。`startMove_returned=1`、`stop_reason=duration_completed`、`loop_joined=1`、`power_off_readback=1` 表明正常计时结束后由程序停止并下电；前后最近十条告警相同。这不是启动失败，也没有触发本程序漂移/速度护栏。

默认 `--duration 5`；程序不会无限保持阻抗。终端现在明确打印“正常完成，主动停止并已确认下电”。如需延长同一受控观察窗口，时间参数为 `--duration 20`，到时仍自动停止；不能仅为了不断电而删掉退出清理。**先核对实际启动打印的质量/重心：当前 YAML 可被现场修改，不等于这次保存的 manifest。**

完整组件并未拆除，所以 **0.2642 kg 是试验中的软件参数，不是已确认的真实重量**。同一负 Z 重心下 0.5642 kg 也完成了五秒观察；此结果说明质量设置会影响运动，但不能证明某个数字就是正确辨识值，更不能只凭静止便验收不同姿态、接触推力或 Quest 跟随。

## 当前测试：移植新版启动流程，保留 SDK 0.3.4

当前控制器为 **2.3.2**；官方最新 **0.7.1 要求控制器 ≥3.2.1**。项目继续成套使用 `rokae/sdk` 的 0.3.4 头文件和动态库，根目录下载的两个新版仓库仅供对照。详见[SDK 兼容性与审查](../docs/official_sdk_update_review.md)。

`hold_pbbkqpew` 在“清空队列”检查处退出，尚未开启阻抗；控制台 `power_off_readback=1` 确认程序清理时已下电。该前置条件已修正：持续 1 kHz 数据流不再必须读到空队列，改为配置完成后停止旧订阅、创建新订阅、限时接收三帧。断流/超时仍拒绝启动。此次结果不能用于判断原有上漂是否改善。

本次修改启动流程：**NRT MoveAbsJ 回任务 home → 连续静止 0.5 秒 → 进入 RT → 设置 TCP、负载、力控参数 → 重建状态订阅并限时接收三帧 → 固定参考阻抗**。主遥操作的阻抗初始化也采用配置后获取新参考；其默认位置控制及任务回位行为不变。

```bash
conda activate dexteleop
python rokae/impedance_test/run.py --stage hold --profile robotassist --startup latest-compatible --duration 5 --execute
```

输入一次 `y` 回车即可，程序先回 Cup home，再观察固定参考阻抗；无需 Quest 或按 e。本次全程不推动，确认回位路径和物理停止装置可用。预期打印 `Startup latest-compatible: NRT MoveAbsJ home`、`Home verified stationary` 和 `Post-setup RT subscription ready; received packets=3`。结果目录保存 `summary.txt`、`console.txt`、`manifest.json`；进入实时回调后才生成 `trace.csv`，初始化失败时可能没有轨迹。

**新启动路径已得到上述现场五秒记录；其它负载设置仍有上漂记录，尚未完成阻抗性能验收。** 保持现有护栏，异常时停止并保留记录，不以放宽限值掩盖偏移。`duration_completed` 也不等于负载准确或阻抗验收通过。

当前统一负载仍来自 `config/robots/rokae.yaml → robot.payload`：质量与重心以启动打印的 `mass_kg/cog_m` 和保存的 manifest 为准。本次审查期间曾读到 -52 mm，最后 dry-run 为 +52 mm；代码修改未覆盖该数值。它们都仍是试验假设，比较两条启动路径时必须固定同一份负载。惯量保留控制器值，零值未验证；TCP 为法兰中心，非零 TCP 会拒绝独立试验。SDK Toolset 与 RT setLoad 使用同一质量/重心，退出不自动恢复负载。

`robotassist` profile 使用 world 力控方向和 `[100,100,100,100,100,100]` 刚度，期望力/力矩为零，仅匹配 RL 截图的这些参数。主遥操作仍使用配置中的刚度 `[300,300,300,15,15,15]`。前三项 N/m、后三项 Nm/rad，没有加入未经验证的阻尼参数。

历史 `hold_81v3kce7` 的 **+52 mm** 在 0.270 秒上升 8.306 mm 后触发程序护栏；均值重心试验也上漂。两次都不能证明新启动流程有效，也不能证明负载是唯一原因。`--startup legacy-rt` 保留旧 RT MoveJ 启动供有需要时对照；不自动连续执行两种试验。

官方接口核对、历史试验和未验证项见[阻抗启动审查](../docs/impedance_startup_audit.md)。

面向当前 xMatePro3 / xCore 2.3.2 / SDK 0.3.4。代码直接链接本地官方 SDK，不运行 Quest、Inspire、相机、主机 IK 或项目遥操作参考发生器。**此前真机曾在无输入时快速运动；本目录是隔离诊断，不是已经通过真机验证的修复。**

## 文件与参数来源

- `run.py`：读取统一配置、默认只打印计划；执行时创建独立结果目录并启动原生程序。
- `main.cpp`：直接 SDK 回位/固定参考阻抗、1 ms 回调采样、停止与资源回收。
- `nrt_home.hpp` / `startup_test.cpp`：NRT 到位判定、取消、错误及新反馈队列的离线测试。
- `../include/realtime_startup.hpp`：主遥操作与独立试验共用的有界订阅初始化与读取。
- `guard.hpp` / `guard_test.cpp`：离线验证状态有效性和漂移/速度/位移约束。
- `analyze.py`：离线分析 CSV，不连接机器人。

质量/重心和六轴刚度来自 [机器人配置](../../config/robots/rokae.yaml)；home 来自 [Cup 配置](../../config/tasks/cup.yaml)，不复制一份 YAML 默认值：

```text
home rad = [0, 0.13323130, 0, 1.17304764, 0, 1.2835298642, 0]
mass kg  = 0.9642
cog m    = robot.payload.cog_m  # 以本次启动 manifest 为准；不在脚本复制默认值
K        = [300,300,300,15,15,15] N/m, Nm/rad
```

配置的质量/重心只写入一次 SDK Toolset 并读回，阻抗阶段将同一负载显式传给 RT setLoad。这是赋值，不将质量相加。惯量保留当前控制器值，并记录为未验证；不由质量/质心猜算惯量。此试验采用当前法兰中心 TCP，检测到非零 TCP/旋转会拒绝运动，不自动覆盖工具坐标。默认 configured profile 的力控方向为工具/法兰轴；本轮 robotassist profile 使用 world 方向。当前平移和转动增益各向同性。

## 编译及离线验证

在仓库根目录，全部使用 dexteleop：

```bash
conda activate dexteleop
cmake -S rokae/impedance_test -B rokae/build/impedance_test -DCMAKE_BUILD_TYPE=Release
cmake --build rokae/build/impedance_test --parallel
ctest --test-dir rokae/build/impedance_test --output-on-failure
python rokae/impedance_test/run.py --stage hold
```

最后一条默认 DRY RUN，不连接、不上电、不写数据目录。只有加 `--execute` 才连接；运动阶段还需输入 y 并回车，没有跳过确认选项。

## 按阶段运行

**1. 只读检查**：不设置负载、不切模式、不上电、不运动。

```bash
python rokae/impedance_test/run.py --stage inspect --execute
```

**2. 单独回 home**：确认运动空间、完整装配与物理停止装置可用后执行。输入 `y` 并回车，默认使用 NRT `MoveAbsJ`（SDK speed=100、online_scale=0.50）；不是阻抗运动。不修改软限位、不恢复报警，不自动避障。回位完成后停止并请求下电，核对下电结果。

```bash
python rokae/impedance_test/run.py --stage home --execute
```

**3. 在 home 进行 5 秒固定参考观察**：输入 `y` 并回车。第一次全程不触碰，观察是否自行漂移。该阶段先以位置控制回 home，连续确认静止 0.5 秒后才进入阻抗；已到位则跳过零长度回位运动。

```bash
python rokae/impedance_test/run.py --stage hold --duration 5 --execute
```

只有这一步无异常运动、无保护且日志正常时，才考虑下一步；软件完成计时不代表完整验收。出现自行运动或保护应停止本轮流程并保留结果，不能反复重启。

**4. 后续轻推观察**：在前一步稳定的条件下运行，前两秒不要触碰，之后轻推手掌/刚性装配处，避免手指和细长支架。缓慢偏离后松手，观察是否向初始平衡位姿恢复。阻抗始终已激活，不需要按 e。

```bash
python rokae/impedance_test/run.py --stage hold --interaction light-push --duration 20 --execute
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

### 用户授权的重心候选对照试验

用户要求先按轴排列错误的假设试验。新增 `--experimental-cog-mm X Y Z`，只允许 `hold`，仅覆盖本次写给 SDK 的重心，质量/刚度/停止阈值不变；原 YAML 保留。候选值不被标记为已标定，manifest 同时保存原配置和本次值。程序会写入 SDK Toolset 和 RT 负载并检查读回；退出不主动恢复上一份负载，不能假定控制器已恢复原值。

```bash
conda activate dexteleop
python rokae/impedance_test/run.py --stage hold --duration 5 \
  --experimental-cog-mm 12.17669684527 8.60984186271 52.71043598571
# 上面仅打印计划；现场确认后执行下面一条，并在提示时输入 y 并回车。
python rokae/impedance_test/run.py --stage hold --duration 5 \
  --experimental-cog-mm 12.17669684527 8.60984186271 52.71043598571 --execute
```

本轮全程不触碰机器人，先观察固定参考下是否自发漂移。程序先以位置控制、0.50 速度比例回任务 home；已到位时跳过零长度运动。随后连续 0.5 秒确认每关节误差≤0.002 rad、速度<0.01 rad/s（最多等 5 秒），失败不进入阻抗。确认执行前须检查整个回位路径。启动不需要 Quest 或键盘 e；输入确认后请求上电、回 home，静止确认通过后进入阻抗。保留关节速度 0.15 rad/s 等原护栏，发生异常运动使用现场物理停止，不提高阈值重试。`duration_completed` 只说明计时完成，还需结合轨迹位移/速度和停止状态判断；一次无漂移不证明质心或接触性能正确。结果保存到新的 `data/diagnostics/impedance_test/hold_*/`。

此前使用原始重心执行 hold，固定参考下约 0.110 秒内 TCP 上升 5.36 mm，J2 达到 0.15097 rad/s，触发软件 joint_speed 护栏；程序随后主动停止并下电，summary 确认下电。没有完成 5 秒，也未达到轻推试验前提。**不重复原配置试验，不提高护栏阈值；本次仅进行上面明确标记的候选对照试验**。详情见[离线分析](../../docs/validation/run_history.md)。q_c/tau_c 在本记录中的零值尚未确认语义，不能据此认定是实际全零伺服目标。

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

2026-10-23 实读与 `tcp-axis.png` 的位置/关节角相符，RPY 在显示精度附近相符；当前 TCP、参考与基座变换均为单位矩阵，工具系与法兰系重合。当前活动 SDK 负载仍为 0 kg，坐标核对成功不能视为负载或阻抗验证通过。数值见[坐标核对记录](../../docs/validation/run_history.md)。

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


### 候选试验 hold_p03k9qz4 与自动回位更新

本次 371 个样本覆盖 0.367093 秒，固定参考下末端上升 9.491 mm，最大相对参考位移 9.812 mm；最大关节速度 0.07749 rad/s。停止原因是测试程序 `startup_joint_drift`（启动前两秒任一关节偏移超过 0.02 rad），随后程序请求下电，读回确认。不是本次记录证明的控制器速度/加速度保护；附带的旧日志不能归因于本次。该轨迹尚未到稳态，不能据此确认重心正确。

现在 `hold` 自动执行位置控制回 home → 连续静止确认 → 采集新的固定参考 → 阻抗观察。使用上面的候选命令即可，确认后会先回位；运动路径必须空闲。回位失败、超时或中断不会进入阻抗。保持原有阻抗漂移/速度保护，不以扩大阈值掩盖原因；回位控制与阻抗观察的阈值分开。当前只通过离线检查，新增回位衔接尚未真机验证。


### 自动回位复测 hold_zlgonasd

已实际回 home 并通过 0.5 秒静止确认。阻抗启动后 0.288287 秒内末端上升 9.395 mm，J4 再次触发程序的 0.02 rad 启动漂移保护；最大关节速度 0.09425 rad/s，未触及 0.15 rad/s 速度护栏。程序请求下电且读回确认，不能将附带旧控制器日志认作本次故障。

这排除了“没有先回 home”作为本次直接原因；固定参考下仍存在可重复漂移。两次约 9.4 mm 的上升是相同关节护栏截断的结果，不代表自然稳定在该高度。候选重心尚未验证正确，不替换默认标定值，不继续重复同参数试验或扩大保护阈值。下一步应核对 RobotAssist 成功程序的实际工具/负载、力控类型、力控坐标和刚度与 SDK 路径，不能仅依据轴排列猜测解决。

独立测试确认现统一为 `y` 加回车，直接回车取消；不需要额外按 e。该简化不改变控制模式、负载或护栏。
