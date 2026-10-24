# Quest 3 + Inspire + ROKAE 操作手册

最新轻推行程调整：`hold_8dr6_xzi` 在 19.395 s 因 J3 偏移 0.500062 rad 触发主机 joint_excursion，停止并确认下电；当时 TCP 偏移 8.16 mm，记录最大 38.48 mm、最大关节速度 0.3833 rad/s，未触发速度检查。轻推阶段（2 s 后）总关节偏移上限现为 **1.0 rad**；baseline 仍 0.50 rad，启动仍 0.04 rad，轻推速度 0.70 rad/s、TCP 100 mm。J1/J3 大幅变化提示冗余姿态运动，放宽行程不是零空间控制修复；若松手后继续自行运动，应停止并核查。新增停止详情会打印具体关节、偏移及阈值。未修改控制器保护、SDK 限速/滤波或负载。


当前独立测试的软件护栏已按要求翻倍：启动关节偏移 0.04 rad、总关节偏移 0.50 rad、TCP 转角 0.30 rad；baseline/启动速度 0.30 rad/s、TCP 位移 60 mm；light-push 在第 2 秒后速度 0.70 rad/s、TCP 位移 100 mm。前 2 秒仍不触碰。回 home 的速度与到位容差、刚度、负载、控制器底层保护未改变。下方旧运行记录中的阈值属于当时配置。


**当前独立阻抗仍未验收通过。** 最新日志确认关节传感器零点标定成功。现启用 `apply_on_impedance_start: true`，启动时通过 SDK `setToolset(结构体)` 设置最新截图负载 **0.96 kg、CoM [-0.05143,0.00823,0.00840] m**，读回校验后上电；不依赖 test 工程的 tool1/tool2 名称，保留当前 TCP/工件坐标及惯量。质量为截图舍入值，不能据此确认完整标定精度。先独立测试，勿直接恢复遥操作：

```bash
conda activate dexteleop
python rokae/impedance_test/run.py --stage hold --duration 5 --execute
```

程序先回 home，随后固定参考观察 5 秒，结束主动停止并下电。先不触碰；出现漂移仍需检查 trace，不能把参数写入成功当成阻抗正常。详见[独立阻抗解决记录](rokae/docs/impedance_resolution.md)。

负载重复设置审查：`apply_on_impedance_start` 控制是否覆盖 **SDK Toolset**，不控制 RT `setLoad`。独立测试此前忽略此开关，现已修复；false 会保留并核验活动工具负载，之后仍按官方用法配置 RT 负载。未发现应用层质量相加，控制器内部重复补偿尚无证据确认或排除。详见[三层负载与检查方式](rokae/docs/payload_review.md)。最新数值以上方配置为准。

独立阻抗测试的回 home 速度已调整：默认 NRT MoveAbsJ 保留 SDK speed=100，在线倍率由 **0.05 提至 0.50（10 倍）**；legacy-rt 的 MoveJ 速度比例同步为 0.50。原启动命令不变，home/hold 两个阶段均生效。实际耗时仍受行程、加减速和控制器限制影响，不承诺恰好缩短至十分之一。到位后仍连续静止确认 0.5 秒，阻抗增益、负载、接触护栏未改。

最新方向性试验：0.0642 kg 仅为软件实验值，不能认定为完整组件质量。八轮新记录确认发送目标与 SDK 指令反馈保持固定（最大平移差 <0.001 mm），仍有方向性回弹问题；当前证据更支持补偿/摩擦偏差，根因未闭环。先核对相同参数下的 RL/SDK 与真实负载，不继续盲目减质量。详见[新增证据与修正顺序](rokae/docs/impedance_startup_audit.md)的“方向不对称与软件质量试验”小节。

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

当前控制器为 **2.3.2**；官方最新 **0.7.1 要求控制器 ≥3.2.1**。项目继续成套使用 `rokae/sdk` 的 0.3.4 头文件和动态库，根目录下载的两个新版仓库仅供对照。详见[SDK 兼容性与审查](rokae/docs/official_sdk_update_review.md)。

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

官方接口核对、历史试验和未验证项见[阻抗启动审查](rokae/docs/impedance_startup_audit.md)。

本手册区分“只遥操作”和“遥操作并采集数据”。所有命令从仓库根目录运行，统一使用现有 `dexteleop` 环境：

```bash
cd /data/Projects/DexRetarget
conda activate /data/miniconda3/envs/dexteleop
```

Quest 发送到本机 UDP 9000；Inspire 串口、机器人网络/TCP/控制器及任务 home 按现场配置确认。
真实机械臂需要匹配当前平台和 Python 的原生 SDK 绑定，准备步骤见 [ROKAE 验收](rokae/docs/commissioning.md)。

若启动提示控制器软限位已关闭，可先运行 `python rokae/examples/06_recover_soft_limits.py` 只读检查。独立的程序恢复入口使用手动单关节 Jog，完成后恢复原限位并保持下电，不自动进入遥操作；默认距离与显式 10° 回退、执行确认及尚未验证的硬件限制见[软限位恢复](rokae/docs/commissioning.md#已关闭软限位的程序恢复)。
恢复默认等待现场物理使能上电（`--manual-power-source operator`）；接有外接使能或示教器时，SDK 不支持代替它进行手动上电。若出现“上下电失败”，可用 `python rokae/examples/01_read_state.py --controller-logs` 只读查询控制器日志。

<a id="teleoperation"></a>

## 模块一：遥操作

### 同时控制机械臂和灵巧手

加载任务配置，关闭相机和数据采集：

```bash
python -m dexretarget \
  --task-config config/tasks/cup.yaml \
  --enable-hand --enable-arm \
  --no-capture
```

当前杯子任务配置为 `home.on_start: true`，启动会先执行任务 home，完成后默认暂停跟随。
cup 使用任务文件 `home.arm_joints` 中的固定 home；启动和 `h` 均使用该目标，不记录每次运行前的位置作为 home。该目标独立于 `config/robots/rokae.yaml` 的 `robot.manipulation_init`。
**默认暂停不代表启动时完全不运动。**先确认初始化整条路径，再启动；示例 home 尚不能视为现场任务已验收姿态。
输入新鲜、程序准备完成后，在终端按 `e` 开始臂手跟随。

成功初始化后，输出应包含：

```text
Outputs: hand=True, arm=True
```

### 另一条路线：机械臂原生阻抗控制

保留位置控制默认值，通过显式参数选择 ROKAE 原生笛卡尔阻抗：

```bash
python -m dexretarget \
  --task-config config/tasks/cup.yaml \
  --enable-hand --enable-arm \
  --arm-control impedance --no-capture --skip-init
```

此命令跳过启动回 home，但仍会连接、请求上电并启动控制。先核实整套末端负载、重心和 TCP；准备完成后按 `e` 跟随。`h` 仍是关节回位。阻抗目标是弹簧平衡位姿，接触时允许偏移；不能保证任何碰撞都不会下电。KWR75B 尚未接入反馈，手指仍为位置控制。

遥操作增益配置为 `config/robots/rokae.yaml` 的 `control.impedance`；`mock_mass`/`mock_damping` 不会设置真机。当前试验负载见本文开头；独立测试的 robotassist profile 会覆盖测试刚度，不更改此处遥操作增益。官方示例对比、参数边界及验收流程见[阻抗路线说明](rokae/docs/impedance.md)。

### 原脚本入口与输出选择

旧 example 入口已移除。下面使用正式入口，不加载任务配置，使用默认 ROKAE 初始化配置，也不启动数据采集：

```bash
python -m dexretarget \
  --enable-hand --enable-arm --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --inspire-port /dev/ttyUSB0 \
  --inspire-baudrate 115200 \
  --inspire-hand-id 1 \
  --no-capture
```

| 输出参数 | 实际行为 |
| --- | --- |
| 只传 `--enable-hand` | 只控制手，**不会连接、上电或控制机械臂** |
| 只传 `--enable-arm` | 只控制机械臂 |
| 同时传 `--enable-hand --enable-arm` | 同时控制臂手 |
| 两个参数都不传 | 默认同时控制臂手 |

因此，仅带 `--enable-hand` 时机械臂不动、没有上电声音，符合该参数的含义。
需要臂手协作时，建议显式写全两个参数。上电请求默认开启；`e` 控制的是跟随门控。
无任务配置的命令不提供任务 home，按 `h` 会提示没有配置 home。

### 键盘操作

| 按键 | 行为 |
| --- | --- |
| `e` | 切换已选臂手的统一跟随使能；使能要求 Quest 输入新鲜 |
| `p` | 明确暂停跟随，保留最后目标，不自动回 home |
| `c` | 重新建立机械臂相对参考 |
| `h` | 已加载任务 home、已暂停且无录制/保存工作时，返回 home；完成后仍暂停 |
| `q` / Ctrl+C | 退出并清理资源 |
| `?` | 显示帮助与当前状态 |

交互终端直接按键，无需回车；管道/非 TTY 每行一个命令。不使用 Quest 按钮或 kill 信号使能。
跟踪过期进入 **Tracking HOLD**：停止新增臂手目标，保留使能意图；连续新鲜 0.20 s 后重新建立机械臂参考并恢复跟随，手部目标用 0.5 s 过渡。手动 `p` 仍需 `e` 恢复。保持最后目标不代表机械锁定或断电，仍可能完成先前接受的运动。
`Arm state: hold_disabled` 表示手动/启动暂停，不是速度超限。正常丢帧显示 `Tracking HOLD`，不再自动关闭键盘使能；数据中记录 `tracking_hold` 和 `tracking_recovered_recenter`，HOLD 段不作为有效训练动作。驱动故障、实时 safe_stop 仍然报错停止。
完整流程见[遥操作说明](docs/usage.md#teleoperation)。

### 初始化、home 与恢复速度

默认 init 使用 SDK 速度设置 250、在线倍率 0.50；cup 的启动/home 使用任务速度设置 150，并使用同一在线倍率。`joint_speed * 1000` 才是 SDK 设置值，`0.25` 不表示关节速度的 25%。独立软限位恢复默认 Jog 速率为 5%，可用 `--jog-rate 0.01` 至 `0.10` 覆盖。参数位置、命令及验证范围见[速度配置](docs/configuration.md#初始化与恢复速度)。

<a id="rokae-recovery"></a>

### ROKAE 恢复与回初始化位置：命令速查

以下独立脚本从仓库根目录、`dexteleop` 环境运行。先退出正在控制机械臂的遥操作程序，避免两个进程同时控制。恢复与回位均不提供自动避障，需要检查整条路径及手指、相机、线缆的空间。

| 当前情况 | 使用入口 |
| --- | --- |
| 查看关节、模式、上下电状态和故障日志 | `01_read_state.py --controller-logs` |
| `controller soft limits enabled: False` | 先用 `06_recover_soft_limits.py` 恢复保护 |
| 软限位开启，但关节接近边界 | 用 `05_move_to_init.py --recover-joint-limits` 审查回退计划 |
| 回通用遥操作初始化姿态 | `05_move_to_init.py --pose teleop` |
| 笛卡尔运动在奇异附近失败，需要改变关节构型 | 条件允许时使用 `05` 的关节回位；不是继续发送笛卡尔目标 |
| 回 cup 任务 home | 加载 cup 的主程序中，暂停且无录制/保存后按 `h` |

#### 1. 只读检查与日志

```bash
python rokae/examples/01_read_state.py --controller-logs
python rokae/examples/06_recover_soft_limits.py
```

第二条不带 `--execute`，只读取状态并输出恢复计划，不发送上电、模式切换、限位设置或运动命令。控制器必须空闲。

#### 2. 软限位保护已经关闭：先恢复保护

如果所有关节都在原限位内，计划应显示 `all joints are inside; enable existing soft limits without Jog`：

```bash
python rokae/examples/06_recover_soft_limits.py --execute
```

输入 `RECOVER` 后，脚本停止运动、请求下电、切到手动模式，恢复并验证原限位；此分支不 Jog、不请求上电。结束后不自动回位或启动遥操作。

如果有一个关节轻微越界，先查看向内回退 10° 的计划，再执行：

```bash
python rokae/examples/06_recover_soft_limits.py --clearance-deg 10
python rokae/examples/06_recover_soft_limits.py \
  --clearance-deg 10 --jog-rate 0.05 \
  --manual-power-source operator --execute
```

仅支持一个关节越界且越界不超过 1°；`--clearance-deg` 范围 0.5–10°，只影响需要回退的越界轴，**不会将已经在限位内的轴再推入 10°**。`--jog-rate 0.05` 是 5% Jog 速率。

输入 `RECOVER` 后，等待 `WAITING FOR MANUAL ENABLE` 提示，再按住现场三段使能的中间档；保持到停止提示后释放。默认不由 SDK 请求手动上电。只有确实没有外接使能/示教器且控制器支持时，才使用 `--manual-power-source sdk`；它不是绕过物理使能的办法。出现 `RECOVERY INCOMPLETE` 时重新读取日志与限位状态，不要直接启动遥操作。

恢复后重新检查：

```bash
python rokae/examples/06_recover_soft_limits.py
```

确认显示软限位已开启，再执行下面的初始化。实现和硬件验证范围见[软限位恢复说明](rokae/docs/commissioning.md#已关闭软限位的程序恢复)。

#### 3. 回通用初始化位置

先预检，再执行；不带 `--execute` 不发送运动：

```bash
python rokae/examples/05_move_to_init.py --pose teleop
python rokae/examples/05_move_to_init.py --pose teleop --execute
```

按提示输入 `MOVE`。如果提示关节接近软限位，则改用以下流程，先检查回退目标：

```bash
python rokae/examples/05_move_to_init.py --pose teleop --recover-joint-limits
python rokae/examples/05_move_to_init.py \
  --pose teleop --execute --recover-joint-limits
```

该选项会在确认后暂时关闭软限位，以关节运动回退受影响轴，再下电并恢复原限位，随后继续初始化流程。**这是会改变保护状态并实际运动的维护流程，不是普通初始化开关；实机恢复尚未完成验收。**初始软限位已关闭时它仍会拒绝执行，必须先走 `06`。

当前回退触发裕量默认 0.15 rad（约 8.59°），回退目标距边界 0.35 rad（约 20.05°），以机器人配置为准。例如 J4 距上限约 0.49° 时，即使 `06` 成功恢复保护，仍会触发 `05` 的边界回退检查。

需要显式降低初始化速度时：

```bash
python rokae/examples/05_move_to_init.py \
  --pose teleop --speed 100 --online-speed-scale 0.25 --execute
```

`--speed` 是 SDK 设置值，不是 rad/s 或百分比；默认 SDK 250、在线倍率 0.50。这些速度参数不改变恢复 Jog 的速率，也不替代对路径的检查。

初始化目标的归属如下，避免混用：

| 入口 | 目标配置 |
| --- | --- |
| `05 --pose teleop`（默认） | `config/robots/rokae.yaml` → `robot.initial_joint_move.joint_position` |
| `05 --pose manipulation` | 同文件 → `robot.manipulation_init.joint_position`，历史独立演示预设 |
| 主程序加载 cup 后启动回位或按 `h` | `config/tasks/cup.yaml` → `home.arm_joints` |

`manipulation` 不等于当前 cup home；以脚本打印的 `target joints` 和实际 TCP 为准，不要仅凭预设名判断手掌朝向。

#### 4. 离开奇异附近的构型

当前没有“一键自动检测并脱离任意奇异位姿”的命令。`05` 使用关节空间 `MoveAbsJ`，不依赖笛卡尔逆解，可在路径允许时改变构型、回到选定初始化姿态：

```bash
# 退出遥操作后，先检查状态和回位路径
python rokae/examples/01_read_state.py --controller-logs
python rokae/examples/05_move_to_init.py --pose teleop
# 确认目标与路径后执行关节回位
python rokae/examples/05_move_to_init.py --pose teleop --execute
```

若同时存在软限位关闭或靠近边界，先按前面的对应分支处理。若控制器有未解除故障，此命令不会清除报警或绕过安全联锁。

关节运动能够经过笛卡尔奇异构型，不代表任意路径都安全；脚本也不保证目标对所有 TCP、安装与任务都远离奇异。完成后确认关节状态和末端姿态，再重新启动遥操作。不要把 `hold_disabled` 或 Tracking HOLD 当作奇异证据：它们首先表示使能/跟踪状态，应结合控制器日志判断实际原因。

<a id="data-collection"></a>

## 模块二：数据采集

同时启用臂手和双相机采集，显式加载任务配置：

```bash
python -m dexretarget \
  --task-config config/tasks/cup.yaml \
  --enable-hand --enable-arm \
  --capture
```

任务配置引用独立的 [collection 配置](config/collection/collection.json)，后者引用[相机配置](config/cameras/realsense.json)。
当前默认 D435i 腕部、D455 全局 RGB 各 640×480、30 FPS，训练采样网格为 20 Hz。
相机与反馈源提前启动、预热，按 `r` 才开始写入当前 episode。

推荐顺序：**`e → 预调整 → r → 示范 → s/f → p → 等待保存完成 → h`**。

| 按键 | 采集行为 |
| --- | --- |
| `r` | 从当前位置开始录制，不要求每段从固定点或 home 开始 |
| `s` / `f` | 成功 / 失败结束当前段，后台排空、对齐、保存；**不会自动暂停跟随** |
| `p → e` | 录制中暂停再继续；观察继续记录，暂停处切断训练有效片段 |
| `d` | 将本会话上一段完整 episode 移入可恢复回收站 |

录制或保存中不能回 home；保存中仍可按 `p` 暂停。回位完成后保持暂停，再按 `e` 开始下一次操作。
按 `q` 或中断退出时，未结束的段按失败/中断处理；完成保存的失败段也不默认用于训练。

原始数据默认保存在 `data/raw/quest_inspire_rokae/`，逐段为 HDF5 + 元数据。
写盘失败或未完成段保留 `.partial-*`，不当作完整训练数据。字段、对齐、校验、回收/恢复和 LeRobot 导出统一见[数据说明](data/README.md)。

## 常见启动问题

### `/dev/ttyUSB0: Permission denied`

在当前终端检查实际生效的用户组和设备权限：

```bash
id -nG
ls -l /dev/ttyUSB0
```

本机已确认设备为 `root:dialout`、权限 `660`，`neal` 已在系统的 `dialout` 成员表中；旧登录会话可能尚未生效。
在运行遥操作的终端刷新组权限，然后在新 shell 中重新激活环境：

```bash
newgrp dialout
conda activate /data/miniconda3/envs/dexteleop
id -nG
```

确认输出含 `dialout` 后重新运行。注销系统后重新登录也会刷新组权限。
若设备不存在，应先确认连接与实际串口路径，再通过 `--inspire-port` 覆盖。

### 手正常，机械臂不动

先确认命令同时带 `--enable-hand --enable-arm`，以及成功启动输出为 `hand=True, arm=True`。
若初始化报错，应先解决 SDK/连接等具体错误；成功初始化后默认暂停，需按 `e` 跟随。
这与仅传 `--enable-hand`、完全不初始化机械臂的情况不同。

### `Restarting with conda C++ runtime`

这是入口优先加载当前 Conda C++ 运行库的提示；若随后出现 traceback，应按后面的具体异常排查。

### `_rokae_cpp` 加载失败或出现 `circular import`

本机为 x86_64，旧文件 `_rokae_cpp.cpython-310-aarch64-linux-gnu.so` 属于 ARM64，不能被本机 Python 加载。
已补齐同版官方 x86_64 SDK、按架构选择的构建配置和本机扩展；父包延迟加载，避免扩展缺失时出现误导性的循环导入提示。

先验证导入，下面的命令不连接、不上电机器人：

```bash
python -c 'import rokae; print(rokae.SDK_VERSION); print(rokae.module_file())'
```

本机应输出版本 `0.3.4`，模块路径以 `_rokae_cpp.cpython-310-x86_64-linux-gnu.so` 结尾。
更换 Python 版本或平台后需重新编译：

```bash
conda activate /data/miniconda3/envs/dexteleop
bash rokae/build.sh
```

`PREEMPT_RT` 提示与导入异常是不同问题。原生模块导入和离线测试通过，不代表实际 1 ms 控制时序或真机运动已经验收。
依赖、SDK 来源及本次验证结果见[原生构建与排错](rokae/docs/native_build.md)。

模块职责见[架构](docs/architecture.md)，配置覆盖规则见[配置管理](docs/configuration.md)，实际执行过的测试及未验证硬件项见[验证报告](docs/refactor_report.md)。

<a id="impedance-first-trial"></a>

## 阻抗控制首次试用：检查结果与启动指令

**本分支是 ROKAE 原生笛卡尔阻抗。弹簧关系在模式运行时就存在，并非碰到物体才切换控制器。**固定平衡点时，外力使末端偏离，控制器产生恢复作用；外力撤去后趋向平衡点，实际回归速度和振荡需要真机验证。机械臂柔顺不代表手指柔顺，也不保证所有接触都不下电。

本次复查修复了阻抗回调读取 TCP 失败仍使用缓存位姿的问题，现会报错并结束实时循环。具体实现、官方示例及其他 xMate 使用资料见[阻抗代码审查与调参依据](rokae/docs/impedance.md#再次代码审查与调参依据2026-10-22)。软件测试不能代替当前装配的接触验收。

### 先确认实际负载

RH56DFX-2R 约 540 g、D435i 约 75 g、KWR75B 约 280 g，三件合计 895 g；含额外法兰、支架及随动线缆，整套估计 **1.1–1.45 kg**。来源与估算假设见[末端重量说明](rokae/docs/impedance.md#当前末端装配重量估算)。1.25 kg 只是估算中心值，尚未作为标定值写入控制器。

还必须确认质心、惯量和 TCP。相机侧置意味着质心未必在法兰轴线上；不能从照片推导可信数值。`mock_mass` 不是真机负载，`mock_damping` 不是真机阻尼。当前通过 `robot.payload` 配置控制阻抗启动时写入质量/重心，再读回 SDK Toolset，并调用实时 `setLoad`；TCP、参考系和惯量保留读回值。启动打印 `active load: mass=... cog=... inertia=...`，读回匹配仅验证配置传递，不能证明辨识精度或接触稳定性。

### 第一阶段：只启用机械臂，隔离手指误握问题

先确认软限位开启、控制器空闲、当前位姿及空间合适；这里只读查询，不恢复或移动：

```bash
conda activate /data/miniconda3/envs/dexteleop
python rokae/examples/01_read_state.py --controller-logs
python rokae/examples/06_recover_soft_limits.py
```

负载与 TCP 已核实后，从当前位置启动，不执行任务 home、不连接手指串口、不采集相机：

```bash
python -m dexretarget \
  --task-config config/tasks/cup.yaml \
  --enable-arm --arm-control impedance \
  --no-capture --skip-init
```

只传 `--enable-arm` 表示仅控制机械臂，但手、相机、力传感器等装配重量仍全部计入负载。命令会请求上电并启动阻抗；`--skip-init` 只跳过启动关节回位。默认暂停 Quest 跟随，可先检查固定平衡点下是否漂移，再进行受控小位移接触；确认响应稳定后，准备新鲜 Quest 腕部数据并按 `e` 开始跟随。`p` 暂停新增目标，`q` 退出；首次接触时不要按 `h`，它仍执行关节回位。

启动日志应包含 `cartesian impedance stiffness N/m, Nm/rad`，当前值是 `[300,300,300,15,15,15]`；另有 `cartesian impedance leaves the arm angle free`，因此必须观察肘部空间。期望力六项为零，平衡点偏差限幅为 40 mm / 0.30 rad。这些是尚未实机验收的试验值，没有因这次审查擅自提高。

### 第二阶段：臂手遥操作及采集

机械臂接触检查完成后，退出第一阶段进程，再运行：

```bash
python -m dexretarget \
  --task-config config/tasks/cup.yaml \
  --enable-hand --enable-arm --arm-control impedance \
  --no-capture --skip-init
```

要采集时将 `--no-capture` 替换为 `--capture`，依旧按 `r` 开始、`s/f` 结束。手指仍为位置控制，Quest 遮挡误握尚未由本次阻抗修改解决。切回位置路线应退出后重启，用 `--arm-control pinocchio`；不支持运行中热切换。

本次软件验证：x86 原生绑定重编译成功；215 项测试及 9 项子测试通过；阻抗模拟臂手、Quest UDP、合成双相机及保存链路重跑通过。新增无效反馈拒绝测试，但真实 SDK 状态读取失败的停止行为尚未做硬件故障注入。当前仍待确认的条件是实际负载/TCP、接触稳定性及实时网络；不能将软件测试通过视为真机已验收。

### 当前现场故障：首次阻抗试用已触发保护，暂勿直接重试

2026-10-22 的只读查询发现最近阻抗记录为 `41451: Axis3 Momentum OverLimit`，控制器随后下电；用户启动日志的负载为 **0 kg**。这不是正常弹簧回弹，`active` 也不能证明电机仍在执行。

现已增加**零/无效负载在上电前拒绝启动**，以及运行中（含暂停）每 0.5 s 检测控制器下电/模式异常并带日志退出。估计重量不能替代质量、重心、惯量及 TCP 标定。上述启动命令保留供配置完成后使用，不能把此次代码修复当作真机振动已解决。故障证据和待排查项见[现场排查记录](rokae/docs/impedance.md#现场抖动与保护下电排查2026-10-22)。

### 最新现场状态：未按 e 也发生运动和保护

2026-10-23 00:25，startup_paused 下仍发生明显关节运动并触发 Axis1 Momentum OverLimit；显式 RT setLoad 并未解决故障。`hold_disabled` 暂停的是跟随，不是电机或阻抗。已将 HOLD 提前至实时循环启动前，并补 TCP/目标记录；未做真机验证。**暂停完整 VR 阻抗试验，优先核查控制器动力学/传感器及七轴零空间行为。**

### 先前现场状态：负载读回一致后仍会触发保护

2026-10-23 00:21，按 e 后无接触移动手腕仍触发 10079 力矩指令超限与 41451 Axis3 Momentum OverLimit。以下命令仅保留为入口说明，**不代表当前真机已通过验收，请勿直接反复尝试完整 VR 跟随**。已补显式 RT setLoad 和有界关节状态历史；尚未验证它们能消除故障，详见[阻抗排查记录](rokae/docs/impedance.md)。

### 历史记录：原始辨识与旧换轴候选（非当前参数）

用户随后授权按重心轴序错误假设做一次独立固定参考试验。以下命令只覆盖本次重心，当时质量为 0.939386 kg；当前脚本将读取新的 YAML 质量，不能用此命令复现当时条件。输入 `y` 并回车 后会直接请求上电及阻抗，不需要按 e；全程 5 秒不触碰、先检查自发漂移。启动后先以位置控制回任务 home，再连续确认静止 0.5 秒后进入阻抗。停止阈值保持原值，退出不主动恢复 SDK 负载。完整说明见[候选试验](rokae/impedance_test/README.md#用户授权的重心候选对照试验)。

```bash
conda activate dexteleop
python rokae/impedance_test/run.py --stage hold --duration 5 \
  --experimental-cog-mm 12.17669684527 8.60984186271 52.71043598571 --execute
```

当时复核：用户确认 RobotAssist 使用未修改的原始辨识数据可正常阻抗。几何复核尚未证实 `[12,8,52] mm` 是正确重心；上面的参数仅用于用户授权的候选试验，未替换默认标定值，后续该候选已在真机试验并触发漂移护栏。遥操作代码已对齐官方初始化顺序，硬件漂移尚未验证解决；分析与验证边界见[重心与阻抗路径复核](rokae/docs/payload_review.md)。

用户已用 RobotAssist 完成全组件空载/带载两步辨识。当时 `config/robots/rokae.yaml → robot.payload` 记录质量 **0.93938639463 kg** 和重心 **[-52.71043598571, 12.17669684527, 8.60984186271] mm**（配置中已换算为米）。此前的重量估算不再作为本装配参数。

`robot.payload.apply_on_impedance_start: true` 时，阻抗启动前将配置质量/重心写入 SDK Toolset，并读回核验；false 时只核验活动值。保留当前 TCP、工件坐标与惯量。当前为 true，启动时应用最新截图值，尚未验收漂移问题；以下遥操作命令待独立验收通过后使用：

```bash
conda activate dexteleop
python -m dexretarget \
  --task-config config/tasks/cup.yaml \
  --enable-arm --arm-control impedance \
  --no-capture --skip-init
```

程序在阻抗上电前应用并核对 SDK 质量/重心，打印实际负载和 TCP；设置失败或读回不匹配会拒绝启动。该设置不会保存回 RobotAssist 的 RL 工具，也不适用于更换后的装配。启动后保持暂停，`e` 使能/暂停，`p` 暂停，`q` 退出。`--skip-init` 只跳过回初始位置，并不禁止电机上电。

截图惯量仍为零，TCP 尚无标定证据；质量/重心匹配不代表已经解决振动，也不代表接触测试通过。完整参数、单位和审查说明见[现场辨识与阻抗核对](rokae/docs/impedance.md)。无需重新拆装来执行软件配置核对。两步辨识 SDK 接口的历史审核保留在同一文档中。

### 当前下一步：只读诊断，不重新启动阻抗

未使能跟随就发生快速运动的问题尚未解决。此阶段不要重复运行 `--arm-control impedance`。先退出遥操作程序，再保存只读报告：

```bash
conda activate dexteleop
LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
python rokae/examples/01_read_state.py \
  --faults-only \
  --json-output data/diagnostics/rokae_snapshot_01.json
```

该命令读取状态、基坐标、软限位及最多 10 条告警/错误日志；不启动实时控制、不上电、不清报警、不设置负载。文件存在时拒绝覆盖，后续改用 `_02.json`。报告中的 YAML 负载明确标记为配置值，不冒充控制器读回。需配合 RobotAssist 导出的完整故障日志，才能进一步分析故障前过程；这份静止快照不能证明阻抗可运行。原配置下的稳定性尚未通过；本次候选试验命令见上面的负载辨识小节。

### 独立 SDK 固定参考阻抗试验（脱离 Quest）

**最新结果：hold_1jjub90t 在 0.110 秒内无输入上升约 5.36 mm，触发软件关节速度护栏，程序主动停止并下电。尚未完成静止观察，不重复以下原配置 hold 命令，勿提高护栏。本次只使用上面带 experimental-cog-mm 的候选试验命令。** 已增加明确的停止来源分析，见 [记录](docs/validation/run_history.md)。

新增 [rokae/impedance_test](rokae/impedance_test/README.md)，直接调用官方 SDK。读取统一质量/重心和 Cup home，提供 `inspect`、`home`、`hold` 三个阶段。默认无 `--execute` 只打印计划，不连接。

```bash
conda activate dexteleop
# 回指定 Cup home，仅回位，不启动阻抗；需要输入 y 并回车
python rokae/impedance_test/run.py --stage home --execute
# 确认回位完成后，固定参考阻抗 5 秒；第一次全程不触碰；输入 y 并回车
python rokae/impedance_test/run.py --stage hold --duration 5 --execute
```

这是面向现场人员的隔离诊断入口，不是已验证的故障修复。5 秒静止观察无异常且日志正常后，才考虑按独立 README 运行较长的轻推试验。两阶段会请求上电，结束后请求下电并读回；Ctrl+C 为软件停止请求，物理停止装置需可用。初始自行漂移、越界或保护发生时停止本轮，不反复重启。1 ms 回调记录保存到 `data/diagnostics/impedance_test/`。用户已执行原配置 hold 并触发护栏；候选重心试验尚未执行。


### 候选试验 hold_p03k9qz4 与自动回位更新

本次 371 个样本覆盖 0.367093 秒，固定参考下末端上升 9.491 mm，最大相对参考位移 9.812 mm；最大关节速度 0.07749 rad/s。停止原因是测试程序 `startup_joint_drift`（启动前两秒任一关节偏移超过 0.02 rad），随后程序请求下电，读回确认。不是本次记录证明的控制器速度/加速度保护；附带的旧日志不能归因于本次。该轨迹尚未到稳态，不能据此确认重心正确。

现在 `hold` 自动执行位置控制回 home → 连续静止确认 → 采集新的固定参考 → 阻抗观察。使用上面的候选命令即可，确认后会先回位；运动路径必须空闲。回位失败、超时或中断不会进入阻抗。保持原有阻抗漂移/速度保护，不以扩大阈值掩盖原因；回位控制与阻抗观察的阈值分开。当前只通过离线检查，新增回位衔接尚未真机验证。


### 自动回位复测 hold_zlgonasd

已实际回 home 并通过 0.5 秒静止确认。阻抗启动后 0.288287 秒内末端上升 9.395 mm，J4 再次触发程序的 0.02 rad 启动漂移保护；最大关节速度 0.09425 rad/s，未触及 0.15 rad/s 速度护栏。程序请求下电且读回确认，不能将附带旧控制器日志认作本次故障。

这排除了“没有先回 home”作为本次直接原因；固定参考下仍存在可重复漂移。两次约 9.4 mm 的上升是相同关节护栏截断的结果，不代表自然稳定在该高度。候选重心尚未验证正确，不替换默认标定值，不继续重复同参数试验或扩大保护阈值。下一步应核对 RobotAssist 成功程序的实际工具/负载、力控类型、力控坐标和刚度与 SDK 路径，不能仅依据轴排列猜测解决。

独立测试确认现统一为 `y` 加回车，直接回车取消；不需要额外按 e。该简化不改变控制模式、负载或护栏。
