# 官方 SDK 下载、版本与持续上漂复核（2026-10-23）

## 下载与运行库边界

按用户要求完整克隆到项目根目录，并从官方 GitHub Releases 补齐 Linux x86_64 二进制；直连成功，没有使用镜像，也没有安装或覆盖 `dexteleop` 中的包。

| 项目 | 路径 | 当前 main 提交 | 最新公开 Release |
| --- | --- | --- | --- |
| C++ | `xCoreSDK-CPP/` | `061c2fb11f360f1585aae2776a4bb713b9e7c49c` | 0.7.1，发布于 2026-06-12 |
| Python | `xCoreSDK-Python/` | `a135059bb14299983798fd7f18e9b3fcacf6813b` | 0.7.1，发布于 2026-06-15 |

main 的合并提交日期均为 2026-07-24；不要与 CHANGELOG 中版本条目的日期混淆。C++ 最新提交与项目此前 `official_sdk_audit/061c…` 的审核副本相同，并非这次才发布的阻抗修复。

- [官方 C++ 仓库](https://github.com/RokaeRobot/xCoreSDK-CPP)、[C++ Release](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v0.7.1)：库已落入 `xCoreSDK-CPP/lib/Linux/x86_64/`。
- [官方 Python 仓库](https://github.com/RokaeRobot/xCoreSDK-Python)、[Python Release](https://github.com/RokaeRobot/xCoreSDK-Python/releases/tag/v0.7.1)：扩展已落入 `xCoreSDK-Python/Release/linux/`，含 CPython 3.8–3.12。
- 两份压缩包大小及 SHA256 均与 GitHub API 的 asset digest 一致；两个仓库 `git fsck` 通过。提交、下载链接和校验值见[下载记录](../../docs/validation/run_history.md)。Python 包中五份已有 stub 仅换行符不同，保留仓库原文件。
- 已在 `dexteleop` 编译并链接官方 C++ `cartesian_impedance_control` 示例，未运行；Python 3.10 扩展导入及 `Load(mass,cog,inertia)` 构造成功，未创建机器人连接。官方 Python 示例中注释掉的 `Load()` 无参写法与当前二进制构造签名不符，不能复制后直接执行。
- `readelf`/`ldd` 确认现有独立测试仍加载 `rokae/sdk/lib/x86_64/libxCoreSDK.so.0.3.4`。新目录是独立上游 checkout，加入项目忽略项，项目自有源码检查不扫描其内部文件。

## 更新了什么，能否直接替换

两个最新仓库 README 都要求 **xCore ≥3.2.1**；当前机器人日志为 **2.3.2**，不满足该要求。CHANGELOG 的 0.7.0 条目写 ≥3.2.0，当前发行包按更严格的 README 要求处理。不能重命名新库覆盖旧库或跳过版本检查。

[C++ CHANGELOG](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/CHANGELOG.md) 中相关变化：

| 版本 | 相关变化 | 对本项目的含义 |
| --- | --- | --- |
| 0.4.0 | 非实时力控接口、`calibrateForceSensor` | 提供更接近 RL 的控制路径和关节力传感器标定入口；本地 0.3.4 头文件没有这些接口 |
| 0.4.1 | 碰撞检测负载设置修复 | 不是本次 `startup_joint_drift` 的已证实修复；该记录不能外推为全部阻抗负载问题 |
| 0.7.0 | 阻抗力/速度限幅、力控带宽与摩擦补偿参数；实时延迟优化 | 新参数位于 `force_control.h` 的非实时力控类，不应当作旧 RT API 可用的新阻尼旋钮 |
| 0.7.1 | `setToolInfo/setWobjInfo` 的旋转角度/负载错误修复；关节跟随奇异处理优化 | 当前程序调用 `setToolset` 和 RT `setLoad`，没有调用上述新工具接口，不能认定命中了这个 bug |

0.4.1 的声明兼容范围覆盖当前控制器，但本轮官方公开 Release 列表没有对应二进制。若要隔离版本差异，先获取厂商确认适配 **2.3.2 / xMatePro3** 的完整头文件和库，单独构建对照入口；不能混用 0.7.1 头文件与 0.3.4 动态库。Python 0.5.0 的兼容条目与同号 C++ 不同，更不能只凭版本号互换。

## 三条控制路径需要区分

1. **RobotAssist RL**：截图 `FcInit tool1,wobj0,0 → SetControlType 1 → SetCartCtrlStiffVec → FcStart`，在控制器内运行。
2. **新版 SDK 非实时力控**：`forceControl().fcInit(world) → setControlType(1) → setCartesianStiffness → fcStart`。官方 [C++ 示例](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/example/force_control_commands.cpp) 和 [Python 示例](https://github.com/RokaeRobot/xCoreSDK-Python/blob/main/example/force_control_example.py) 展示了这一流程。它更接近 RL，仍需核对工具与初始化参数；示例含六轴机型、期望力和工具旋转，不可原样用于当前七轴实机。
3. **现有项目实时阻抗**：`RT setLoad → setCartesianImpedance → startMove(cartesianImpedance) → 1 kHz CartesianPosition`。它是正确的公开阻抗接口类型，但与前两条路径的内部状态初始化不能直接画等号。

最新版 [RT 笛卡尔阻抗示例](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/example/rt/cartesian_impedance_control.cpp) 改为 NRT 回位、切换后读取 RT 位姿、清空旧状态队列、先注册回调再启动运动。旧版使用 RT MoveJ。项目已采用显式 RT TCP、实时状态作起点和先注册回调的顺序。本次新增 `latest-compatible` 启动路径，使用 0.3.4 已有 NRT MoveAbsJ 接口回位，并有界清空旧队列、等待新反馈；`legacy-rt` 保留旧路径。此差异并非上漂的已证实根因，尚待真机对照。

新官方力矩示例的主机阻抗代码甚至把阻尼矩阵乘数设为零；不能把“官方示例”当成当前装配下调好的稳定参数集。此次未移植该控制器，也未执行其大幅运动例程。

## “怎么改负载都向上”的证据

不能再仅凭上漂方向反复改变 XYZ 符号。也不能从这个现象直接推断负载接口没有作用：几次质量基本相同，修改的主要是质心；同一 home 下这些修改可能不足以改变总残余力矩的方向。

以下三次均为 home + world + 六项 100 刚度，比较固定时刻，避免相同护栏造成终点位移相似的误导：

| 记录 | 重心 mm | 质量 kg | 0.1 s 上升 mm | 0.2 s 上升 mm |
| --- | --- | --- | --- | --- |
| hold_ysb3lesz | [-26.90277,23.48004,-41.45851] | 0.968391 | 1.895 | 5.278 |
| hold_1bci3ms9 | [-28.30,22.43,-38.17] | 0.9642 | 1.906 | 5.238 |
| hold_81v3kce7 | [0,0,52] | 0.9642 | 2.156 | 5.886 |

方向和曲线量级相近，但不能说完全相同。最新一轮 274 帧、0.270240 s，上升 8.306 mm，J4 达 0.020035 rad 后程序护栏终止，已读回下电。启动前后最近十条告警相同。数据及理想重力计算见[对照记录](../../docs/validation/run_history.md)。

进一步检查：

- 三次发送的 TCP 参考在各自试验中恒定。首帧平移误差分别约 0.058、0.051、0.063 mm，未看到目标初始就向上错开数毫米；这不能验证控制器内部目标解释或全部状态时序。
- `tau_c` 在上述轨迹中全为零，`q_c` 进入阻抗后也出现全零。现有记录不能将它们当作实际伺服力矩、关节执行目标，更不能据此得出“控制器在发零关节角”。`tau_m` 是测量反馈，也不能单独辨识补偿残差。
- 回调约 1 ms 不代表控制器反馈绝对新鲜。[官方 issue #4](https://github.com/RokaeRobot/xCoreSDK-CPP/issues/4) 报告过 1 ms 订阅的延迟；公开非 PR 问题 #1/#2/#3/#4/#6 中未找到本机上漂的确定修复。固定参考试验不依赖反馈更新运动目标，因此时延如何导致当前现象仍需证据。

### 只有 Z 重心并不是只在 Z 方向控制负载

负载重力在基座正装条件下为 `F_B=[0,0,-mg]`；法兰系中的重力为 `F_F=R_BF^T F_B`，重力矩为 `τ_F=r_F×F_F`。把重心写成 `[0,0,0.052]` 只改变力臂，不改变重力的方向。

按最新 home 姿态与 0.9642 kg 理想计算：法兰系重力约 `[4.9584,0,8.0550] N`。均值重心与轴线 52 mm 两种假设的法兰重力矩差约 **0.305 N·m**。这是几何假设下的法兰力矩差，既不是实测残余力矩，也不是每个关节的补偿变化。即使设置生效，仍可能被其它补偿误差主导而持续同向运动。

## 当前判断与后续隔离顺序

目前证据支持“静止参考下存在重复的控制/补偿偏差”，不足以锁定为 CoM、固件 bug、零偏或初始化中的某一个。停止盲目交换轴和增加质量。优先排查：

1. **实际工具是否一致。** 以能够正常运行的 RL 工程为基准，核对成功执行时的完整 tool1 定义及同一 home 表现。此前 SDK 查询的工具表与截图不一致，tool2 未定义也仍在日志中。这一差异不能用手工重心均值代替。
2. **零偏与动力学模型。** 新 SDK 的 `calibrateForceSensor` 和 [官方力矩排查](https://docs.rokae.com/docs/SDK/cpp/实时模式常见问题/cpp_rt_torque_control/) 将传感器标定、正确负载、RL/拖动对照列为检查方向。它标定的是机器人关节力传感器，不是外接坤维传感器，也不是重新辨识质量。要求负载准确；不得用当前猜测的 52 mm 把真实负载误差吸收进零偏。官方文档未证明 FcInit 自动完成这一步，不能凭 RL 成功推断其自动标定。
3. **隔离 RT 与控制器内路径。** 获取兼容 SDK 后，在同一工具、home、刚度下先做非实时 `fcInit/fcStart` 对照，再比较 RT。非实时稳定而 RT 漂移，才更有力指向 RT 初始化/负载通道/版本；两者均异常，则优先负载、零偏和模型。此对照尚未执行。

本轮下载和离线检查完成，不等于已解决实机漂移。未更改质量、重心、刚度、护栏、控制器固件或机器人状态。

## 兼容移植的代码审查（2026-10-23）

- 独立试验默认 `latest-compatible`：NRT home 使用 speed=100 / online_scale=0.05。仅 idle 不判定成功，还核对目标关节、速度、执行事件和供电；60 秒超时/取消/错误请求停止并清空队列，外层请求下电。到位后还要连续静止 0.5 秒。
- RT 参考在 TCP、负载、刚度、期望力和滤波配置之后取得。新共享 helper 清空队列最多 128 帧/50 ms，再等待新帧最多 20 ms；断流或持续积压拒绝启动。SDK 调用本身阻塞不能由这些主机时限硬中断。
- 主遥操作阻抗将负载/力控参数设置前移到初始参考之前，也使用共享 helper；初始化上电后失败会请求下电。固定参考不随漂移重定；Quest 跟随逻辑未被更换。
- 独立原生程序运动前检查 SDK=0.3.4、控制器=2.3.2、机型=xMatePro3。未知版本仅允许只读 inspect；版本变化需重新审查。新参数协议 v3 防止旧二进制静默忽略启动选项；原生仍接受 v1/v2 并使用旧启动方式。
- 没有移植 0.7.1 独有 `ForceControl` 或 `calibrateForceSensor` 符号，没有替换固件/SDK 库、猜测轴排列或放宽保护。NRT 回 home 后仍是 RT 笛卡尔阻抗，不能称为完整复现 RL `FcInit/FcStart`。
- 审查时 YAML 曾为 `[0,0,-0.052]` m，最后 dry-run 已变为 `[0,0,+0.052]` m。此次代码修改未覆盖该数值，注释改为以 YAML/manifest 为准；正负号及原始辨识坐标均新增原样传递测试。固定负载再比较启动路径，避免混入多个试验变量。

本次构建与离线验证记录见 `docs/validation/impedance_startup_backport_20261023.json`（仓库根目录相对路径）。这些检查不验证真实重力补偿、阻尼、零偏、1 ms 实时稳定性或上漂根因；仍需现场固定参考对照。

## 现场反馈后的启动订阅修正

`hold_pbbkqpew`：NRT 回位和静止检查通过；尚未读取阻抗初始参考、未执行 startMove，就因 `realtime state backlog did not drain before impedance` 退出。console.txt 有 `power_off_readback=1`，这是本程序前置检查触发的退出，不能作为力控碰撞或漂移判断。

前述 128 帧/50 ms 清队列方案已撤销。原实现把连续读取非空数据解释为积压，日志没有单次耗时/帧序号，无法区分正常持续流、接收开销与真实积压。官方新 helper 的循环不能直接作为旧版 SDK 下可靠的启动门槛。本地头文件说明 updateRobotState 返回数据长度，0 表示超时，但并未提供此处源时间戳或帧序号保证。

修正后的共享入口 `receive_startup_state` 只用于 startMove 之前：停止旧接收 → 配置后创建 1 ms 新订阅（官方接口等待首帧）→ 三次 updateRobotState(20 ms)，每次必须成功，读取阶段总耗时超过 60 ms 则拒绝。调用方仍检查 TCP/关节反馈，独立测试还检查 home 偏差/速度，运行中不追随漂移重定参考。此方式避免依赖“队列为空”，不能证明报文绝对年龄或机器人动力学正确。失败清理仍停止接收并请求下电，未修改负载、刚度或运行护栏。

新增离线测试覆盖持续非空流正常完成、首帧缺失、第二帧断流、SDK 异常、订阅异常、读取总耗时超限。真机订阅初始化与原有上漂问题仍需复测。

接口依据：[官方方法说明](https://docs.rokae.com/en/docs/SDK--0-7-x-AR/cpp/cpp_method/)；旧版行为以本地 `rokae/sdk/include/rokae/robot.h` 为准。本次未连接或驱动真机。
