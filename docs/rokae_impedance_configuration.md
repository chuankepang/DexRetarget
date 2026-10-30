# ROKAE 阻抗遥操作：配置与调参

跟踪恢复等待现为 **10 ms**（此前 200 ms），入口为 `config/robots/rokae.yaml` → `teleoperation.tracking_resume_delay_s: 0.01`。设为 0 表示首次新鲜数据即重定参考并恢复。此计时不阻塞接收线程；不改变手腕 150 ms/手指 200 ms 的丢失阈值、键盘暂停、实时 watchdog 或手部 0.5 秒平滑。实际恢复受 Quest 帧率和主循环调度影响，10 ms 不是整条链路延迟保证。以下旧记录中的 200 ms 为历史行为。


更新：2026-10-29。适用本项目 xMatePro3 / 控制器 2.3.2 / SDK 0.3.4。
本轮只做代码、配置、原生编译及离线检查；没有自动上电或移动真机。

## 平衡位姿与实际位姿

阻抗模式的动作是平衡位姿，不是保证到达的硬位置。VR 使用相对映射，
输出仍是机器人基座坐标系中的完整 TCP 位姿。改变控制器不改变这个坐标约定。
接触时实际 TCP 可以偏离平衡点；松手后的回弹还受重力补偿、摩擦、零空间和控制器内部阻尼影响。

局部近似：`F_spring = K * (x_equilibrium - x_measured)`。
300 N/m 刚度下，10 mm 误差对应约 3 N 弹簧力；这是理论近似，不是测得接触力。
当前参考误差限制 40 mm，所以单轴名义弹簧项约 12 N；这不包含阻尼、惯性与补偿项，不能当作硬性接触力上限。

```mermaid
flowchart LR
    Q[Quest 新鲜腕部位姿] --> G[键盘使能与相对映射]
    G --> P[平衡点与实测 TCP 的误差限制]
    P --> B[最新目标缓冲区]
    B --> R[C++ 1 kHz 参考滤波与速度加速度限制]
    R --> L[再次限制平衡点误差]
    L --> I[SDK 原生笛卡尔阻抗]
    I --> M[实测关节与 TCP]
    M --> P
    M --> L
    P --> D[数据模块记录接受的目标]
    M --> D
```

Python 限制用于接受目标和记录；实时限制用于每周期约束，二者使用同一组误差参数。
数据中的 action 是控制器输入目标，位于实时平滑之前；不是实际位置，也不等于实测力。
相机、存储、模型推理不进入 1 ms 回调。暂停冻结跟随，不自动回 home；`h` 单独触发 home。

## 参数归属与唯一修改入口

| 内容 | 修改位置 | 生效范围 |
|---|---|---|
| 连接、负载、SDK 限速/滤波 | `config/robots/rokae.yaml` → `robot` | 设备和原生控制 |
| 刚度、期望力、力控坐标系 | 同文件 → `control.impedance` | 原生阻抗；独立 configured 测试与遥操作共享 |
| 平衡点速度、加速度 | 同文件 → `control.impedance.reference_motion` | 仅阻抗参考发生器 |
| 平衡点相对实测 TCP 的误差 | 同文件 → `control.impedance.reference_limit` | Python 与 C++ 共同使用 |
| 通用笛卡尔参考限制 | 同文件 → `safety` | 位置路线默认；阻抗未覆盖的字段继承 |
| VR 缩放、轴映射 | 同文件 → `teleoperation` | 全部遥操作路线 |
| Cup home、任务说明、采集配置引用 | `config/tasks/cup.yaml` | 该任务 |
| 关节速度/加速度/jerk、IK、零空间参考 | `config/models/xmate_er_pro_7.yaml` | 仅 Pinocchio 位置路线 |
| 记录频率、同步、存储 | `config/collection/collection.json` | 数据采集 |

优先级：CLI > 阻抗 reference_motion 覆盖值 > safety 通用值。
任务 home 覆盖通用启动姿态；模型 neutral_position 是 IK 零空间参考，不是另一份任务 home。
各路线限制不同职责，禁止将 Pinocchio 关节流速度误认为原生笛卡尔阻抗的参数。
代码中的缺省值服务旧调用兼容；日常调参只改上述 YAML，不同时修改 Python/C++ 默认值。
机器人、模型、任务及独立测试入口已使用拒绝重复键的 YAML 读取器，避免静默覆盖。

## 当前基线

| 参数 | 当前值 | 本轮变化 |
|---|---|---|
| 平移/旋转刚度 | [300,300,300] N/m；[15,15,15] Nm/rad | 保留最近独立测试基线 |
| 期望力/力矩 | 全零 | 不用附加期望力抵消漂移 |
| 力控坐标系 | tool | 正式路线由写死 world 改为读取配置，与 configured 测试一致 |
| 平衡点平移/转动速度 | 0.20 m/s；0.60 rad/s | 原通用 0.15；0.50 |
| 平衡点平移/转动加速度 | 0.60 m/s²；2.0 rad/s² | 原通用 0.40；1.50 |
| 平衡点误差 | 0.04 m；0.30 rad | 不变 |
| 参考低通 / SDK 滤波 | 5 Hz / 30 Hz | 不变；滤波不是阻尼比 |
| 负载 | 0.96 kg；[-0.05143,0.00823,0.00840] m | 不变；质量来自舍入截图 |

速度/加速度限制作用于平衡点参考，外力推动后的实测速度并不受该数值硬性约束。
独立测试 `joint_excursion`、`joint_speed`、`tcp_displacement` 是主机试验护栏，
与生产遥操作参考限幅、控制器内部力矩保护是三层不同机制。
本轮不扩大 SDK/控制器力矩、力矩变化率、动量等保护，不关闭 SDK rate limiter。
旧 SDK 没有单独的笛卡尔阻尼或实时零空间刚度设置接口；
`mock_damping`/`mock_mass` 只在模拟模型生效，IK damping 是数值求解阻尼。
误填 `damping`、`damping_ratio`、`nullspace_stiffness` 会明确报错，不静默忽略。

## 操作与调参顺序

先检查最终配置，不连接硬件：

```bash
conda activate dexteleop
python -m dexretarget --task-config config/tasks/cup.yaml --arm-control impedance --print-arm-config
```

独立测试先只打印计划，再由操作者执行：

```bash
python rokae/impedance_test/run.py --stage hold --duration 5
python rokae/impedance_test/run.py --stage hold --duration 5 --execute
python rokae/impedance_test/run.py --stage hold --duration 30 --interaction light-push --execute
```

前 2 秒不接触。先以 5–10 mm 偏移轻推松手，记录回弹、峰值速度、最终偏差及关节构型。
不通过静态保持时先核对标定、负载和 TCP，不用提高刚度或放宽保护掩盖漂移。
保持通过但手感太软/硬时，每次只改一组刚度约 10–20%，重复相同姿态和推移量；
这是现场试验步长建议，不是厂家验证配方。各轴可以不同，方向以 force_frame 为准。
当前实时接口不提供独立阻尼调节，不能将 Franka 的力矩算法直接叠加在原生阻抗上。

独立保持/回弹验证后，再使用明确的阻抗遥操作命令：

```bash
python -m dexretarget --task-config config/tasks/cup.yaml --enable-hand --enable-arm --arm-control impedance --no-capture
```

`e` 使能/暂停，`p` 暂停，`c` 重定参考，`h` 回 home。启动后默认暂停跟随。
正式采集去掉 `--no-capture`，`r` 开始；`s/f` 保存成功/失败段。采集与策略部署应使用同一套阻抗参数。
默认 position 路线和旧命令兼容性保留，没有静默改为阻抗。

## 官方依据与版本边界

- [官方实时阻抗示例](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/example/rt/cartesian_impedance_control.cpp)：设置力控坐标、负载、刚度及期望力，然后以笛卡尔位姿作为回调输出。
- [官方力矩控制说明](https://docs.rokae.com/docs/SDK/cpp/实时模式常见问题/cpp_rt_torque_control/)：力矩与其变化率都受限制，状态字段也有模式/版本条件。
- 本机匹配头文件 `rokae/sdk/include/rokae/motion_control_rt.h` 是当前能力边界：原生笛卡尔刚度最大 [1500,1500,1500,100,100,100]，本项目未提高；新网页/RL 接口不等于旧 RT 接口具备同样能力。

尚未实测本轮提高的参考速度/加速度、正式路线 force_frame 配置以及完整接触遥操作。
离线测试验证参数传递、优先级、无硬件预览与异常拒绝，不代表实际刚度/阻尼性能通过。

## 本轮 Review 与验证结果

- 原生桥接在 dexteleop 环境重新编译，仍链接本机 SDK 0.3.4。
- 300 项 pytest、9 项子测试通过；独立测试 CTest 5/5 通过。
- 检查 193 个 Python 文件、42 个规范模块、199 个本地 Markdown 引用，无错误。
- 新测试覆盖 tool/world 到 native 配置传递、无硬件 CLI 预览、CLI 优先级、位置路线行为保留、YAML 重复键拒绝、不支持参数拒绝。
- 独立保持测试以前固定使用零期望力而未核对 YAML；现显式拒绝非零期望力，避免误做带主动施力的 baseline。
- 采集元数据继续写入有效 realtime 参数与 controller spec，包括新增 force_frame；数据 schema 与 action 含义不变。
- C++ 1 ms 控制循环未增加磁盘写入、配置解析或 Python 回调；初始化阶段增加坐标系选择。
- 未执行硬件试验，未验证 force_frame API 的实际物理效果、新参考上限下的接触性能或真实阻尼。
