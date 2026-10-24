# VR 逆运动学与配置逐项审查

## 当前执行的逆解

默认 `config/robots/rokae.yaml: control.mode=pinocchio`，Cup 未覆盖 `teleop.arm_control`。
实际调用链：Quest 相对目标 → PositionController.set_target → 本项目 DifferentialIK.step → set_target_joints → 原生 JointStream → ROKAE jointPosition。
Pinocchio 提供正解、雅可比和 SE(3) 运算；逆解控制策略是本项目实现，**不是 ROKAE 官方 IK，也不是直接调用一个 Pinocchio 完整逆解器**。
每个新腕部样本推进一次微分 IK；1 kHz 是后面的参考平滑/硬件回调频率。

另一条 `--arm-control cartesian` 路线把末端目标发到 xCore cartesianPosition，由控制器处理笛卡尔运动。本项目七轴位置路线启动时读取 `elbow_m`，之后显式发送固定 `command.elbow` / `hasElbow`。不是每次按 e 都重新读取臂角。
阻抗路线不固定 hasElbow；不能将阻抗自由度策略误认为当前位置路线的行为。

ROKAE SDK 0.3.4 本地头文件区分两类接口：

- `Model_T::calcIk(CartesianPosition, ec)`：控制器模型接口，输入末端相对外部参考系的位姿。本项目 Python/C++ 桥目前没有暴露逆解，只暴露了 `calculate_fk`。
- `xMateModel::getJointPos(cartPos, elbow, jntInit, jntPos)`：条件编译的主机模型接口，输入**法兰**位姿、臂角和初始关节角，选择邻近解，返回不可达、跳变量/速度、限位或奇异等错误码。
- 当前 x86 构建链接官方 `nomodel` 库，没有启用 `XMATEMODEL_LIB_SUPPORTED`；不能因头文件存在就宣称官方主机 IK 已启用，也不能将控制器请求未经耗时测量放进 1 ms 回调。

[ROKAE 官方概念说明](https://docs.rokae.com/docs/)定义 elbow 为臂平面相对参考平面的夹角，**不是第七轴角度**。
在线[SDK 方法文档](https://docs.rokae.com/docs/SDK/cpp/cpp_method/)为 0.7.x；接口可用性以本仓库 0.3.4 头文件及实际链接选项为准，未进行官方 IK 实机性能比较。

## 冗余解如何指定

一般非奇异位形中，7 轴跟踪 6 维末端任务，剩一个冗余自由度。当前不是显式给定 elbow，而是副任务投影：

```text
J# = Jᵀ (J Jᵀ + λ² I)⁻¹
N  = I - J# J
dq = J# error_step + N secondary_velocity * dt
secondary_velocity = posture + joint_limit_repulsion + bend_joint_repulsion
```

- 姿态参考：`config/models/xmate_er_pro_7.yaml: neutral_position`；`posture_gain=1.2`。
- 关节限位排斥：`limit_gain=4.0`，`limit_margin=0.17 rad`，依据主机模型上下界。
- J2/J4/J6 远离 0 的启发式：`singularity_gain=0.8`；不能保证所有几何奇异都被规避。
- 副任务总速度投影前限制为 `null_max_speed=0.6 rad/s`；之后还有逐轴限速与关节位置截断。
- `e/c` 或跟踪恢复重置积分状态到当前命令关节角，但**不会把 neutral_position 改成当前关节角**。Cup home 也不会改变 neutral。

λ>0 时 N 不是严格零空间投影，J N 通常非零；副任务可能引入末端漂移，之后逐轴截断也会破坏理想任务优先级。
手不动而肘缓慢调整可能来自副任务；不应把它直接判定成 VR 抖动。

## 离线量化与质量判断

运行 `python tools/diagnostics/audit_rokae_ik.py --output docs/validation/ik_audit_20261022.json`。
以下为 50 Hz、500 步、固定目标、同一个拟合模型计算目标与误差；不包含真实机器人、1 kHz 平滑、通信或标定误差。

| 场景 | 10 秒后位置残差 | 姿态残差 | 关节总位移范数 |
| --- | --- | --- | --- |
| 当前 neutral 保持 | 0.030 mm | 0.00043° | 0.00010 rad |
| 偏离 neutral 的姿态保持 | 0.084 mm | 0.00202° | 0.09955 rad |
| neutral 附近平移 [20,-10,15] mm | 0.039 mm | 0.00110° | 0.11711 rad |
| 近伸直姿态平移 [10,0,-5] mm | 1.249 mm | 0.00797° | 0.66873 rad |

此机器本次单步 P95 约 181–198 μs；这是离线耗时，不是实时最坏耗时保证。
偏离 neutral 的保持场景说明末端基本不动时，关节仍会因副任务变化。
近奇异案例起始最小奇异值约 0.00310，阻尼上升到约 0.1777，收敛质量下降；不能据四个案例声称全工作区都好/坏。
起始 `||J N||` 在上述案例约 0.020–0.081；同点 SVD 严格核投影约 1e-16，仅为代数对照，没有替换在线算法。

原始结果见 [JSON](validation/run_history.md)。当前证据不支持“算得慢是主要原因”；**模型准确度、滤波链、奇异区和参考策略更值得优先测量**。

## 仍需解决的质量问题与优化顺序

1. **模型/TCP 校验优先**：当前四个连杆长度来自拟合，不是厂家标定。启动只在一个姿态检查 FK，容差 20 mm / 0.10 rad（约 5.73°）；一个点通过不能证明整片空间准确。应在多个已知关节姿态用官方 FK 做只读比较，统一 base、法兰和工具 TCP，再决定是否替换模型。当前不能宣称机器人末端达到上述亚毫米精度。
2. **IK 与伺服限位一致**：真实关节流与控制器软限位取交集，但 IK 模型仍使用 ±170/120/360° 的本地边界。现场曾读到更窄的 ±165/115/355°。靠近限位时，IK 可能继续向伺服拒绝/截断的方向积分。后续应把已验证的有效边界同步给 IK，不应放宽控制器限制掩盖问题。
3. **明确零空间意图**：可选择保持使能时姿态、任务指定参考姿态，或显式臂角连续调度。当前仅支持修改模型的固定 neutral，不存在任务 home 自动变零空间参考的功能。任务参考应属于 task；模型中保留默认值。
4. **改善任务优先级**：在仿真比较严格核投影/分层 QP、关节限位/速度约束与方向统一缩放。阻尼、自适应秩、米与弧度的权重必须一起评估；不能只把阻尼调小。当前 SE(3) 小步近似没有完整 Jlog 修正，较大姿态误差也应专项测试。[Pinocchio 官方 IK 示例](https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/inverse-kinematics.py)展示了误差雅可比修正及收敛判据，不能直接把该示例当成已验证的机器人控制器。
5. **可观察性与 A/B**：记录 σ_min、阻尼、IK 残差、目标关节/伺服命令/实测差和拒绝次数；用相同 VR 回放比较 pinocchio 与 cartesian。官方路线也会受固定臂角可达性/奇异影响，不保证切换后一定更好。
6. **路线参数要分清**：当前 Pinocchio 路线不执行 safety 中的笛卡尔工作空间盒/平移滤波；它执行模型 IK 步长和关节流限制。模型 15 Hz 关节参考滤波与 SDK 30 Hz 设置可能共同影响延迟。此轮明确标注，不以参数名字相同暗示等价；碰撞避障并不存在。

本轮没有切换后端、替换主机模型、放宽限位或上线新的零空间算法。修复一个确定的恢复问题：`reset_tracking()` 现在同时清除上一帧时间戳，避免长暂停后的首个 IK 步长由暂停时长放大到 50 ms；恢复首步使用既有 20 ms 初始步长，并补充回归测试。

## 配置归属与清理

- `robots/rokae.yaml`：设备/默认控制/无任务启动，以及已明确标注的历史独立工具预设。
- `tasks/cup.yaml`：Cup 指令、设备选择、采集引用、任务 home。相同的角度数值不意味着同一语义；home 不等于 IK neutral。
- `models/xmate_er_pro_7.yaml`：主机模型、默认零空间参考、IK 和关节流参数。
- `robot.manipulation_init` 原本是“工具朝下”的旧演示姿态名字，仅供 05 `--pose manipulation`、cup_flip、box_vertices 等使用；正式 Cup 采集从不读它。当前保存值已被改成腕部向前，原注释错误，已修正并保留原角度注释。未擅自恢复旧角度；要求工具朝下的演示仍可能拒绝它。保留该兼容字段是因为有实际调用者，不另设一个隐式 Cup home。
- 任务 home 覆盖通用 init 的 enabled、joint_position、joint_speed、timeout、max_segment_delta；在线倍率仍继承 init。home.joint_tolerance_rad 是任务完成反馈检查，joint_waypoints 是任务回位路径。
- 删除从未被 Python/C++ 读取的 `robot.sdk_callback_hz` 和 init 的三个 `min_*_bend` 字段（4 项）；它们并没有提供运行时保护。
- 删除与全局 position 完全重复的 `tasks.cup.controller` / `tasks.box.controller`（2 项）。主遥操作解析器本来不传这些 demo task 名；任务覆写应使用 `teleop.arm_control`。旧外部配置的解析兼容代码仍保留。
- 所有其他配置数值、Cup 新角度及历史角度注释保留；三份 YAML 的所有当前条目均加中文解释。下面逐项列出实际归属与生效条件。

### config/robots/rokae.yaml

| 条目 | 归属/生效条件 |
| --- | --- |
| `robot.type` | 设备型号；当前主机 Pinocchio 路线只支持此七轴型号。 |
| `robot.ip` | 机器人控制器 IP；CLI 可覆盖。 |
| `robot.local_ip` | 本机机器人网口 IP；CLI 可覆盖。 |
| `robot.control_hz` | Hz；Python mock 调度/实时配置使用；原生真实回调固定 1 ms，改此值不能改变硬件周期。 |
| `robot.rt_network_tolerance` | SDK 实时丢包容忍设置，不是 Quest 跟踪超时。 |
| `robot.controller_rate_limit` | 全部真实实时路线都传给 SDK；关节流另有 model.stream 限制。 |
| `robot.controller_filter_cutoff_hz` | 全部真实实时路线都传给 SDK；关节流另有 model.stream 限制。 |
| `robot.initial_joint_move.enabled` | 通用启动回位开关；任务 home.on_start 覆盖，--skip-init 可跳过。 |
| `robot.initial_joint_move.joint_position` | rad，J1..J7；无任务时使用；有任务时由 home.arm_joints 覆盖。 |
| `robot.initial_joint_move.joint_speed` | 归一化 SDK 设置：乘 1000 得 250，不是 rad/s；任务 home.joint_speed 覆盖。 |
| `robot.initial_joint_move.online_speed_scale` | NRT 在线速度倍率；任务启动和 h 也继承此值，非遥操作速度。 |
| `robot.initial_joint_move.max_segment_delta` | rad/段；启动分段规划上限；任务 home.max_segment_delta 覆盖。 |
| `robot.initial_joint_move.timeout` | 秒；每次初始化目标的超时；任务 home.timeout_s 覆盖。 |
| `robot.initial_joint_move.recovery_trigger_margin` | rad；05 维护工具的近软限位判定，不是 IK 零空间参数。 |
| `robot.initial_joint_move.recovery_clearance` | rad；05 旧 NRT 回退目标余量。软限位已关闭时应使用 06 手动 Jog。 |
| `robot.initial_joint_move.recovery_speed` | SDK 整数设置；仅 05 --recover-joint-limits 旧 NRT 路线。 |
| `robot.initial_joint_move.recovery_jog_rate` | 0.01..0.10；06 手动 Jog 速度比例，--jog-rate 可覆盖。 |
| `robot.initial_joint_move.recovery_timeout` | 秒；05 旧 NRT 回退超时，不控制 06 的等待使能超时。 |
| `robot.initial_joint_move.min_joint_limit_margin` | rad；05 对目标姿态做的手册限位余量检查；不是运行时 IK 边界。 |
| `robot.manipulation_init.joint_position` | rad；当前保留现场改值（与腕部向前姿态相同），不再声称工具朝下。演示有朝下检查，可能拒绝此姿态。 |
| `robot.base_transform.pose_xyzw` | [x,y,z,qx,qy,qz,qw]，米/无量纲；当前假设基座与世界重合。 |
| `robot.require_controller_base_rotation_identity` | 检查控制器基座安装旋转是否为单位阵，适用于当前正装场景。 |
| `robot.controller_base_rotation_tolerance` | rad；上述安装旋转检查阈值。 |
| `non_realtime.online_speed_scale` | 独立 NRT 工具速度倍率；遥操作启动优先使用 initial_joint_move 的倍率。 |
| `non_realtime.joint_speed` | SDK 整数速度设置；joint_move 等工具使用。 |
| `non_realtime.cartesian_speed` | SDK 笛卡尔速度设置（mm/s）；cartesian_move 等工具使用。 |
| `non_realtime.timeout` | 秒；独立 NRT 路径超时。 |
| `non_realtime.max_joint_delta` | rad；独立关节小步工具允许的单次目标变化。 |
| `non_realtime.max_translation_delta` | 米；独立 NRT 笛卡尔目标跳变限制。 |
| `non_realtime.max_rotation_delta` | rad；独立 NRT 笛卡尔姿态跳变限制。 |
| `control.mode` | pinocchio=主机微分 IK+关节流；cartesian=控制器笛卡尔逆解。当前不是官方主机 IK。 |
| `control.controller` | position=位置控制；impedance=笛卡尔阻抗。只选控制类型，不指定任务 home。 |
| `control.impedance.backend` | 原生 SDK 笛卡尔阻抗后端。 |
| `control.impedance.stiffness` | 平移 N/m、旋转 Nm/rad；占位试验值，不是厂家推荐配方。 |
| `control.impedance.desired_wrench` | 前三项 N、后三项 Nm；期望外力/力矩。 |
| `control.impedance.reference_limit.translation` | 米；平移平衡点距离上限。 |
| `control.impedance.reference_limit.rotation` | rad；姿态平衡点角距离上限。 |
| `control.impedance.mock_mass` | 仅离线 mock 弹簧模型质量，不发给控制器。 |
| `control.impedance.mock_damping` | 仅离线 mock 阻尼，不发给控制器。 |
| `teleoperation.translation_scale` | 相对位移缩放，0.8 倍；不是速度设置。 |
| `teleoperation.rotation_scale` | 相对旋转向量缩放，1.0 倍；不代表绝对姿态映射。 |
| `teleoperation.quest_to_robot_rotation` | 3x3 按行展开；Quest 相对运动轴到机器人/世界轴的正交旋转。 |
| `safety.translation_cutoff_hz` | Hz；笛卡尔路线平移低通。 |
| `safety.rotation_cutoff_hz` | Hz；笛卡尔路线姿态低通。 |
| `safety.translation_deadband` | 米；笛卡尔路线平移死区。 |
| `safety.rotation_deadband` | rad；笛卡尔路线姿态死区。 |
| `safety.max_translation_speed` | m/s；笛卡尔路线速度限制。Pinocchio IK 同名限制在 config/models 中。 |
| `safety.max_angular_speed` | rad/s；笛卡尔路线角速度限制。Pinocchio IK 同名限制在 config/models 中。 |
| `safety.max_translation_acceleration` | m/s²；笛卡尔路线平移加速度限制。 |
| `safety.max_angular_acceleration` | rad/s²；笛卡尔路线角加速度限制。 |
| `safety.max_translation_jerk` | m/s³；笛卡尔路线平移 jerk 限制。 |
| `safety.max_angular_jerk` | rad/s³；笛卡尔路线角 jerk 限制。 |
| `safety.max_target_translation_delta` | 米；笛卡尔路线拒绝过大目标跳变。 |
| `safety.max_target_rotation_delta` | rad；笛卡尔路线拒绝过大姿态跳变。 |
| `safety.workspace_min` | 米；笛卡尔路线基座坐标 TCP 盒下界，当前仅为宽泛占位范围。 |
| `safety.workspace_max` | 米；笛卡尔路线基座坐标 TCP 盒上界，不是碰撞避障。 |
| `safety.hold_timeout` | 秒；全部路线的 Python 腕部过期门控；笛卡尔实时 hold；关节实时 hold 另在 model.stream。 |
| `safety.stop_timeout` | 秒；笛卡尔实时断流停止；Pinocchio 的对应参数在 model.stream。 |
| `safety.max_source_age` | 秒；启动等待新鲜腕部及笛卡尔源时间检查；关节流另用 model.stream。 |
| `safety.future_tolerance` | 秒；笛卡尔源时间允许的未来偏差；关节流另用 model.stream。 |
| `realtime_test.amplitude` | 米；试验正弦位移幅值。 |
| `realtime_test.frequency` | Hz；试验正弦频率。 |
| `realtime_test.duration` | 秒；试验持续时间。 |
| `realtime_test.update_hz` | Hz；Python 试验目标发布频率，不是 1 kHz 伺服周期。 |
| `realtime_test.axis` | 试验平移轴 x/y/z。 |
| `mock.initial_pose_xyzw` | Cartesian/impedance mock 起始 TCP；Pinocchio mock 从启动关节角做 FK。 |

### config/tasks/cup.yaml

| 条目 | 归属/生效条件 |
| --- | --- |
| `schema_version` | 任务配置格式版本；加载器严格校验字段。 |
| `task_id` | 任务唯一标识；写入 episode 元数据。 |
| `instruction` | 默认自然语言任务描述；CLI --task 可覆盖。 |
| `teleop.arm_config` | 机器人默认配置唯一入口；路径相对仓库根目录。 |
| `teleop.hand` | 左右手选择。 |
| `teleop.quest3_protocol` | Quest 接收协议。 |
| `teleop.quest3_port` | Quest 接收端口。 |
| `teleop.inspire_port` | 灵巧手串口设备。 |
| `teleop.inspire_baudrate` | 串口波特率。 |
| `teleop.inspire_hand_id` | 灵巧手协议地址，不是左右手标识。 |
| `teleop.arm_print_hz` | 终端周期状态打印频率，0 关闭；不改变控制/采样频率。 |
| `collection.enabled` | 默认启用采集源；r 才创建 episode。 |
| `collection.config` | 采样/同步/存储配置引用；不是把采集参数复制到任务。 |
| `home.on_start` | 是否启动回任务 home；覆盖机器人 initial_joint_move.enabled。 |
| `home.arm_joints` | rad，J1..J7；覆盖通用启动 joint_position，并作为 h 的目标。 |
| `home.hand_angles` | 六个原生 0..1000 寄存器；null 保持手目标，不自动张开。 |
| `home.joint_waypoints` | 关节路径中间点，单位 rad；空列表走分段直达，不能自动避障。 |
| `home.joint_speed` | 乘 1000 得 SDK 150；覆盖通用启动速度，在线倍率继承机器人 init。 |
| `home.timeout_s` | 秒；覆盖启动 timeout；h 路径也使用。 |
| `home.max_segment_delta` | rad/段；覆盖通用初始化分段上限，也用于 h。 |
| `home.joint_tolerance_rad` | rad；启动/home 完成时的实测关节误差阈值。 |
| `scene_setup` | 人读场景说明，仅记录，不执行动作。 |

### config/models/xmate_er_pro_7.yaml

| 条目 | 归属/生效条件 |
| --- | --- |
| `name` | 主机模型标识；不是从控制器下载的厂家标定模型。 |
| `d_bs` | 米；基座到肩部高度，拟合值，不是厂家 CAD/标定参数。 |
| `d_se` | 米；肩到肘长度，拟合值。 |
| `d_ew` | 米；肘到腕长度，拟合值。 |
| `d_wf` | 米；腕到模型 TCP 的固定轴向偏移；不能替代实际工具 TCP 标定。 |
| `singular_joints` | 从 0 开始的 J2/J4/J6 索引；副任务远离零角的启发式，不是完整奇异性判定。 |
| `lower_deg` | 度；主机模型下界。真实关节流还与控制器软限位取交集；当前 IK 模型本身未同步该交集。 |
| `upper_deg` | 度；主机模型上界；当前 ±170/120/360 与现场软限位不同，不得视为控制器设置。 |
| `max_velocity` | rad/s，各关节上限；IK 每步和 1 kHz 关节流都使用。 |
| `max_acceleration` | rad/s²，1 kHz 关节流使用；IK 自身不是加速度约束优化器。 |
| `max_jerk` | rad/s³，1 kHz 关节流使用。 |
| `neutral_position` | rad，固定零空间姿态参考；不随任务 home 或 e/c 自动改变，也不是动作命令。 |
| `stream.reference_cutoff_hz` | Hz，关节目标低通截止频率。 |
| `stream.deadband` | rad，关节目标死区。 |
| `stream.max_target_delta` | rad，允许的关节目标跳变量。 |
| `stream.hold_timeout` | 秒，原生关节流无新目标时 hold；不是 Python 的腕部 freshness 阈值。 |
| `stream.stop_timeout` | 秒，原生关节流无新目标时 safe_stop；显式 hold 与断流 watchdog 是不同状态。 |
| `stream.max_source_age` | 秒，关节流拒绝过期来源时间戳。 |
| `stream.future_tolerance` | 秒，关节流来源时间戳的未来偏差容忍。 |
| `ik.damping` | 基础阻尼 λ；矩阵中加 λ²I，越大通常越稳但跟随误差也可能增大。 |
| `ik.damping_max` | 接近奇异时增大的阻尼上界。 |
| `ik.singular_sigma` | 最小奇异值低于此阈值时增加阻尼；平移/旋转混合雅可比未做尺度归一化。 |
| `ik.posture_gain` | 副任务趋向 neutral_position 的增益。 |
| `ik.limit_gain` | 副任务远离模型关节限位的增益；不是硬约束。 |
| `ik.singularity_gain` | 副任务推动 J2/J4/J6 离开零角的增益；不是保证全局无奇异。 |
| `ik.null_max_speed` | rad/s，副任务速度向量的整体范数上限；投影前限幅。 |
| `ik.limit_margin` | rad，接近模型限位时开始排斥的距离。 |
| `ik.max_translation_speed` | m/s，将末端误差步长限制为速度×dt；与笛卡尔路线同名设置相互独立。 |
| `ik.max_angular_speed` | rad/s，将旋转误差步长限制为角速度×dt。 |

## 本轮验证与边界

- 配置前后机器比较：机器人剩余 66 项、Cup 22 项、主机模型 29 项，共 117 个叶子条目；保留条目的解析值全部不变，仅删除上述 6 个无效/重复项。用户历史角度注释保留。
- `python -m pytest -q`：214 passed，9 subtests passed，7.80 s；包括新增恢复时间基准测试。
- `python tools/check_project.py`：184 个自有 Python 文件、42 个核心模块、134 个本地文档链接，无错误。
- `tests/check_capture_entrypoint.py --synthetic-cameras --seconds 2`：完整伪 Quest UDP / Inspire PTY / mock Pinocchio / 合成相机链路通过。主动断流 HOLD、无需 e 恢复、手动暂停、home、成功/失败保存及退出均验证。第一段 145 帧中 121 帧有效并分成三个训练段，第二段 24/24 帧有效；无缺失流。无效帧来自主动断流/暂停，未伪装成有效数据。报告位于 `docs/validation/ik_review_entrypoint_20261022.json`。
- 未连接/移动真实机器人，未测官方 IK 时延或精度，未验证真实碰撞/负载/工具标定；上文剩余改进项不因离线测试通过而视为解决。
