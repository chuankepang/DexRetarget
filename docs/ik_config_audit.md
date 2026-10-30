# 运动学、逆解与配置审查

更新：2026-10-30。跨项目范式、论文与公司报告见[relatedworks.md](../relatedworks.md)。本轮实现模型一致性检查、有效限位统一和冗余策略改进；保留现有手腕映射、默认 Pinocchio 路线、原生 SDK 0.3.4、负载与阻抗参数。

后续同日的链路优化、IK 等价计算复用、`response_hz` 参数和最新 **416 项测试 / 9 项 subtest** 结果见[跟随与延迟审查](teleop_latency_review.md)。下文较早的测试数字是当时审查记录；腕部映射未改变。

## 实际数据流

```text
Quest UDP / USB TCP → 新鲜度与臂手使能门 → RelativePoseMapper
  → EECommand（base_T_TCP 位姿目标）
  → PositionController → DifferentialIK（每个新目标一步）
  → set_target_joints → 原生 JointRealtimeCore（1 kHz）
  → SDK jointPosition → 机器人
```

`tool_relative`、`wrist_to_tool_rotation`、位置轴变换、平移倍率与旋转倍率保持原值。IK 不读欧拉角，也不改手腕到工具的轴线关系。`cartesian` 和 `impedance` 后端分别由控制器跟踪位姿/平衡位姿，不经过本项目主机 IK。手部重定向、采集结构和时间戳接口未修改。

## 三项实现

### 1. 多姿态模型一致性检查

启动原生 Pinocchio 关节控制前：读取并合并软限位 → 检查当前关节有效 → 查询官方 FK → 比较当前实测 TCP → 确认查询期间关节位置没有明显变化 → 才调用原生启动。

采样包含当前关节、历史 neutral（裁到有效限位内）、各轴 ±0.15 rad，共 16 个虚拟关节配置。只计算这些点的 FK，**不会运动到采样点**。非有限/非刚体变换、模型失配、软限位关闭或查询期间运动均阻止启动。

SDK `calcFk` 返回 ref_T_end；新增 `calculate_fk_in_base` 显式计算：

```text
base_T_end = inverse(world_T_base) @ world_T_ref @ ref_T_end
```

旧 `calculate_fk` 的外部参考系语义保留。工具 TCP 必须和 RT `tcpPose_m` 一致；当前实测对照用于检查这一点。没有拟合一个额外刚体变换去掩盖错误的 TCP / base / 安装旋转。

配置只在 `config/models/xmate_er_pro_7.yaml: validation`。位置 20 mm、姿态 0.10 rad 保留原容差，只扩大检查覆盖；它们是启动一致性门槛，**不能当作亚毫米标定精度证明**。四段几何长度仍是既有拟合值，没有在控制柜关机时猜测厂家参数。

### 2. IK 与伺服共用有效边界

`effective_min=max(model_min,controller_min)`，`effective_max=min(model_max,controller_max)`。
同一结果更新模型位置界、Pinocchio 模型界、`StreamLimits`，再构造原生 `JointRealtimeConfig`。不放宽任何控制器边界；无交集、失序/非有限边界和界外初始状态报错。运行期间不热修改边界，控制器配置改变后应重新启动。

速度/加速度/jerk 的既有设置不变。IK 在每步位置/速度盒内先整体缩放主任务，再按剩余可用范围整体缩放副任务；避免逐轴截断改变任务方向。它是保守的可行步生成，不等于完整约束 QP 的全局最优解，边界处仍可能需要离合或换姿态。

### 3. 明确零空间参考与任务优先级

默认 `ik.posture_reference: initial`，每次使能、c、跟踪恢复从当前原生命令关节重置参考；`model` 可使用保留的历史 neutral。Cup home 不再被误认为隐式零空间参考，也不在 task 中重复保存同一套 IK 参数。

```text
primary = Jᵀ (J Jᵀ + λ²I)⁻¹ error_step
B = 6×7 雅可比 SVD 的结构冗余核基（7-6 个方向）
secondary_step = B Bᵀ secondary_velocity * dt
```

主任务仍用自适应 DLS；副任务改用严格几何核投影，避免旧 `I-J#J` 在 λ>0 时的主任务泄漏。接近秩退化时平滑衰减副任务，不突然扩展零空间维度。关节限位排斥保留；J2/J4/J6 弯曲启发式仅在最小奇异值下降时平滑启用，不在正常静止姿态持续推肘。

旧 neutral 和所有用户历史 home 注释保留。新的 `posture_reference` 只属于模型 IK 配置；默认缺少该字段的外部旧配置仍按 `model` 参考解析，但投影与边界算法使用当前实现。

严格几何核是局部一阶概念，不保证有限步末端完全不变；奇异阈值仍来自平移/旋转混合雅可比，未做单位归一化。启发式不等于完整奇异规避或碰撞检测。当前没有全局最优保证。

## Pinocchio 与官方能力取舍

| 维度 | 本项目 Pinocchio + DifferentialIK | ROKAE 官方 |
| --- | --- | --- |
| 模型 | 四段拟合几何，新增多姿态官方 FK 一致性门；精度受模型/TCP 限制 | xMateModel 创建时读取控制器模型参数；是否包含全部个体标定需实测确认 |
| 冗余 | 使能姿态参考、精确几何核、限位/近奇异副任务；可修改策略 | 主机 getJointPos 接受 elbow + seed，说明选邻近 seed 的解；本项目 Cartesian 位置路线固定启动 elbow |
| 最优性 | 局部 DLS + 可行步生成；不是全工作区/全轨迹最优 | 邻近 seed 不等于最佳限位、可操作度或最小能耗；固定臂角可能排除其他可行解 |
| 可用性 | 默认在用；不改现有映射 | 当前 nomodel 库有 calcFk/calcIk，但 Python 桥只新增了基座系 FK，没有接入官方 IK；完整 xMateModel 未启用 |
| 时延 | 当前离线诊断单步约 0.1–0.25 ms（P95，随负载变化）；不含链路延迟 | 未完成同目标官方 IK 实测，不能宣称更快、更准 |

接口依据：[本地匹配的官方 0.3.4 model.h](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/77d2fd7546e657d4e66a82b29c2a5f21d01db3e3/include/rokae/model.h)、[官方模型示例](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/example/xmatemodel_er3_er7p.cpp)。elbow 是臂角，不是第七轴角度。SDK calcIk 模型查询与 cartesianPosition 执行路线必须分开计时。

[Pinocchio 官方 IK 示例](https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/inverse-kinematics.py)包含 Jlog6 误差雅可比与收敛判据；本项目仍采用限速小步 CLIK，没有把该示例直接当成在线控制器。[官方零空间练习](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_d_practical_exercises_3_invkine.html)说明了任务优先级和数值秩选择。

官方 [CHANGELOG](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/CHANGELOG.md) 的 0.7.0 引入 calcAllIkSolutions / DH 查询并要求 xCore ≥3.2.0；现场 2.3.2 不直接替换新版 SDK 或头文件。后续如评估官方 IK，使用同一批映射后的 TCP 目标，比成功率、模型残差、关节路径、限位裕量、奇异指标和耗时，而不是凭库名切换。

## 离线证据与验证范围

固定目标 50 Hz、10 秒，同一拟合模型：

| 场景 | 修改前最大单轴偏移 | 修改后最大单轴偏移 | 含义 |
| --- | --- | --- | --- |
| Cup home 保持 | 5.742731° | <0.000001° | 正常静止目标不再吸向旧 neutral |
| 偏离 neutral 的保持姿态 | 3.855822° | <0.000001° | initial 参考按当前姿态建立 |
| 近伸直姿态平移 [10,0,-5] mm | 26.488759° | 以当前诊断输出为准 | 奇异启发式已改变，不能只用偏移量评估全部性能 |

工具 X/Z ±90° 轨迹（4 秒渐变 + 8 秒保持）均能在拟合模型中收敛。新增两条测试从非零腕参考、实际映射矩阵开始，经 IK 和 C++ 1 kHz JointRealtimeCore，再用 FK 检查工具 Z ±90°：位置误差 <0.5 mm、角度误差 <0.1°，全程满足模拟的控制器有效关节边界。此阈值是模拟回归标准，不是真机误差。

已完成 C++ 桥构建，仍链接原 0.3.4 nomodel SDK。最终 `pytest -q`：**398 passed，9 subtests passed，26.98 s**，没有跳过测试。覆盖虚拟 FK 错误只在非当前点出现、非法变换、有效限位传到原生配置、失败/运动时禁止启动、边界向内恢复、50/90/100 Hz 零空间保持、完整工具 Z ±90° 映射链及原有测试。

合成相机 + 伪 Quest UDP + Inspire PTY + mock arm 的完整入口检查：

| 后端 | 成功 episode：总帧/有效帧 | 失败 episode：总帧/有效帧 | 缺失流 |
| --- | --- | --- | --- |
| cartesian | 141 / 121 | 24 / 24 | 无 |
| pinocchio | 140 / 119 | 24 / 24 | 无 |

覆盖 r 录制、s/f 保存、主动断流 HOLD、恢复重定参考、手动暂停、home 与退出；成功段有三段训练片段，无效帧来自主动断流/暂停，不跨间断拼接。原始 smoke episode 已按本次输出精确清理，保留此汇总，不删除既有采集数据。

Review 修复了：近奇异启发式在阈值处突变（改为平滑启用）；原生启动前后 IK 种子可能不一致（从原生命令重置）；无效增益/变换可能混入（增加校验）；模型参考越界（裁到同一有效界）。现有映射文件、机器人映射 YAML、任务 home/历史注释、SDK 库和阻抗参数均未修改。

控制柜未开机，SDK 只读连接返回网络错误；用户确认暂不便连接，因此**未验证实际多姿态官方 FK、真实运动、接触阻抗或物理轴向**，没有把未执行项视为通过。模型一致性检查代码已完成，实际几何参数是否需要进一步标定，仍须开机后执行下方只读命令确认。

已有两段 .96 kg、法兰中心 TCP 的历史 trace 中抽取 63 对 q/TCP 记录，拟合模型与记录 TCP 的最大差约 0.788 mm / 0.00975°；这只是已保存控制器状态的局部对照，不是本轮官方 FK 查询或物理标定验收。

```bash
conda activate dexteleop
# 新增原生接口需重建；SDK 版本不变。
bash rokae/build.sh
# 离线，无硬件连接
python tools/diagnostics/audit_rokae_ik.py
python tools/diagnostics/check_rokae_orientation.py --include-ik
# 控制柜开机、机器人静止时：只读 FK 对照，不上电、不发运动指令
python rokae/control/examples/check_model.py --execute
```

## 配置归属

- `config/robots/rokae.yaml`：设备、映射、默认控制、独立工具启动预设；本轮不改该文件的参数。
- `config/tasks/cup.yaml`：任务指令、采集引用、home；本轮不改原 home 与历史注释。
- `config/models/xmate_er_pro_7.yaml`：主机几何、IK / 零空间、模型验证和关节流参数，统一维护。
- `manipulation_init` 是旧独立演示的兼容预设，home 是任务回位目标，neutral 是可选副任务参考；不能混用。

下列为现有关键配置清单。参数名相同不意味着跨后端同时生效。

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
| `teleop.quest_config` | Quest 通信默认配置引用；`--config` 可覆盖，协议/监听地址/端口仅在 config/quest 中维护。 |
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
| `lower_deg` | 度；主机模型下界。真机启动前与控制器软限位取交集，同步到 IK / Pinocchio / 原生关节流。 |
| `upper_deg` | 度；主机模型上界；当前 ±170/120/360 与现场软限位不同，不得视为控制器设置。 |
| `max_velocity` | rad/s，各关节上限；IK 每步和 1 kHz 关节流都使用。 |
| `max_acceleration` | rad/s²，1 kHz 关节流使用；IK 自身不是加速度约束优化器。 |
| `max_jerk` | rad/s³，1 kHz 关节流使用。 |
| `neutral_position` | rad，`posture_reference=model` 时的默认参考；`initial` 模式取每次重定参考时的关节姿态。 |
| `stream.reference_cutoff_hz` | Hz，关节目标低通截止频率。 |
| `stream.deadband` | rad，关节目标死区。 |
| `stream.max_target_delta` | rad，允许的关节目标跳变量。 |
| `stream.hold_timeout` | 秒，原生关节流无新目标时 hold；不是 Python 的腕部 freshness 阈值。 |
| `stream.stop_timeout` | 秒，原生关节流无新目标时 safe_stop；显式 hold 与断流 watchdog 是不同状态。 |
| `stream.max_source_age` | 秒，关节流拒绝过期来源时间戳。 |
| `stream.future_tolerance` | 秒，关节流来源时间戳的未来偏差容忍。 |
| `validation.translation_tolerance_m` | 多姿态官方 FK 一致性位置容差，米；不是真机精度指标。 |
| `validation.rotation_tolerance_rad` | 多姿态官方 FK 一致性角度容差，rad。 |
| `validation.joint_offset_rad` | 当前关节两侧的虚拟 FK 采样偏移，rad；不执行运动。 |
| `ik.posture_reference` | initial：使能/c/恢复时关节姿态；model：历史 neutral。仅决定副任务，不改变 VR 映射。 |
| `ik.damping` | 基础阻尼 λ；矩阵中加 λ²I，越大通常越稳但跟随误差也可能增大。 |
| `ik.damping_max` | 接近奇异时增大的阻尼上界。 |
| `ik.singular_sigma` | 最小奇异值低于此阈值时增加阻尼；平移/旋转混合雅可比未做尺度归一化。 |
| `ik.posture_gain` | 副任务趋向所选 posture_reference 的增益；初始参考/历史 neutral 二选一。 |
| `ik.limit_gain` | 副任务远离模型关节限位的增益；不是硬约束。 |
| `ik.singularity_gain` | 仅接近雅可比奇异时平滑启用的弯曲关节启发式增益；不保证全局无奇异。 |
| `ik.null_max_speed` | rad/s，副任务速度向量的整体范数上限；投影前限幅。 |
| `ik.limit_margin` | rad，接近模型限位时开始排斥的距离。 |
| `ik.max_translation_speed` | m/s，将末端误差步长限制为速度×dt；与笛卡尔路线同名设置相互独立。 |
| `ik.max_angular_speed` | rad/s，将旋转误差步长限制为角速度×dt。 |
