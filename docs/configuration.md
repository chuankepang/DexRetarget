# 配置管理

配置按职责分开，统一路径解析与入口组装；不把设备、任务和存储塞进一个大文件。

| 文件 | 唯一职责 |
| --- | --- |
| [config/quest/udp.yaml](../config/quest/udp.yaml) / [usb_tcp.yaml](../config/quest/usb_tcp.yaml) | Quest 监听 IPv4/端口；UDP 直连，USB TCP 的服务/消费者共用 |
| [config/tasks/cup.yaml](../config/tasks/cup.yaml) | 任务 ID/语言、设备默认参数、collection 引用和任务 home |
| [config/robots/rokae.yaml](../config/robots/rokae.yaml) | 机器人、坐标/TCP 映射、控制器、滤波与运动限制 |
| [config/cameras/realsense.json](../config/cameras/realsense.json) | 双相机角色、序列号、分辨率、FPS、曝光与深度开关 |
| [config/collection/collection.json](../config/collection/collection.json) | 数据位置、频率、同步阈值、字段表示和存储；引用相机配置 |
| `config/retargeting/{adaptive,vector}/` | 各手型/输入源的重定向配置 |

阻抗调参入口见[阻抗遥操作配置说明](rokae_impedance_configuration.md)。`control.impedance` 唯一管理刚度、力控坐标及平衡点误差；`reference_motion` 只覆盖阻抗参考运动，位置控制继续用 `safety`。可用 `--print-arm-config` 查看 CLI 覆盖后的实际配置，不连接硬件。机器人/任务/模型 YAML 的重复键会报错。

任务配置：

```yaml
schema_version: dexretarget.task.v1
task_id: cup_pick_place
instruction: "pick up the cup and place it in the tray"
teleop:
  arm_config: config/robots/rokae.yaml
  quest_config: config/quest/usb_tcp.yaml
collection:
  enabled: true
  config: config/collection/collection.json
# home 的实际关节/手角度与路点应由现场示教确认；参考现有任务文件。
```

显式 CLI > 任务配置 > 程序默认值。正式入口的 `--arm-config`、`--hand-config` 和任务内引用统一相对仓库根目录；绝对路径可直接使用。
Quest 通信使用 `--config config/quest/udp.yaml` 或 `--config config/quest/usb_tcp.yaml`。UDP 由遥操作程序直接接收，一个终端即可；USB TCP 使用独立通信服务，服务与遥操作加载同一份配置。文件仅定义 `schema_version`、`mode`、`bind_host`、`port`，严格拒绝未知字段、重复键和无效值。UDP 不调用 ADB；USB TCP 自动建立 ADB reverse，限定监听 127.0.0.1。UDP 的 `0.0.0.0` 是电脑监听地址，HTS 目标应填电脑 Wi-Fi IPv4。

Cup 的 `teleop.quest_config` 只引用默认 USB 配置，显式 `--config` 优先；任务不得同时定义该引用与旧协议/端口/USB 开关。UDP 配置默认 `quest3_source=direct`，不查找本机服务、不调用 ADB；USB TCP 配置默认 `quest3_source=relay`，服务缺失就报错，不回退直连。两条路线保持原始主机时间戳和已有过期/HOLD 逻辑。

高级覆盖（USB TCP 服务）：服务 `--port` 对应消费者 `--quest3-port`，服务 `--socket` 对应消费者 `--quest3-relay-socket`；两端保持一致。旧协议参数保留兼容，和显式 `--config` 模式冲突时报错。详见[简明操作手册](../README_QUEST3_INSPIRE_ROKAE.md#teleoperation)。
所有真实运行配置集中在根目录 `config/`。`integrations/config` 仅为旧手型工具保留指向 `config/retargeting` 的符号链接，不含第二份配置。旧 ROKAE preset 原文归档到 `archive/config/rokae_teleop.yaml.disabled`，不再参与运行。

home 只用于启动回位和显式 `h`，不作为 `r` 的位置条件。控制模式在运行中不切换；任务配置不热加载。
任务 profile 与实际生效参数、CLI、采集配置及 schema 哈希随 episode 保存。
相机进程接收父进程已验证的配置快照，避免启动时重读被修改的文件。

## 初始化与恢复速度

| 路径 | 当前设置 | 配置位置 |
| --- | --- | --- |
| 默认启动 / `05_move_to_init.py` | SDK 250，在线倍率 0.50（此前 0.20） | `robot.initial_joint_move.joint_speed: 0.25`、`online_speed_scale: 0.50` |
| cup 启动与键盘 `h` | SDK 150（此前 50），在线倍率 0.50 | 任务 `home.joint_speed: 0.15`；在线倍率继承机器人 init |
| `06_recover_soft_limits.py` | Jog 5%（此前固定 1%） | `robot.initial_joint_move.recovery_jog_rate: 0.05` |

`joint_speed * 1000` 是 xCoreSDK 的 NRT 速度设置，不是 rad/s 或关节速度百分比。SDK 按设置值划分关节速度档位，在线倍率额外乘于 NRT 轨迹；实际耗时还取决于行程、加减速和分段停止，不能按参数倍数保证实机提速。

```bash
# 只读检查当前 init 计划；显式覆盖速度设置与在线倍率
python rokae/examples/05_move_to_init.py --pose manipulation --speed 250 --online-speed-scale 0.50
# 只读检查恢复计划，Jog 5%；执行时另加 --execute 并输入 RECOVER
python rokae/examples/06_recover_soft_limits.py --clearance-deg 10 --jog-rate 0.05
```

init CLI 优先于 init YAML；Jog CLI 优先于 `recovery_jog_rate`，缺省回退为 0.05。恢复入口允许 1%–10%，不修改单轴、回退范围、手动使能及原软限位要求。遥操作实时速度/加速度/jerk 限幅、数据新鲜度阈值和 1 kHz 周期均保持原配置。

`hold_disabled` 表示键盘/启动门控关闭。跟踪过期独立进入 Tracking HOLD，不关闭操作者使能；手指目标过期阈值 0.20 s，腕部使用 `safety.hold_timeout`（默认 0.15 s）。连续新鲜 0.01 s（`teleoperation.tracking_resume_delay_s`，0 表示首次新鲜数据即恢复）后重建机械臂相对参考并恢复，手目标过渡 0.5 s。`p` 和硬件故障不会被跟踪恢复自动解除。原实时 watchdog 保留，进程卡住导致 safe_stop 仍需排障重启。

SDK 单位、档位、在线倍率及 Jog 范围依据[官方方法说明](https://docs.rokae.com/docs/SDK/cpp/cpp_method/)。本次测试及尚未验证的实机范围见[验证记录](validation/teleop_speed_20261022.md)。

## 环境与安装

```bash
conda activate /data/miniconda3/envs/dexteleop
python -m pip install --no-deps --no-build-isolation -e .
bash tools/install_collection.sh
# 需要导出时：
bash tools/install_collection.sh --export
```

使用已有环境，不再创建 anydex/dexdata/venv。首次手部重定向所需 Pinocchio、串口等仍按
现有 `requirements.txt` 和硬件支持情况安装；不为只读相机或离线处理强制加载所有机器人依赖。
环境版本清单为 [validated_environment.json](../docs/validation/validated_environment.json)。

原生相机 SDK 在 `/data/Software/realsense`，安装说明见[相机模块](../realsense/README.md)。
机器人原生构建默认使用 dexteleop 的 Python，可用 `ROKAE_PYTHON` 显式覆盖；按架构选择内置 aarch64/x86_64 SDK，构建目录默认 `rokae/build/<架构>/`。本机扩展与构建工具见[原生构建说明](../rokae/docs/native_build.md)。
本机 Python 3.10 的采集/导出与官方 OpenPI Python ≥3.11 的训练要求仍需区分。

## 逆解与参数生效范围

三份 YAML 的参数归属与算法限制见[IK/配置审查](ik_config_audit.md)，跨框架映射范式见[调研表](../relatedworks.md)。
Cup home 覆盖通用启动姿态，`manipulation_init` 是独立演示预设；IK 默认 `posture_reference: initial`，在使能/c/恢复时取当前关节参考。`model` 模式才吸向 `neutral_position`。
真机启动前将控制器软限位与模型限位取交集，共同用于 IK 和原生关节流；并查询 16 个虚拟配置的官方 FK。`models/...yaml: validation` 维护容差，不会运动到这些采样点。
当前 Pinocchio 路线主要使用模型中的 IK/关节流限幅，不能把 `safety` 的笛卡尔滤波和工作空间盒视为该路线已执行的约束。

## 手腕与工具旋转轴

`teleoperation.rotation_mapping` 决定空间轴还是工具相对轴；`rotation_scale` 决定角度比例。当前为 tool_relative + 1.0，`wrist_to_tool_rotation` 明确右手系手腕局部轴到工具局部轴，不与位置映射矩阵混用。[具体公式、轴约定与验证](rokae_orientation_mapping.md)。参数在启动时加载，运行中改 YAML 不会热切换。

## 参考响应与计时

`models/xmate_er_pro_7.yaml: stream.response_hz` 只作用于 Pinocchio 关节流；`robots/rokae.yaml: safety.response_hz` 作用于官方笛卡尔位置/阻抗参考。当前均为 2 Hz，各自有唯一来源，不在任务配置重复。正值选择包含加速度反馈的三阶参考生成，0 保留旧递推；这不是实际阻抗阻尼、通信频率或电机带宽。现有速度/加速度/jerk 和超时限制仍生效。

`--timing-report PATH` 可选收集主机分段耗时，每项最多保留最近 4096 样本，退出后写 JSON。`--config` 仍选择通信配置，`--arm-control` 选择后端。完整语义、测量边界与回退方法见[延迟审查](teleop_latency_review.md)。
