# 配置管理

配置按职责分开，统一路径解析与入口组装；不把设备、任务和存储塞进一个大文件。

| 文件 | 唯一职责 |
| --- | --- |
| [config/tasks/cup.yaml](../config/tasks/cup.yaml) | 任务 ID/语言、设备默认参数、collection 引用和任务 home |
| [config/robots/rokae.yaml](../config/robots/rokae.yaml) | 机器人、坐标/TCP 映射、控制器、滤波与运动限制 |
| [config/cameras/realsense.json](../config/cameras/realsense.json) | 双相机角色、序列号、分辨率、FPS、曝光与深度开关 |
| [config/collection/collection.json](../config/collection/collection.json) | 数据位置、频率、同步阈值、字段表示和存储；引用相机配置 |
| `config/retargeting/{adaptive,vector}/` | 各手型/输入源的重定向配置 |

任务配置：

```yaml
schema_version: dexretarget.task.v1
task_id: cup_pick_place
instruction: "pick up the cup and place it in the tray"
teleop:
  arm_config: config/robots/rokae.yaml
collection:
  enabled: true
  config: config/collection/collection.json
# home 的实际关节/手角度与路点应由现场示教确认；参考现有任务文件。
```

显式 CLI > 任务配置 > 程序默认值。正式入口的 `--arm-config`、`--hand-config` 和任务内引用统一相对仓库根目录；绝对路径可直接使用。
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

`hold_disabled` 表示键盘/启动门控关闭。跟踪过期独立进入 Tracking HOLD，不关闭操作者使能；手指目标过期阈值 0.20 s，腕部使用 `safety.hold_timeout`（默认 0.15 s）。连续新鲜 0.20 s 后重建机械臂相对参考并恢复，手目标过渡 0.5 s。`p` 和硬件故障不会被跟踪恢复自动解除。原实时 watchdog 保留，进程卡住导致 safe_stop 仍需排障重启。

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

三份 YAML 已逐项补充中文注释，完整 117 项归属与当前算法限制见[IK/配置审查](ik_config_audit.md)。
Cup home 覆盖通用启动姿态，`manipulation_init` 是独立演示预设，`neutral_position` 是 IK 副任务参考；三者不能混用。
当前 Pinocchio 路线主要使用模型中的 IK/关节流限幅，不能把 `safety` 的笛卡尔滤波和工作空间盒视为该路线已执行的约束。
