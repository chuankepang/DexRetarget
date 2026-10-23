> 重构前操作手册备份。当前入口、目录与测试结果以[使用指南](../../docs/usage.md)和[重构报告](../../docs/refactor_report.md)为准。

# Quest 3 + Inspire Hand + ROKAE：遥操作与数据采集操作手册

本文按两个功能模块组织：

| 功能模块 | 用途 | 启动入口 |
| --- | --- | --- |
| [模块一：遥操作](#teleoperation) | Quest 3 控制机械臂和灵巧手，完成连接、调试与现场操作 | `python example/teleop_arm_hand.py` |
| [模块二：数据采集](#data-collection) | 在联合遥操作过程中录制双相机、臂手状态与动作，导出 π0.5 训练数据 | 同一入口增加 `--capture` |

两个模块共用遥操作控制链路和 **`dexteleop` Conda 环境**。每个终端先执行：

```bash
cd /data/Projects/DexRetarget
conda activate /data/miniconda3/envs/dexteleop
```

下文命令均从仓库根目录运行。采集依赖与导出依赖也安装在此环境，不另建 Python 环境。
项目总导航见[中文 README](../../README.zh.md)，目录职责见[架构说明](../../docs/architecture.md)。

现场设备与部署约定：

| 项目 | 配置 |
| --- | --- |
| Quest | Meta Quest 3 + Hand Tracking Streamer（HTS），默认 UDP 9000 |
| 灵巧手 | Inspire Hand，默认 `/dev/ttyUSB0`、115200 baud、hand id 1 |
| 机械臂 | ROKAE xMate ER Pro 7 / xMatePro3，7 轴 |
| 机器人网络 | 控制器 `192.168.0.160`，控制主机机器人网卡 `192.168.0.100/24` |
| 机器人 SDK | xCoreSDK-CPP 0.3.4，现场控制器 xCore 2.3.2 |
| 双相机 | D435i 腕部/掌侧，D455 全局；各 640×480、30 FPS RGB |

原有真机部署说明来自 Jetson AGX Orin（aarch64）；下文的 `Orin`、`eth0`、`wlan0`
需对应实际控制主机和网卡名。当前内置机器人 SDK 仅支持 aarch64，本机 x86_64 尚无
可用 `_rokae_cpp`；数据处理和模拟检查可运行，真实机器人操作须先完成对应平台构建与验收。
ROKAE 上层保留 Python，实时控制通过 pybind11 下沉到 C++，不使用 `xCoreSDK-Python`。

> 保留原有现场要求：首次真机测试须有人手持急停、清空工作区，并逐级验证只读状态、
> 小关节运动、小 MoveL 和实时小轨迹。启动主程序可能按现有配置执行初始化运动。

<a id="teleoperation"></a>

## 模块一：遥操作

本模块可单独控制手、单独控制臂或联合控制臂手。不加 `--capture` 时不启动训练数据采集。
常用入口见 [1.9 真实 ROKAE 遥操作启动](#teleop-start)；首次部署先完成前面的环境、网络和验收步骤。

### 1.1 系统架构

```text
Quest 3 HTS App
  └── Wi-Fi UDP/TCP：wrist pose + 21 hand landmarks
        ↓
控制主机 / DexRetarget Python
  ├── landmarks → AnyDexRetarget → Inspire 串口指令
  └── wrist SE(3)
        → 相对位姿映射
        → Quest/机器人轴映射与尺度
        → world target
        → world/base 坐标转换
        → 控制路线：默认 pinocchio 主机逆解 → C++ 关节流
          或 cartesian TCP 目标 → C++ 笛卡尔流
              ↓
C++ latest-value slot（仅保留最新目标，不使用 FIFO）
              ↓
C++ / xCoreSDK 1 ms realtime callback
  ├── timestamp/sequence/stale rejection
  ├── translation low-pass
  ├── quaternion SLERP
  ├── 速度、加速度、jerk、workspace 限制
  └── watchdog hold / safe-stop
              ↓
ROKAE jointPosition / CartesianPosition
```

Quest 常见更新率为 60/72/90 Hz；机器人回调为 1 kHz。两者完全解耦。Python
收包、手部重定向、日志或 policy 推理暂时变慢时，不会阻塞 C++ 实时回调，也不会
在恢复后补执行旧 VR 数据。

### 1.2 目录和关键文件

```text
DexRetarget/
├── example/teleop_arm_hand.py          # Quest + Inspire + ROKAE 主程序
├── example/input/quest3.py             # HTS UDP/TCP 解析
├── example/output/real/
│   ├── drivers_inspire.py              # Inspire 串口输出
│   └── drivers_rokae.py                # C++ ROKAE Python 封装
├── rokae/
│   ├── include/rokae_driver.hpp
│   ├── src/                            # 笛卡尔实时 + 1 kHz 关节流
│   ├── python/rokae.py
│   ├── config/rokae.yaml               # 遥操作配置；control.mode 默认 pinocchio
│   ├── examples/                       # SDK 工具：读状态、MoveJ、MoveL
│   ├── control/                        # 主机逆解和任务脚本（长方体、马克杯）
│   ├── sdk/                            # 内置 xCoreSDK-CPP 0.3.4
│   ├── third_party/pybind11/
│   ├── CMakeLists.txt
│   └── build.sh
├── realsense/
│   ├── config/cameras.json             # 双相机角色、序列号和流规格
│   └── scripts/check_cameras.py        # 相机枚举与同时采集诊断
├── data/
│   ├── config/collection.json          # 频率、同步、存储和数据表示
│   ├── collection/                     # 采集、对齐、校验、回收和导出
│   ├── openpi/                         # π0.5 数据适配与训练注册入口
│   └── raw/                           # 运行时生成的原始 episode
└── tests/
```

工程内已经包含运行需要的 SDK 文件：

```text
rokae/sdk/include/rokae/
rokae/sdk/include/Eigen/
rokae/sdk/lib/aarch64/libxCoreSDK.so.0.3.4
rokae/third_party/pybind11/include/pybind11/
```

该 SDK 的 ARM64 编译链使用仓库内置依赖。当前内置运行库仅支持 aarch64；
x86_64 主机须先取得匹配架构的官方 SDK 并适配构建，不能直接使用此 ARM64 动态库。

### 1.3 编译并检查 C++ ROKAE 模块

以下构建步骤适用于 aarch64 部署机。`rokae/build.sh` 尚有历史 Python 默认路径，
因此显式指定 `ROKAE_PYTHON`，确保绑定编译到 `dexteleop` 使用的 Python ABI。
本机 x86_64 尚未具备可用的原生绑定，不应把下面步骤视为已完成。

```bash
cd /data/Projects/DexRetarget
ROKAE_PYTHON=/data/miniconda3/envs/dexteleop/bin/python bash rokae/build.sh
python -c 'import rokae; print(rokae.SDK_VERSION); print(rokae.module_file())'
```

预期包含：

```text
0.3.4
/data/Projects/DexRetarget/rokae/python/_rokae_cpp.cpython-310-aarch64-linux-gnu.so
```

检查动态库确实使用工程内部 SDK：

```bash
ldd rokae/python/_rokae_cpp*.so | grep xCoreSDK
```

预期指向：

```text
/data/Projects/DexRetarget/rokae/sdk/lib/aarch64/libxCoreSDK.so.0.3.4
```

### 1.4 网络连接

#### 1.4.1 Orin 与 ROKAE 控制柜

ROKAE 控制柜通过网线连接 Orin `eth0`。执行：

```bash
sudo ip addr flush dev eth0
sudo ip addr add 192.168.0.100/24 dev eth0
sudo ip link set eth0 up
```

检查：

```bash
ip -br -4 addr show eth0
ip route get 192.168.0.160
ping -c 3 192.168.0.160
```

应看到 Orin 通过 `eth0`、源地址 `192.168.0.100` 访问机器人
`192.168.0.160`。机器人链路不能填写 Orin 的 Wi-Fi/热点地址。

#### 1.4.2 Quest 3 与 Orin

Quest 和 Orin `wlan0` 必须连接同一 Wi-Fi 或同一手机热点，并且 AP 不能开启客户端
隔离。先在 Orin 查询无线地址：

```bash
ip -br -4 addr show wlan0
```

例如显示 `172.20.10.2/28`，HTS App 的 IP 就填写 `172.20.10.2`。不要填写
`192.168.0.100`，那是 Orin 到机器人的有线地址。

HTS App 推荐配置：

```text
IP:       <Orin wlan0 当前 IPv4>
Port:     9000
Protocol: UDP
Hands:    Right（右手）或 Both Hands
```

优先使用单播到 Orin `wlan0` 地址。`255.255.255.255` 广播在手机热点上可能被拦截。
如 App 支持 TCP，也可选择 TCP；此时 Orin 程序是监听端，App 主动连接 Orin 的
`wlan0_IP:9000`。

在启动完整程序前可检查网络层：

```bash
sudo tcpdump -ni wlan0 udp port 9000
```

TCP 模式则使用：

```bash
sudo tcpdump -ni wlan0 tcp port 9000
```

如果 `tcpdump` 完全没有数据，问题在热点、IP、协议或 HTS App，不在 DexRetarget
解析器。

#### 1.4.3 Inspire 串口

```bash
ls -l /dev/ttyUSB*
```

默认使用：

```text
/dev/ttyUSB0
115200 baud
hand id = 1
```

若没有权限：

```bash
sudo usermod -aG dialout "$USER"
```

重新登录后生效。Inspire 使用直接串口，不经过 ROS 2 topic。

### 1.5 ROKAE 统一配置

配置文件：

```text
config/robots/rokae.yaml
```

#### 1.5.1 机器人与 init

```yaml
robot:
  type: xmate-er-pro-7
  ip: 192.168.0.160
  local_ip: 192.168.0.100
  control_hz: 1000
  rt_network_tolerance: 20
  controller_rate_limit: true
  controller_filter_cutoff_hz: 30.0

  initial_joint_move:
    enabled: true
    joint_position: [0.0, 0.35, 0.0, 1.85, 0.0, -0.6292036732, 1.5707963268]
    joint_speed: 0.25
    online_speed_scale: 0.20
    max_segment_delta: 0.35
    timeout: 60.0
    recovery_trigger_margin: 0.15
    recovery_clearance: 0.35
    recovery_speed: 20
    recovery_timeout: 60.0
    min_joint_limit_margin: 0.174533
    min_shoulder_bend: 0.35
    min_elbow_bend: 0.35
    min_wrist_bend: 0.35

non_realtime:
  # 对 MoveAbsJ/MoveL 命令速度的二次倍率；每次 NRT 运动前显式写入控制器。
  online_speed_scale: 1.0
  # SDK 分档：<100≈10%，100..200≈30%，200..500≈50%，
  # 500..800≈80%，>800≈100%。独立关节测试默认使用中间档。
  joint_speed: 300
  cartesian_speed: 20
  timeout: 60.0
```

- 关节单位为 rad；
- 当前正装高肘 init 为 `[0, 0.35, 0, 1.85, 0, -0.62920367, +pi/2]` rad；
- 名义运动学下，J7 腕部旋转轴（`+TCP_Z`）严格指向 `+world_X`，因此绕世界 X
  的滚转与 J7 腕部滚转一致；`TCP_X` 与世界 Y 轴平行；
- xCore 手册明确列出 ER Pro 的 J2=0、J4=0、J6=0 和腕心位于 J1 轴正上方四类
  奇异；本点 J2=`20.1°`、J4=`106.0°`、J6=`-36.1°`，名义 TCP 约为
  `[0.681, 0.000, 0.496] m`。名义肘点约 `[0.135, 0, 0.712] m`、腕点约
  `[0.431, 0, 0.496] m`，形成高肘、前臂前下倾、腕轴水平向前的构型，并分别
  避开前三类和基座正上方腕心区域。[xCore 控制系统手册：ER Pro 奇异位置](https://static.rokae.com/Downloads/Manual/xCore%E6%8E%A7%E5%88%B6%E7%B3%BB%E7%BB%9F%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8CV3.2_A.pdf)
- 手册定义的精确奇异点是关节等于零；YAML 的 `0.35 rad` 只用于笛卡尔逆解选构型。
  `05_move_to_init.py` 走的是 MoveAbsJ，关节轨迹可以靠近或穿过 J2/J4/J6 = 0，
  不会因此拒绝运动；
- ER3 Pro 手册规定 J1/J3/J5 为 ±170°、J2/J4/J6 为 ±120°、J7 为 ±360°；脚本还
  强制至少保留 10° 机械限位余量。[ER3 Pro 硬件安装手册](https://static.rokae.com/Downloads/Manual/xMate%20ER3%20Pro%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85%E6%89%8B%E5%86%8C.pdf)
- `enabled: true`，主程序连接机器人后、建立 Quest 参考前会先执行 MoveAbsJ；
- 路径先保持实测 J7，只把 J1–J6 移到目标臂形；然后才将 J7 移到 `+pi/2`。
  两段均按 `max_segment_delta=0.35 rad` 自动插值；总行程不设人为上限；
- 当前 YAML 的 init `joint_speed=0.25`，代码换算为 SDK speed `250`，另乘独立 NRT
  在线倍率 `0.20`。这与旧文档的 speed `50` 不同，现场需重新确认初始化速度与路径；
  本次只更新说明，没有修改运动配置。参数仅作用于初始化 `MoveAbsJ`，不决定实时跟踪速度；
- 调试时可以用 `--skip-init` 临时跳过；
- 七轴机器人建议使用 joint init，确保每次肘部构型一致。
- `joint_speed` 是 init 轨迹速度档，init 下的 `online_speed_scale` 是额外乘法倍率；
  二者都不决定 1 kHz 遥操作速度。

#### 1.5.2 正装 `T_world_base`

当前机械臂正装，base 与 world 的三轴方向完全一致；原点是否重合仍由 XYZ 标定：

```text
base X → world [1, 0, 0]
base Y → world [0, 1, 0]
base Z → world [0, 0, 1]
```

因此：

```text
R_world_base = I
T_world_base = [I, p_world_base; 0 0 0 1]
```

YAML 使用 `X Y Z QX QY QZ QW`：

```yaml
base_transform:
  pose_xyzw: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
```

这里四元数是单位四元数 `QX QY QZ QW = 0 0 0 1`。前三个零仍是 XYZ 平移占位值；
若 world 原点不在机器人 base 原点，必须填入测得的 `p_world_base`。程序统一使用：

```text
T_base_world      = inverse(T_world_base)
T_base_tcp_target = T_base_world @ T_world_tcp_target
```

不要在其他文件中额外写 `y=-y` 或 `z=-z`。

还要区分两个容易混淆的量：

- YAML 的 `base_transform` 是 DexRetarget 用于 Quest/world 映射的外部标定；
- xCoreSDK 的 `baseFrame()` 是控制器内部保存的“机器人 base 相对于控制器 world”。

机械臂由倒装改成正装后，Robot Assist 中的安装方式/基座姿态和负载参数也必须同步
修改。仅改 YAML 不能修正控制器的重力补偿。可用下面的无运动 dry-run 查看 SDK
报告的 `baseFrame xyz-rpy`：

```bash
python rokae/examples/03_move_cartesian.py \
  --relative 0.001 0 0 0 0 0 --relative-frame base
```

本次正装应接近 `[0, 0, 0, 0, 0, 0]`。如果仍显示旧的
`[0, 0, 0, 3.14159, 0, 0]`，先在 Robot Assist 修正安装配置并重新做负载/TCP
确认，不要启动实时遥操作。C++ 的普通 MoveL 坐标转换虽然会读取并补偿当前
`baseFrame()`，但这不能代替控制器正确的安装方向和重力模型。

`05_move_to_init.py` 和 `teleop_arm_hand.py` 现在都会在上电、发运动指令之前读取该值。
当旋转偏离单位阵超过 `0.02 rad` 时直接退出，并明确保证没有发送运动；不要通过关闭
此检查来绕过尚未完成的正装配置。

若输出为 `diag(1,-1,-1)`，即 `Rx(pi)`，说明控制器仍是倒装配置。进入 Robot Assist：

1. 打开“设置 → 基坐标系标定”（不同 Robot Assist 版本可能归在“设置 → 标定”下）；
2. 选择“手动输入”，安装方式选择“正装”；
3. 若 world 与 base 原点重合，位置填 `[0,0,0]`，Euler 姿态填 `[0,0,0]`；若原点
   不重合，只填写实测 XYZ，姿态仍为零；
4. 保存/应用，并按界面要求重启控制器；
5. 同步确认当前工具负载、质心和动力学参数符合 Inspire 手及转接件；
6. 重启后先运行 `python rokae/examples/01_read_state.py`，确认
   `controller world_T_base` 的旋转为单位阵，再运行 init dry-run。

不要进行机械零点标定；这里修改的是“基坐标系标定/安装方式”，不是七个关节零点。

#### 1.5.3 Quest 映射

```yaml
teleoperation:
  translation_scale: 0.80
  rotation_scale: 0.70
  quest_to_robot_rotation:
    [1.0, 0.0, 0.0,
     0.0, 1.0, 0.0,
     0.0, 0.0, 1.0]
```

- `translation_scale`：手移动 10 cm 时，机械臂目标移动 8 cm；
- `rotation_scale`：手转动 1 rad 时，目标转动 0.7 rad；
- `quest_to_robot_rotation`：Quest 右手坐标轴到机器人遥操作增量坐标轴的旋转矩阵；
- 初次实机应先减小尺度，逐轴检查 X/Y/Z 和旋转正负方向。

#### 1.5.4 安全参数

```yaml
safety:
  translation_cutoff_hz: 5.0
  rotation_cutoff_hz: 5.0
  translation_deadband: 0.001
  rotation_deadband: 0.008
  max_translation_speed: 0.15
  max_angular_speed: 0.50
  max_translation_acceleration: 0.40
  max_angular_acceleration: 1.50
  max_translation_jerk: 2.5
  max_angular_jerk: 8.0
  max_target_translation_delta: 0.25
  max_target_rotation_delta: 1.0
  workspace_min: [-1.2, -1.2, -1.2]
  workspace_max: [1.2, 1.2, 1.2]
  hold_timeout: 0.15
  stop_timeout: 0.60
  max_source_age: 0.20
  future_tolerance: 0.05
```

这是稳定调试配置之上的候选“一档提速”：目标滤波由 3 Hz 提到 5 Hz，控制器滤波由
20 Hz 提到 30 Hz，同时仍保留 1 mm/0.008 rad 死区以及速度、加速度、jerk 三层约束。
不要一次继续提高多项参数；若重新出现嗡鸣或 S 型摆动，立即停机并退回下述旧参数。
这些软件上限不替代控制柜安全区、碰撞检测、关节限制和急停。第一次实机前必须根据
当前 base 坐标系测量并缩小 `workspace_min/max`。

这里“恢复”指退回上一轮已稳定的 commissioning 值：controller filter `20 Hz`、上层
平移/旋转滤波 `3 Hz`、死区 `0.002 m / 0.015 rad`、最大速度 `0.10 m/s / 0.35 rad/s`、
最大加速度 `0.25 m/s² / 1.0 rad/s²`、最大 jerk `1.5 m/s³ / 6.0 rad/s³`。只改 YAML
即可，不需要重新编译。

### 1.6 机械臂独立验收流程

以下步骤不需要 Quest，也不需要 Inspire。所有运动工具默认 dry-run。真实运动必须添加
`--execute`，之后在终端输入大写 `MOVE`。只有完成现场验收后才考虑用 `--yes` 跳过
输入确认。

#### 1.6.1 第一步：只读机器人状态

```bash
python rokae/examples/01_read_state.py
```

也可显式指定连接参数：

```bash
python rokae/examples/01_read_state.py \
  --robot-type xmate-er-pro-7 \
  --robot-ip 192.168.0.160 \
  --local-ip 192.168.0.100
```

输出包括：连接状态、SDK/控制器版本、电源状态、操作模式、运行状态、7 个关节角、
关节速度和 4×4 `base_T_tcp`。只读脚本不上电、不切换运动模式、不发送目标。

代码使用官方 `tcpPose_m` 读取 TCP；该字段明确定义为末端相对于机器人 base 的
行优先齐次矩阵，平移单位为 m。

#### 1.6.2 第二步：小范围关节运动

先把当前 7 个关节角复制出来，只改一个关节很小的量，例如不超过 0.02 rad。

先 dry-run：

```bash
python rokae/examples/02_move_joint.py \
  --joints J1 J2 J3 J4 J5 J6 J7 \
  --speed 300 --max-joint-delta 0.05
```

确认当前位置、目标和差值正确后执行：

```bash
python rokae/examples/02_move_joint.py \
  --joints J1 J2 J3 J4 J5 J6 J7 \
  --speed 300 --max-joint-delta 0.05 \
  --execute
```

遥操作初始位置init_pose

```bash
joint_position: [0.0, 0.35, 0.0, 1.85, 0.0, -0.6292036732, 1.5707963268]
```

终端提示后输入 `MOVE`。目标是绝对关节角，单位 rad。任一关节目标差超过
`max_joint_delta` 会在发送前拒绝。程序等待目标稳定；超时会 stop 并清理运动队列。

#### 1.6.3 第三步：普通笛卡尔 MoveL

基座坐标系 +X 移动 5 mm、姿态不变：

```bash
python rokae/examples/03_move_cartesian.py \
  --relative 0.005 0 0 0 0 0 \
  --relative-frame base --speed 10
```

确认 dry-run 后：

```bash
python rokae/examples/03_move_cartesian.py \
  --relative 0.005 0 0 0 0 0 \
  --relative-frame base --speed 10 \
  --execute
```

`--relative DX DY DZ RX RY RZ` 的前三项单位 m，后三项是 rotation vector，单位 rad，
不是 Euler 角直接相减。

工具坐标系 +Z 移动 3 mm：

```bash
python rokae/examples/03_move_cartesian.py \
  --relative 0 0 0.003 0 0 0 \
  --relative-frame tcp
```

绝对目标格式：

```bash
python rokae/examples/03_move_cartesian.py \
  --pose-xyzw X Y Z QX QY QZ QW
```

NRT `MoveLCommand` 接收的是当前 SDK Toolset 下的 `ref_T_end`，而本工程对外统一使用
实时状态/遥操作接口的 `base_T_tcp`。两者不能直接等同，也不要求 work object 与 base
重合。桥接层会读取 `baseFrame()`、`toolset.ref/end`、`flangeInBase`、`endInRef` 和
`tcpPose_m`，保持现有工具、工件与负载配置，并执行：

```text
flange_T_tcp = inverse(base_T_flange) * base_T_tcp_current

ref_T_end_target = inverse(world_T_ref) * world_T_base
                 * base_T_tcp_target * inverse(flange_T_tcp)
                 * flange_T_end
```

发送前会用当前状态闭环重建 `ref_T_end_current`。平移误差超过 2 mm 或旋转误差超过
0.02 rad 时拒绝运动；正常输出应接近：

```text
[ROKAE] MoveL frame check: translation_error=0 m rotation_error=0 rad
```

`03_move_cartesian.py` 在输入 `MOVE` 前就完成该只读预检，并打印转换后的
`converted NRT ref_T_end target`。不加 `--execute` 可安全检查整条坐标链而不运动。

`max_translation_delta`/`max_rotation_delta` 是闭区间上限。程序在用户确认后会再次读取
TCP，因此传感器微小抖动和浮点舍入可能让名义 `0.05 m` 变成 `0.050000000x m`。边界
比较仅加入 `1e-6 m` 和 `1e-6 rad` 的数值容差；真正超过上限时，异常会同时打印实际
增量和配置上限。

`Frame(array6)` 的接口语义是只初始化 `trans/rpy`，不会自动同步内部 `pos`；直接读取
`Frame.pos` 会表现为 `end-in-reference TCP pose has an invalid bottom row`。桥接层使用
官方六维 `posture(endInRef)` 后，必须调用 SDK 的 `Utils::postureToTransArray()` 显式按
`Rz * Ry * Rx` 约定生成完整的 4×4 矩阵。

反方向同样不能省略：NRT `MoveLCommand` 的控制器规划器读取
`CartesianPosition::trans/rpy`。将 16 维矩阵直接传给 `CartesianPosition(array16)` 只会
填充 `pos`，在 xCore 2.3.2 上会报 `[SDK MoveL] 输入位姿有误`。发送前必须使用
`Utils::transArrayToPosture()` 转成 `[x,y,z,rx,ry,rz]`，再构造 NRT 目标。代码同时显式
设置 `setDefaultConfOpt(false)`，让七轴逆解选取距离当前关节角最近的解，而不是要求目标
携带 `confData`。

另外，Eigen 的 `Block` 不能引用已经销毁的临时矩阵。原 `rotation_distance()` 中
`auto block = pose_matrix(...).topLeftCorner()` 会形成悬空引用并产生随机大角度误差；
现在强制求值到拥有自身存储的 `Eigen::Matrix3d/Vector3d`。

#### 1.6.4 第四步：保存当前位置为候选 init

手动示教或小范围调试到合适姿态后：

```bash
python rokae/examples/save_current_as_init.py
```

该程序只读并打印 YAML，不会修改配置文件。检查无碰撞、远离奇异点和 workspace
边界后，把输出的关节角复制到：

```yaml
robot:
  initial_joint_move:
    enabled: true
    joint_position: [J1, J2, J3, J4, J5, J6, J7]
```

#### 1.6.5 第五步：单独验证 init

先 dry-run：

```bash
python rokae/examples/05_move_to_init.py
```

确认后执行：

```bash
python rokae/examples/05_move_to_init.py --execute
```

程序执行：连接 → 读取控制器真实软限位与当前 joints/TCP → 检查 ER3 Pro 手册范围与
10° 余量 → 检查 J2/J4/J6 官方奇异保护角 → 自动插值小段 →
控制器规划 MoveAbsJ → 每段等待控制器平滑停止 → 读取最终 joints/TCP → 计算并打印
`world_T_tcp`、J7 腕轴与世界 +X 的对齐误差及夹爪横轴误差。总关节路程不再受
`max_joint_delta` 限制：规划器先保持实测 J7 不变，将 J1–J6 插值到目标形状，再插值
J7；每段不超过 `0.35 rad`，每段仍由控制器完成同步加减速。

预期完成后两个姿态误差接近 0°：

```text
hand alignment: wrist_axis(+TCP_Z/J7 vs +world_X)=... deg,
                gripper lateral(TCP_X parallel world_Y)=... deg
```

若任一误差超过 5°，脚本会警告；此时不要进入遥操作，先核对
`base_transform.pose_xyzw`、Robot Assist 当前 TCP，以及灵巧手相对法兰的安装方向。

如果当前位置离 init 很远，程序会自动增加插值段数；但插值只约束关节步长，不会做
环境避障，首次路径仍需低速、空场地并随时准备急停。

##### 关节已到软限位、普通 MoveAbsJ 无法退回

当程序打印某轴距控制器软限位不超过 `0.15 rad` 时，正常 init 和遥操作入口都会拒绝
运动。清空工作区并确认目标确实朝限位内侧后，运行：

```bash
python rokae/examples/05_move_to_init.py \
  --execute --recover-joint-limits
```

先输入 `RECOVER`：程序保存控制器原软限位，下电并切到手动模式，临时关闭软限位，
仅将临界轴低速退到距原限位 `0.35 rad` 的位置；随后停止、再次下电/手动、恢复并回读
验证原软限位。之后程序重新规划 init，并要求第二次输入 `MOVE`。不要把该参数加入日常
启动命令；遥操作程序绝不会自动关闭软限位。若出现 `CRITICAL: failed to restore...`，
保持机器人停止，在 Robot Assist 中恢复软限位后再进行任何运动。

#### 1.6.6 第六步：C++ realtime 小轨迹

默认轨迹为 base X 方向 ±5 mm、0.1 Hz、5 s，Python 以 100 Hz 更新最新目标，C++
和 SDK 以 1 kHz 输出控制命令。

先 dry-run：

```bash
python rokae/examples/04_realtime_cartesian_test.py
```

确认后执行：

```bash
python rokae/examples/04_realtime_cartesian_test.py --execute
```

也可指定更保守参数：

```bash
python rokae/examples/04_realtime_cartesian_test.py \
  --axis x --amplitude 0.002 --frequency 0.05 \
  --duration 3 --update-hz 100 \
  --execute
```

`amplitude` 被限制为不超过 0.01 m。此步骤成功并验证 stop 后，才进入 Quest 真机
遥操作。

#### 1.6.7 关于上电

所有会产生运动的 ROKAE 程序现在默认通过 xCoreSDK 请求自动上电，不再要求先在
Robot Assist 中手动上电，也不需要额外添加 `--power-on`。只读程序
`01_read_state.py` 仍然不会上电、切换模式或发送运动指令。

如现场安全流程明确禁止 SDK 自动上电，可以给独立运动工具添加 `--no-power-on`，给
联合遥操作程序添加 `--no-rokae-power-on`。`--execute` 和人工输入 `MOVE` 的实机运动
确认仍然保留；自动上电不等于自动跳过运动确认。

### 1.7 无硬件测试和仿真

Python 测试使用 pytest；原生实时测试还要求可用的 `_rokae_cpp`：

```bash
python -m pytest tests -q
```

本机尚有原生绑定缺失和既有初始化速度测试不一致；已运行的范围及结果见
[验证记录](data_collection_validation_20261022.md)，不应将本命令理解为全套测试已通过。

运行实际 C++ realtime core 的频率、抖动、丢包、延迟尖峰和 Python 暂停仿真：

```bash
python example/test_rokae_realtime_sim.py \
  --quest-hz 60 72 90 --robot-hz 1000 \
  --duration 2.0 \
  --jitter-ms 12 --delay-ms 25 --loss 0.08 \
  --spike-rate 0.03 --spike-min-ms 50 --spike-max-ms 200 \
  --pause-start 1.0 --pause-duration 0.25 \
  --csv /tmp/rokae_cpp_timing.csv
```

输出包含接收/拒绝数量、最大速度/加速度/jerk、最大单周期步长、平均延迟、跟踪误差、
hold 周期和 safe-stop 周期。

### 1.8 Quest 输入和 Inspire 独立验证

#### 1.8.1 只运行 Inspire

先启动控制主机程序，再在 Quest HTS App 中点击 `Stream Started`。程序就绪后按 `e` 使能手：

```bash
python example/teleop_arm_hand.py \
  --enable-hand --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --inspire-port /dev/ttyUSB0 \
  --inspire-baudrate 115200 \
  --inspire-hand-id 1
```

这条命令不会连接 ROKAE。确认手指关键点和 Inspire 动作正常后再测试机械臂。

#### 1.8.2 用 Mock ROKAE 验证 Quest wrist

Mock 不连接机械臂，但走完整相对位姿映射和安全状态机；同样默认暂停，就绪后按 `e` 使能：

```bash
python example/teleop_arm_hand.py \
  --enable-arm --mock-rokae --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --arm-print-hz 10
```

看到连续变化的 `ARM_TARGET`、非零 `xyz` 和递增输入帧率，说明 Quest wrist 已进入
遥操作链路。

TCP 模式：

```bash
python example/teleop_arm_hand.py \
  --enable-arm --mock-rokae --hand right \
  --quest3-protocol tcp --quest3-port 9000
```

<a id="teleop-start"></a>

### 1.9 真实 ROKAE 遥操作启动

#### 1.9.1 启动前检查表

1. 急停可用，现场有人监护；
2. 工作区内无人、无障碍物；
3. Orin `eth0=192.168.0.100/24`，机器人可 ping；
4. Quest 和 Orin `wlan0` 同一局域网；
5. HTS App IP 是 Orin `wlan0` 地址，端口和协议一致；
6. `01_read_state.py` 已成功；
7. 小关节、MoveL、init、realtime sine 已逐项验收；
8. `T_world_base`、Quest 三轴、workspace 和 TCP/tool 已确认；
9. 初次运行降低 translation/rotation scale 和速度上限；
10. 急停、安全门、报警及外接使能条件正常，控制柜允许 SDK 自动上电和控制。

#### 1.9.2 只遥操作 ROKAE

推荐明确写出 `--enable-arm`：

```bash
python example/teleop_arm_hand.py \
  --enable-arm --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --arm-config ../config/robots/rokae.yaml \
  --rokae-robot-ip 192.168.0.160 \
  --rokae-local-ip 192.168.0.100 \
  --arm-print-hz 10
```

当前 YAML 的 `initial_joint_move.enabled: true`，该命令连接后会自动运动到上述
joint init。首次实机必须先用 `05_move_to_init.py` 验证路径无碰撞；尚未完成验收时，
启动主程序必须临时添加 `--skip-init`。

程序启动顺序：

```text
连接 ROKAE
→ 读取当前状态
→ 可选 MoveAbsJ 到 joint init
→ 等待稳定并读取 base_T_tcp_ref
→ 确认 Quest wrist 流存在
→ 按 control.mode 启动对应实时路线并立即 HOLD
→ 丢弃模式切换期间的旧 wrist
→ 等待实时模式启动后到达的新 wrist
→ 用当前 command pose 同时建立机器人/Quest 相对参考
→ arm 保持 DISABLED/HOLD
→ 操作者手动使能
```

以下臂角说明针对 `--arm-control cartesian` 路线；当前默认 `pinocchio` 路线使用主机逆解
与关节流。对于 ER Pro 七轴机型，C++ 笛卡尔驱动在进入实时模式时读取当前 `elbow_m`（臂角），并在
每个 1 kHz `CartesianPosition` 命令中设置 `hasElbow=true` 和同一个 `elbow`。这会固定
七轴冗余构型，避免控制器在只要求 TCP 姿态时自行用 J5/J6 改变臂角。当前 init 又令
J7 腕轴与世界 +X 对齐，因此绕世界 X 的纯滚转应主要表现为 J7 运动。使用笛卡尔路线时，若终端没有打印
`locking 7-axis realtime elbow angle at ... rad`，应先核对路线与臂角锁定状态。固定臂角只约束
七轴冗余参数，并不等价于强制 `J7-only`；离开精确init构型后，xCore为保持完整TCP
位姿而联动J5/J6可能是合理的。若必须始终只动J7，需要另建关节空间实时控制模式，不能
继续把它当作笛卡尔位姿控制。

`ARM_TARGET` 现在同时打印旋转轴和七轴状态，专门用于判断“手腕翻转但TCP不转”：

- `vr_body_dr_deg=[...]`：Quest参考手腕坐标下的原始相对旋转；
- `mapped_world_dr_deg=[...]`：经过Unity左手系转换、轴映射和
  `rotation_scale` 后，实际要求机器人执行的世界坐标系旋转向量；
- `tracking_world_dr_deg=[...]`：C++平滑目标相对控制器实测TCP仍未完成的世界坐标系
  旋转；
- `q=[J1,...,J7]`：1 kHz状态流中的实测关节角；
- `elbow(meas/cmd)=.../...`：控制器实测臂角与每周期固定发送的臂角。

在当前正装、`world_R_base=I`、J7轴对齐`+world_X`的init下，缓慢绕世界X翻转时，
`mapped_world_dr_deg` 应主要只有第一个分量。若它不是X，问题在Quest姿态/动作定义或
`quest_to_robot_rotation`；若它是X但`tracking_world_dr_deg`持续接近该值、TCP实测矩阵
不变，问题在xCore笛卡尔实时执行；若跟踪误差接近零，则控制器实际上已经完成TCP
旋转，J5/J6伴随运动只是七轴逆解分配或工具外观造成的观感。固定臂角后
`elbow(meas/cmd)`应基本一致；明显漂移表示当前xCore 2.3.2没有按该实时字段约束冗余解。

机械臂和灵巧手默认均暂停跟随。在运行程序的同一个终端使用键盘：

| 按键 | 功能 |
| --- | --- |
| `e` | 同时使能/暂停臂手跟随；使能时重新建立机械臂相对参考 |
| `p` | 明确暂停臂手；不自动返回 home |
| `c` | 重设机械臂 Quest 相对参考，保留当前机器人命令 TCP |
| `h` | 暂停且没有活动录制时，独立执行任务配置中的 home 路径 |
| `q` / Ctrl+C | 退出程序 |
| `?` | 显示帮助和当前状态 |

交互终端直接按键，无需回车；管道/非 TTY 输入使用一行一个命令。启用前须有新鲜的
Quest 手腕和手部关键点。暂停后不再向臂手下发新的 Quest 目标；手保留最后下发目标，
这不等于机械释放或断电。追踪过期会暂停统一门控，恢复输入后需要再次按 `e`。
不再使用 Quest 按钮或进程信号进行使能。

#### 1.9.3 Inspire + ROKAE 联合遥操作

```bash
python example/teleop_arm_hand.py \
  --enable-hand --enable-arm --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --inspire-port /dev/ttyUSB0 \
  --inspire-baudrate 115200 \
  --inspire-hand-id 1 \
  --arm-config ../config/robots/rokae.yaml \
  --rokae-robot-ip 192.168.0.160 \
  --rokae-local-ip 192.168.0.100 \
  --arm-print-hz 10
```

同样在当前终端按 `e` 同时使能臂手，按 `p` 同时暂停。

#### 1.9.4 调试阶段直接使能

`--arm-start-enabled` 会跳过默认的键盘 `e` 使能步骤，并使能统一臂手门控。只应在完成全部实机验收
并确认 Quest tracking 稳定后使用，首次运行不要添加。

#### 1.9.5 安全停止

正常退出使用 `Ctrl+C`。程序会请求 hold、停止 realtime、停止状态接收、切回非实时
模式并断开机器人。异常情况下优先按现场急停，不要依赖 SSH 或 Python 退出作为唯一
安全措施。

### 1.10 相对遥操作计算方法

启动/重新中心化时同时记录：

```text
T_vr_ref
T_base_tcp_ref
```

运行中计算：

```text
dp_quest_world = p_vr_current - p_vr_ref
dR_quest_world = R_vr_current @ transpose(R_vr_ref)

dp_world = translation_scale * R_quest_to_robot @ dp_quest_world
dR_world = Exp(rotation_scale * Log(
    R_quest_to_robot @ dR_quest_world @ transpose(R_quest_to_robot)))

T_world_tcp_ref = T_world_base @ T_base_tcp_ref
p_world_target  = p_world_tcp_ref + dp_world
R_world_target  = dR_world @ R_world_tcp_ref
T_base_tcp_target = inverse(T_world_base) @ T_world_tcp_target
```

平移和旋转现在都是 Quest tracking world 中的 spatial delta，不再被初始手腕方向或初始
TCP 方向二次旋转；这可避免直线手部运动映射成非预期曲线。旋转使用 SO(3)、rotation
vector 和 quaternion，不使用 Euler angle subtraction。
最终发送给 ROKAE 的是 base 坐标系下绝对 4×4 TCP 目标；“增量式遥操作”指目标由
启动参考的相对手腕运动生成，不是每帧向机器人累加一个无界增量。

### 1.11 C++ 实时控制与 watchdog

下述 TCP latest-value 与 `CartesianPosition` 规则针对笛卡尔路线；默认 Pinocchio
关节流的实现与诊断见 [ROKAE 控制模块](../../rokae/control/README.md)。

每条 Python → C++ 目标包含：

```text
base_T_tcp_target
source_timestamp
receive_timestamp
sequence_id
```

处理规则：

- 新目标直接覆盖旧目标，没有 FIFO；
- 重复或倒退 sequence 拒绝；
- source timestamp 倒退拒绝；
- source age 超过 `max_source_age` 拒绝；
- 单帧目标跳变超过 `max_target_translation_delta` 或
  `max_target_rotation_delta` 拒绝；
- 输入年龄小于 `hold_timeout` 时跟踪；
- `hold_timeout` 至 `stop_timeout` 之间平滑保持最后安全命令；
- 超过 `stop_timeout` 返回 `CartesianPosition.setFinished()` 并结束实时运动；
- 新输入不会触发历史命令追赶。

平移使用按真实 callback `dt` 计算的一阶低通，旋转使用 quaternion SLERP。速度、
加速度和 jerk 的限制同样基于真实 `dt`。C++ callback 异常会保存错误文本、安全结束
运动，并传回 Python 主线程。

### 1.12 调试日志和目标接口

`--arm-record` 保存机械臂调试 JSONL。双相机、臂手实测状态与训练 episode 的采集
使用[模块二](#data-collection)，两者用途不同。调试记录示例：

```bash
python example/teleop_arm_hand.py \
  --enable-arm --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --arm-record logs/rokae_session.jsonl
```

每条记录包含 Quest wrist、相对变换、world/base 目标、sequence、测量 TCP、watchdog
和 command age。

Python 侧的目标下发接口示例：

```python
accepted = driver.set_target_pose(
    base_T_tcp_target,
    source_timestamp=time.monotonic(),
    sequence=sequence,
)
```

策略输出频率由训练数据与部署方案决定；当前数据模块按 20 Hz 对齐。此处仅说明底层
目标接口，不是完整的策略执行程序。部署时应沿用已验证的控制路线，并保持采集与推理
的动作含义一致；1 kHz 控制仍由底层负责。

### 1.13 常见问题

#### 1.13.1 `packets=0` 或一直等待 Quest reference

```bash
ip -br -4 addr show wlan0
ss -ulnp | grep ':9000'
sudo tcpdump -ni wlan0 udp port 9000
```

检查 HTS IP 是否为 Orin `wlan0` 当前地址、App 与程序的 UDP/TCP 是否一致、热点是否
隔离客户端、是否有其他进程占用 9000。手机热点地址变化后需要重新填写 App。

#### 1.13.2 `GLIBCXX_3.4.29 not found`

```bash
conda activate /data/miniconda3/envs/dexteleop
```

先确认 `which python` 指向该环境，再在同一终端重试。若仍报错，检查动态库解析与
环境内 `libstdc++` 版本；不要复用旧 `anydex` 的激活脚本或库路径。

#### 1.13.3 `_rokae_cpp` 未找到

```bash
cd /data/Projects/DexRetarget
ROKAE_PYTHON=/data/miniconda3/envs/dexteleop/bin/python bash rokae/build.sh
python -c 'import rokae; print(rokae.module_file())'
```

#### 1.13.4 机器人可以 ping，但 SDK 连接失败

检查机器人 IP、Orin 源地址、控制器是否已有其他客户端、机器人型号、自动模式和控制柜
许可。本项目对应 xCore 2.3.2 + SDK C++ 0.3.4，不要加载 Python SDK 0.7.1。

#### 1.13.5 MoveL 报 reference/work object 不对齐

这是安全拒绝。`endInRef` 当前不是 base-relative TCP。检查 Robot Assist 中 TCP、工具、
工件和 SDK toolset，使外部 reference 与 base 对齐后再测试。

#### 1.13.6 启动后机械臂方向不对

立即 disable/急停。先用 2 mm 小 MoveL 验证 base X/Y/Z，再用 Mock 和极小
`translation_scale` 验证 `quest_to_robot_rotation`，最后检查 `T_world_base`。禁止同时
在多个位置手工翻转 Y/Z。

#### 1.13.7 `watchdog_state=safe_stop`

说明超过 `stop_timeout` 没有收到合法 wrist target，或者 C++ callback 已请求停止。
检查 Quest 网络和 `callback_error`，然后完整退出并重启；safe-stop 后不要尝试在原实时
会话中直接恢复。

#### 1.13.8 输入 `MOVE` 后机械臂不动或终端像“卡住”

新版驱动会打印每个阶段：

```text
[ROKAE] requesting automatic operate mode ...
[ROKAE] requesting motor power-on ...        # 电源关闭时自动执行
[ROKAE] power=on; selecting motion control mode ...
[ROKAE] NRT online speed scale=1
[ROKAE] clearing NRT motion buffer ...
[ROKAE] appending MoveAbsJ ...
[ROKAE] starting trajectory id=... ...
[ROKAE] MoveAbsJ waiting: state=... power=... \
        max_joint_error=J1:... rad actual=... target=... q=[...]
```

上电不再只检查一次：默认会请求上电并等待最多 10 秒确认 `power=on`。轨迹开始后
每秒打印状态，同时查询官方 `moveExecution` 事件。控制器返回的规划/执行错误会立即
显示；命令 5 秒内没有进入 moving 状态会 stop/reset 并打印最近控制器 warning/error，
不再无输出地等待完整 60 秒。若运动中发生掉电，异常也会附带最近控制器日志，用于
区分碰撞/力矩保护、急停、安全回路、驱动故障与普通轨迹问题；不要自动重新上电重试。

只要 `state=moving` 且同一关节的 error 持续减小，机器人就在运动。`MoveAbsJ` 的
整数 speed 还会乘以 xCoreSDK 的 NRT 在线倍率。旧代码没有显式设置后者，若之前的
SDK 客户端留下了很低的倍率，就会出现“speed 不低但 60 秒仍未到位”。现在每次 NRT
运动前都会按 YAML 的 `online_speed_scale` 写入并打印确认。普通独立关节工具默认
使用 `non_realtime.online_speed_scale=1.0` 和 speed `300`；`05_move_to_init.py` 与
遥操作启动 init 使用独立倍率 `0.20`；当前 YAML 对应 SDK speed `250`，以启动日志为准。不要放宽到位误差或盲目
扩大目标角度来掩盖慢速问题，应先看日志中的实际关节误差是否以合理速度下降。

`MoveAbsJ` 只用于连接检查和移动到初始化构型。正式 Quest 遥操作使用所选路线的 C++ 1 kHz
实时回调，不使用上述 NRT speed；跟手性由 `safety.max_translation_speed`、
`max_angular_speed`、加速度/jerk、滤波和网络延迟共同决定。遥操作不应设成机械臂
物理极限速度：正确目标是始终跟踪“最新目标”，同时保留速度、加速度、工作区和
watchdog 限制。

默认命令会由 SDK 自主上电。例如：

```bash
python rokae/examples/02_move_joint.py \
  --joints J1 J2 J3 J4 J5 J6 J7 \
  --speed 300 --max-joint-delta 0.05 \
  --execute
```

只有现场明确要求人工上电时才添加 `--no-power-on`。如果自动上电超时，检查急停、
安全门、控制模式、报警和外接使能条件；程序不能绕过控制柜安全链。

如果停在某个 `[ROKAE]` 阶段，把从该阶段到异常结尾的完整输出保留下来。常见外部原因
包括示教器/外接使能开关未允许上电、非自动模式、安全门或急停、未复位报警、其他 SDK
客户端占用，以及控制器拒绝目标规划。

#### 1.13.9 `Initial Quest target was rejected as stale or out of order`

这个错误表示 Quest 数据流存在，但程序曾把“进入 ROKAE realtime 模式之前收到的帧”
作为第一个实时目标。控制器切换模式和启动状态接收可能超过
`safety.max_source_age`（默认 0.20 秒），因此该帧会被安全层判为过期；它不是机器人
断网或 Quest 未连接。

当前程序已经改为以下安全启动顺序：

```text
确认 Quest 数据流存在
→ 启动 ROKAE realtime
→ 立即 HOLD 当前机械臂姿态
→ 丢弃启动前的 wrist 帧
→ 等待 realtime 启动后到达的新 wrist 帧
→ 用当前 command pose 建立相对遥操作参考
→ 默认继续 HOLD，不向 C++ 发布目标
→ 按下 e 后重新建立参考并发布零跳变首目标
```

正常日志应依次出现：

```text
Waiting for a fresh Quest wrist before starting ROKAE realtime mode...
ROKAE realtime mode is HOLDING; waiting for a post-start fresh Quest wrist...
ROKAE realtime mode remains HOLDING; arm is disabled until keyboard e.
```

第二行之后保持手在 Quest 视野内，程序会自动继续，不需要重启 HTS。如果超过
`--quest-reference-timeout`（默认 15 秒）仍没有启动后的新帧，检查 HTS 是否仍在持续
发送，而不是只看 App 是否显示 `stream started`。如果首帧仍被拒绝，新版异常会附带
`rejected_source_age`、`rejected_source_time`、`rejected_sequence` 和
`rejected_target_delta` 等 diagnostics；保留完整 diagnostics 再定位，不要直接放宽
`max_source_age` 来绕过时序问题。

#### 1.13.10 `rejected_target_delta=1`，尚未按 `e` 就退出

这表示首目标被判定为相对当前 C++ command pose 跳变过大。旧实现存在两个问题：默认
DISABLED 时仍发布首目标，而 C++ 对两个临时 Eigen 矩阵取 block 后保留了悬空表达式，
极少数情况下会把完全相同的位姿误算成大跳变。

当前实现已修复矩阵生命周期，并严格区分 HOLD 与发布：默认启动完成后只更新 Quest
参考，不调用 `set_target_pose()`，所以不会解除 C++ HOLD；只有按下 `e` 后才用
当时的 Quest wrist 和当前 command pose 重新定基准并发布目标。不要通过增大
`max_target_translation_delta` 或 `max_target_rotation_delta` 掩盖这个问题。

#### 1.13.11 机械臂 S 型摆动、静止漂移或明显嗡鸣

明显摆动和大幅嗡鸣不是正常跟手。立即 `Ctrl+C`/disable，必要时按急停；不要靠提高
速度、滤波截止频率或映射比例尝试“冲过去”。旧版 C++ 轨迹生成器只有速度、加速度和
jerk 限幅，没有制动项。固定 5 cm 目标的离线复现会在目标两侧持续越过，形成极限环。
当前版本已换成阻尼比 1.25 的过阻尼位置伺服，并增加固定目标 3 秒无极限环测试。

HTS 官方说明 wrist 是 Unity world-space 位姿，因此理想情况下只移动头盔、不移动空间
中的手，wrist world pose 不应随头盔作同幅运动；但 Quest inside-out 定位、遮挡和
重新定位仍会带来毫米级/角度噪声。先在不连接机器人的情况下测两次，每次保持手不动：

```bash
# 第一次头也不动；第二次只缓慢移动头，手保持在空间中不动
python example/check_quest_wrist_stability.py \
  --protocol udp --port 9000 --hand right --duration 10
```

如果第二次的 `position_drift_first_last_mm` 或旋转明显随头运动增大，问题来自 Quest/HTS
跟踪，不应由机器人增益补偿。改善照明和手部可见性，避免 tracking origin 在运行中重置；
无线 UDP 在部分 Wi-Fi 上会 batching，HTS 官方建议优先试无线 TCP，USB TCP 最稳定。
[HTS 数据和网络说明](https://github.com/wengmister/hand-tracking-streamer/blob/main/CONNECTIONS.md)

然后断开 Quest 对控制的影响，只运行 2 mm、0.05 Hz 的固定正弦测试：

```bash
python rokae/examples/04_realtime_cartesian_test.py \
  --axis x --amplitude 0.002 --frequency 0.05 \
  --duration 5 --update-hz 100 --execute
```

- 若该测试也摆动/嗡鸣：优先检查 ROKAE 实时链路、控制器参数、工具负载和内核，不是
  Quest 映射问题；
- 若该测试安静而 Quest 模式异常：查看 `ARM_TARGET` 中的 `vr_delta`。静止时它持续
  变化说明 HTS 跟踪漂移；`tracking_error` 很大则说明机器人没有跟上平滑命令；
- realtime 启动现在会打印 `active load: mass=... cog=... inertia=...`。若机械臂实际
  安装了 Inspire/转接件而这里仍是零或明显错误，先在 Robot Assist 做负载辨识并填写
  正确质量、质心和惯量。错误负载会破坏重力补偿和碰撞检测；不要猜数值；
- 历史 Orin 部署机检测到的是 `5.10.192-tegra ... PREEMPT`，不是 `PREEMPT_RT`。ROKAE 要求
  realtime callback 按 1 ms 输出，计算较重或调度不稳定时建议实时内核与低抖动直连
  网卡。[ROKAE 实时模式说明](https://docs.rokae.com/docs/SDK/quick_start/)

本工程现使用两级平滑：C++ 5 Hz 目标低通/过阻尼限速器，以及 ROKAE
`setFilterLimit` + `setFilterFrequency` 30 Hz。官方给出的控制器截止频率推荐范围是
10–100 Hz，且要求 callback 以 1 ms 规划并返回笛卡尔位置。
[ROKAE C++ 实时接口说明](https://docs.rokae.com/docs/SDK--0-7-x-AR/cpp/cpp_method/)

### 1.14 每次现场启动的最短流程

```bash
# 终端 1：环境、机器人网络、只读检查
cd /data/Projects/DexRetarget
conda activate /data/miniconda3/envs/dexteleop
sudo ip addr flush dev eth0
sudo ip addr add 192.168.0.100/24 dev eth0
sudo ip link set eth0 up
ping -c 3 192.168.0.160
python rokae/examples/01_read_state.py

# Quest：连接与 Orin 相同 Wi-Fi，HTS 填 wlan0_IP:9000、UDP、Right/Both

# 终端 1：联合遥操作
python example/teleop_arm_hand.py \
  --enable-hand --enable-arm --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --inspire-port /dev/ttyUSB0 \
  --rokae-robot-ip 192.168.0.160 \
  --rokae-local-ip 192.168.0.100

# 当前终端：程序就绪后按 e 使能臂手；p 暂停；q 退出。
```

### 1.15 仍需现场确认的参数

- `T_world_base` 的 X/Y/Z 平移；
- Robot Assist 中实际 TCP、工具和工件/参考坐标系；
- joint init 的无碰撞路径和七轴肘部构型；
- base-frame workspace 与控制柜安全区；
- Quest 到机器人 X/Y/Z 和旋转方向；
- `rt_network_tolerance`、controller filter cutoff 和现场 1 kHz 丢包率；
- 急停、碰撞、Quest 断流、网线断开和程序异常退出响应；
- Inspire 与机械臂同时运动时的自碰撞和环境碰撞风险。

只有上述项目逐项完成实机验收后，才能提高映射尺度、速度上限或使用
`--arm-start-enabled`。

<a id="data-collection"></a>

## 模块二：数据采集

本模块在模块一的联合遥操作入口中增加录制能力。先完成 Quest、机械臂、灵巧手的
独立与联合验收，再采集真机数据。启动 `--capture` 会初始化采集源，但需要手动输入
`r` 才开始写入 episode；按 `e` 使能/暂停臂手，按 `p` 明确暂停。
允许在使能后预调整，再按 `r` 从当前位置开始；不要求每段从 home 开始。

### 2.1 配置与依赖

任务配置与传感器/存储配置分开管理。直接加载示例任务：

```bash
python example/teleop_arm_hand.py --task-config config/tasks/cup.yaml
# 同一个任务仅遥操作，不启用相机录制：
python example/teleop_arm_hand.py --task-config config/tasks/cup.yaml --no-capture
```

[任务配置示例](../../config/tasks/cup.yaml) 包含 task_id、instruction、teleop、
collection.config 引用与 home；优先级为显式 CLI > 任务配置 > 程序默认值。
配置内路径相对于仓库根目录。修改任务或 home 时复制该文件并使用新的 task_id。
示例 home 复用已有正装七轴 init，并非已标定的杯子任务姿态；首次使用前核对路径。

home 可设置 `on_start`、七轴 `arm_joints`、可选六通道 `hand_angles`、有序
`joint_waypoints`、速度、超时、分段步长和回位误差。任务 home 速度默认 0.05，
与旧公共 init 配置分开。路点之间使用原有关节分段规划，不具备自动避障。
每段 metadata 保存任务配置快照与 SHA256；`--skip-init` 可跳过启动回位，
按 `r` 不检查是否在 home。

| 配置文件 | 管理内容 |
| --- | --- |
| [config/tasks/cup.yaml](../../config/tasks/cup.yaml) | 任务指令、设备默认值、collection 引用和任务 home |
| [config/robots/rokae.yaml](../../config/robots/rokae.yaml) | 机器人网络、初始化、控制路线、TCP/world 映射和运动限制 |
| [config/cameras/realsense.json](../../config/cameras/realsense.json) | 相机序列号、角色、640×480/30 FPS、曝光及可选深度 |
| [config/collection/collection.json](../../config/collection/collection.json) | 数据目录、采样频率、对齐阈值、存储格式和动作表示 |

采集配置引用相机配置，不需要重复填写相机序列号。当前绑定为：

| 角色 | 相机 | 序列号 |
| --- | --- | --- |
| `wrist` 腕部/掌侧 | D435i | `243522070004` |
| `global` 全局 | D455 | `234322305920` |

在已激活的 `dexteleop` 环境中安装/补齐依赖：

```bash
bash data/install_env.sh
# 需要 LeRobot 导出时安装导出依赖：
bash data/install_env.sh --export
```

### 2.2 采集前检查

先确认相机绑定，再检查双路同时运行的帧率：

```bash
python realsense/scripts/check_cameras.py --inventory
python realsense/scripts/check_cameras.py --seconds 60
```

相机检查程序结束后再启动正式采集，避免多个进程同时占用设备。更换相机应先更新
序列号配置。相机安装、曝光与 USB 诊断详见[双相机说明](../../realsense/README.md)。

确认磁盘空间、任务描述、TCP、控制模式以及相机视野后开始采集。默认最低剩余空间
5 GB，单段最长 300 秒。`--capture` 要求同时启用臂与手；`--mock-rokae` 会把数据
标记为模拟数据，默认禁止导出到训练集。

### 2.3 启动联合遥操作与采集

终端 1：

```bash
python example/teleop_arm_hand.py \
  --task-config config/tasks/cup.yaml \
  --enable-hand --enable-arm --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --inspire-port /dev/ttyUSB0 \
  --inspire-baudrate 115200 --inspire-hand-id 1 \
  --arm-config /data/Projects/DexRetarget/config/robots/rokae.yaml \
  --rokae-robot-ip 192.168.0.160 \
  --rokae-local-ip 192.168.0.100 \
  --capture \
  --collection-config config/collection/collection.json \
  --task "pick up the cup and place it in the tray" \
  --arm-print-hz 0
```

该命令沿用模块一的初始化与控制流程，默认可能先执行 joint init。也可以使用 2.1 中的
任务配置命令，一并加载任务 home、语言指令、设备参数与 collection 配置引用。

程序就绪后，在同一终端按 `e` 使能臂手。可以先做预调整，再按 `r` 开始录制；
`r` 不会移动到 home，也不会改变遥操作使能。开始录制要求传感器就绪、臂处于 active。

### 2.4 开始、结束与连续采集

交互终端直接按键；管道/非 TTY 输入时每个命令后加回车：

| 命令 | 行为 |
| --- | --- |
| `e` / `p` | 切换使能 / 明确暂停，统一控制臂手 |
| `r` | 使用任务配置或 `--task` 中的指令开始录制 |
| `s` | 任务成功，结束并保存当前段 |
| `f` | 任务失败，结束并保留当前段；默认不导出到训练集 |
| `h` | 已暂停且录制/保存均结束后，返回任务 home；完成后仍暂停 |
| `c` | 重新建立机械臂 Quest 相对参考 |
| `d` | 将本会话上一段完整 episode 移入回收站 |
| `q` / Ctrl+C | 退出；未结束的段按失败/中断保存 |

推荐流程：`e → 预调整 → r → 示范 → s/f → p → h`。`s/f` 仅结束录制，不会自动
暂停或回 home。录制中可 `p → e` 暂停再继续；原始观察仍保存，暂停区间无动作标签，
有效片段分别导出，训练窗口不跨间隙。录制中禁止 `h`，防止复位动作混入示范。
保存和对齐在后台执行，期间仍可暂停；新的录制或 home 要等待保存完成。

`h` 是明确的运动指令，不等同于暂停。先确保已经退出接触、物体已妥善放置、整条
返回路径无障碍，再触发。回 home 时可 `p` 请求停止，或 `q` 停止并退出；取消/失败后
保持暂停并禁止使能，需成功重新执行 home 或重启。SDK 停止和运动超时依赖机器人链路，
不构成急停保证。默认 home 不修改手指值；设置 `hand_angles` 时在臂回位后发送该目标，
写入确认不代表手指已经到达。

不同任务通过不同 `--task-config` 启动；运行中不更换任务配置或语言标签。失败 episode
保留便于排查，不应使用 `s` 标记未完成任务。

### 2.5 传感器规格与同步

| 数据流 | 目标频率 | 保存规格 |
| --- | --- | --- |
| 两路 RGB | 各 30 FPS | 640×480 RGB8，JPEG 质量 95；默认不开深度 |
| 机械臂实测状态 | 50 Hz | TCP 4×4、7 轴关节角/速度和状态读取时间窗 |
| 灵巧手实测状态 | 20 Hz | 六通道 `ANGLE_ACT`，原始范围 0..1000；可选原始力寄存器及有效标记 |
| 臂手动作、Quest、控制事件 | 随原控制/输入更新 | 接受的臂目标、手下发值与匹配确认、手腕/关键点、使能/重置等 |
| 对齐训练样本 | 20 Hz | 两路图像索引、`state[12]`、`action[12]`、时差与有效性 mask |

各源保存原始时间戳与主机接收时间，停止录制后构建 20 Hz 时间网格。观察取时间点
`t` 之前最近收到的样本；动作取 `[t,t+50 ms)` 内最早接受的臂目标和最早确认的手目标。
缺失、陈旧、相机时差过大、状态读取过慢或臂手动作时差过大等情况被标为无效。

默认相机/臂/手最大帧龄为 100/100/150 ms，双相机估计采集时差上限为 40 ms。
这属于软件时间对齐，**不保证两台相机同时曝光，也不保证臂手指令原子下发**。
图像编码和磁盘写入在实时机器人回调之外进行。

实测状态和动作指令分别保存。两者标准表示均为 12 维：

```text
TCP xyz（m） + 绝对旋转向量（rad） + 六个原生手角度寄存器 / 1000
     3                   3                            6
```

TCP 使用机器人 base 坐标系；手通道依次为小指、无名指、中指、食指、拇指弯曲、
拇指旋转。π0.5 保留 32 维网络接口，适配器使用前 12 维并由 OpenPI 处理补维。
采集与推理应保持 TCP、坐标系、控制模式、动作表示和相机预处理一致。

### 2.6 保存、检查与可恢复删除

默认目录由 `output_root` 与 `dataset_name` 决定：

```text
data/raw/quest_inspire_rokae/
├── .partial-episode_.../      # 录制中或未完成写盘的异常段
├── episode_.../
│   ├── episode.h5            # streams/ 原始流，samples/ 对齐结果
│   └── metadata.json         # 任务、成功状态、配置快照、质量摘要
└── .trash/                   # 可恢复删除
```

结束时先排空写入队列、关闭文件、生成对齐数据，再将完整段原子重命名。失败任务在
正常完成写盘后也可成为完整 episode，但 `success=false`；`.partial-*` 表示尚未完成，
不能用于训练。当前实现不提供断电下 HDF5 的事务级恢复保证。

```bash
python -m data list
python -m data inspect data/raw/quest_inspire_rokae/episode_ID
python -m data validate data/raw/quest_inspire_rokae/episode_ID
# 需要重新生成派生对齐结果时：
python -m data align data/raw/quest_inspire_rokae/episode_ID

# 把 episode_ID 换成实际完整目录名；删除命令会输出回收站路径。
python -m data trash episode_ID
# 使用回收站路径最后一段目录名恢复：
python -m data restore 回收站目录名
```

删除只在配置数据集内操作，移入 `.trash` 后可恢复；不提供永久删除命令，也不会
自动删除既有训练导出。若使用自定义采集配置，管理命令应保持一致：

```bash
python -m data --config /path/to/collection.json list
```

### 2.7 导出 π0.5 微调数据

将示例 episode 路径替换为实际成功段，可一次传入多个路径：

```bash
python -m data export data/raw/quest_inspire_rokae/episode_ID \
  --repo-id local/rokae_inspire \
  --output data/lerobot/local/rokae_inspire
```

导出使用与当前 OpenPI 对接的 **LeRobot v2.1 image-backed Parquet**，保留 task、
`observation.images.global`、`observation.images.wrist`、`observation.state` 和 `action`。
仅连续有效段进入导出，默认最短 20 帧，action chunk 不跨无效间隙。
默认拒绝失败、模拟、不完整以及混合控制模式/采集配置的数据；输出目录不覆盖、不自动上传。

完整格式、OpenPI 适配与训练入口见[数据模块说明](../../data/README.md)。
采集与导出均使用当前 `dexteleop`；本机 Python 3.10 尚不满足官方 OpenPI 的 Python
≥3.11 训练依赖，本次没有运行完整微调或真机策略推理。

### 2.8 无机器人运动的采集检查与验证范围

```bash
python -m pytest tests/test_data_collection.py tests/test_openpi_data_adapter.py -q
python -m data simulate --seconds 3 --output-root /tmp/dex-data-check
# 只读取真实相机，臂、手和 Quest 仍为合成输入：
python -m data simulate --real-cameras --seconds 8 \
  --output-root data/camera_tests/collection_smoke
```

模拟段明确标记 `simulated=true`。已完成真实双相机配合模拟 Quest/串口手/mock 臂的
原入口联测：5 秒、100 个 20 Hz 时间槽全部有效，并通过官方 LeRobot 导出与回读。
这不代表真实臂手运动和串口反馈时延已经验收。完整结果与限制见
[验证记录](data_collection_validation_20261022.md)，调研依据见[数据采集设计](../../docs/data_collection_design.md)。
