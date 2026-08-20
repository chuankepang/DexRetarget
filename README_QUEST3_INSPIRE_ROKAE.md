# Quest 3 遥操作 Inspire Hand 与 ROKAE

本工程在 AnyDexRetarget 原有 Inspire Hand 重定向基础上，集成了 ROKAE xCoreSDK 实时笛卡尔控制链。

```text
Quest 3 Hand Tracking
        ├── 手指关键点 → AnyDexRetarget → Inspire 串口
        └── 手腕 SE(3) → 相对位姿映射 → 安全限制 → ROKAE CartesianPosition
```

真实 ROKAE 未连接时使用 `--mock-rokae`。完整实现、SDK API、数值验证和剩余风险见 `ROKAE_TELEOP_IMPLEMENTATION.md`。

## 1. 通信方式

| 链路 | 方式 | 默认参数 |
| --- | --- | --- |
| Quest 3 → Jetson | UDP | `9000` |
| Jetson → Inspire Hand | 直接串口，不使用 ROS 2 | `/dev/ttyUSB0`，115200 baud，ID 1 |
| Jetson → ROKAE | xCoreSDK C++ bridge | 1 kHz `CartesianPosition` callback |

Quest App 当前可以设置：

```text
Protocol: UDP
Destination: 255.255.255.255
Port: 9000
```

调试完成后建议把广播地址换成 Jetson 的固定局域网 IP。Quest、Jetson 应处于同一子网，Wi-Fi AP 不应启用客户端隔离。

检查 Jetson 地址和 UDP 数据：

```bash
ip -br addr
ss -ulnp | grep ':9000'
sudo tcpdump -ni any udp port 9000
```

ROKAE 实时控制必须使用 Jetson 到控制器的独立有线网络，不应经过 Quest 使用的 Wi-Fi 链路。

## 2. 环境与编译

```bash
cd /SSD-512G/Project/AnyDexRetarget
conda activate /SSD-512G/conda_envs/anydex
export LD_LIBRARY_PATH=/SSD-512G/conda_envs/anydex/lib
```

编译 xCoreSDK 小型桥接层：

```bash
bash example/output/real/rokae_cpp/build_bridge.sh
```

桥接层默认使用：

```text
/SSD-512G/Project/rokae-cpp/xCoreSDK-CPP
```

若 SDK 位于其他位置：

```bash
ROKAE_SDK_ROOT=/path/to/xCoreSDK-CPP \
  bash example/output/real/rokae_cpp/build_bridge.sh
```

## 3. 配置

配置文件：

```text
example/config/rokae_teleop.yaml
```

主要内容：

```yaml
robot:
  type: xmate-er-pro-7
  ip: 192.168.0.160
  local_ip: 192.168.0.100
  rt_network_tolerance: 20
  base_transform:
    pose_xyzw: [0, 0, 0, 0, 0, 0, 1]

teleoperation:
  translation_scale: 0.5
  rotation_scale: 0.7
  quest_to_robot_rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]

safety:
  max_translation_speed: 0.20
  max_angular_speed: 0.80
  max_translation_delta: 0.005
  max_rotation_delta: 0.02
  low_pass_alpha: 0.25
  command_timeout: 0.20
  workspace_min: [-1.2, -1.2, -0.1]
  workspace_max: [1.2, 1.2, 1.5]
```

`base_transform.pose_xyzw` 明确定义为 `T_world_base`，即 ROKAE 基座在共享 world 中的位姿，格式为：

```text
X Y Z QX QY QZ QW
```

正装、吊装和倾斜安装全部通过它处理，不要在代码中零散修改轴符号。

## 4. 无硬件验证

运行全部数值测试：

```bash
python -m unittest discover -s tests -v
```

运行独立仿真：

```bash
python example/test_rokae_teleop_sim.py --base normal --csv /tmp/normal.csv
python example/test_rokae_teleop_sim.py --base upside-down --csv /tmp/upside.csv
python example/test_rokae_teleop_sim.py --base tilted --csv /tmp/tilted.csv
```

连接 Quest，使用 Mock ROKAE，机械臂路径独立验证：

```bash
python example/teleop_arm_hand.py \
  --enable-arm \
  --robot rokae \
  --mock-rokae \
  --quest3-port 9000 \
  --quest3-protocol udp \
  --arm-record /tmp/rokae_targets.jsonl
```

Mock + Inspire Hand：

```bash
python example/teleop_arm_hand.py \
  --enable-hand \
  --enable-arm \
  --robot rokae \
  --mock-rokae \
  --hand right \
  --quest3-port 9000 \
  --inspire-port /dev/ttyUSB0 \
  --inspire-baudrate 115200 \
  --inspire-hand-id 1
```

仅控制 Inspire，不读取 ROKAE 配置、不加载 SDK：

```bash
python example/teleop_arm_hand.py \
  --enable-hand \
  --hand right \
  --quest3-port 9000 \
  --inspire-port /dev/ttyUSB0
```

如果 `--enable-hand` 和 `--enable-arm` 都不写，为兼容之前的使用方式，默认同时启用两条路径；没有 ROKAE 时应明确加 `--mock-rokae`。

## 5. 相对位姿映射

启动时同时记录当前 Quest wrist 和当前 ROKAE TCP：

```text
Delta_T_vr = inverse(T_vr_ref) @ T_vr_current
T_world_tcp_target = T_world_tcp_ref @ map(Delta_T_vr)
T_base_tcp_target = inverse(T_world_base) @ T_world_tcp_target
```

旋转使用 SO(3) rotation vector 的 `Log/Exp` 做比例缩放，不使用 Euler angle 直接相减。对外发送的是相对操作者锚点计算得到的、ROKAE 基座坐标系下的绝对行优先 4×4 TCP 目标。

终端示例：

```text
ARM_TARGET status=active xyz=[+0.45100, ...] base_T_tcp=[16 values]
```

JSONL 记录同时包含：

- `vr_pose`
- `vr_relative`
- `world_target`
- `base_target_raw`
- `base_target_safe`
- `status`

## 6. Headless 控制

建议使用 `tmux`：

```bash
tmux new -s quest_teleop
cd /SSD-512G/Project/AnyDexRetarget
conda activate /SSD-512G/conda_envs/anydex
python example/teleop_arm_hand.py [参数]
```

重新锚定并保持目标连续：

```bash
kill -USR1 <PID>
```

切换机械臂运动使能：

```bash
kill -USR2 <PID>
```

Quest wrist 超过 `command_timeout` 未更新时，状态进入 `hold_timeout`。恢复跟踪的第一帧会自动重新锚定，避免补执行掉线期间的人手位移。

## 7. 真实 ROKAE 启动

在启动前确认：

- 机型对应 `xmate-6`、`xmate-er-pro-7` 或 `standard-6`；
- 控制器版本和 xCoreSDK 授权；
- ROKAE IP 与 Jetson 有线网卡 IP；
- `tcpPose_m` 对应的 TCP/工具配置；
- `T_world_base` 和 Quest 控制轴映射；
- 当前 TCP 位于配置的 base-frame workspace 内；
- 急停、deadman、碰撞检测和控制器安全区已验证。

保守参数示例：

```bash
python example/teleop_arm_hand.py \
  --enable-arm \
  --robot rokae \
  --rokae-robot-type xmate-er-pro-7 \
  --rokae-robot-ip 192.168.0.160 \
  --rokae-local-ip 192.168.0.100 \
  --rokae-power-on \
  --arm-start-enabled \
  --arm-translation-scale 0.2 \
  --arm-rotation-scale 0.3 \
  --arm-max-translation-speed 0.05 \
  --arm-max-angular-speed 0.20
```

真实模式会从 xCoreSDK `tcpPose_m` 读取当前 TCP 作为机械臂 reference；`--robot-initial-pose` 只用于 Mock，不会覆盖真实机器人状态。

真实机械臂默认 `hold_disabled`。只有明确加入 `--arm-start-enabled` 或运行时发送 `SIGUSR2` 才跟随 Quest。`--rokae-power-on` 也必须显式提供；若由控制柜/示教器管理上电，则不要加该参数。

## 8. 安全边界

当前已实现：

- 平移与旋转比例；
- 低通滤波；
- 最大线速度与角速度；
- 最大单帧平移与转角；
- base-frame TCP 工作空间；
- timeout hold；
- enable/disable；
- recenter；
- latest-value buffer。

当前软件限制不能替代：

- 控制器安全区；
- 碰撞检测；
- 关节限位和奇异规避；
- 环境/自碰撞检查；
- 硬件急停和现场操作规程。

第一次实机测试应低速、空载、无障碍，并由另一人守在急停旁。

## 9. 常见问题

### Inspire 正常，但没有机械臂目标

Quest App 必须同时发送 wrist pose。手指 landmarks 和 wrist 是两种数据。用 `tcpdump` 检查 UDP 9000。

### UDP 9000 被占用

```bash
ss -ulnp | grep ':9000'
```

同一时间只运行一个 Quest 接收程序。

### Inspire 串口权限不足

```bash
groups
sudo usermod -aG dialout "$USER"
```

加入用户组后重新登录 SSH。

### ROKAE bridge 找不到

```bash
bash example/output/real/rokae_cpp/build_bridge.sh
ldd example/output/real/rokae_cpp/build/libanydex_rokae_bridge.so
```

也可以通过 `ANYDEX_ROKAE_BRIDGE` 或 `--rokae-bridge` 指定桥接库。

### 当前 TCP 在 workspace 外

程序会拒绝启动 reference，避免目标瞬间吸附到边界。先读取真实 TCP，根据现场安全区域修改 `workspace_min/max`，不要简单删除检查。

### 手移动方向不正确

先使用 `--mock-rokae`，分别验证三轴平移和三轴旋转，再修改 `quest_to_robot_rotation`。矩阵必须满足 `C.T @ C = I` 且 `det(C)=+1`。

