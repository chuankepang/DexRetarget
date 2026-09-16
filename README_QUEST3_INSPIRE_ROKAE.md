# Quest 3 遥操作 Inspire Hand + ROKAE 完整操作手册

本文档对应以下现场配置：

```text
计算设备：     NVIDIA Jetson AGX Orin（aarch64）
VR：           Meta Quest 3 + Hand Tracking Streamer（HTS）
灵巧手：       Inspire Hand，串口直连 Orin
机械臂：       ROKAE xMate ER Pro 7 / xMatePro3，7 轴
控制器地址：   192.168.0.160
控制器 xCore： 2.3.2
Orin 有线地址：192.168.0.100/24
机器人 SDK：   xCoreSDK-CPP 0.3.4
Quest 端口：   9000
```

ROKAE 链路不再使用 `xCoreSDK-Python`。DexRetarget 保留 Python 上层，机器人
实时控制通过 pybind11 下沉到 C++ 和 xCoreSDK-CPP 0.3.4。

> 真实机器人首次测试必须有人手持急停，清空工作区，并从只读、小关节运动、
> 小 MoveL、实时 5 mm 轨迹逐级验证。不要跳过中间步骤。

## 1. 系统架构

```text
Quest 3 HTS App
  └── Wi-Fi UDP/TCP：wrist pose + 21 hand landmarks
        ↓
Orin / DexRetarget Python
  ├── landmarks → AnyDexRetarget → Inspire 串口指令
  └── wrist SE(3)
        → 相对位姿映射
        → Quest/机器人轴映射与尺度
        → world target
        → world/base 坐标转换
        → pybind11 set_target_pose()
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
ROKAE CartesianPosition
```

Quest 常见更新率为 60/72/90 Hz；机器人回调为 1 kHz。两者完全解耦。Python
收包、手部重定向、日志或 policy 推理暂时变慢时，不会阻塞 C++ 实时回调，也不会
在恢复后补执行旧 VR 数据。

## 2. 目录和关键文件

```text
DexRetarget/
├── example/teleop_arm_hand.py          # Quest + Inspire + ROKAE 主程序
├── example/input/quest3.py             # HTS UDP/TCP 解析
├── example/output/real/
│   ├── drivers_inspire.py              # Inspire 串口输出
│   └── drivers_rokae.py                # C++ ROKAE Python 封装
├── rokae/
│   ├── include/rokae_driver.hpp
│   ├── src/rokae_driver.cpp
│   ├── src/pybind_module.cpp
│   ├── python/rokae.py
│   ├── config/rokae.yaml               # 唯一推荐的 ROKAE 配置
│   ├── examples/                       # 独立机械臂测试工具
│   ├── sdk/                            # 内置 xCoreSDK-CPP 0.3.4
│   ├── third_party/pybind11/
│   ├── CMakeLists.txt
│   └── build.sh
└── tests/
```

工程内已经包含运行需要的 SDK 文件：

```text
rokae/sdk/include/rokae/
rokae/sdk/include/Eigen/
rokae/sdk/lib/aarch64/libxCoreSDK.so.0.3.4
rokae/third_party/pybind11/include/pybind11/
```

编译和运行不依赖外部 `/SSD-512G/Project/xCoreSDK-CPP-0.3.4`。

## 3. 每个终端都要先设置环境

使用已经安装好的 `anydex` 环境，不要使用 `base`：

```bash
cd /SSD-512G/Project/DexRetarget
conda activate /SSD-512G/conda_envs/anydex
export LD_LIBRARY_PATH=/SSD-512G/conda_envs/anydex/lib
```

最后一行很重要，可避免 Pinocchio 加载系统旧版 `libstdc++.so.6` 时出现
`GLIBCXX_3.4.29 not found`。

`example/teleop_arm_hand.py` 还带有启动保护：若当前 conda 环境存在自己的
`lib/libstdc++.so.6`，但它没有位于 `LD_LIBRARY_PATH` 首位，脚本会在导入
Pinocchio 前自动用正确路径重启一次。因此忘记手动 `export` 时主遥操作也能启动；
其他独立 Python 工具仍建议按上面的环境命令运行。

程序不依赖桌面显示，Orin 使用 SSH/headless 运行即可。建议用两个 SSH 终端：第一个
运行遥操作主程序，第二个查看网络、发送 enable/recenter 信号或执行紧急诊断；不要让
SSH 连接本身充当机器人安全停止装置。

## 4. 编译并检查 C++ ROKAE 模块

```bash
cd /SSD-512G/Project/DexRetarget
bash rokae/build.sh
python -c 'import rokae; print(rokae.SDK_VERSION); print(rokae.module_file())'
```

预期包含：

```text
0.3.4
/SSD-512G/Project/DexRetarget/rokae/python/_rokae_cpp.cpython-310-aarch64-linux-gnu.so
```

检查动态库确实使用工程内部 SDK：

```bash
ldd rokae/python/_rokae_cpp*.so | grep xCoreSDK
```

预期指向：

```text
/SSD-512G/Project/DexRetarget/rokae/sdk/lib/aarch64/libxCoreSDK.so.0.3.4
```

## 5. 网络连接

### 5.1 Orin 与 ROKAE 控制柜

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

### 5.2 Quest 3 与 Orin

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

### 5.3 Inspire 串口

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

## 6. ROKAE 统一配置

配置文件：

```text
rokae/config/rokae.yaml
```

### 6.1 机器人与 init

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
    joint_position: [0.0, -0.60, 0.0, 1.40, 0.0, 0.75, 1.5707963268]
    joint_waypoints:
      - [0.0, -0.60, 0.0, 1.40, 0.0, 0.75, -1.5707963268]
      - [0.0, -0.60, 0.0, 1.40, 0.0, 0.75, 0.0]
    joint_speed: 0.05
    online_speed_scale: 0.20
    max_segment_delta: 0.30
    timeout: 60.0
    max_joint_delta: 1.60
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
- 当前正装 init 为 `[0, -0.60, 0, 1.40, 0, 0.75, +pi/2]` rad；
- 当前正装及手/TCP 安装约定下，`+TCP_Z` 指向 `+world_X`（水平前伸），
  `-TCP_Y` 指向 `-world_Z`（掌心向下）；
- 该点经 ER3 Pro 运动学模型复核：最小 Jacobian 奇异值约 `0.189`，条件数约
  `10.10`，优于上一候选点的 `0.172`、`11.16`；
- xCore 手册明确列出 ER Pro 的 J2=0、J4=0、J6=0 和腕心位于 J1 轴正上方四类
  奇异；本点 J2=`-34.4°`、J4=`80.2°`、J6=`43.0°`，TCP 前伸约 0.290 m，分别
  避开前三类和基座正上方腕心区域。[xCore 控制系统手册：ER Pro 奇异位置](https://static.rokae.com/Downloads/Manual/xCore%E6%8E%A7%E5%88%B6%E7%B3%BB%E7%BB%9F%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8CV3.2_A.pdf)
- 手册定义的精确奇异点是关节等于零；YAML 的 `0.35 rad` 是本工程额外设置的保守
  commissioning 警戒带，不是厂家给出的硬阈值。若启动时已处于警戒带内，路径只允许
  保持同一符号并严格远离零点，禁止靠近或穿越零点；
- ER3 Pro 手册规定 J1/J3/J5 为 ±170°、J2/J4/J6 为 ±120°、J7 为 ±360°；脚本还
  强制至少保留 10° 机械限位余量。[ER3 Pro 硬件安装手册](https://static.rokae.com/Downloads/Manual/xMate%20ER3%20Pro%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85%E6%89%8B%E5%86%8C.pdf)
- `enabled: true`，主程序连接机器人后、建立 Quest 参考前会先执行 MoveAbsJ；
- 从旧的 `J7≈-pi/2` 姿态进入新 init 需要约 176° 的掌面翻转。路径先保持
  `J7=-pi/2`，只把 J2/J4/J6 移到远离奇异区和限位的安全臂形；然后才让 J7 经零位
  翻到 `+pi/2`。所有粗路径再按 `max_segment_delta=0.30 rad` 插值成小关节段；
- init 使用 SDK speed `50`（低于 `100` 分档点）和独立 NRT 在线倍率 `0.20`。这些参数
  只作用初始化 `MoveAbsJ`，进入 1 kHz `RtCommand` 后不再参与，不会增加 Quest 遥操作
  延迟，也不会降低实时跟踪速度；
- 调试时可以用 `--skip-init` 临时跳过；
- 七轴机器人建议使用 joint init，确保每次肘部构型一致。
- `joint_speed` 是 init 轨迹速度档，init 下的 `online_speed_scale` 是额外乘法倍率；
  二者都不决定 1 kHz 遥操作速度。

### 6.2 正装 `T_world_base`

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

### 6.3 Quest 映射

```yaml
teleoperation:
  translation_scale: 0.60
  rotation_scale: 0.70
  quest_to_robot_rotation:
    [1.0, 0.0, 0.0,
     0.0, 1.0, 0.0,
     0.0, 0.0, 1.0]
```

- `translation_scale`：手移动 10 cm 时，机械臂目标移动 6 cm；
- `rotation_scale`：手转动 1 rad 时，目标转动 0.7 rad；
- `quest_to_robot_rotation`：Quest 右手坐标轴到机器人遥操作增量坐标轴的旋转矩阵；
- 初次实机应先减小尺度，逐轴检查 X/Y/Z 和旋转正负方向。

### 6.4 安全参数

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

## 7. 机械臂独立验收流程

以下步骤不需要 Quest，也不需要 Inspire。所有运动工具默认 dry-run。真实运动必须添加
`--execute`，之后在终端输入大写 `MOVE`。只有完成现场验收后才考虑用 `--yes` 跳过
输入确认。

### 7.1 第一步：只读机器人状态

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

### 7.2 第二步：小范围关节运动

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
joint_position: [0.0, -0.60, 0.0, 1.40, 0.0, 0.75, 1.5707963268]
```

终端提示后输入 `MOVE`。目标是绝对关节角，单位 rad。任一关节目标差超过
`max_joint_delta` 会在发送前拒绝。程序等待目标稳定；超时会 stop 并清理运动队列。

### 7.3 第三步：普通笛卡尔 MoveL

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

### 7.4 第四步：保存当前位置为候选 init

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

### 7.5 第五步：单独验证 init

先 dry-run：

```bash
python rokae/examples/05_move_to_init.py
```

确认后执行：

```bash
python rokae/examples/05_move_to_init.py --execute
```

程序执行：连接 → 读取当前 joints/TCP → 检查 ER3 Pro 手册关节范围与 10° 余量 →
检查 J2/J4/J6 官方奇异保护角 → 规划直达或受限中间点 → 对每段检查最大关节变化 →
控制器规划 MoveAbsJ → 每段等待控制器平滑停止 → 读取最终 joints/TCP → 计算并打印
`world_T_tcp`、前伸误差和掌心向下误差。旧姿态首次切换时先经过粗路径点，再按
`0.30 rad` 上限细分成多个小段；从新 init 附近启动只生成必要的小段或无需运动。
每段均由控制器完成同步加减速。

预期完成后两个姿态误差都约为 `1.2°`：

```text
hand alignment: forward(+TCP_Z vs +world_X)=... deg,
                palm_down(-TCP_Y vs -world_Z)=... deg
```

若任一误差超过 5°，脚本会警告；此时不要进入遥操作，先核对
`base_transform.pose_xyzw`、Robot Assist 当前 TCP，以及灵巧手相对法兰的安装方向。

如果当前位置离 init 很远，应规划多个安全中间点，而不是直接增大
`max_joint_delta`。

### 7.6 第六步：C++ realtime 小轨迹

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

### 7.7 关于上电

所有会产生运动的 ROKAE 程序现在默认通过 xCoreSDK 请求自动上电，不再要求先在
Robot Assist 中手动上电，也不需要额外添加 `--power-on`。只读程序
`01_read_state.py` 仍然不会上电、切换模式或发送运动指令。

如现场安全流程明确禁止 SDK 自动上电，可以给独立运动工具添加 `--no-power-on`，给
联合遥操作程序添加 `--no-rokae-power-on`。`--execute` 和人工输入 `MOVE` 的实机运动
确认仍然保留；自动上电不等于自动跳过运动确认。

## 8. 无硬件测试和仿真

编译后运行全部测试：

```bash
python -m unittest discover -s tests -v
```

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

## 9. Quest 输入和 Inspire 独立验证

### 9.1 只运行 Inspire

先启动 Orin 程序，再在 Quest HTS App 中点击 `Stream Started`：

```bash
python example/teleop_arm_hand.py \
  --enable-hand --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --inspire-port /dev/ttyUSB0 \
  --inspire-baudrate 115200 \
  --inspire-hand-id 1
```

这条命令不会连接 ROKAE。确认手指关键点和 Inspire 动作正常后再测试机械臂。

### 9.2 用 Mock ROKAE 验证 Quest wrist

Mock 不连接机械臂，但走完整相对位姿映射和安全状态机：

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

## 10. 真实 ROKAE 遥操作启动

### 10.1 启动前检查表

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

### 10.2 只遥操作 ROKAE

推荐明确写出 `--enable-arm`：

```bash
python example/teleop_arm_hand.py \
  --enable-arm --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --arm-config ../rokae/config/rokae.yaml \
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
→ 启动 C++ realtime Cartesian 并立即 HOLD
→ 丢弃模式切换期间的旧 wrist
→ 等待实时模式启动后到达的新 wrist
→ 用当前 command pose 同时建立机器人/Quest 相对参考
→ arm 保持 DISABLED/HOLD
→ 操作者手动使能
```

真实机械臂默认不会立刻跟随手移动。另开一个 SSH 终端使能：

```bash
pgrep -af teleop_arm_hand.py
kill -USR2 <PID>
```

`SIGUSR2` 用于 enable/disable。使能后的下一帧 wrist 会自动重新建立参考，避免手腕
当前位置造成跳变。

运行中重新中心化：

```bash
kill -USR1 <PID>
```

此操作保持当前机器人命令 TCP，并把当前 Quest wrist 设为新参考。

### 10.3 Inspire + ROKAE 联合遥操作

```bash
python example/teleop_arm_hand.py \
  --enable-hand --enable-arm --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --inspire-port /dev/ttyUSB0 \
  --inspire-baudrate 115200 \
  --inspire-hand-id 1 \
  --arm-config ../rokae/config/rokae.yaml \
  --rokae-robot-ip 192.168.0.160 \
  --rokae-local-ip 192.168.0.100 \
  --arm-print-hz 10
```

同样使用 `kill -USR2 <PID>` 单独使能机械臂。Inspire 手指控制仍会正常运行。

### 10.4 调试阶段直接使能

`--arm-start-enabled` 会跳过默认的人工 `SIGUSR2` 使能步骤。只应在完成全部实机验收
并确认 Quest tracking 稳定后使用，首次运行不要添加。

### 10.5 安全停止

正常退出使用 `Ctrl+C`。程序会请求 hold、停止 realtime、停止状态接收、切回非实时
模式并断开机器人。异常情况下优先按现场急停，不要依赖 SSH 或 Python 退出作为唯一
安全措施。

## 11. 相对遥操作计算方法

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

## 12. C++ 实时控制与 watchdog

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

## 13. 数据记录和 policy 接口

遥操作记录示例：

```bash
python example/teleop_arm_hand.py \
  --enable-arm --hand right \
  --quest3-protocol udp --quest3-port 9000 \
  --arm-record logs/rokae_session.jsonl
```

每条记录包含 Quest wrist、相对变换、world/base 目标、sequence、测量 TCP、watchdog
和 command age。

Policy 仍在 Python 侧以 50–250 Hz 产生目标：

```python
accepted = driver.set_target_pose(
    base_T_tcp_target,
    source_timestamp=time.monotonic(),
    sequence=sequence,
)
```

Policy 不应直接实现 1 kHz xCoreSDK 循环，也不应等待每个目标“执行完成”。C++ 层继续
负责 latest-value、滤波、运动限制和 watchdog。

## 14. 常见问题

### 14.1 `packets=0` 或一直等待 Quest reference

```bash
ip -br -4 addr show wlan0
ss -ulnp | grep ':9000'
sudo tcpdump -ni wlan0 udp port 9000
```

检查 HTS IP 是否为 Orin `wlan0` 当前地址、App 与程序的 UDP/TCP 是否一致、热点是否
隔离客户端、是否有其他进程占用 9000。手机热点地址变化后需要重新填写 App。

### 14.2 `GLIBCXX_3.4.29 not found`

```bash
conda activate /SSD-512G/conda_envs/anydex
export LD_LIBRARY_PATH=/SSD-512G/conda_envs/anydex/lib
```

然后在同一终端重新运行。

### 14.3 `_rokae_cpp` 未找到

```bash
cd /SSD-512G/Project/DexRetarget
bash rokae/build.sh
python -c 'import rokae; print(rokae.module_file())'
```

### 14.4 机器人可以 ping，但 SDK 连接失败

检查机器人 IP、Orin 源地址、控制器是否已有其他客户端、机器人型号、自动模式和控制柜
许可。本项目对应 xCore 2.3.2 + SDK C++ 0.3.4，不要加载 Python SDK 0.7.1。

### 14.5 MoveL 报 reference/work object 不对齐

这是安全拒绝。`endInRef` 当前不是 base-relative TCP。检查 Robot Assist 中 TCP、工具、
工件和 SDK toolset，使外部 reference 与 base 对齐后再测试。

### 14.6 启动后机械臂方向不对

立即 disable/急停。先用 2 mm 小 MoveL 验证 base X/Y/Z，再用 Mock 和极小
`translation_scale` 验证 `quest_to_robot_rotation`，最后检查 `T_world_base`。禁止同时
在多个位置手工翻转 Y/Z。

### 14.7 `watchdog_state=safe_stop`

说明超过 `stop_timeout` 没有收到合法 wrist target，或者 C++ callback 已请求停止。
检查 Quest 网络和 `callback_error`，然后完整退出并重启；safe-stop 后不要尝试在原实时
会话中直接恢复。

### 14.8 输入 `MOVE` 后机械臂不动或终端像“卡住”

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
遥操作启动 init 则单独使用更保守的 `0.20` 和 speed `50`。不要放宽到位误差或盲目
扩大目标角度来掩盖慢速问题，应先看日志中的实际关节误差是否以合理速度下降。

`MoveAbsJ` 只用于连接检查和移动到初始化构型。正式 Quest 遥操作使用 C++ 1 kHz
实时笛卡尔回调，不使用上述 NRT speed；跟手性由 `safety.max_translation_speed`、
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

### 14.9 `Initial Quest target was rejected as stale or out of order`

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
→ 收到 SIGUSR2 后重新建立参考并发布零跳变首目标
```

正常日志应依次出现：

```text
Waiting for a fresh Quest wrist before starting ROKAE realtime mode...
ROKAE realtime mode is HOLDING; waiting for a post-start fresh Quest wrist...
ROKAE realtime mode remains HOLDING; arm is disabled until SIGUSR2.
```

第二行之后保持手在 Quest 视野内，程序会自动继续，不需要重启 HTS。如果超过
`--quest-reference-timeout`（默认 15 秒）仍没有启动后的新帧，检查 HTS 是否仍在持续
发送，而不是只看 App 是否显示 `stream started`。如果首帧仍被拒绝，新版异常会附带
`rejected_source_age`、`rejected_source_time`、`rejected_sequence` 和
`rejected_target_delta` 等 diagnostics；保留完整 diagnostics 再定位，不要直接放宽
`max_source_age` 来绕过时序问题。

### 14.10 `rejected_target_delta=1`，尚未 `SIGUSR2` 就退出

这表示首目标被判定为相对当前 C++ command pose 跳变过大。旧实现存在两个问题：默认
DISABLED 时仍发布首目标，而 C++ 对两个临时 Eigen 矩阵取 block 后保留了悬空表达式，
极少数情况下会把完全相同的位姿误算成大跳变。

当前实现已修复矩阵生命周期，并严格区分 HOLD 与发布：默认启动完成后只更新 Quest
参考，不调用 `set_target_pose()`，所以不会解除 C++ HOLD；只有收到 `SIGUSR2` 后才用
当时的 Quest wrist 和当前 command pose 重新定基准并发布目标。不要通过增大
`max_target_translation_delta` 或 `max_target_rotation_delta` 掩盖这个问题。

### 14.11 机械臂 S 型摆动、静止漂移或明显嗡鸣

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
- 当前 Orin 检测到的是 `5.10.192-tegra ... PREEMPT`，不是 `PREEMPT_RT`。ROKAE 要求
  realtime callback 按 1 ms 输出，计算较重或调度不稳定时建议实时内核与低抖动直连
  网卡。[ROKAE 实时模式说明](https://docs.rokae.com/docs/SDK/quick_start/)

本工程现使用两级平滑：C++ 5 Hz 目标低通/过阻尼限速器，以及 ROKAE
`setFilterLimit` + `setFilterFrequency` 30 Hz。官方给出的控制器截止频率推荐范围是
10–100 Hz，且要求 callback 以 1 ms 规划并返回笛卡尔位置。
[ROKAE C++ 实时接口说明](https://docs.rokae.com/docs/SDK--0-7-x-AR/cpp/cpp_method/)

## 15. 每次现场启动的最短流程

```bash
# 终端 1：环境、机器人网络、只读检查
cd /SSD-512G/Project/DexRetarget
conda activate /SSD-512G/conda_envs/anydex
export LD_LIBRARY_PATH=/SSD-512G/conda_envs/anydex/lib
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

# 终端 2：确认程序已建立参考并保持后，使能机械臂
pgrep -af teleop_arm_hand.py
kill -USR2 <PID>
```

## 16. 仍需现场确认的参数

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
