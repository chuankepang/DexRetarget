# Quest 3 遥操作双 KUKA iiwa + Robotiq 3F

> **范围说明：** 本文件只适用于 KUKA/Robotiq 的自定义 TCP + UDP bridge 链路。
> ROKAE/Inspire 使用内置 xCoreSDK-CPP 0.3.4/pybind11 和 `example/teleop_arm_hand.py`，请只看
> `README_QUEST3_INSPIRE_ROKAE.md`。两套 YAML、端口、driver 和停止协议不能交叉
> 使用；KUKA 的 `CLEANUP# / GETSTOP# / quit#` 也不是 ROKAE xCoreSDK 指令。

本文说明 Quest 3 上的 Hand Tracking Streamer
（HTS）采集双手，Orin 接收 UDP/TCP，生成左右 KUKA iiwa 的笛卡尔 TCP
目标及左右 Robotiq 3F 的 `0/1` 开合指令，再通过 UDP 单播发送给运行机器人
底层控制程序的小伙伴主机。

> `teleop_dual_iiwa_robotiq.py` 不直接驱动真机；新增的
> `kuka_iwaa/cartesian_teleop_bridge.py` 接收它的 UDP JSON，使用文档给出的 iiwa 14
> 空间旋量做 FK/多候选 IK，再通过 `kuka_iiwa.py` 的 SmartServo 接口控制机械臂。
> 当前配置只启用左臂 `192.168.1.101`。该实现不包含环境、自碰撞检测，首次真机
> 运行必须低速、空载，并由人员手持急停。

## 0. 当前左臂直接启动（Quest → Orin → 192.168.1.101）

系统使用两个 Orin 进程，本机 UDP `10000` 将 VR 映射与机器人实时执行解耦：

```text
Quest HTS --UDP:9000--> teleop_dual_iiwa_robotiq.py
              --UDP:10000--> cartesian_teleop_bridge.py
              --TCP:30002--> 左 KUKA 192.168.1.101
```

当前 Quest 最新目标以 `30 Hz` 发布，bridge 计算 IK 后由 `kuka_iiwa.py` 的独立
发送线程以 `100 Hz` 持续刷新最新关节目标。所有层级都只保留最新值，不会排队
重放已经过时的手部动作：

```yaml
publisher:
  command_period_s: 0.0333333333
```

可临时通过 `--command-period-s 0.05` 改成 20 Hz。执行桥 watchdog 为 `0.25 s`，
底层最新目标刷新超时为 `0.20 s`；通信中断后不再重复发送旧目标，并由 bridge
发送 `CLEANUP`。单次变化超过执行桥安全步长时仍会被拒绝。

先确认配置中的 `flange_T_tcp`。图片表 10 的零位矩阵为
`B_T_E(0)=Trans_z(1.332m)`；当前暂按该 E 与控制 TCP 相同处理，额外 TCP 偏置为
单位阵。如果 Robotiq 安装后控制柜使用了不同工具坐标，必须把法兰到实际 TCP 的
4×4 固定变换填入 `kuka_iwaa/cartesian_teleop.yaml`，并重新验证。

先运行纯运动学测试：

```bash
cd /SSD-512G/Project/DexRetarget
python kuka_iwaa/cartesian_teleop_bridge.py --self-test
```

然后开终端 A，连接左臂并监听 Orin 本机 UDP。启动时程序会比较 POE-FK 与控制柜
返回的笛卡尔位姿，超出阈值会拒绝控制：

```bash
python -u kuka_iwaa/cartesian_teleop_bridge.py \
  --config kuka_iwaa/cartesian_teleop.yaml \
  --debug
```

终端 B 启动 Quest 输入和相对增量发布器（先不要 `--start-armed`）：

```bash
python -u example/teleop_dual_iiwa_robotiq.py \
  --config example/config/iiwa_robotiq_teleop.yaml \
  --quest-host 0.0.0.0 \
  --quest-port 9000 \
  --quest-protocol udp \
  --output udp \
  --command-host 127.0.0.1 \
  --command-port 10000
```

Quest HTS 填 Orin 无线网卡的实际 IP、UDP `9000`。操作者站好、左手被稳定跟踪、
机器人周围清空后，在终端 C 使能发布器：

```bash
kill -USR2 <终端B打印的PID>
```

使能动作会同时重置手腕参考。机械臂此刻的真实 TCP 被执行桥作为零增量锚点，
之后按 `world_translation_delta_m` 和 `world_rotation_delta_vector_rad` 跟踪；不是把
过期的 YAML 起始 TCP 硬发给机器人。再次发送 `USR2` 停用，`Ctrl+C` 退出时执行桥
发送 `CLEANUP`。

控制权切换采用防跳变顺序：程序先以 Observer 身份启动状态流、读取实际关节角并
验证 FK；随后才申请 Controller，立即发送 `CLEANUP#` 清除控制柜可能保留的旧
目标，并将刚读取的实际关节角作为首个 SmartServo 保持目标。退出顺序固定为
`CLEANUP# → GETSTOP# → quit#`。

仅检查状态时使用：

```bash
python -u kuka_iwaa/kuka_iiwa.py --ip 192.168.1.101 --port 30002 --read
```

此时程序保持 Observer，不发送 `REQUEST#` 或任何二进制运动指令。

图片给出的左臂固定外参已同时写入两个 YAML：

```text
W0_T_BL = Tz(1.217) Ty(0.193) Rx(-30deg) Rz(45deg)
```

数值为：

```text
[[ 0.70710678, -0.70710678, 0.0, 0.000],
 [ 0.61237244,  0.61237244, 0.5, 0.193],
 [-0.35355339, -0.35355339, 0.86602540, 1.217],
 [ 0.0,         0.0,        0.0, 1.0]]
```

这里严格按文档中变换的书写顺序做右乘矩阵连乘。如果原作者软件对 `Rot/Trans`
采用不同的主动/被动旋转或左乘约定，现场轴向测试必须据此修正；不能跳过验证。

### 控制柜笛卡尔状态使用的特殊坐标系

实机样本表明，这套自定义 TCP 服务返回的笛卡尔状态不是原始 `B_L` 坐标，也不是
完整的 `W0` 坐标。它使用 `W0_T_BL` 的旋转方向，但原点仍在 `B_L`，即：

```text
C_T_TCP_reported = [R_W0_BL, 0; 0, 1] B_L_T_TCP
```

对于实测关节角 `[0,0,0,-1.6467,0,0,0]`：

```text
POE 的 B_L 系位置       = [550.411,   0.000, 738.141] mm
旋转到控制柜报告方向后 = [389.199, 706.127, 444.650] mm
控制柜实际返回          = [389.200, 706.000, 444.600] mm
```

误差约 `0.136 mm / 0.000051 rad`，说明 POE、关节符号和 KUKA ABC 解析均与该样本
吻合。配置中的 `controller_pose_T_base` 专门表达这个报告坐标系；完整的
`world_T_base` 仍用于 VR 世界运动到机械臂 base 的转换，两者不能混用。

## 1. 当前实现程度

| 环节 | 状态 | 说明 |
|---|---|---|
| HTS UDP/TCP 接收 | 已实现 | 双手腕 6DoF 和每手 21 个关键点；严格检查 7/63 个数值 |
| Unity 坐标转换 | 已实现 | `[右, 上, 前] → [前, 左, 上]`，位置和姿态同时转换 |
| 相对式机械臂映射 | 已实现 | 输出世界系参考增量；执行桥以使能时真实 TCP 为锚点 |
| 双 KUKA base/world 换算 | 已实现 | 每侧使用独立的 `world_T_base` |
| Robotiq `0/1` 映射 | 已实现 | 关节弯曲分数 + 双阈值滞回，`0=开、1=合` |
| 超时、使能、重定位 | 已实现 | 默认未使能；跟踪超时 `valid=false`；重定位不引起目标跳变 |
| Orin → 控制主机 UDP JSON | 已实现 | 目标 IP 可配置；同一包包含双臂和双夹爪；典型包小于以太网 MTU |
| 左 KUKA/Robotiq 真机控制 | 已接入、待现场验收 | POE IK → SmartServo；夹爪沿用 0/1 接口 |
| 左臂外参 | 已填入 | 使用图片给出的 `W0_T_BL`；右臂已禁用 |
| TCP/VR 朝向 | 待现场确认 | `flange_T_tcp` 暂为单位阵；`R_W_Q` 暂按 FLU 对齐 |
| 真机安全验收 | 待完成 | 必须先离线、空载、单臂、低速，再进入双臂测试 |

结论：左臂从 Quest 到 KUKA SmartServo 的代码链路已经接通；仍不能在未做
`flange_T_tcp`、FK/控制柜位姿以及 VR/world 三轴小步验证前宣称真机验收完成。

相关文件：

- `example/input/quest3.py`：HTS UDP/TCP 接收与 Unity → 右手 FLU 转换；
- `anydexretarget/teleop/bimanual.py`：世界坐标增量映射和夹爪二值映射；
- `example/teleop_dual_iiwa_robotiq.py`：双臂/双夹爪 JSON 发布器；
- `example/config/iiwa_robotiq_teleop.yaml`：网络、坐标、限幅及阈值；
- `tests/test_bimanual_teleop.py`、`tests/test_quest3_input.py`：硬件无关测试。

原有的 `example/teleop_arm_hand.py`（Inspire + ROKAE）没有被替换。

## 2. 最短启动流程（先照这里运行）

所有命令都在 Orin 的 SSH 终端执行。本机使用的 Conda 环境名为 `anydex`，完整
路径是 `/SSD-512G/conda_envs/anydex`（Python 3.10），不要在 `base` 中运行。先进入
工程和环境：

```bash
cd /SSD-512G/Project/DexRetarget
conda activate /SSD-512G/conda_envs/anydex
which python      # 应为 /SSD-512G/conda_envs/anydex/bin/python
python --version  # 应为 Python 3.10.x
```

配置文件是：

```text
example/config/iiwa_robotiq_teleop.yaml
```

### 2.1 只接收 Quest，并打印期望 world TCP（不会发给机器人）

先启动 Orin 程序：

```bash
python example/teleop_dual_iiwa_robotiq.py \
  --config example/config/iiwa_robotiq_teleop.yaml \
  --quest-host 0.0.0.0 \
  --quest-port 9000 \
  --quest-protocol udp \
  --output summary \
  --start-armed
```

再在 Quest HTS App 中配置并开始 streaming：

```text
Protocol: UDP
IP:       <Orin 的 wlan0 IPv4>  # 每次用 ip -br -4 addr 实测
Port:     9000
Hands:    Both
```

终端会持续出现类似：

```text
ENABLED | left:arm=active world_p=[+0.4500,+0.2000,+0.5500]
world_qxyzw=[+0.0000,+0.0000,+0.0000,+1.0000] grip=0 score=0.12(active) | ...
```

- `world_p`：期望 KUKA TCP 在共享世界系中的 `[x,y,z]`，单位米；
- `world_qxyzw`：期望姿态四元数 `[qx,qy,qz,qw]`；
- `grip=0/1`：打开/闭合；
- `--output summary` 不创建机器人指令 UDP socket，因此适合第一轮验证；
- `--start-armed` 在这里仅表示允许目标随手运动。去掉它时目标保持，状态为
  `DISARMED`。

控制该打印的 YAML 参数是：

```yaml
publisher:
  print_hz: 2.0
  print_world_tcp_target: true
```

也可临时用 `--no-print-world-tcp-target` 关闭位姿打印，或用
`--print-world-tcp-target` 覆盖 YAML 开启。

### 2.2 配置正确后，向小伙伴控制主机发 UDP，同时打印 world TCP

假设小伙伴主机在遥操作局域网中的 IP 是 `192.168.1.120`。先在小伙伴主机启动
KUKA/Robotiq 底层接收器，使其监听所有网卡的 UDP `0.0.0.0:10000`，不能只
绑定 `127.0.0.1`。在 Orin 上先确认路由：

```bash
CONTROL_PC_IP=192.168.1.120
ping -c 3 "$CONTROL_PC_IP"
ip route get "$CONTROL_PC_IP"
```

然后仍在 Orin 启动本程序：

```bash
python example/teleop_dual_iiwa_robotiq.py \
  --config example/config/iiwa_robotiq_teleop.yaml \
  --quest-host 0.0.0.0 \
  --quest-port 9000 \
  --quest-protocol udp \
  --output udp \
  --command-host "$CONTROL_PC_IP" \
  --command-port 10000
```

也可以直接在 YAML 中填写，之后启动时省略两个 `--command-*` 参数：

```yaml
publisher:
  command_host: 192.168.1.120  # 改成小伙伴主机的真实 LAN IP
  command_port: 10000
```

配置里的 `127.0.0.1` 只是安全的本机测试默认值；它只能把包发回 Orin 自己，
无法到达小伙伴主机。

程序默认 `DISARMED`，但仍会打印左右目标。双手放好并确认底层处于安全保持后，
复制启动日志里的 PID，在第二个 SSH 终端使能：

```bash
kill -USR2 <PID>
```

再执行一次 `kill -USR2 <PID>` 会停用。需要在不改变当前机器人目标的情况下重置
手腕参考位置：

```bash
kill -USR1 <PID>
```

若 YAML 里的真实外参/启动 TCP 尚未全部确认，UDP 模式会主动拒绝启动。这时先
完成第 6 节标定，不要绕过保护。

## 3. 设计依据：沿用官方 Franka 的相对式方法

参考资料：

- [HTS 官方连接说明](https://github.com/wengmister/hand-tracking-streamer/blob/main/CONNECTIONS.md)
- [HTS 官方仓库](https://github.com/wengmister/hand-tracking-streamer)
- [HTS 官方 Python SDK](https://github.com/wengmister/hand-tracking-sdk)
- [官方 Franka VR teleop](https://github.com/wengmister/franka-vr-teleop)
- [Franka 的 VR-to-robot 转换代码](https://github.com/wengmister/franka-vr-teleop/blob/main/src/franka_vr_teleop/franka_vr_teleop/vr_to_robot_converter.py)

官方 Franka 转换器采用以下核心逻辑：

1. 将 Unity `[x=右, y=上, z=前]` 转换为机器人常用的
   `[x=前, y=左, z=上]`，即 `[z, -x, y]`；
2. 收到第一帧后记录初始 VR pose；
3. 平移使用 `p_now - p_start`；
4. 空间旋转使用 `R_now * inverse(R_start)`；
5. 平滑后发送绝对机器人 pose；暂停/恢复时重新记录参考，保持机器人目标。

本实现保留上述语义，但双臂共用一个 `world`。安装差异只由
`world_T_left_base` 和 `world_T_right_base` 表达，不能靠“左右手各自镜像一下”来
猜测。夹爪不是多自由度灵巧手，所以没有套用 Inspire 的 Retargeter，而是把 21
点关节弯曲映射为 Robotiq 所需的一个二值量。

## 4. 数据流

```text
Quest 3 / HTS
  │  wrist: xyz + qxyzw；landmarks: 21×xyz
  │  无线 UDP / 无线 TCP / USB TCP
  ▼
Orin: example/input/quest3.py
  │  Unity-LH → FLU-RH；严格包长；单调时钟时间戳
  ▼
ReferenceRelativeWorldMapper
  │  Quest 相对运动 → 左/右 world_T_tcp_target
  │  inverse(world_T_base) → 左/右 base_T_tcp_target
  ▼
UDP JSON 单播（Orin → 小伙伴控制主机:10000，默认 DISARMED）
  ▼
小伙伴主机：底层双臂控制进程（另行实现）
  │  valid/watchdog → 限幅 → IK/轨迹 → KUKA
  └  gripper 0/1 的边沿变化 → Robotiq
```

## 5. 坐标关系——必须先理解再填参数

### 5.1 坐标系定义

- `Q`：`quest3.py` 转换后的 Quest tracking world，右手系，`x前/y左/z上`；
- `W`：你为双臂工作站定义的共享世界系，例如支架中心或地面固定点；
- `B_L`、`B_R`：左右 KUKA 的 base link；
- `TCP_L`、`TCP_R`：左右机械臂实际控制使用的工具中心点。

`A_T_B` 表示把 B 中坐标转换到 A 中。4×4 矩阵按行展开，平移单位米，四元数
统一使用 `[qx, qy, qz, qw]`。

### 5.2 Quest 相对运动到共享世界目标

程序分别记录每只手的参考 wrist pose `(p_Q_ref, R_Q_ref)`，并由配置提供
`R_W_Q`。启动时机器人真实 TCP 为 `W_T_TCP_start`。之后：

```text
dp_W = translation_scale * R_W_Q * (p_Q_now - p_Q_ref)

dR_Q = R_Q_now * transpose(R_Q_ref)
dR_W = R_W_Q * dR_Q * transpose(R_W_Q)

p_W_TCP_target = p_W_TCP_start + dp_W
R_W_TCP_target = scaled(dR_W) * R_W_TCP_start
```

这是**相对于程序参考时刻的空间/世界增量**，不是速度，也不是让下游逐包累加的
局部增量。低层每周期应追踪最新的绝对目标。

### 5.3 world 与两个 base

YAML 分别保存固定外参：

```text
W_T_TCP_start  = W_T_B * B_T_TCP_start
B_T_TCP_target = inverse(W_T_B) * W_T_TCP_target
```

因此操作者双手都沿世界 +X 移动时，两个 `world_T_tcp_target` 都沿同一个 +X
变化；由于左右 base 的安装位置/朝向不同，两侧的 `base_T_tcp_target` 数值可以完全
不同。这正是支架安装场景需要的关系。

## 6. 现场标定配置

编辑：

```bash
cd /SSD-512G/Project/DexRetarget
nano example/config/iiwa_robotiq_teleop.yaml
```

每侧都必须填写并确认三组参数：

1. `world_base_pose_xyzw`
   - 固定的 `W_T_B_L` / `W_T_B_R`；
   - `[x, y, z, qx, qy, qz, qw]`，米，四元数 xyzw；
   - 来自支架 CAD、手眼/外参标定或精确测量；
   - 确认后设置 `world_base_pose_configured: true`。
2. `base_tcp_reference_pose_xyzw`
   - 遥操作开始瞬间由 KUKA 状态读取的 `B_T_TCP_start`；
   - 不应长期照抄示例单位矩阵；机器人每次起始姿态变化时都要更新；
   - 确认后设置 `reference_pose_configured: true`，或通过命令行传入。
3. `quest_to_world_rotation`
   - 3×3 的 `R_W_Q`，必须正交且行列式为 `+1`，不能是反射矩阵；
   - 若操作者面向 world +X，且 world 也是 x前/y左/z上，可先用单位阵做离线轴向
     检查；否则按实际站位标定 yaw/roll/pitch；
   - 通常左右手应使用同一个矩阵。左右机械臂安装差异归入 `world_T_base`；
   - 确认后设置 `quest_to_world_rotation_configured: true`。

以下参数先保守设置：

- `translation_scale`：平移倍率，初次真机建议 `0.3`～`0.5`；
- `rotation_scale`：旋转倍率，初次建议 `0.3`～`0.5`；
- `low_pass_alpha`：越小越平滑但延迟更大；当前滤波定义中 `1.0` 等于不滤波；
- `max_translation_m`：相对启动目标的总平移上限，不是每周期步长；
- `max_rotation_rad`：相对启动目标的总角度上限；
- `tracking_timeout`：VR 数据超过此时间即 `valid=false`。

也可用命令行覆盖，以便底层在启动前读取真机姿态后传入：

```bash
python example/teleop_dual_iiwa_robotiq.py \
  --left-base-tcp-start 0.45 0.20 0.55 0 0 0 1 \
  --right-base-tcp-start 0.45 -0.20 0.55 0 0 0 1 \
  --left-world-base 0 0.40 0.75 0 0 0 1 \
  --right-world-base 0 -0.40 0.75 0 0 0 1 \
  --left-quest-to-world-rotation 1 0 0 0 1 0 0 0 1 \
  --right-quest-to-world-rotation 1 0 0 0 1 0 0 0 1 \
  --output summary
```

上面只是格式示例，**不是你的实机参数**。不要直接复制 `iwaa-pyfri` 的
`arms.yaml`，除非逐项确认其坐标系定义、单位、TCP 和四元数顺序完全相同。

为防止占位矩阵发往真机，`--output udp`/`both` 会检查六个
`*_configured` 标志；`summary` 和 `stdout` 可在未标定时用于验证。

## 7. Orin 与 Quest 网络连接

### 7.1 Orin 网络地址必须现场读取

Wi-Fi IP 通常由 DHCP/手机热点分配，会改变。2026-09-09 的一次实测为
`wlan0=10.102.178.177/24`、SSID=`Xiaomi Civi 4 Pro`，该数值只对当次连接有效。
每次实验前必须重新查看：

```bash
ip -br -4 addr
ip route
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
```

目前是 2.4 GHz Wi-Fi。能先做功能验证，但为降低抖动建议专用 5 GHz/6 GHz AP，
减少同频设备，并给 Orin 和控制主机固定 DHCP 租约。Quest 通过 Wi-Fi 连接；
Orin 和小伙伴主机最好通过网线接同一路由器/交换机。关键是 Quest 能访问 Orin，
Orin 能访问小伙伴主机。使用 HTS 广播 UDP 时，Quest 与 Orin 还应在同一广播域且
AP 未开启客户端隔离。

### 7.2 方案 A：无线 UDP（最简单）

1. Quest 连接 `DualArmPlatform_2_4`；确认不是手机热点的隔离访客网络。
2. Orin 启动接收端：

   ```bash
   cd /SSD-512G/Project/DexRetarget
   conda activate anydex
   python example/teleop_dual_iiwa_robotiq.py \
     --quest-host 0.0.0.0 \
     --quest-port 9000 \
     --quest-protocol udp \
     --output summary
   ```

3. Quest HTS App 选择 `UDP`，推荐先配置：

   ```text
   IP:   <上述命令显示的 wlan0 IPv4>
   Port: 9000
   Hands: Both / Left + Right
   ```

   这是单播，排错更明确。官方默认广播也可用：

   ```text
   IP:   255.255.255.255
   Port: 9000
   ```

`0.0.0.0` 只用于 Orin 的监听绑定，不能填到 App 作为目标；`192.168.1` 或
单独一个 `172` 也不是完整主机地址。机械臂的 `192.168.1.101` 也绝不能作为
Quest 目标。App 必须填写当次实际的 Orin `wlan0` 完整 IPv4 和端口 `9000`。

### 7.3 方案 B：无线 TCP（无线下更可靠）

官方要求主机 TCP server 先启动，然后 App 才连接：

```bash
python example/teleop_dual_iiwa_robotiq.py \
  --quest-host 0.0.0.0 \
  --quest-port 9000 \
  --quest-protocol tcp \
  --output summary
```

Quest App：

```text
Mode: Wireless TCP
IP:   192.168.1.100
Port: 9000
```

若重新启动 Orin 程序，App 可能需要停止/重新开始 streaming 来重连。

### 7.4 方案 C：USB TCP（正式低抖动实验优先）

HTS 官方文档认为 USB TCP 最稳定。用可传数据的 USB-C 线连接 Quest 和 Orin：

```bash
adb devices
adb reverse tcp:8000 tcp:8000
adb reverse --list
```

先启动 Orin server：

```bash
python example/teleop_dual_iiwa_robotiq.py \
  --quest-host 0.0.0.0 \
  --quest-port 8000 \
  --quest-protocol tcp \
  --output summary
```

然后在 Quest App 中选择 `TCP Wired`：

```text
IP:   127.0.0.1
Port: 8000
```

首次连接时需在头显中允许 USB/调试。如果 `adb devices` 显示 `unauthorized`，
戴上头显确认授权；没有设备时检查线材是否支持数据。

### 7.5 网络排错

```bash
# 是否已有程序占用 9000
ss -lunp | grep ':9000'
ss -ltnp | grep ':9000'

# UDP 包是否真正到 wlan0（需 sudo）
sudo tcpdump -ni wlan0 udp port 9000

# Ubuntu firewall（若已启用，按实际协议开放）
sudo ufw status
sudo ufw allow 9000/udp
sudo ufw allow 9000/tcp
```

不要同时启动两个监听同一端口的 teleop 程序。若 `tcpdump` 有数据而程序一直
`waiting_for_reference`，检查 HTS 是否启用了 wrist 和左右手；若完全无包，检查
Quest/Orin 子网、AP client isolation、防火墙和 App 目标地址。

当前状态行还会输出 `QuestRX` 诊断计数：

- `packets=0`：Orin 没收到 UDP，检查 App 目标 IP、Wi-Fi、端口和防火墙；
- `packets>0` 但 `left_wrist=0` 且 `rejected>0`：数据已经到达，但 HTS 标签或字段
  数量不符合当前 CSV 协议，查看同一行的 `last_rejected`；
- `right_wrist>0`、`left_wrist=0`：App 只输出/只识别到了右手；
- `left_wrist>0`、`left_landmarks=0`：手腕位姿有效，但没有开启或没有识别 21 点
  landmarks；
- 两个左手计数持续增长后，状态应从 `waiting_for_reference/waiting_for_hand` 变为
  `disarmed`；此时才进入机械臂使能步骤。

### 7.6 Orin 到小伙伴控制主机

推荐网络拓扑：

```text
                      ┌─ Wi-Fi ─ Quest 3
专用路由器/交换机 ───┼─ Ethernet/Wi-Fi ─ Orin（VR 计算）
                      └─ Ethernet ─ 控制主机（KUKA/Robotiq 底层）
                                         └─ 第二块网卡 ─ 机器人控制柜
```

控制主机如果使用双网卡，建议：

- 网卡 A 接遥操作 LAN，例如 `192.168.1.120/24`；
- 网卡 B 接 KUKA 控制柜的专用网段；
- 两个网卡不要配置成相同子网，否则系统可能把 Orin 数据走错接口；
- 底层 UDP socket 绑定 `0.0.0.0:10000` 或网卡 A 的具体 IP；
- KUKA 实时接口仍走网卡 B，不受 Quest 无线网络影响。

在小伙伴主机先临时验证是否收到 Orin JSON：

```bash
ip -br -4 addr
sudo ufw allow 10000/udp
nc -u -l 10000
```

如果该系统的 `nc` 参数不兼容，可用底层程序自己的 UDP receiver。然后在 Orin
运行 `--output udp --command-host 192.168.1.120 --command-port 10000`。收到连续
JSON 后再接机器人；不要用 `ping` 成功代替 UDP 收包验证。

Orin 与控制主机时钟不要求严格同步：底层以本机 UDP 接收时间和递增
`sequence` 做 watchdog。若需要录制多机时间对齐，可额外配置 chrony/PTP，但它
不应成为安全超时的唯一时钟来源。

## 8. 推荐的分阶段验证流程

### 阶段 0：离线测试

```bash
cd /SSD-512G/Project/DexRetarget
conda activate anydex
LD_LIBRARY_PATH="$CONDA_PREFIX/lib" python -m unittest discover -s tests -v
```

如果 Pinocchio 报 `GLIBCXX_3.4.29 not found`，务必确认运行的是 AnyDex 的
Conda Python，并让 `$CONDA_PREFIX/lib` 优先于系统旧版 `libstdc++`。不要通过
替换 `/lib/aarch64-linux-gnu/libstdc++.so.6` 来解决。

### 阶段 1：只验证 Quest 输入和手势，不发 JSON

```bash
python example/teleop_dual_iiwa_robotiq.py \
  --quest-protocol udp \
  --quest-port 9000 \
  --output summary \
  --rate-hz 60 \
  --print-hz 5
```

戴好头显并让双手保持在摄像头视野内。应看到：

- 左右 `arm=disarmed`，`world_xyz` 稳定；移动手腕时，因为未使能，目标仍保持；
- 张手时 `grip=0` 且 score 接近 0；握拳时 `grip=1` 且 score 接近 1；
- 手移出视野约 `0.25 s` 后状态不再有效；
- 没有某只手时该侧显示 `waiting_for_reference/hand`。

注意：summary 模式默认未使能，所以手腕目标不会随动。要在完全无机器人输出的
条件下验证坐标随动，可加 `--start-armed`；此模式没有 UDP sink，仍不会发给底层：

```bash
python example/teleop_dual_iiwa_robotiq.py \
  --output summary \
  --start-armed \
  --print-hz 10
```

一次只测试一个方向：向前、向左、向上分别移动 5 cm，检查 world XYZ 的符号和
量级；再分别绕 world X/Y/Z 小角度旋转。若轴不对，修改
`quest_to_world_rotation`，不要在 KUKA 侧用经验交换轴来掩盖问题。

### 阶段 2：检查完整 JSON 和录制

```bash
python example/teleop_dual_iiwa_robotiq.py \
  --output stdout \
  --start-armed \
  --rate-hz 10 \
  --record /tmp/quest_dual_test.jsonl
```

确认没有 `NaN`，矩阵最后一行为 `[0,0,0,1]`，旋转矩阵正交，序号递增。

### 阶段 3：接入底层，但真机仍保持/仿真

当前“Orin 直接连接左臂控制柜”的运行链路为：Quest UDP `:9000` → Orin
发布器 → Orin 本机 UDP `127.0.0.1:10000` → Cartesian bridge → KUKA
`192.168.1.101:30002`。

先启动桥接器：

```bash
cd /SSD-512G/Project/DexRetarget
python -u kuka_iwaa/cartesian_teleop_bridge.py \
  --config kuka_iwaa/cartesian_teleop.yaml \
  --debug
```

当前 `move_to_initial_on_start: false`，且 `initial_world_T_tcp` 已暂时注释，因此
这条命令取得左臂 Controller 权限并验证 FK 后不会执行启动移动，而是直接保持
当前位置并监听控制指令。以下初始位姿参数已保留为注释，稍后需要时可恢复：

```text
x = 400 mm, y = 500 mm, z = 1400 mm
Euler XYZ = 0 deg, 0 deg, 0 deg
```

恢复时取消 `initial_world_T_tcp` 和 `initial_ik_seed_candidates_rad` 的注释，并把
`move_to_initial_on_start` 改成 `true`。启动移动是关节空间插补，当前没有环境碰撞
规划，因此必须先确认整条运动路径无障碍。

然后在第二个 SSH 终端启动 Quest 发布器（默认 30 Hz）：

```bash
cd /SSD-512G/Project/DexRetarget
python -u example/teleop_dual_iiwa_robotiq.py \
  --quest-host 0.0.0.0 \
  --quest-port 9000 \
  --quest-protocol udp \
  --output udp \
  --command-host 127.0.0.1 \
  --command-port 10000
```

此时发布器默认仍为 `DISARMED`。这里所说的“kill 后开始遥操作”是对**发布器
PID** 执行 `kill -USR2 <PID>`，不是杀死进程，也不是给 bridge PID 发信号。
使能瞬间会以当前 Quest 手腕姿态及机械臂当前 TCP 为相对遥操作参考，因此不会因
手腕绝对位置产生跳变。

1. 小伙伴主机底层绑定 `0.0.0.0:10000`，只打印或在仿真里显示目标；
2. 填好六个标定确认项；
3. 启动底层接收器；
4. 启动发布器（默认 DISARMED）：

   ```bash
   python example/teleop_dual_iiwa_robotiq.py \
     --output udp \
     --command-host 192.168.1.120 \
     --command-port 10000
   ```

5. 底层确认 `teleop_enabled=false`、所有 `valid=false` 时只保持；
6. 手腕放在舒适参考位置后再使能。

### 阶段 4：使能、clutch/recenter 与停用

程序启动时会打印 PID。另一个 SSH 终端执行：

```bash
# 开/关遥操作。由 DISARMED → ENABLED 时自动在当前手腕位置重置参考，不跳变。
kill -USR2 <PID>

# 保持当前 world TCP 目标，同时把当前双手位置重新设为参考（clutch/recenter）。
kill -USR1 <PID>
```

`Ctrl-C` 停止发布器。**信号不是工业急停**；低层和机器人控制柜仍必须有独立
使能、硬急停、超时制动和安全速度限制。

### 阶段 5：真机顺序

1. 机器人无负载或轻质假负载，人员离开工作空间；
2. 两臂均处于可达、远离奇异位形和互碰的初始姿态；
3. 底层使用小速度、小加速度、小 jerk，并验证 watchdog；
4. 先只允许左臂，右臂保持；再只允许右臂；
5. 再测试夹爪；
6. 最后才允许双臂同时运动，并加入双臂/环境碰撞检查。

## 9. Robotiq 3F 的 `0/1` 手势映射

程序对五指的两个屈曲关节计算无尺度角度分数：直指约为 0，90° 或更弯约为
1。四个非拇指各占 `0.225`，拇指占 `0.10`。默认 Schmitt trigger：

```text
当前为开(0)，closure_score >= 0.62 → 合(1)
当前为合(1)，closure_score <= 0.38 → 开(0)
中间区间保持上一命令
```

调整 YAML：

```yaml
grippers:
  close_threshold: 0.62
  open_threshold: 0.38
```

先看 summary 中不同人的张手、自然弯曲、握拳 score，再定阈值。始终满足
`0 <= open_threshold < close_threshold <= 1`。底层建议只在 `valid=true` 且
`command` 发生边沿变化时向夹爪下发，避免重复触发；失效时保持，不要把失效误
解释为打开或闭合。当前实现已经贯通以下命令链：

```text
Quest 21 点手部姿态
  → BinaryGripperMapper（带迟滞的 0/1）
  → UDP JSON grippers.left.command
  → CartesianTeleopBridge（只在 0/1 变化时执行）
  → KukaIiwa.set_gripper_binary()
  → 控制柜夹爪二进制帧
```

根据已验证的 `kuka_iiwa(2).py`，控制柜帧头固定为 4 字节：

```text
Basic 模式：_CM 04 02 00 00 _EE#
张开：      _CM 03 1e 07 00 _EE#   # position=30
闭合：      _CM 03 64 07 00 _EE#   # position=100
```

张开和闭合不是通过不同的 motion type 区分，而是由位置 `30/100` 区分。两帧之间
默认等待 `0.15 s` 让 Basic 模式切换完成；机械臂 SmartServo 的独立 100 Hz 线程
在这段时间仍持续工作。当前协议没有返回夹爪实际位置，因此这里是完整的命令执行
环，但不是带夹爪位置反馈的闭环控制。

## 10. UDP JSON v2 契约

```json
{
  "schema": "dexretarget.dual_iiwa_robotiq.v2",
  "sequence": 12,
  "timestamp_unix_ns": 1780000000000000000,
  "teleop_enabled": true,
  "arms": {
    "left": {
      "valid": true,
      "status": "active",
      "reference_id": 2,
      "source_age_s": 0.008,
      "base_frame": "left_base",
      "tcp_frame": "left_tcp",
      "base_T_tcp_target": [16],
      "world_T_tcp_target": [16],
      "world_translation_delta_m": [0.03, -0.01, 0.02],
      "world_rotation_delta_vector_rad": [0.0, 0.1, 0.0]
    }
  },
  "grippers": {
    "left": {
      "valid": true,
      "status": "active",
      "source_age_s": 0.009,
      "command": 0,
      "closure_score": 0.12
    }
  }
}
```

`[16]` 表示按行展开的 4×4 数组，实际包中是 16 个数。右侧字段结构相同。

底层消费规则：

```python
if packet["schema"] != "dexretarget.dual_iiwa_robotiq.v2":
    hold_robot()
elif not packet["teleop_enabled"]:
    hold_robot()
else:
    for side in ("left", "right"):
        arm = packet["arms"][side]
        if arm["valid"]:
            target = np.asarray(arm["base_T_tcp_target"]).reshape(4, 4)
            submit_latest_cartesian_target(side, target)
        else:
            hold_arm(side)
```

还必须实现：

- 本地接收超过约 `0.25 s` 无新 `sequence` 时立即保持/安全停；
- 丢弃重复或倒序包，只使用最新值，不能排队补执行旧目标；
- 检查矩阵有限、刚性、可达，并做工作空间、速度、加速度、jerk 限制；
- 两臂碰撞、环境碰撞、关节限位和奇异性检查；
- 明确底层究竟接收 base 还是 world。若用 `base_T_tcp_target`，不要再乘一次
  `world_T_base`；
- 低层闭环频率可以高于 60 Hz，在相邻 VR 目标间平滑/轨迹化，不能把 60 Hz
  的绝对目标直接当关节速度。

## 11. 常见问题

### App 报没有 active 网络连接

- Quest 必须先成功连接 Wi-Fi；
- 填完整 Orin IPv4，例如当前 `192.168.1.100`，不是 `172` 或网段前缀；
- 无线 TCP/UDP 时 Quest 与 Orin 必须可互通；
- USB TCP 则用 `127.0.0.1:8000` 并先执行 `adb reverse`。

### 程序能启动但一直 waiting

- UDP：先用 `tcpdump` 看包是否到达；
- TCP：必须先启动 Orin server，再点 App streaming；
- App 选择双手输出，双手放入 Quest 摄像头视野；
- 确认输入端口没有被其他程序占用。

### 一使能目标就跳

- 当前实现由未使能转使能时会自动 recenter；
- 仍跳通常说明 YAML 的 `B_T_TCP_start` 不是当前真实 TCP；
- 检查下游是否错误地把绝对目标当增量重复累加；
- 检查矩阵的行/列展开和四元数 xyzw 顺序。

### 左右臂沿世界方向不一致

- 先比较两个 `world_T_tcp_target` 是否按预期变化；
- 若 world 正确而 base/真机不对，检查对应 `world_T_base`；
- 若 world 本身不对，检查 `quest_to_world_rotation`；
- 不要用负行列式的镜像矩阵修左右手。

### 无线 UDP 偶发一批一批到达

这是 Wi-Fi/AP 省电和 DTIM 设置下可能出现的 batching。先换无线 TCP；正式追求
稳定低抖动时优先 USB TCP。官方 Franka 作者报告其设备上 USB TCP 约 65 Hz、
Wi-Fi UDP 约 20 Hz，这只是该设备/网络的实测，不是保证值。

## 12. 最终上机检查表

- [ ] Quest 与 Orin 网络/USB TCP 连通，双手数据持续更新；
- [ ] Orin 能向小伙伴控制主机 UDP 10000 连续发送，控制主机按接收时钟做 watchdog；
- [ ] `W_T_B_L`、`W_T_B_R` 都由实际安装标定；
- [ ] 两侧启动 `B_T_TCP_start` 来自当前机器人状态；
- [ ] `R_W_Q` 完成三轴小位移与小旋转验证；
- [ ] `summary` 和 `stdout` 验证通过；
- [ ] 底层对 schema、sequence、valid、watchdog 做严格处理；
- [ ] 底层已有限位、速度/加速度/jerk、IK、碰撞与奇异性保护；
- [ ] `0/1` 极性与 Robotiq 驱动约定一致；
- [ ] 默认 DISARMED、硬急停有效、低速单臂试验通过；
- [ ] 最后才进入低速双臂协同测试。

# 杯子预抓取
关节(rad): -0.4121  1.2230 -0.3498 -1.7168  0.2731 -1.1565 -2.2999
位姿: xyz(mm)= 635.0  89.5  83.7 abc=-0.8306 -1.4815 -2.4143
