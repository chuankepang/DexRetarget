# Quest 3 + Inspire + ROKAE 操作手册

所有命令在仓库根目录运行，每个终端先激活环境：

```bash
cd /data/Projects/DexRetarget
conda activate dexteleop
```

启动遥操作会按任务配置回 home；先确认运动路径、手指及线缆空间。机械臂维护时先退出遥操作，Quest 通信服务可以保持运行。

<a id="teleoperation"></a>

## 1. 无线 UDP：一个终端直接运行

电脑和 Quest 连接同一可互通 Wi-Fi。在 HTS 选择 **UDP、电脑 Wi-Fi IPv4、端口 9000、右手 wrist/landmarks**，先按 Start Streaming，再运行：

```bash
python -m dexretarget --config config/quest/udp.yaml \
  --task-config config/tasks/cup.yaml --arm-control pinocchio --no-capture
```

**无需运行通信服务，不需要 USB 或 ADB。**程序直接接收 UDP，按任务配置回 home 后按 `e` 使能。电脑未运行时，HTS 发出的数据不会被接收；重启本地程序即可继续接收新数据。

Wi-Fi 地址用 `ip -4 -brief address` 查看；不要填机械臂专用网卡地址或监听地址 `0.0.0.0`。局域网需允许设备互通和 UDP/9000。

## 2. USB 有线 TCP：通信与遥操作分开

**终端 A：**接好已授权的 Quest USB，启动并保持通信服务：

```bash
python -m dexretarget.quest --config config/quest/usb_tcp.yaml
```

程序自动建立 ADB reverse、打开 HTS。HTS 选择 **TCP Wired/ADB、127.0.0.1:8000、右手 wrist/landmarks**，按一次 Start Streaming。

**终端 B：**运行遥操作：

```bash
python -m dexretarget --config config/quest/usb_tcp.yaml \
  --task-config config/tasks/cup.yaml --arm-control pinocchio --no-capture
```

等待有效数据 → 初始化臂手并回 home → 按 `e` 使能。`q` 退出遥操作后，终端 A 保留头显连接，可反复启动终端 B；终端 A 按 `Ctrl+C` 才关闭通信，再启动需重新 Start Streaming。服务不控制机器人、不录制数据、不设置自启。

两种模式默认同时控制臂手。切换模式时先退出遥操作，离开 USB 模式时停止终端 A，再调整 HTS 配置。首次运行用小幅平移和 10–20°拧转核对工具轴与方向；详见[姿态配置](docs/rokae_orientation_mapping.md)。

### 键盘速查

交互终端直接按键，无需回车。

| 按键 | 功能 |
| --- | --- |
| `e` | 臂手统一使能 / 暂停；使能要求新鲜手部数据 |
| `p` | 暂停跟随，保留最后目标，不自动回 home |
| `c` | 重新建立机械臂相对参考 |
| `h` | 暂停且无录制/保存工作时回任务 home；完成后仍暂停 |
| `r` | 开始录制；需要以 `--capture` 启动并已使能 |
| `s` / `f` | 按成功 / 失败结束并保存当前段；**不会自动暂停跟随** |
| `d` | 将本会话上一段完整数据移入可恢复回收站 |
| `q` / `Ctrl+C` | 退出遥操作并清理资源 |
| `?` | 显示帮助和当前状态 |

`hold_disabled` 表示启动或键盘暂停。`Tracking HOLD` 表示跟踪过期，停止新增臂手目标；数据恢复后按配置重定参考并恢复。手动暂停需按 `e` 恢复。暂停保持目标不等于机械锁定。

<a id="data-collection"></a>

## 3. 遥操作并采集数据

无线 UDP 直接在一个终端运行：

```bash
python -m dexretarget --config config/quest/udp.yaml \
  --task-config config/tasks/cup.yaml --arm-control pinocchio --capture
```

USB TCP 采集先保持终端 A 的通信服务，再将上述 `--config` 换成 `config/quest/usb_tcp.yaml`。

操作顺序：**`e` 使能 → 预调整 → `r` 录制 → 示范 → `s/f` 保存 → `p` 暂停 → 等保存完成 → `h` 回位**。

- 不要求每段从 home 开始；可以先遥操作调整位置，再按 `r`。
- 相机和反馈源提前启动，`r` 才开始录制。录制或保存中不能回 home。
- 原始数据位于 `data/raw/quest_inspire_rokae/`，每段为 HDF5 + 元数据；`.partial-*` 为未完成段。
- 当前配置：D435i 腕部、D455 全局 RGB 各 640×480、30 FPS，训练对齐网格 20 Hz。

查看已保存的数据：

```bash
python -m dexretarget.dataset list
```

校验、回收/恢复与导出见[数据操作说明](data/README.md)。

<a id="impedance-first-trial"></a>

## 4. 独立阻抗测试

此测试不需要 Quest；先退出遥操作。负载、重心和刚度读取 `config/robots/rokae.yaml`，home 读取 Cup 任务配置。

```bash
# 回 home 后固定参考观察 5 秒，全程不触碰
python rokae/impedance_test/run.py --stage hold --duration 5 --execute

# 静态稳定后，轻推并松手观察回弹
python rokae/impedance_test/run.py \
  --stage hold --interaction light-push --duration 30 --execute
```

按提示输入 `y` 回车。程序先回 home，前 2 秒不触碰；轻推阶段先缓慢偏移 10–20 mm 再松手。计时结束主动停止并下电，`duration_completed` 表示正常定时结束；其他停止原因查看输出目录的 `summary.txt`。

**阻抗性能尚未完成真机验收。**正式使用前先核实负载/TCP并完成独立测试；遥操作或采集命令将 `--arm-control pinocchio` 改为 `--arm-control impedance` 即选择阻抗路线，VR 控制的是平衡位姿。参数说明见[阻抗配置](docs/rokae_impedance_configuration.md)。

<a id="rokae-recovery"></a>

## 5. 机械臂状态、回位与恢复

**读取状态和控制器日志：**

```bash
python rokae/examples/01_read_state.py --controller-logs
```

**回通用初始化位置：**先检查目标，再执行；按提示输入 `MOVE`。

```bash
python rokae/examples/05_move_to_init.py --pose teleop
python rokae/examples/05_move_to_init.py --pose teleop --execute
```

此处使用机器人配置的通用 init，**不是 Cup home**；Cup home 在主程序暂停后按 `h` 返回。奇异附近可在路径允许时通过上述关节回位改变构型，没有通用的一键脱离奇异命令。

**软限位已关闭：**先检查，再执行恢复；按提示输入 `RECOVER`。

```bash
python rokae/examples/06_recover_soft_limits.py
python rokae/examples/06_recover_soft_limits.py --execute
```

若单关节轻微越界，需要向内回退，可先检查 10°回退计划，再执行：

```bash
python rokae/examples/06_recover_soft_limits.py --clearance-deg 10
python rokae/examples/06_recover_soft_limits.py \
  --clearance-deg 10 --manual-power-source operator --execute
```

越界恢复仅支持单轴越界不超过 1°；等待 `WAITING FOR MANUAL ENABLE` 后使用现场物理使能。完成后重新检查软限位已开启。软限位开启但关节接近边界时，`05` 的 `--recover-joint-limits` 属于会暂时改变保护状态的维护流程，先阅读[恢复步骤与限制](rokae/docs/commissioning.md#已关闭软限位的程序恢复)。

## 6. 常见问题

| 现象 | 处理 |
| --- | --- |
| Quest 无数据 | UDP 检查同网段互通、Wi-Fi 地址和 UDP/9000 放行；USB 检查通信服务、`adb devices` 为 `device`、TCP Wired/8000；两者均需 Start Streaming |
| 退出遥操作后 HTS 报断开 | USB 模式确认没有同时退出终端 A；拔线或停止服务会断开。UDP 无需通信服务 |
| 接收端口被占用 | 只保留所选模式的一个服务；停止使用同一端口的输入测试或直连程序 |
| `/dev/ttyUSB0: Permission denied` | 用 `id -nG`、`ls -l /dev/ttyUSB0` 检查；已加入 dialout 但当前会话未生效时执行 `newgrp dialout`，重新激活环境 |
| `_rokae_cpp` 加载失败 | 在 dexteleop 环境执行 `bash rokae/build.sh`，详见[原生构建](rokae/docs/native_build.md) |
| 手正常但机械臂不动 | 确认未只传 `--enable-hand`，已按 `e`，腕部数据新鲜；异常时读取控制器日志 |

通信服务按 `Ctrl+C` 后会有序清理，连续按键不会打断清理。异常退出遗留的 socket 会在下次启动时检查并自动回收；运行中的服务仍会拒绝重复启动。通信服务停止后，正在运行的遥操作输入会失效；恢复服务后重新启动遥操作。

## 7. 配置入口

| 配置 | 用途 |
| --- | --- |
| [config/quest/udp.yaml](config/quest/udp.yaml) / [usb_tcp.yaml](config/quest/usb_tcp.yaml) | Quest 通信模式、监听地址和端口；UDP 直连，USB TCP 使用通信服务 |
| [config/tasks/cup.yaml](config/tasks/cup.yaml) | 任务指令、Cup home、设备默认值、采集配置引用 |
| [config/robots/rokae.yaml](config/robots/rokae.yaml) | 网络、负载/TCP、坐标映射、控制模式与限制 |
| [config/cameras/realsense.json](config/cameras/realsense.json) | 相机序列号、角色、图像规格 |
| [config/collection/collection.json](config/collection/collection.json) | 保存目录、采样频率、同步与数据格式 |

只查看机械臂生效配置，不连接硬件：

```bash
python -m dexretarget --task-config config/tasks/cup.yaml \
  --arm-control pinocchio --print-arm-config
```

Cup 任务只引用默认 `teleop.quest_config`，不重复定义协议/端口；显式 `--config` 优先。UDP 配置默认直接接收；USB TCP 配置默认读取通信服务，无服务则报错。配置启动时加载，修改后重启对应程序。更多内容：[配置说明](docs/configuration.md) · [架构](docs/architecture.md)。
