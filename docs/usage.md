# 遥操作与数据采集

所有命令从仓库根目录运行，使用同一环境：

```bash
cd /data/Projects/DexRetarget
conda activate /data/miniconda3/envs/dexteleop
```

任务配置示例在 [config/tasks/cup.yaml](../config/tasks/cup.yaml)。该文件会按配置启动回 home，并默认启用采集源。
cup 的固定 home 以 `config/tasks/cup.yaml` 的 `home.arm_joints` 为准（当前为原腕部向前姿态的舍入值），启动回位与 `h` 使用同一目标；不是每次启动的位置快照。整条路径仍需现场确认，真机先完成[硬件验收](../rokae/docs/commissioning.md)。

<a id="teleoperation"></a>

## 模块一：遥操作

只控制，不启动相机和数据记录：

```bash
python -m dexretarget --task-config config/tasks/cup.yaml --no-capture
```

统一命令为 `python -m dexretarget ...`。启动后默认臂手暂停；`--skip-init` 可显式跳过启动回位。

| 按键 | 行为 |
| --- | --- |
| `e` | 同时使能/暂停机械臂和手跟随；使能要求 Quest 输入新鲜，并重新建立机械臂参考 |
| `p` | 明确暂停；保留最后命令目标，不自动回 home |
| `c` | 重新建立机械臂 Quest 相对参考 |
| `h` | 已暂停且无录制/保存任务时，独立执行任务 home 路径；完成后仍暂停 |
| `q` / Ctrl+C | 退出、停止控制并清理资源 |
| `?` | 帮助与会话状态 |

交互终端直接按键，无需回车；非 TTY 每行一个命令。没有 Quest 按钮/kill 信号使能接口。
跟踪过期进入 **Tracking HOLD**：停止新增臂手目标，保留使能意图；连续新鲜 0.20 s 后重新建立机械臂参考并恢复跟随，手部目标用 0.5 s 过渡。手动 `p` 仍需 `e` 恢复。保持最后目标不代表机械锁定或断电，仍可能完成先前接受的运动。

原有手/臂独立入口仍支持：

```bash
python -m dexretarget --enable-hand --inspire-port /dev/ttyUSB0
python -m dexretarget --enable-arm --mock-rokae --arm-control cartesian
```

这两项默认同样暂停，输入新鲜后按 `e`。真实臂入口需可用的原生绑定，mock 不证明真实控制链路已验收。

<a id="data-collection"></a>

## 模块二：数据采集

```bash
python -m dexretarget --task-config config/tasks/cup.yaml
```

相机和状态源先启动、预热，按 `r` 才接入 writer。摄像头不由遥操作控制循环初始化或操作。
配置优先级为显式 CLI > 任务配置 > 默认值；例如 `--collection-config /path/to/collection.json` 覆盖任务引用。

| 按键 | 行为 |
| --- | --- |
| `r` | 从当前位置开始录制，不要求回 home；语言指令来自任务配置或 `--task` |
| `s` / `f` | 成功/失败结束当前段，后台排空、对齐和保存；跟随状态不变 |
| `d` | 回收本会话上一段完整 episode；恢复使用 data CLI |

推荐顺序：`e → 预调整 → r → 示范 → s/f → p → h`。可以连续采集多段。
录制中允许 `p → e` 暂停再继续：观察流继续保存，停止新增臂手目标，暂停处切断训练窗口。
录制或保存中禁止回 home；保存中可以暂停，但必须等待保存完成后再开始下一段或回位。

回 home 前先处理持物、退出接触并确认路径。`h` 使用现有分段关节运动，不具备自动避障。
`p` 可请求取消回位，`q` 请求停止并退出；失败/中止后使能锁定，成功重新回位或重启后恢复。
可选手寄存器 home 在臂到位后发送；默认 `hand_angles: null` 保留目标。写确认不等于手已到位。
SDK stop 受设备/网络响应约束，不能替代急停。

## 数据检查与导出

原始数据默认在 `data/raw/quest_inspire_rokae/`。写盘失败/未完成段保留 `.partial-*`，禁止训练；不自动永久删除。

```bash
python -m dexretarget.dataset list
python -m dexretarget.dataset validate data/raw/quest_inspire_rokae/episode_ID
python -m dexretarget.dataset trash episode_ID
python -m dexretarget.dataset restore 回收站目录名
python -m dexretarget.dataset export data/raw/quest_inspire_rokae/episode_ID \
  --repo-id local/rokae_inspire --output data/lerobot/local/rokae_inspire
```

字段、频率、因果对齐、有效性阈值与导出限制统一见[数据模块](../data/README.md)。
不把失败/模拟段默认混入训练，不把暂停两侧的动作拼成一个窗口。

## 不驱动真实机器人检查

```bash
python -m dexretarget.dataset simulate --seconds 3 --output-root /tmp/dex-capture-check
python tests/check_capture_entrypoint.py --synthetic-cameras --arm-control pinocchio --seconds 2
python tools/diagnostics/check_cameras.py --inventory
```

前两项使用合成状态或模拟臂手；最后一项只枚举相机。
完整联测与硬件限制见[验证报告](refactor_report.md)。
