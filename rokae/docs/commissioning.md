# ROKAE 真机验收入口

命令从仓库根目录运行，统一激活 `dexteleop`。本机 x86_64 原生绑定已构建并完成离线检查；以下硬件验收流程尚未执行，不能将导入成功当作真机运动通过。SDK/控制器配置以 [rokae.yaml](../../config/robots/rokae.yaml) 为准。

## 平台与连接

内置 xCoreSDK-CPP 0.3.4 运行库分别位于 `sdk/lib/aarch64` 和 `sdk/lib/x86_64`；CMake 根据本机架构选择，Python 扩展也必须匹配架构与 ABI。在支持的平台构建：

```bash
conda activate /data/miniconda3/envs/dexteleop
bash rokae/build.sh
python -c 'import rokae; print(rokae.SDK_VERSION); print(rokae.module_file())'
```

本机编译依赖、SDK 变体及加载检查见[原生构建与排错](native_build.md)。

控制主机机器人网卡和控制器必须在配置子网；不要把 Quest 的 Wi-Fi 地址填入机器人本机地址。
网卡名以现场为准，修改网卡配置可能中断远程连接，不在启动脚本中自动执行。
Quest HTS 单播到控制主机无线 IPv4，默认 UDP 9000；手串口默认值来自任务/CLI。

## 必须核对的坐标和配置

- 机器人型号、七轴关节数、IP/本机源地址；TCP、工具负载、质心和惯量。
- Robot Assist 中正装/baseFrame 与项目 `T_world_base` 的区别；当前配置要求控制器基座旋转接近单位阵。
- `T_world_base` 平移占位、Quest 三轴方向、映射尺度、workspace 与控制柜安全区。
- 初始化和 home 整条路径；关节分段不提供避障。
- 位置、Pinocchio 关节流与 impedance 是不同路线；采集和推理保持同一路线及限幅。

SDK 上电、模式切换、1 kHz 回调由原有驱动负责。init/home 与恢复速度现按操作者要求调整，配置说明见[速度配置](../../docs/configuration.md#初始化与恢复速度)；硬件验收仍需现场执行。

## 分级验证

先保证急停可用、工作区清空并有人监护，再依次执行：

```bash
# 只读，不上电、不运动
python rokae/examples/01_read_state.py
# 当前关节附近小运动；占位符替换为实际测量值
python rokae/examples/02_move_joint.py --joints J1 J2 J3 J4 J5 J6 J7 --speed 300 --max-joint-delta 0.05
# +X 5 mm，姿态不变
python rokae/examples/03_move_cartesian.py --relative 0.005 0 0 0 0 0 --relative-frame base --speed 10
# 检查初始化路径
python rokae/examples/05_move_to_init.py
# 小实时轨迹
python rokae/examples/04_realtime_cartesian_test.py --axis x --amplitude 0.002 --frequency 0.05 --duration 3 --update-hz 100
```

运动工具默认 dry-run；实际执行添加 `--execute` 并按提示输入 `MOVE`。
不要用 `--yes` 跳过首次现场验收。失败时保留完整日志，不放宽限位、断流阈值或到位误差来绕过问题。
验收完成后才进入[联合遥操作](../../docs/usage.md)。

独立工具的细节见[原生模块](../README.md)与[控制器说明](../control/README.md)。
完整旧版 SDK 诊断说明保存在[历史手册](../../archive/docs/quest_rokae_manual_before_refactor.md)，
其中旧路径、旧环境和当时的测试结果不能直接当作当前状态。

## 已关闭软限位的程序恢复

`controller soft limits enabled: False` 时，不需要强制依赖 Robot Assist 界面；SDK 0.3.4 本地头文件提供手动 `startJog()` 和 `setSoftLimit()`。
[官方手册](https://docs.rokae.com/docs/xCore/11-%E5%AE%89%E5%85%A8%E5%8A%9F%E8%83%BD/)说明超限关节的恢复应使用向内 Jog；关闭关节位置限制后不能运行普通程序。
此前 `05_move_to_init.py --recover-joint-limits` 使用 MoveAbsJ，且拒绝初始限位已关闭状态；本入口独立使用手动 Jog，不将该旧流程当作已验证的恢复方式。

```bash
# 只连接读取并显示计划，不改变上电、模式、限位或运动状态
python rokae/examples/06_recover_soft_limits.py

# 计划回到边界内 10°，用于避开当前 init 的 0.15 rad（约 8.6°）保护区
python rokae/examples/06_recover_soft_limits.py --clearance-deg 10

# 接有手持使能/示教器：使用现场使能，不发送 SDK 上电请求（默认 operator）
python rokae/examples/06_recover_soft_limits.py --clearance-deg 10 --manual-power-source operator --execute

# 仅适用于厂家配置确认无外接使能开关/示教器的设备：请求 SDK 手动上电
python rokae/examples/06_recover_soft_limits.py --clearance-deg 10 --manual-power-source sdk --execute

# 只读检查实际控制器报警，不改变模式、上电或限位
python rokae/examples/01_read_state.py --controller-logs
```

默认回退到原边界内 0.5°；`--clearance-deg` 显式接受 0.5–10°。只允许一个关节处于/超出边界，超出不得超过 1°；多个超限轴、非空闲状态、非法读数或确认后位置/限位变化均拒绝运动。其余关节不发运动目标。
日志中的 J6 为约 115.048°、原上界为 115°，默认计划约 114.5°；指定 10° 时约 105°，回退约 10.048°。这些只是给定日志的计算值，现场始终读取实时状态。

流程：复查确认时快照 → 停止/下电/手动 → 等待现场使能或显式 SDK 请求上电 → 有限步长单关节 Jog → 始终调用 stop → 下电/手动 → 检查全部关节在原范围内 → 只开启原限位 → 回读限位值、开关、下电及手动状态。没有关闭限位、放宽范围、清除安全报警或自动回 home 的动作。
Jog 默认从原先固定 1% 调为配置中的 5%（`robot.initial_joint_move.recovery_jog_rate: 0.05`）。可用 `--jog-rate 0.01..0.10` 覆盖；显示计划与原生实际下发使用同一值，非法值在连接前拒绝，原生入口再次验证。

**SDK 0.3.4 的 `setPowerState(true)` 明确限定：只有无外接使能开关或示教器的机器人才能手动模式上电。** 因而接有装置时，不使用 Robot Assist 界面仍可以让程序发送 Jog，但不能让软件代替现场物理使能。`operator` 不调用 SDK 上电：输入 `RECOVER` 前保持使能释放；看到 `WAITING FOR MANUAL ENABLE` 后，在 30 秒内按住三位使能中间位置并保持；看到 `Jog stopped` 后释放，程序等待下电并恢复限位。保持工作区可见，现场使能由人操作。

`sdk` 只在符合上述硬件条件时使用，禁止通过拔除/短接使能或切换自动模式来绕过条件。`上下电失败` 只是 SDK 的通用错误，不能单凭它确认外部使能、报警或安全门中的哪一个导致拒绝。失败时程序读取最近 10 条控制器日志（含时间、ID、内容和建议，可能包含历史事件），供继续定位；日志读取失败不覆盖原始错误。

控制器拒绝上电或 Jog 时会报错退出。Ctrl+C 在原生轮询中检查，随后进入停止/下电/尝试恢复流程；它不代替硬件急停。若通讯中断、关节仍超限或恢复失败，会明确报告 `RECOVERY INCOMPLETE`，并区分是否请求过 Jog、是否回读确认下电及限位恢复是否确认。程序不会在关节仍超限时强制开启限位。

恢复成功后，先只读检查，再检查所需 init 的目标与路径；`--pose manipulation` 读取本目录 YAML 的 `robot.manipulation_init`，不读取任务 YAML 的 home。
本入口已在 dexteleop 中编译并完成离线规划、CLI 门控测试；手动上电、实机 Jog、RSC 差异及异常停止/恢复尚未实机验证。测试范围与限制见[本次验证记录](../../docs/validation/rokae_soft_limit_recovery_20261022.md)。
