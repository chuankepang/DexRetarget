# 跟随暂停诊断与初始化/恢复提速

日期：2026-10-22。使用既有 dexteleop 环境，未安装依赖、未连接或移动机器人。

## 触发条件与修改

`anydexretarget/teleop/arm.py` 中 `hold_disabled` 来自跟随使能为 false；速度限制对目标做限幅，不会设置该状态。主循环可因手指目标数据超过 0.20 s 未处理更新、腕部超过配置间隔（默认 0.15 s）而关闭统一臂手门控；键盘暂停也走同一路径。没有现场连续日志，不能确认用户每次暂停的具体来源。

此前这些自动暂停调用 `following(False)`，输出和采集事件均被误写为 `keyboard_pause`。现显式传入 `quest_hand_stale` / `quest_wrist_timeout`，在状态输出中显示 `following_reason`，手指过期时打印年龄，腕部超时时打印阈值。输入恢复后仍需重新使能，避免自动续动。数据字段和时间戳格式不变；事件原因文本纠正。

| 路径 | 修改前 | 修改后 |
| --- | --- | --- |
| 默认 init | SDK 250 × online 0.20 | SDK 250 × online 0.50 |
| cup 启动 / h | SDK 50 × online 0.20 | SDK 150 × online 0.50 |
| 独立恢复 Jog | 固定 1% | 默认 5%，CLI/YAML 可选 1%–10% |
| 旧 init recovery NRT 设置 | SDK 20 | SDK 50；原流程未改为已验证的推荐路径 |

init 新增 `--speed` / `--online-speed-scale`，参数优先于 YAML。恢复新增 `--jog-rate`，优先于 `initial_joint_move.recovery_jog_rate`；CLI 与原生层分别验证范围及有限值，计划显示的值与下发值一致。单轴、方向、回退距离、人工确认、手动使能、停止和原限位恢复流程保持原约束。

用户自定义的 cup 关节目标、分段路径、停止/断流阈值、遥操作实时速度/加速度/jerk、关节限位及 1 kHz 回调不作改动。修正 cup 配置与使用手册中“任务 home 等于 manipulation_init”的过时文字，以实际任务 YAML 为准。

## 验证与 Review

x86_64 原生模块已用 dexteleop 编译、链接成功。针对性离线检查 94 项通过，覆盖速度参数穿过 init CLI → Python 驱动 → 模拟原生边界、Jog 配置/CLI 优先级与非法值拒绝、暂停原因传至控制门和采集事件、速度限幅仍保持 active、暂停/重建参考，以及已有安全和恢复回归。

既有 init 测试要求 SDK 设置 <100、online <=0.20，与操作者本次明确要求的提速配置不再一致。将该断言更新为有界新档位（SDK 5–500、online 0.01–0.50），并补充实际值向原生边界传递的集成检查；不是以修改断言代替运动测试。

最终执行 `/data/miniconda3/envs/dexteleop/bin/python -m pytest -q`：202 passed、9 subtests passed，耗时 7.67 s，没有失败或跳过项。`python tools/check_project.py`：187 个 Python 文件、37 个规范模块、151 个本地 Markdown 引用，errors=[]。两个运动工具的 `--help` 检查通过。

Review 检查了所有自动关闭跟随的调用点、collector 的 control/event 路径、任务 home 对 init 参数的覆盖、startup/home 共用驱动的在线倍率、默认值到 pybind 的传播及原生校验。没有在实时回调中增加逻辑。保留对临时切换状态、异常停止和数据源失联的原有保护。

## 未验证范围

没有实测提速后的启动/home 时长、轨迹、振动、跟踪误差或恢复 Jog；参数倍率不等于实际耗时倍率。无法用离线检查证明现场可安全运行，也没有确认用户偶发暂停究竟来自哪路追踪数据。新版日志将给出触发原因供进一步定位。旧 `--recover-joint-limits` 的 MoveAbsJ 关闭限位流程仍不是已验证的手动 Jog 替代方案。

SDK 的速度档位、在线倍率、Jog 单位和范围核对了本地 0.3.4 头文件及[官方方法说明](https://docs.rokae.com/docs/SDK/cpp/cpp_method/)。
