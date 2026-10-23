# DexRetarget：遥操作与示范数据集

Quest 3 → ROKAE 七轴臂 + Inspire 灵巧手；D435i / D455 图像与臂手反馈同步记录，用于 π0.5 微调。

```bash
conda activate dexteleop
# 遥操作与采集：启动先回任务 home，e 使能、r 录制、s/f 保存
python -m dexretarget --task-config config/tasks/cup.yaml
# 只遥操作
python -m dexretarget --task-config config/tasks/cup.yaml --no-capture
# 数据检查与处理
python -m dexretarget.dataset --help
```

机械臂维护命令集中见[软限位恢复、回初始化位置与奇异附近处理](README_QUEST3_INSPIRE_ROKAE.md#rokae-recovery)。

## 目录职责

| 目录 | 内容 |
| --- | --- |
| `config/` | 唯一运行配置目录：robots、tasks、cameras、collection、retargeting、models、tools |
| `dexretarget/app/` | 应用入口、键盘会话、模块组装和生命周期 |
| `dexretarget/teleop/` | Quest 接收、重定向、臂手控制、跟踪 HOLD、home |
| `dexretarget/camera/` | 图像采集与时间戳，独立进程 |
| `dexretarget/dataset/` | 数据记录、HDF5、同步对齐、质量检查、导出、回收站 |
| `dexretarget/policy/` | OpenPI 训练/推理表示适配 |
| `data/` | 采集数据、导出数据与数据说明 |
| `anydexretarget/` | 可复用的手部重定向和几何算法 |
| `rokae/` | 原生 SDK 桥、实时控制器、运动学和调试工具 |
| `tools/` | 诊断、仿真、维护工具 |
| `integrations/` | 其他手型/输入设备、双 IIWA、可视化标定；主链路不依赖它 |
| `docs/`、`archive/` | 当前文档；历史资料与停用配置 |

机器人唯一默认配置是 **`config/robots/rokae.yaml`**。Cup 任务的 home 明确由
`config/tasks/cup.yaml` 定义，覆盖通用启动姿态。任务中的设备与 collection 引用均相对仓库根目录。
旧 `example/` 已迁移，旧脚本路径和 `python -m data` 需要换成上面的新入口。

跟踪丢失时臂手保持最后目标，保留使能意图；连续新鲜 0.20 秒后重建机械臂参考，
手部用 0.5 秒过渡到最新目标。`p` 手动暂停后必须按 `e`；驱动异常或实时 safe_stop 仍停止运行。
跟踪 HOLD 期间数据继续记录，控制状态无效，训练片段不跨过 HOLD 拼接。
手保持最后目标不代表断电或机械锁定，仍可能完成此前已接受的运动。

[操作手册](README_QUEST3_INSPIRE_ROKAE.md) · [配置](docs/configuration.md) ·
[架构](docs/architecture.md) · [数据格式](data/README.md) · [验证与迁移](docs/layout_tracking_review.md)

[原项目与引用](docs/retargeting.en.md) · [MIT License](LICENSE)
