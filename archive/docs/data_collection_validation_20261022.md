> 历史记录：只对应当时的机器与版本，不代表当前验证结果。当前状态见[重构与验证报告](../../docs/refactor_report.md)。

# 数据采集验证（2026-10-22）

运行环境：全部使用 `dexteleop` Conda，版本记录在
[validated_environment.json](../../docs/validation/validated_environment.json)。

## 已通过

- 新采集、适配器及相关控制逻辑的针对性检查：22 项通过。
- 排除未编译原生 SDK 与一项原有配置不一致测试后，Python 回归：68 项通过、2 项 deselect，另有两个原生绑定相关测试文件未纳入。
- 原臂手入口集成：本地模拟 Quest UDP、虚拟 Inspire 串口、mock 臂，真实 D435i/D455；
  实际执行 `r`、`s`、`q`，5 秒保存 100 个 20 Hz 样本，100 个有效，两个相机各 152 帧（含起始历史帧）。
  实测臂状态记录 245 条、手反馈 100 条、臂目标 251 条、手目标 473 条，未发生队列溢出或相机帧号缺口。
- 同一段通过官方 LeRobot v2.1 API 导出与回读：100 帧、一个 episode，
  动作窗口为 `[20,12]`；RGB、state、action 适配器与末尾 padding 检查通过。
- Python/Bash 语法及新文档本地链接检查通过。

集成检查脚本：[tests/check_capture_entrypoint.py](../../tests/check_capture_entrypoint.py)。
原始结果在 `data/camera_tests/entrypoint_smoke/`，导出结果在
`data/camera_tests/export_smoke/local/entrypoint/`，都已忽略版本控制，且标记 `simulated=true`。

## 验证中发现并已修复

- 单条 resize 的 HDF5 写法在全遥操作记录频率下导致队列溢出，已改成批量追加。
- 录制起点早于记录器实际接入，造成首槽没有可用历史；已将起点放到接入时刻。
- mock 臂 HOLD 用发布目标实现、占用初始命令序号；已改成独立 HOLD latch。
- 阻塞 `input()` 的 daemon 终端线程可能在解释器退出时触发 stdin 锁错误；
  改为可退出的 select/os.read 处理，并等待正在结束的 episode 排空。

## 当前边界

此次没有向真实机械臂或灵巧手发送运动目标。真实串口读取协议通过含碎片、噪声和错误
checksum 的测试帧验证，真实硬件的应答时延和稳定性仍需现场验收。

本机尚无 `_rokae_cpp` 编译产物，因此原生实时核心相关测试不能运行；
真实机械臂采集必须先构建与当前 x86_64/Python 匹配的绑定。
原有 `test_init_nrt_speed_is_independent_and_conservative` 要求
`initial_joint_move.joint_speed * 1000 < 100`，而当前配置为 0.25，仍不满足。
本次没有擅自改变现有机械臂初始化速度、刚度或运动参数。

本机 `dexteleop` 为 Python 3.10，官方 OpenPI 训练环境要求 Python ≥3.11；
训练配置入口已提供，但本次没有运行完整 OpenPI 训练、模型推理或测量微调成功率。
PyTorch 安装了 CUDA 12.8 构建，但本次也没有据此声称 GPU 驱动或 5080 推理已验证。

## 键盘与任务配置更新

后续已增加任务 YAML、键盘统一臂手门控、独立 home 与后台保存。
功能、数据流 review 及最新回归/联测结果见[键盘与任务 review](keyboard_task_review_20261022.md)。
