# 存储与同步设计依据（2026-10-22）

需求是为 ROKAE 七轴臂、Inspire 六通道手及两路 RealSense 图像保留可追溯的真机示范，
并导出 π0.5 微调数据；VR 主端只用于辅助追溯，自主推理不依赖 Quest 骨架。

| 作者实现/官方资料 | 观察与决定 |
| --- | --- |
| [OpenPI 转换示例](https://github.com/Physical-Intelligence/openpi/blob/main/examples/libero/convert_libero_data_to_lerobot.py) | 图像、state、action、语言 task 通过官方 LeRobot API 导出，逐 episode 保存 |
| [OpenPI 依赖](https://github.com/Physical-Intelligence/openpi/blob/main/pyproject.toml) | 固定 LeRobot revision `0cf864870cf29f4738d3ade893e6fd13fbd7cdb5`，对应 v2.1；本项目匹配该版本 |
| [LeRobot v3 说明](https://huggingface.co/docs/lerobot/lerobot-dataset-v3) | v3 将多 episode 聚合到文件；不能默认与 v2.1 loader 兼容，本次不直接采用最新布局 |
| [Diffusion Policy RealEnv](https://github.com/real-stanford/diffusion_policy/blob/main/diffusion_policy/real_world/real_env.py) | Zarr＋MP4、独立采集、时间对齐与动作执行时间分开；借鉴其时序职责，不照搬 UR 控制器 |
| [DROID 采集器](https://github.com/droid-dataset/droid/blob/main/droid/trajectory_utils/misc.py) / [writer](https://github.com/droid-dataset/droid/blob/main/droid/trajectory_utils/trajectory_writer.py) | 保留观察、动作和控制阶段时间，使用异步队列写盘 |
| [X-VLA handler](https://github.com/2toinf/X-VLA/blob/main/datasets/domain_handler/base.py) | 加载器构建未来窗口，可利用时间数组插值；不能恢复采集时没有保存的真实时间 |
| [RoboTwin 转换器](https://github.com/RoboTwin-Platform/RoboTwin/blob/main/envs/utils/pkl2hdf5.py) | episode 与结构化 HDF5 便于管理；仿真下一帧 state 作 action 不适用于真机阻抗标签 |
| [Inspire 厂商 RH56 手册](https://en.inspire-robots.com/wp-content/uploads/2023/11/INSPIRE-ROBOTS-THE-DEXTEROUS-HAND-RH56-SERIES-USER-MANUAL.pdf) | ANGLE_ACT 从 1546 开始，六个 short、0..1000；读取回复头 90 EB、请求头 EB 90，需校验包长、ID、寄存器和 checksum |

原始层选择 **逐 episode HDF5，JPEG 图像与可选 16 位 PNG 深度内嵌**：保留不规则频率、
逐帧时间与序号，单段可完整检查和回收；JPEG 降低写盘压力，数值使用无损 LZF。
相比先写固定帧率 MP4 再以帧下标推断时间，更容易追溯实际掉帧；相比 JSONL 保存全部内容，
它保留数值类型并避免将图像变成持久化文本。项目没有 ROS，因此无需为采集额外引入 ROSbag 系统。
这是针对本工程的设计选择，不是关于某一种格式普遍优于其他格式的结论。

训练层使用官方 LeRobot v2.1 的 image-backed Parquet：与 OpenPI 示例兼容，
图像由官方 API 转为 PNG 并嵌入 Parquet，不会再产生 JPEG 类的有损编码。
需要 MP4 版本时可用同 revision 官方工具转换；原始 HDF5 不覆盖。

20 Hz 是本次的工程起点：相机已实测约 30 FPS，手反馈目标 20 Hz，臂状态目标 50 Hz，
能保留一定的时间选择余量。它不是 π0.5 固定要求；真实接触任务是否足够要在动作速度、
串口延迟和机械臂读取耗时实测后判断。20 步 action horizon 对应 1 秒，不应忽略频率直接套用别人的 horizon。

对齐采用“因果观察＋下一时间槽的已接受目标”，保留臂/手命令的独立时间和序号。
没有使用未来观察，也没有把实际运动误差藏进 action。依靠软件阈值只能保证规则可审计，
不能保证跨设备曝光/采样零误差；不满足阈值的片段保留原始数据并从训练窗口中排除。

配置、字段、操作命令及实现限制见 [data/README.md](../data/README.md)。
