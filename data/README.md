# Quest3 臂手系统的数据采集与处理

项目总导航在[根目录 README](../README.zh.md)，职责划分在[架构说明](../docs/architecture.md)。
本目录实现采集、原始存储、对齐、校验、回收与 LeRobot 导出；研究依据见
[存储与同步设计](../docs/data_collection_design.md)，实测与限制见
[验证报告](../docs/refactor_report.md)。

## 配置与数据规格

主配置：[config/collection.json](../config/collection/collection.json)。它引用
[相机配置](../config/cameras/realsense.json)，不维护第二份序列号/分辨率/曝光参数。
每段保存配置快照、字段 schema 和 SHA256。类型、维度和单位在
[collection/config.py](../dexretarget/dataset/config.py) 定义，修改表示方式应升级 schema。

| 流 | 目标频率 | 数据规格 |
| --- | --- | --- |
| `camera_wrist` / `camera_global` | 各 30 FPS | 640×480 RGB8，JPEG 质量 95；帧号、设备时间、接收时间、估计采集时间、时钟域、内参、固件 |
| 可选深度 | 随 frameset | Z16→16 位 PNG，独立保存深度帧号/设备时间/时钟域；默认关闭 |
| `arm_state` | 50 Hz | qpos[7]/qvel[7]，rad/rad·s⁻¹；实测 TCP[4,4]；SDK 读取起止时间 |
| `hand_state` | 20 Hz | 实测 ANGLE_ACT[6]，0..1000；可选 FORCE_ACT[6] 原始寄存器及有效标记；读取时间窗 |
| `arm_command` / `hand_command` | 原遥操作频率 | 请求/接受 TCP、六通道下发值及匹配写确认、各自序号和时间 |
| `quest` / `control` / `event` | 随输入/状态 | 手腕、21×3 关键点及各自接收时间、使能/重置/错误事件 |
| `samples/` | 20 Hz | state[12]、action[12]、原始流索引、有效性、时差及无效原因位 |

**state 是实测，action 是指令，绝不互相替代。**两者的 12 维表示为
`TCP xyz(米) + 旋转向量(弧度) + 六个原生手角度寄存器/1000`。
手通道顺序为小指、无名指、中指、食指、拇指弯曲、拇指旋转；沿用厂商寄存器方向，
不再翻转。统一编码/解码见 [representation.py](../dexretarget/dataset/representation.py)。
辅助关节、力和 Quest 数据留在原始层，不作为标准 π0.5 的额外传感输入。

动作是控制器接受的 TCP 参考：位置路线为 IK 前目标，阻抗路线为限幅后平衡点；
它不是 1 kHz 插值参考或未来实测值。臂手链路异步，不声称物理原子下发。
采集和推理必须保持控制模式、TCP、坐标系、滤波、限速、表示和相机预处理一致。

## 统一 Conda 环境

```bash
conda activate /data/miniconda3/envs/dexteleop
bash tools/install_collection.sh
# 需要官方 LeRobot 格式导出时：
bash tools/install_collection.sh --export
```

所有 Python 包只安装到 `dexteleop`。`--export` 使用 PyTorch 2.7.1 CUDA 12.8 构建，
以及 OpenPI 固定 revision 的 LeRobot 数据模块与 dataset I/O 依赖子集，不安装其额外机器人驱动/训练套件。
本机 Python 3.10 可采集与导出；官方 OpenPI 要求 Python ≥3.11，训练主机须满足其依赖。

## 开始与结束

先激活 `dexteleop`，在仓库根目录直接加载[任务配置](../config/tasks/cup.yaml)：

```bash
python -m dexretarget --task-config config/tasks/cup.yaml
```

任务配置管理指令、设备参数和 home，并通过 `collection.config` 引用本目录主配置。
显式 CLI 参数优先；`--no-capture` 只遥操作。home 可按任务调整；cup 使用 manipulation 姿态
任务配置内的 `home.arm_joints`，需实际验证返回路径，程序不自动避障。配置内路径相对仓库根目录，任务快照与哈希
随 episode 保存。真实机械臂需要已编译且验证过的 `_rokae_cpp`。

| 按键 | 行为 |
| --- | --- |
| `e` / `p` | 切换臂手统一使能 / 明确暂停；默认均暂停 |
| `r` | 传感器新鲜且臂 active 时，从当前位置开始录制，不要求在 home |
| `s` / `f` | 成功 / 失败结束并保存；跟随状态不变 |
| `h` | 已暂停且没有录制/保存任务时，独立回任务 home；完成后仍暂停 |
| `c` | 重新建立机械臂 Quest 相对参考 |
| `d` | 将本会话上一段完整 episode 移入回收站 |
| `q` / Ctrl+C | 退出；未结束的段按失败/中断保存 |
| `?` | 帮助与当前状态 |

交互终端单键，无需回车；管道/非 TTY 每行一个命令。统一入口只使用一个 stdin 读取器，
不再依赖 Quest 按钮或 kill 信号。推荐 `e → 预调整 → r → 示范 → s/f → p → h`。
录制中可暂停/继续，原始观察继续保存，暂停区间标无效且动作窗口不跨间隙；录制中禁止回 home。
保存与对齐在后台进行，键盘暂停仍可用。手暂停保留最后目标；复位前先处理持物和接触。
home 取消/失败后保持暂停并锁定使能，须重新成功回位或重启；`p` 可请求取消回位。
`--arm-record` 保留为调试日志。完整操作见[双模块手册](../README_QUEST3_INSPIRE_ROKAE.md)。

## 同步、保存与删除

源流异步写盘，停止后构建 20 Hz 网格：观察只取接收时间 ≤ t 的最近样本；
标签取 `[t,t+50 ms)` 中最早接受的臂目标与最早确认的手目标。缺失不填旧指令。
默认相机/臂/手最大帧龄为 100/100/150 ms，相机估计采集时间差最多 40 ms，
SDK 状态读取窗口最多 30 ms；详见配置。陈旧、禁用、重置、帧号缺口等均保留 mask。
只导出连续有效段，最短 20 帧；不把坏行两侧拼接，action chunk 不跨边界。

设备时间和主机时间分开保存；SDK global time 映射与接收时间都属于软件估计，
不能保证两台相机曝光同时发生。SDK 配对也可能复用深度帧，因此深度序号单独保留。

```text
data/raw/quest_inspire_rokae/
├── .partial-episode_.../          # 正在写入或失败；禁止训练
├── episode_.../
│   ├── episode.h5                # streams/ 原始流，samples/ 派生对齐结果
│   └── metadata.json             # task、状态、配置、控制上下文、质量摘要
└── .trash/                       # 可恢复的回收站
```

有界队列溢出/写盘错误会保持无效 `.partial-*`；结束时排空队列、关闭文件、对齐并校验，
最后同文件系统原子重命名。不宣称断电下 HDF5 已实现事务级恢复。
默认最大时长 300 秒、最低剩余磁盘 5 GB，达到限制按失败结束。

```bash
python -m dexretarget.dataset list
python -m dexretarget.dataset inspect data/raw/quest_inspire_rokae/episode_ID
python -m dexretarget.dataset validate data/raw/quest_inspire_rokae/episode_ID
python -m dexretarget.dataset align data/raw/quest_inspire_rokae/episode_ID
python -m dexretarget.dataset trash episode_ID
python -m dexretarget.dataset restore 回收站目录名
```

`align` 只重建 `samples/`；回收只接受配置数据集内的完整 episode ID，不接受任意路径。
没有永久删除命令，也不自动删除已有训练导出。生成目录已忽略 Git，QA/代码保留。

## 导出与 π0.5 适配

```bash
python -m dexretarget.dataset export data/raw/quest_inspire_rokae/episode_ID \
  --repo-id local/rokae_inspire --output data/lerobot/local/rokae_inspire
```

可传多个 episode；拒绝混合采集配置/控制模式，默认拒绝失败、模拟和不完整段。
测试时须显式加 `--allow-simulated` / `--include-failed` 绕过相应过滤。
不覆盖现有输出、不上传 Hub；临时导出经官方 LeRobotDataset 回读后才提交。

导出为 OpenPI 对应的 **LeRobot v2.1 image-backed Parquet**，特征为
`observation.images.global`、`observation.images.wrist`、`observation.state[12]`、`action[12]` 与 task。
PNG 图像由官方库嵌入 Parquet，避免额外建立视频时基。`raw_provenance.json` 记录每段来源。

[OpenPI 适配器](../dexretarget/policy/rokae_policy.py) 在训练与推理使用同一输入契约，保留 32 维网络，
由 OpenPI 做归一化及补维；输出取前 12 维。这里使用绝对 TCP，不对旋转向量逐元素做 DeltaActions。
在满足 OpenPI 依赖的训练环境，可用[注册入口](../dexretarget/policy/run.py)启动官方脚本：

```bash
python -m dexretarget.policy.run --openpi-root /path/to/openpi \
  --repo-id local/rokae_inspire --dataset-root /path/to/data/lerobot \
  compute_norm_stats --config-name pi05_rokae
```

随后用同一入口运行官方 train/serve 脚本；不能沿用其他机器人的归一化统计。
本次不声称已经验证真实机器人上的微调效果。

## 不启动机器人的检查

```bash
python -m pytest tests/test_data_collection.py tests/test_openpi_data_adapter.py -q
python -m dexretarget.dataset simulate --seconds 3 --output-root /tmp/dex-data-check
python -m dexretarget.dataset simulate --real-cameras --seconds 8 --output-root data/camera_tests/collection_smoke
```

最后一项只读取真实双相机，臂/手/Quest 仍为合成输入，始终标记 `context.simulated=true`。
真实图像写盘测试不等于真实臂手采集验证。
