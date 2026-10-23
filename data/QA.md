> 本文保留调研问题与当时的判断；其中“尚未实现/下一步”描述已由文末落地说明替代。当前入口、模块边界和实际测试结果以[架构](../docs/architecture.md)、[使用指南](../docs/usage.md)及[重构验证报告](../docs/refactor_report.md)为准。

# 背景

真机实验条件： rokae er3pro机械臂、inspire hand灵巧手、5080笔记本、realsense D435i手眼相机安装在掌心侧，realsense D455相机全局视场。

任务：pi05 全参数微调 学习灵巧操作

技术路线：
1. quest3 遥操作 机械臂—灵巧手，现在dexretarget已经实现
2. 按照规范同步保存相机、本体状态、机器人动作等数据

# 问题TBD

> 调研日期：2026-10-22。以下优先依据论文、作者代码和厂商 SDK；“项目建议”表示针对本设备的设计判断，不代表已经完成真机验证。注意：当前 `config/robots/rokae.yaml` 配置为 `xmate-er-pro-7`，是 **7 轴**；笛卡尔位姿的 6 维不能理解成机械臂只有 6 个关节。

1. 修改pi05的action expert的输出维度，将平行夹爪迁移到inspire hand的6个独立自由度，我在网上小红书等论坛看到这种微调方式的合理性，并且进行全参数微调是有效的！！！这个方案你做一下调研！！！！

**解答：迁移到 Inspire 六自由度手有直接研究依据，全参数微调也是官方支持的路线；但本项目通常不需要修改 action expert 的网络维度。**

- **直接证据：**Ego-Pi 使用 π0.5，在搭载六个可控关节的 Inspire 手上完成真机任务，并用 Quest 控制器与 Manus 手套遥操作采集数据。其手部动作采用机器人自身的关节表示，说明“VLA 从夹爪迁移到灵巧手”可行。不过，它研究的是人类/机器人数据共训练，不能将其成功率当作本项目“只改输出层并全参数微调”的效果保证。论文还报告：在更高维双手设置中，直接扩大最终投影层导致训练损失升高，因而采用动作 token 交错方案。[Ego-Pi §3.2–3.3、§4](https://arxiv.org/html/2606.08107v1)
- **实际需要的动作维度：**若输出关节目标，是 `7 个臂关节 + 6 个手执行器 = 13 维`；若输出 TCP 目标，是 `xyz(3) + 旋转向量(3) + 手(6) = 12 维`，采用旋转 6D 表示则为 15 维。Inspire 的六个独立执行通道不等于 URDF 中所有连动关节都可独立控制。应固定通道顺序、开合方向和单位。
- **优先实现：**OpenPI 的 `Pi0Config.action_dim` 默认是 **32**，已有 `PadStatesAndActions`。保留 32 维预训练结构，将上述真实动作补零到 32 维，推理时取回有效维度；修改机器人输入/输出适配器、数据配置和归一化统计即可。32 个通道不是“31 个臂关节加一个夹爪”的固定语义，六个手通道的含义由新数据学习。[模型配置](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/models/pi0_config.py)、[补维与动作变换](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/transforms.py)
- **若坚持改成 12/13/15 维：**需要同时适配 `action_in_proj`、`action_out_proj`，以及 checkpoint 的形状不匹配、状态输入和数据变换；不能只改输出层。π0 的连续状态投影也与维度相关；π0.5 默认将状态编码为离散 token，不能完全照搬 π0 的修改方案。对本项目没有必要先承担这部分改动。[PyTorch 模型](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/models_pytorch/pi0_pytorch.py)
- **全参数微调与资源：**从 `pi05_base` 初始化、使用不冻结参数的训练配置，是合理的起点；应以独立场景/物体的真机成功率检验效果。官方给出的单卡全参数微调显存参考为 **>70 GB**，因此 5080 笔记本不能据此视为足够的训练设备；可负责采集和部署适配，完整训练安排在大显存服务器，其他省显存训练方案需另外实测。[官方训练说明与资源要求](https://github.com/Physical-Intelligence/openpi#requirements)

2. 调研一下用的最多的franka机械臂，其他研究中遥操作、策略推理使用的franka位置控制模式，还是阻抗控制模式呢？这样做的目的是什么呢？调研一下rokae官方支不支持这种控制器架构呢？

**解答：应区分“策略输出位置目标”和“底层用阻抗执行”。两者可以同时成立，DROID 就是具体例子；不能由几个项目断言所有 Franka 研究都用同一种模式。**

- **Franka / DROID：**DROID 可以接收笛卡尔速度或位置动作，转换为关节目标；其非阻塞执行路径启动 `start_cartesian_impedance()`，然后更新期望关节位置。遥操作与策略推理复用采集循环中的机器人环境接口。因此数据里的 action 可以是位置/速度，底层仍是阻抗控制。[DROID 机器人实现](https://github.com/droid-dataset/droid/blob/main/droid/franka/robot.py)、[遥操作/策略共用循环](https://github.com/droid-dataset/droid/blob/main/droid/trajectory_utils/misc.py)
- **这样做的目的：**策略以较低频率给目标，底层高速闭环处理跟踪和接触。阻抗把机器人与目标之间的关系设成弹簧阻尼，受外力时允许偏离平衡点，适合接触、装配和遥操作中的小误差；代价是实际位置不一定等于目标。Franka 官方示例通过末端误差、雅可比和动力学项计算关节力矩，明确展示了这种架构。[libfranka 官方阻抗例程](https://github.com/frankarobotics/libfranka/blob/main/examples/cartesian_impedance_control.cpp)
- **ROKAE 官方支持：**xCore 的协作机器人实时接口提供关节阻抗、笛卡尔阻抗和力矩模式。官方 ER Pro 七轴例程先设置 `setCartesianImpedance(...)`，再通过 `startMove(cartesianImpedance)` 启动，并在 1 kHz 回调中返回 `CartesianPosition`；这里的位姿是阻抗平衡点。[ROKAE 官方例程](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/integrations/rt/cartesian_impedance_control.cpp)、[官方实时接口](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/include/rokae/motion_control_rt.h)
- **本项目落地：**仓库已经接入 `--arm-control impedance`，默认仍是 `pinocchio` 关节位置路线。现有文档只确认笛卡尔位置路线经过真机验证，阻抗实现仍有待真机确认；SDK 有接口不等于当前机器人授权、负载和参数已验证。建议接触任务评估“TCP 目标 + 原生笛卡尔阻抗”，采集与推理保持相同控制模式、刚度、限速和滤波。记录目标与实测值，不能混为一个字段。详见 [项目控制说明](../rokae/control/README.md)、[阻抗实现及限制](../rokae/docs/ROKAE_IMPEDANCE_IMPLEMENTATION.md)。

3. 调研一下pi0，pi05，diffusion policy，xvla等等VLA/policy框架，VR遥操作采集数据这种方式，需要采集哪些信息？采集遥操作主端还是从端的信息？一般多模态数据用什么格式保存？这种格式满足pi05官方提供的微调要求吗，需要进行格式转换嘛？？？

**解答：训练样本以机器人从端的观察和机器人动作指令为核心；主端数据建议保留用于追溯，但通常不作为自主推理时的输入。**

| 数据类别 | 本项目应保存的内容 | 用途 |
| --- | --- | --- |
| 视觉 | D455 全局 RGB、D435i 掌侧 RGB；各自帧号、采集时间、接收时间 | π0/π0.5 标准视觉输入；两视角分别映射到全局与腕部图像键 |
| 从端实测状态 | 臂关节角/速度、实测 TCP 位姿、Inspire 六通道实际位置及读取时间 | 机器人本体状态；手反馈不可用时应明确标记，不能把指令冒充实测 |
| 从端动作 | 重定向后、与部署接口一致且已接受的臂目标和六通道手目标；命令序号、下发/预定执行时间 | 模仿学习标签；同时保留原始目标、限幅后目标和底层插值参考以便排查 |
| 任务与标定 | 语言指令、episode、成功/失败、起止；相机内参/外参、TCP、坐标系、单位、控制模式及参数版本 | 语言条件训练、拆分数据和复现实验 |
| 主端与扩展传感 | Quest 手腕/骨架、置信度、离合/暂停事件；深度、IMU、手部力反馈/电流等可用信号 | 调试和后续研究；并非原版 π0.5 全部必需或可直接消费 |

**action 与 state 必须分开：**state 是动作前的实测状态，action 是在该观察下发给机器人的目标。阻抗接触时二者尤其不同。若自主运行没有 Quest 输入，就不应让策略依赖 Quest 骨架。深度/触觉可以原样留档，但接入标准 RGB VLA 需要额外模型或输入适配设计。

| 框架/数据源 | 官方代码中的格式与适配方式 |
| --- | --- |
| π0 / π0.5（OpenPI） | 自有数据微调示例先转为 **LeRobot**，定义图像、state、actions、task，再配置输入输出映射、动作变换及归一化；另有 DROID RLDS 专用加载路径。[官方转换示例](https://github.com/Physical-Intelligence/openpi/blob/main/examples/libero/convert_libero_data_to_lerobot.py)、[数据配置](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/training/config.py) |
| Diffusion Policy | 原版真机示例为低维轨迹 **Zarr + 分相机 MP4**，dataset 类读取并构建观察/动作窗口；不同仿真任务也有 HDF5 等来源。它本身不要求 VR 或语言输入。[真机环境](https://github.com/real-stanford/diffusion_policy/blob/main/diffusion_policy/real_world/real_env.py) |
| X-VLA | 通过 domain handler 适配多数据源，既有 **HDF5** handler，也有读取 **Parquet + 视频**的 LeRobot v2.1 handler；动作键、维度和表示仍需按设备修改。[HDF5 基类](https://github.com/2toinf/X-VLA/blob/main/datasets/domain_handler/base.py)、[LeRobot handler](https://github.com/2toinf/X-VLA/blob/main/datasets/domain_handler/lerobotv21.py) |
| DROID / RoboTwin | DROID 采集器写 HDF5，并支持单独相机录制；公开训练发布格式还可为 RLDS。RoboTwin 是仿真 benchmark，当前转换代码把逐帧 PKL 合并为 HDF5 并生成视频，不能视为真机 VR 的通用采集标准。[DROID writer](https://github.com/droid-dataset/droid/blob/main/droid/trajectory_utils/trajectory_writer.py)、[RoboTwin 转换](https://github.com/RoboTwin-Platform/RoboTwin/blob/main/envs/utils/pkl2hdf5.py) |

**项目建议：原始层用逐 episode 的 HDF5 + 两路 RGB 视频及逐帧时间索引，训练层离线导出 LeRobot。**Parquet 存数值/索引、MP4 或图像文件存视觉、JSON/JSONL 存元数据是常见组合，但“同为 LeRobot”也要核对版本：本次查看的 OpenPI 将 LeRobot 固定到指定 git revision，不能默认最新格式直接兼容。[OpenPI 依赖版本](https://github.com/Physical-Intelligence/openpi/blob/main/pyproject.toml)

转换还必须完成：固定采样率、时间对齐、通道与单位统一、动作绝对/增量语义、任务文本、episode 边界以及 `compute_norm_stats.py`。文件后缀一致不足以满足微调要求；应先导出一小批 episode，确认官方 loader 能加载并检查图像与动作回放。旋转增量需要正确的 SO(3) 运算，不应直接对四元数或旋转 6D 向量逐元素相减。

4. 遥操作能实现的情况下，数据同步和保存的架构是怎样的呢？如何从代码框架上保证各类数据同步保存呢？这部分你要调研pi0，pi05，diffusion policy，xvla等等VLA/policy框架，看看大家是如何处理的？看一下DROID，robotwin等等这些优质开源benchmark！！

**解答：采用“异步采集 + 统一时间轴 + 训练频率组帧 + 异步落盘”。同一线程依次读设备，或给它们设置相同 FPS，都不能保证采样同时发生。**

各项目的可借鉴点如下：

- **Diffusion Policy：**相机与机器人独立采集，通过共享内存缓冲；`get_obs()` 依据时间戳选取对齐时刻之前的最新相机帧和机器人状态，`exec_actions()` 接收带目标执行时间的动作，再用时间累积器写 episode。原例使用 UR 机械臂，适合借鉴时间架构，不能当作 Franka 控制模式证据。[RealEnv 实现](https://github.com/real-stanford/diffusion_policy/blob/main/diffusion_policy/real_world/real_env.py)
- **DROID：**主循环依次取得观察、生成动作、执行，并保存控制阶段时间戳；机器人环境默认 15 Hz，观察中另保留机器人和相机时间。writer 用队列和后台线程写入，避免把所有磁盘工作放到控制循环，但仍不等于相机曝光硬同步。[采集循环](https://github.com/droid-dataset/droid/blob/main/droid/trajectory_utils/misc.py)、[RobotEnv](https://github.com/droid-dataset/droid/blob/main/droid/robot_env.py)、[writer](https://github.com/droid-dataset/droid/blob/main/droid/trajectory_utils/trajectory_writer.py)
- **X-VLA / OpenPI：**重点是读取已采集数据并构建训练窗口。X-VLA 的 HDF5 handler 可依时间数组或配置频率插值未来动作；OpenPI 按数据集帧率构建 action chunk。它们不能补救原始采集时遗失的时间信息。[X-VLA 时间处理](https://github.com/2toinf/X-VLA/blob/main/datasets/domain_handler/base.py)、[OpenPI loader](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/training/data_loader.py)
- **RoboTwin：**在仿真推进中按步采集观察，再组织 episode；仿真时钟下的多模态一致性比真机简单。当前转换器还显式用 `values[:-1]` 作 state、`values[1:]` 作 action，这种下一帧标签约定不能直接套到真机阻抗指令记录。[仿真任务实现](https://github.com/RoboTwin-Platform/RoboTwin/blob/main/envs/_base_task.py)、[转换器](https://github.com/RoboTwin-Platform/RoboTwin/blob/main/envs/utils/pkl2hdf5.py)

本项目建议的代码职责与数据流：

```text
Quest → 重定向/目标约束 → 统一机器人动作接口 → 臂实时伺服 + 手控制
                           │命令事件                 │实测反馈
D435i 采集进程 ─┐           ↓                         ↓
D455 采集进程 ─┼→ 带时间戳/序号的缓冲 → 20 Hz 组帧器 → 队列 → episode writer
臂/手状态采集 ─┘                                  ↓
                            原始数据 + 时间索引 → 校验/重采样 → LeRobot
```

1. **统一时钟但保留原始时间：**主机内部用单调时钟；每条流保存设备时间、主机接收时间、序号和时钟域。设备时间需估计偏移/漂移后才能与主机时间比较；跨机器则另做时钟同步。给队列取出时刻打时间戳会把排队延迟误当采样时间。
2. **分开控制频率与训练频率：**项目现有臂回调是 1 kHz；相机可先配置到实测稳定的 30 FPS，组帧/动作时间网格可先选 20 Hz，手按通信能力独立运行。这是起步配置，不是 π0.5 的硬性要求；动作 horizon 必须结合该频率确定实际覆盖时间。
3. **组帧要有时间契约：**定义 `obs_t` 为参考时刻之前已可获得的观察，`action_t` 为对应控制时间槽的已接受目标，同时记录下发、预定生效及反馈时间；臂和手共享动作 ID。对连续状态可离线插值，图像选择已有帧并记录帧龄，离合/模式切换用离散事件。在线观察不得使用未来帧，动作 chunk 不得跨 episode；相机与手通信延迟要实测，不能简单固定错开一帧。
4. **不阻塞实时循环：**SDK 回调只取目标、输出控制量并发布轻量状态；图像编码、HDF5 写入放独立进程/线程。缓冲要有限，队列溢出、漏帧、陈旧状态必须计数并标记该段无效，不能静默补成“完整数据”。episode 结束先排空队列、关闭文件并校验，再从临时目录提交为有效 episode。
5. **验证同步质量：**逐 episode 检查时间单调性、帧号缺口、图像帧龄、跨流偏差及动作 ID；用可见的手部开合/机械臂运动核对视频与状态曲线，输出延迟统计。软件组帧只能给出有误差界限的对齐，不能保证曝光同时发生。D435i 与 D455 的硬件同步、深度同步和 RGB 同步能力要分别核对，接上同步线不代表两路 RGB 已同步。[RealSense 官方多相机说明](https://dev.realsenseai.com/docs/multiple-depth-cameras-configuration/)

**与当前代码的差距：**`integrations/teleop_arm_hand.py::_record_command()` 的 `--arm-record` 仅保存 VR/机械臂目标、TCP 实测及部分诊断，并每次 `flush()`；没有两路相机、手部实测/命令和逐流采样时间，不能直接作为 π0.5 数据集。下一步应增加统一命令事件、独立传感采集器、组帧器和 episode writer，再做 LeRobot 导出；以上为架构建议，本次未修改控制或采集代码。


# 数据采集实现落地（2026-10-22）

- 主配置为 `config/collection/collection.json`，相机参数引用 `config/cameras/realsense.json`；Python 包和相机子进程统一使用 dexteleop Conda 环境。
- 原始存储进一步选择逐 episode HDF5，内嵌 JPEG RGB 与可选 PNG16 深度，保留逐帧时间和序号；不先写定帧率视频再推断原时间。
- 相机 30 FPS、臂状态目标 50 Hz、手反馈目标 20 Hz；停止后离线生成 20 Hz 因果观察和对应时间槽的已接受动作。缺失、暂停、陈旧和时差不达标的行保留原始记录及无效标记。
- state/action 为机器人基座系 TCP xyz+旋转向量六维、原生手寄存器归一化六维；不修改 π0.5 的 32 维投影。实际 ANGLE_ACT 通过校验回复读取，不以目标值冒充。
- r 开始、s 成功结束、f 失败结束、d 移入可恢复回收站、q 退出；中断段默认不训练。写入失败保留 .partial，完整段原子提交。
- 使用 OpenPI 对应 revision 的官方 LeRobot v2.1 API 导出 image-backed Parquet，并回读验证；连续有效段分别成为训练 episode，不跨错误间隙拼接。

字段、命令和限制见[数据采集说明](README.md)，研究对比见[存储与同步设计](../docs/data_collection_design.md)。这部分是前述调研建议的实际落地，替代“尚未实现”的历史状态描述。

### 任务起点、键盘门控与 home 的落地补充（2026-10-22）

每段不要求固定 home：允许先使能、预调整，再按 r 从当前状态录制。统一键盘 e/p
使能/暂停臂手，r 录制，s/f 保存，h 在暂停且结束录制后独立回任务 home；不使用 Quest
按钮或 kill 信号。录制中暂停保留原始观察并切断有效窗口，复位不进入训练示范。
任务配置独立于 collection 配置，以引用方式关联；逐段保留配置快照和实际生效参数。
启动示例与操作表见[双模块手册](../README_QUEST3_INSPIRE_ROKAE.md)，功能及数据流
review 见[本轮验证报告](../docs/refactor_report.md)。
