# 工具纵轴 90° 拧转：映射与 IK 审查

更新：2026-10-29。用户确认使用 Pinocchio/默认路线，希望绕工具自身纵轴 Z 拧转。
本轮没有连接或上电；下列数值来自离线拟合模型和原生关节参考流，不代表真机精度。

## 找到的问题

1. 原配置 `rotation_scale: 0.8`：人手 90°的相对旋转只生成约 72°。
2. 旧姿态映射为 spatial：把手腕空间增量作用于机器人初始姿态。
   这不保证手腕自身轴对应工具自身轴；Cup 的初始 TCP 方向和旧 commissioning 姿态不同。
3. Pinocchio 路线使用本项目微分 IK，不是 SDK 官方主机 IK。
   存在姿态保持、限位排斥、奇异回避副任务，不是没有零空间控制。
   阻尼投影并非严格零空间，可能留下小残差，但本轮没有复现工具 Z 90°无法求解。
4. 跟踪丢失后会自动重定参考。即便恢复等待只有 10 ms，重定参考仍会改变本次相对动作的起点；
   不能把经过多次重定参考的手腕总转角当成一次连续 90°目标。

映射使用四元数/旋转矩阵和 SO(3) 旋转向量，没有 Euler 角相减。
`*_dr_deg` 表示旋转向量的分量，不是一般情况下的 roll/pitch/yaw。

## 修改后的配置与公式

唯一入口：`config/robots/rokae.yaml` → `teleoperation`：

```yaml
rotation_scale: 1.0
rotation_mapping: tool_relative
wrist_to_tool_rotation:
  [0.0, 0.0, -1.0,
   0.0, 1.0, 0.0,
   1.0, 0.0, 0.0]
```

接收端已经将 Unity 左手系转换为项目右手系。这里的 A 矩阵进一步约定：
项目手腕局部 X → 工具 Z、Y → Y、Z → -X，是合法右手旋转。
HTS 原始纵轴的数值符号还经过左/右手系转换，不应再手动对四元数分量换轴或反号。
该约定用于满足用户指定的纵轴操作，不是通过照片或模型完成的安装标定；必须现场核对三轴和符号。

在 scale=1 时，新姿态映射为：

`R_base_tool_goal = R_base_tool_ref * A * (R_wrist_ref.T * R_wrist_now) * A.T`

先求初始手腕坐标中的旋转，再映射到初始工具坐标。位置仍按空间位移映射，
不会因为转手腕而旋转平移控制轴。`mapped_relative` 仍输出世界系增量，保持既有观察/采集接口含义。
旧调用缺省仍为 spatial；改回 YAML `rotation_mapping: spatial` 可恢复旧轴规则。
映射变化会改变操作语义；旧数据不应在未核对控制参数的情况下和新数据混为同一训练配置。
episode 元数据新增保存映射模式和 A 矩阵，状态/action 的数据形状不变。

## 离线结果与限制

Cup home 为 [0,0.13323130,0,1.17304764,0,1.2835298642,0] rad。
按当前模型，`R_base_tool` 约为：

```text
[-0.8516, 0,  0.5242]
[ 0,      1,  0     ]
[-0.5242, 0, -0.8516]
```

旧 spatial 映射中，空间 X 转轴在此工具坐标里混合 X/Z 分量；不能当作纯工具 Z。
新映射对任意不共轴的初始手腕/工具姿态，合成手腕局部 X 的 90°输入得到纯工具 Z 90°输出。

4 秒目标渐变、8 秒保持的拟合模型微分 IK 结果：

| 目标 | 最终位置误差 | 最终姿态误差 |
|---|---:|---:|
| 工具 Z +90° | 0.0725 mm | 0.00124° |
| 工具 Z -90° | 0.0780 mm | 0.00185° |
| 工具 X +90° | 0.0858 mm | 0.00161° |
| 工具 X -90° | 0.0473 mm | 0.00123° |

另有精确模型见证：仅把 home 的 J7 加/减 90°，FK 得到原位置的工具 Z 正/负 90°。
这证明该拟合模型存在解，不证明实际标定模型、碰撞环境和控制器约束都允许。
已有主机模型限位与控制器限位不完全相同，真机仍以两者交集为准。
工具 Y 的大角度不在此结论范围内；额外检查中该局部求解路径可能碰到 J4/J6 模型限位。

新增离线测试覆盖：任意初始姿态/基座旋转、三轴正反角度、重定参考无跳变、平移不变、
原 0.8 缩放复现、Cup 90°映射→微分 IK→原生 1 kHz 关节流。
原生关节流测试只验证软件参考，不包含真实伺服和硬件。

## 检查指令

```bash
conda activate dexteleop
# 不连接硬件，检查映射模式、增益和轴矩阵
python -m dexretarget --task-config config/tasks/cup.yaml --arm-control pinocchio --print-arm-config
# 不连接硬件，合成三轴输入并检查拟合 IK
python tools/diagnostics/check_rokae_orientation.py --include-ik
```

现场先从 10–20°小角度核对纵轴与符号，再缓慢扩大角度。需要现场执行的命令：

```bash
python -m dexretarget --task-config config/tasks/cup.yaml --enable-hand --enable-arm --arm-control pinocchio --no-capture --arm-print-hz 2
```

按 `e` 使能。打印新增 `mapped_tool_dr_deg`、`raw_to_command_deg` 和 `rejected_target_delta`：

- mapped_tool 的 Z 分量正确、raw_to_command 大：目标已生成，继续查 IK、参考流限幅或目标拒绝。
- raw_to_command 小、tracking_error 大：参考已跟上，继续查模型与实际反馈/控制器约束。
- 出现 Tracking HOLD/recovered 或重定参考：先处理数据连续性，再评估连续 90°。
- `--arm-record` 的可选调试记录新增实时命令 TCP、实测关节和拒绝计数；默认不创建记录文件。

本轮没有增大力矩、速度或软限位，也没有通过锁住冗余轴掩盖映射问题。

本轮软件验证：324 项 pytest、9 项子测试通过；项目检查 195 个 Python 文件、42 个规范模块、202 个本地文档引用，无错误。真机三轴标定、实际90°跟随与跨姿态性能尚未验证。
