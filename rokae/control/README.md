# ROKAE 控制：怎么跑、怎么选控制器

所有命令都在仓库根目录执行：

```bash
cd /data/Projects/DexRetarget
conda activate /data/miniconda3/envs/dexteleop
```

遥操作入口会在需要时重启进程，让当前 Conda 的 libstdc++ 优先加载。仅激活环境不保证其他独立脚本的动态库搜索路径；仍应使用匹配平台的 SDK 和绑定。

不带 `--execute` 的任务脚本只打印轨迹，机械臂不动。真机运动必须加 `--execute`，再在提示里输入 `MOVE`。

配置文件是 `config/robots/rokae.yaml`。

## 先分清两件事

**遥操作**有三条实时路线，默认暂停，按键盘 `e` 后跟踪 Quest：

| `--arm-control` | 实际下发 | 含义 |
| --- | --- | --- |
| `pinocchio` | `jointPosition` | 这台电脑做微分逆解，1 kHz 发关节角。默认。 |
| `cartesian` | `cartesianPosition` | 控制器笛卡尔位置伺服；历史有真机记录，本轮未重新验收。 |
| `impedance` | `cartesianImpedance` | 控制器原生笛卡尔阻抗。目标是平衡点，不是必须到达的位姿。 |

**杯子 / 方盒**只有两种执行方式：

| `--controller` | 实际下发 | 含义 |
| --- | --- | --- |
| `position`（默认） | `MoveAbsJ` 到操作初始位，然后 `MoveL` | 和以前一样的位置轨迹。 |
| `impedance` | `MoveAbsJ` 仍然是位置控制，轨迹段改成阻抗平衡点 | 同一条路点，换成柔顺跟踪。 |

杯子和方盒在 `position` 下始终是 `MoveL`。给它们传 `--controller pinocchio` 或 `--controller cartesian` 不会改成关节流，仍然是 `MoveL`。`pinocchio` / `cartesian` 只对 Quest 遥操作有区别。

位置和阻抗不能在机械臂还在动的时候热切换。先停掉当前程序，再开另一个。

## 怎么选择控制器

优先级从高到低：

1. 命令行
2. `tasks.cup.controller` 或 `tasks.box.controller`
3. `control.controller`
4. 都没写时用位置控制。遥操作再看 `control.mode`：`pinocchio` 或 `cartesian`

现在的默认是：

```yaml
control:
  mode: pinocchio          # 只影响遥操作的位置路线
  controller: position     # position 或 impedance
tasks:
  cup:
    controller: position
  box:
    controller: position
```

先看解析结果，不连机器人：

```bash
python rokae/control/examples/print_controller.py
python rokae/control/examples/print_controller.py --task cup
python rokae/control/examples/print_controller.py --controller impedance --task cup
```

### 遥操作

不写 `--arm-control` 时用 YAML。现在 YAML 是位置控制 + `pinocchio`。

```bash
# 默认：Pinocchio 关节位置。--mock-rokae 不连真机
python -m dexretarget --enable-arm --mock-rokae

# 真机，仍是 Pinocchio
python -m dexretarget --enable-arm

# 控制器笛卡尔位置
python -m dexretarget --enable-arm --arm-control cartesian

# 控制器笛卡尔阻抗
python -m dexretarget --enable-arm --arm-control impedance
```

也可以不传命令行，改 `config/robots/rokae.yaml`：

```yaml
control:
  controller: impedance
```

命令行会盖过这一行。`--arm-control pinocchio` 或 `--arm-control cartesian` 会强制回到位置控制。

Quest 遥操作开始前会走到 `initial_joint_move` 里的类人伸腕姿态。杯子和方盒用的是另一套、末端朝下的操作初始位，不要混用。

上面的无任务入口沿用原初始化配置。加载 `--task-config` 后，任务 home 按 `on_start` 决定启动回位，`--skip-init` 可显式跳过。统一臂手门控、采集与 home 的当前操作见[使用指南](../../docs/usage.md)。

### 杯子和方盒

干运行，只看位姿，机械臂不动：

```bash
python rokae/control/examples/cup_flip.py
python rokae/control/examples/box_vertices.py
```

真机，默认位置控制（`MoveL`）。程序会先 `MoveAbsJ` 到操作初始位：

```bash
python rokae/control/examples/cup_flip.py --execute
python rokae/control/examples/box_vertices.py --execute
```

提示出现后输入 `MOVE`。不想每次输入就加 `--yes`，但仍然必须有 `--execute`。

这一次改成阻抗，不改 YAML：

```bash
python rokae/control/examples/cup_flip.py --controller impedance --execute
python rokae/control/examples/box_vertices.py --controller impedance --execute
```

以后都用阻抗，改 YAML 后再直接 `--execute`：

```yaml
tasks:
  cup:
    controller: impedance
  box:
    controller: position
```

命令行 `--controller position` 可以临时盖回位置控制。

杯子任务：杯子躺着，杯口朝左。手从左下方 45° 抓杯底，上抬 5 cm，翻转后杯口朝下，手仍与竖直方向成 45°，换一个位置放下，停 2 秒，回到操作初始位。方盒是末端朝下，绕上下两个矩形走一圈再回来。

## 阻抗参数

在 `config/robots/rokae.yaml` 的 `control.impedance`：

```yaml
impedance:
  backend: sdk_cartesian
  stiffness: [300.0, 300.0, 300.0, 15.0, 15.0, 15.0]  # X Y Z Rx Ry Rz，N/m 和 Nm/rad
  desired_wrench: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # N 和 Nm，默认不加额外力
  reference_limit:
    translation: 0.04   # 平衡点离当前末端最远 4 cm
    rotation: 0.30      # 最远 0.30 rad
```

这是低于 SDK 上限的占位值，不是厂家推荐值。上限是 `{1500, 1500, 1500, 100, 100, 100}`。SDK 0.3.4 没有单独的笛卡尔阻尼参数，阻尼在控制器内部。`mock_damping` 只给离线弹簧模型用，不会发给机器人。

## 单独动机械臂

到两个初始位之一。`teleop` 是 Quest 用的伸腕姿态，`manipulation` 是杯子和方盒用的末端朝下姿态。

```bash
python rokae/examples/05_move_to_init.py --pose teleop
python rokae/examples/05_move_to_init.py --pose manipulation --execute
```

其他单步脚本同样是先干运行，加 `--execute` 才动：

```bash
python rokae/control/examples/check_model.py
python rokae/control/examples/joint_move.py
python rokae/control/examples/cartesian_move.py
python rokae/control/examples/servo_joint.py
python rokae/control/examples/diff_ik_track.py
```

第一次真的用关节流之前，用 `check_model.py --execute` 对一下主机模型和控制器正解。连杆长度在 `config/models/xmate_er_pro_7.yaml`。

不经过 Pinocchio 的 SDK 例程在 `rokae/examples/`。

## 坐标

`cell.py`：人站在机械臂侧看向桌面，+X 向前，+Y 向左，+Z 向上。基座原点离地 0.83 m，桌面离地 0.825 m，所以桌面在基座坐标里是 z = −5 mm。张开的 Inspire 指尖在法兰 +Z 方向 0.18 m 处。
