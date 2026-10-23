# D435i 掌侧相机与 D455 全局相机

本目录管理训练/推理所用双相机的配置、安装脚本和诊断。现有
`integrations/input/realsense.py` 是“相机识别人手后用于遥操作”的单相机输入，
与这里记录机器人场景的两台相机用途不同。

项目目录规划与 SDK 安装见[根目录中文 README](../README.zh.md)。

Python SDK 和相机采集统一运行于 `/data/miniconda3/envs/dexteleop` Conda 环境。
正式 episode 采集、保存和导出见[数据模块](../data/README.md)。

## 相机绑定

当前设备绑定如下，程序按序列号选择，不依赖 USB 枚举顺序：

| 角色 | 型号 | 序列号 | 本次读取的固件 |
| --- | --- | --- | --- |
| `wrist` 掌侧/腕部 | D435i | 243522070004 | 5.17.3.10 |
| `global` 全局 | D455 | 234322305920 | 5.16.0.1 |

更换设备后重新确认绑定：

```bash
/data/miniconda3/envs/dexteleop/bin/python realsense/scripts/check_cameras.py --inventory
/data/miniconda3/envs/dexteleop/bin/python realsense/scripts/check_cameras.py --inventory --bind-serials
```

已有序列号绑定时不会悄悄替换成另一台相机；更换设备需先更新配置中的序列号，
或清空相应 `serial` 后再按唯一型号绑定。

## 采集与推理配置

统一使用 `config/cameras.json`：RGB8、640×480、30 FPS，默认不开深度。
这足以作为 RGB VLA 的第一版配置，是否看得清细小接触还需检查实际画面、
掌侧遮挡和模型缩放后的图像；不能仅凭分辨率保证任务效果。

检查器显式开启 RGB 自动曝光，并关闭 `auto_exposure_priority`，防止自动曝光优先级
主动降低帧率。光照、曝光、相机位置和预处理在采集/推理时应保持一致。
如果改用固定曝光，先检查画面亮度，再同步更新采集和推理配置。
`--color-exposure` 接受设备 SDK 的曝光单位，不是统一的微秒单位。

```bash
# 两台同时采集，预热 5 秒，计时 60 秒；仅测量，不保存画面。
/data/miniconda3/envs/dexteleop/bin/python realsense/scripts/check_cameras.py --seconds 60

# 增加深度，检查未经 SDK frameset 配对的独立传感器帧。
/data/miniconda3/envs/dexteleop/bin/python realsense/scripts/check_cameras.py --depth --unpaired --seconds 60

# 单独排查 D455。
/data/miniconda3/envs/dexteleop/bin/python realsense/scripts/check_cameras.py --only global --seconds 15
```

程序保存实际流配置、设备信息、帧率、帧号缺口、重复帧、接收间隔和时间戳域；
检查不通过时返回非零退出码。默认 pipeline 模式测的是送达应用的 frameset，
其中 SDK 可能复用某一路帧；`--unpaired` 用独立 sensor callback 检查源流，
因此两种模式的帧号统计不能混淆。

RGB8 双路解码后像素数据约 **55.30 MB/s**；双路 RGB8 + Z16 约 **92.16 MB/s**。
这是数组数据量估算，不是 USB 线上实际吞吐，后者还涉及传输格式、协议和开销。
本次两台均以 5 Gbps 链路接在同一 10 Gbps Hub 下，最终判断以实测为准。

## 2026-10-22 真机测试结果

每项先预热 5 秒，再测量 60 秒。最终显式曝光配置下：

| 测试模式 | D435i RGB | D435i 深度 | D455 RGB | D455 深度 | 帧号缺失/重复 |
| --- | --- | --- | --- | --- | --- |
| 双路 RGB，pipeline | 29.978 FPS / 1799 帧 | 未开启 | 29.987 FPS / 1799 帧 | 未开启 | 全部 0 |
| 双路 RGB＋深度，独立回调 | 29.978 FPS / 1798 帧 | 29.978 FPS / 1799 帧 | 29.987 FPS / 1799 帧 | 29.987 FPS / 1800 帧 | 全部 0 |

第一项无接收超时；第二项按各源流独立计数，通过频率、帧数及连续帧号检查。
计时窗内帧数略有差异来自各传感器相位和实际帧率，不应直接按数组下标跨流配对。
在本次短时采集负载下，**640×480、30 FPS 双相机未发现带宽不足**。

报告：
[双路 RGB](../data/camera_tests/dual_rgb_explicit_ae_640x480_30.json)、
[双路 RGB＋深度独立源流](../data/camera_tests/dual_rgbd_independent_640x480_30.json)。

同时保留两项初始异常记录，未将其改写为通过：

- 首轮 RGB 测试中 D455 为 29.035 FPS，未达到严格的帧率阈值；重新启动并显式设置
  自动曝光后，单机与双机复测均约 30 FPS。尚不能仅凭此确定初始偏差的唯一原因。
- 首轮 RGB＋深度的 pipeline 配对模式，两台深度均出现一次重复及一次后续跳号；
  独立传感器回调复测没有源流帧号缺失。该现象符合组帧复用/匹配的影响，
  不能直接认定为 USB 传输丢帧，也不能保证未来 pipeline 配对不会再次发生。

测试没有运行视频编码、磁盘录制、机械臂控制或 VLA 推理；后续联合运行需要再做
长时间压力验证。软件时间戳对齐、SDK 单机 RGB/深度配对和两台相机曝光硬同步
是不同问题，本工具不保证跨相机曝光同时发生。

## 后续接入

`dexretarget/camera/` 管理独立相机进程，应用组装后送入 `dexretarget/dataset/` 存储与对齐；
不在 1 kHz 臂控制回调内做图像编码或写盘。SDK 内参随 episode 保存，
但相机到 TCP/基座的外参仍需实际标定。

安装依据：[RealSense 官方 Linux 安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md)、
[官方 Python 包](https://pypi.org/project/pyrealsense2/)。
