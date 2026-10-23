# 可选设备集成与标定

正式采集入口为 `python -m dexretarget`，本目录不承载主链路实现。

- `teleop_real.py`、`teleop_sim.py`：多手型硬件与仿真。
- `teleop_dual_iiwa_robotiq.py`：双 IIWA。
- `input/`、`output/`：可选设备及正式驱动的转发接口。
- `test/`：可视化、标定和调参工具，不属于 pytest 测试集。
- `config`：仅为旧工具保留的符号链接，真实参数统一在 `../config/retargeting/`。

迁移保留原脚本功能与资源；未连接的可选硬件和 GUI 没有实机验收。
