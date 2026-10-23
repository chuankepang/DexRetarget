# 维护工具

从仓库根目录、`dexteleop` 环境运行。工具不被正式控制模块导入。

| 目录/命令 | 用途 |
| --- | --- |
| `check_project.py` | 标准库 AST 检查自有 Python 语法、核心依赖/循环及本地文档链接 |
| `diagnostics/check_cameras.py` | 相机枚举、绑定和双路带宽检查；不驱动机器人 |
| `diagnostics/check_quest_wrist_stability.py` | 原始 wrist 抖动/漂移；不驱动机器人 |
| `diagnostics/check_rokae_connection.py` | SDK 只读连接 |
| `robot/rokae_joint_tool.py` | 关节读取/明确确认后的小范围运动 |
| `simulation/rokae_teleop.py` | 无硬件的坐标映射与限幅演示 |
| `simulation/rokae_realtime.py` | 需要已编译原生绑定的离线实时核心检查 |

重复的 example/integrations 转发脚本已删除，请直接使用上表中的工具。
原生 SDK 工具仍在 `rokae/examples/`；可选手型 GUI 和标定工具在 `integrations/`。
`install_collection.sh` 为现有 dexteleop 环境安装采集/导出依赖，本轮未执行安装。
