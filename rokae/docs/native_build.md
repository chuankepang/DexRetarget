# ROKAE 原生构建、加载与离线检查

本机 x86_64/Python 3.10 的桥接扩展已编译并可加载。SDK 导入、同一模块身份及原生离线测试通过；本次未连接、上电或运动真实机器人。

## 原因与修复

旧仓库只携带 ARM64 扩展与 ARM64 SDK，CMake 也显式限制 aarch64。本机 Python 需要 `_rokae_cpp.cpython-310-x86_64-linux-gnu.so`，不能加载 ARM64 文件。父包此前提前导入 facade，使缺失扩展的异常看起来像 circular import。

- CMake 按 x86_64/aarch64 选择 SDK，build.sh 分开构建缓存；原 ARM64 二进制保留。
- `rokae.python` 延迟加载 facade；直接导入子模块不再绕进提前初始化。新诊断在导入前检查兼容 ABI，并保留链接器的具体错误。
- 同一 SDK 只以同一个包限定名加载，避免 pybind 类型重复注册。
- 使用已有官方 0.3.4 的 x86_64 运行库，头文件、许可证、changelog 与原 vendored 版本一致；C++ 控制实现及物理参数不变。

## SDK 变体与来源

来源：`/data/Projects/xCoreSDK-CPP-0.3.4/lib/Linux/cpp/x86_64/`。运行只依赖已复制到本项目的文件，不依赖外部源码目录。

官方 `lib/CMakeLists.txt` 默认 `XCORE_USE_XMATE_MODEL=OFF` 时选择 `libxCoreSDK.nomodel.so.0.3.4`。模型版还需要额外静态 xMateModel/KDL，单独加载会报 `_ZN3KDL2PIE`。本项目不启用该额外模型宏，与原 aarch64 路线一致，使用官方默认 nomodel 变体；SDK 的 `Model_T::calcFk` 控制器接口仍存在。

实际文件为 `rokae/sdk/lib/x86_64/libxCoreSDK.nomodel.so.0.3.4`，其 ELF SONAME 为 `libxCoreSDK.so.0.3.4`，同目录以相对符号链接提供该名称。文件内容未修改：

```text
SHA256 270614ea618179b41983f65831732af156bae1533ce27f80b0e92e028af24650
```

### 官方平台与下载核查（2026-10-22）

- [官方平台说明](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/README.md)列出 Linux x86_64/aarch64；[预编译库说明](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/lib/README.md)说明当前按版本和平台在 Releases 分发。
- 项目沿用的 [v0.3.4 发布页](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v0.3.4)标明面向 xCore V2.1；当前该页附件只列源码归档，不能虚构独立的 v0.3.4 Linux 平台包下载地址。本机已经有同版官方 SDK，本次直接使用上述本地 x86_64 库。
- 官方 [v0.7.1 附件列表](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/expanded_assets/v0.7.1)确实提供 `xCoreSDK-0.7.1-linux-x86_64.tar.gz`。[新版本更新记录](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/CHANGELOG.md)有不同的控制器版本要求；本次没有为了平台适配升级 SDK 或控制器。
- 若“镜像”指整套系统 ISO/Docker 镜像，本次未核实到可直接用于此项目的官方镜像下载；查到的是 SDK 平台库和[实时 Linux 配置说明](https://docs.rokae.com/docs/SDK/quick_start/)，两者不能混同。

## 在 dexteleop 构建

本机缺少编译工具，已在同一 Conda 环境新增 CMake、make、gxx_linux-64 及必要构建依赖；安装使用 `--freeze-installed`，没有替换已有包。未另建环境或安装系统编译器。

本机 Linux x86_64 如需重建环境中的构建工具：

```bash
conda install -n dexteleop --override-channels -c conda-forge \
  --freeze-installed cmake make gxx_linux-64
```

重新激活以加载 Conda 编译器环境变量，再构建：

```bash
cd /data/Projects/DexRetarget
conda activate /data/miniconda3/envs/dexteleop
bash rokae/build.sh
python -c 'import rokae; print(rokae.SDK_VERSION); print(rokae.module_file())'
```

本机预期版本 `0.3.4`，扩展位于 `rokae/python/`，文件名以 `cpython-310-x86_64-linux-gnu.so` 结尾。
Python ABI 或架构变化后必须重新构建，不能重命名旧 ARM64 文件冒充本机扩展。构建产物按现有 `.gitignore` 忽略，源码与 SDK 用于重建。

## 检查范围

以下命令只执行导入与离线算法，不连接机器人：

```bash
python -m pytest -q tests/test_rokae_native_loading.py \
  tests/test_rokae_realtime.py tests/test_rokae_differential_ik.py \
  tests/test_rokae_controllers.py
python -B tools/check_project.py
```

本轮原生相关测试 **37 passed，另有 9 个子测试通过**；完整测试 **139 passed、1 failed、0 skipped**，另有 9 个子测试通过。机器可读证据见[本次验证记录](../../docs/validation/run_history.md)。原重构报告中的 10 项跳过属于修复前的历史结果；本次已真实执行它们。唯一失败仍是既有 `test_init_nrt_speed_is_independent_and_conservative`：SDK 速度 250 不满足测试要求小于 100，未改物理速度或放宽断言。

`PREEMPT_RT` 警告不是此次加载错误的原因。普通内核的实际 1 ms 回调时序、设备网络、真机初始化/home、位置/阻抗运行仍须按[硬件验收流程](commissioning.md)单独验证。
