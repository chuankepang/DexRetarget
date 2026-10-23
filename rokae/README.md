# ROKAE C++ realtime module

This directory is the self-contained ROKAE backend for DexRetarget. It uses
only xCoreSDK-CPP 0.3.4 and is compatible with the current xCore 2.3.2
controller. There is no xCoreSDK-Python runtime or import path.

## Layout

```text
rokae/
├── include/rokae_driver.hpp       # stable C++ driver interface
├── src/rokae_driver.cpp           # state, NRT motion, Cartesian 1 kHz loop
├── src/joint_realtime.cpp         # 1 kHz joint stream used by Pinocchio IK
├── src/pybind_module.cpp          # pybind11 boundary
├── python/rokae.py                # public Python facade
├── config/rokae.yaml              # teleop robot, mount and safety values
├── control/                       # host kinematics, cell frame, task scripts
├── examples/                      # SDK hardware tools (MoveJ / MoveL / read)
├── sdk/
│   ├── include/rokae/             # official 0.3.4 headers
│   ├── include/Eigen/             # required header-only dependency
│   ├── lib/aarch64/libxCoreSDK.so.0.3.4
│   └── lib/x86_64/               # official nomodel runtime + SONAME symlink
└── third_party/pybind11/          # pinned v2.13.6 headers and license
```

The copied SDK files come from
`/SSD-512G/Project/xCoreSDK-CPP-0.3.4`. Runtime and build do not depend on that
external directory. SDK license, upstream README and changelog are preserved
under `rokae/sdk/`. The x86_64 runtime was added from the local official 0.3.4
release with identical headers; its provenance and validation are documented in
[native build and diagnostics](docs/native_build.md).

## Build and import

```bash
cd /data/Projects/DexRetarget
conda activate /data/miniconda3/envs/dexteleop
bash rokae/build.sh

python -c 'import rokae; print(rokae.SDK_VERSION)'
```

Expected SDK version: `0.3.4`. The extension suffix matches the active Python ABI
and host architecture; Python 3.10 x86_64 emits
`rokae/python/_rokae_cpp.cpython-310-x86_64-linux-gnu.so`. The configured library
search path selects `sdk/lib/<architecture>`. Build caches default to
`rokae/build/<architecture>`; `ROKAE_BUILD_DIR` and `ROKAE_PYTHON` remain available.

## Architecture and safety

Python performs Quest decoding, coordinate mapping, configuration and data
logging. `set_target_pose()` only replaces one immutable C++ slot containing a
4x4 target, monotonic timestamp and sequence ID. It never queues poses.

The SDK-owned callback runs every 1 ms in C++ and performs:

1. latest sequence/source-time/source-age validation;
2. short-loss hold and long-loss finished-command watchdog;
3. base-frame workspace clamp;
4. time-constant translation low-pass and quaternion SLERP;
5. translation/rotation deadbands;
6. linear/angular velocity, acceleration and jerk limiting;
7. one `CartesianPosition` result for the current callback.

Python socket delays, printing, retargeting and policy inference therefore
cannot create an xCoreSDK command backlog or block the robot callback.

All motion examples are dry-run by default. Actual motion requires `--execute`
and typing `MOVE`, unless the operator additionally supplies `--yes`.

## Independent tools

Run from the repository root after building:

```bash
# Read only: never powers on or changes motion mode
python rokae/examples/01_read_state.py

# Absolute joint motion, radians
python rokae/examples/02_move_joint.py \
  --joints J1 J2 J3 J4 J5 J6 J7 --execute

# Relative base-frame +X 5 mm MoveL; orientation rotvec is zero
python rokae/examples/03_move_cartesian.py \
  --relative 0.005 0 0 0 0 0 --relative-frame base --execute

# Very small C++ realtime sine test
python rokae/examples/04_realtime_cartesian_test.py --execute

# Move smoothly to the high-elbow init whose J7/TCP +Z wrist axis is +world X.
# Total joint travel is automatically interpolated; dry-run reads soft limits.
# teleop: wrist +Z along +world X. manipulation: tool-down box/cup pose.
python rokae/examples/05_move_to_init.py --pose teleop
python rokae/examples/05_move_to_init.py --pose manipulation --execute

# Inspect already-disabled soft limits; no motion by default.
python rokae/examples/06_recover_soft_limits.py

# Read and print copyable init YAML; never overwrites the config
python rokae/examples/save_current_as_init.py
```

Motion and realtime examples request SDK-controlled power-on by default. Use
`--no-power-on` only when the cell procedure explicitly requires external
power-on. The read-only example never powers on or changes motion mode.

For already-disabled soft limits, the independent maintenance tool uses bounded
single-axis manual Jog and re-enables the original limits with read-back. It
defaults to waiting for physical manual enable (`--manual-power-source operator`);
SDK manual power-on is only available without an external enabling device or
teach pendant. It does not start init or teleoperation. Read-only controller
logs are available with `01_read_state.py --controller-logs`. See [recovery procedure and hardware
validation limits](docs/commissioning.md#已关闭软限位的程序恢复). The legacy
`05_move_to_init.py --recover-joint-limits` route uses MoveAbsJ and is not a
validated substitute for manual Jog recovery.

## Official 0.3.4 APIs used

- connection/state: `xMateErProRobot`, `robotInfo`, `powerState`,
  `operateMode`, `operationState`, `jointPos`, `jointVel`, `cartPosture`;
- NRT: `MoveAbsJCommand`, `MoveLCommand`, `moveReset`, `moveAppend`,
  `moveStart`, `stop`;
- realtime: `setRtNetworkTolerance`, `startReceiveRobotState`,
  `getStateData`, `getRtMotionController`, `setFilterLimit`, `setControlLoop`,
  `startMove(cartesianPosition)` or `startMove(jointPosition)`, `startLoop`,
  `stopMove`, `stopLoop`;
- teardown: `stopReceiveRobotState`, switch back to `NrtCommand`, then
  `disconnectFromRobot`.

`tcpPose_m` and realtime commands are row-major `base_T_tcp` matrices in metres.
NRT `MoveL` uses the active SDK external reference; the bridge compares
`endInRef` against `tcpPose_m` and rejects the command unless that reference is
aligned with the base (2 mm / 0.02 rad check). Configure the intended TCP/tool
and a base-aligned work object before the first Cartesian motion.
