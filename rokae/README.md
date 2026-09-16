# ROKAE C++ realtime module

This directory is the self-contained ROKAE backend for DexRetarget. It uses
only xCoreSDK-CPP 0.3.4 and is compatible with the current xCore 2.3.2
controller. There is no xCoreSDK-Python runtime or import path.

## Layout

```text
rokae/
├── include/rokae_driver.hpp       # stable C++ driver interface
├── src/rokae_driver.cpp           # state, NRT motion, 1 kHz realtime loop
├── src/pybind_module.cpp          # pybind11 boundary
├── python/rokae.py                # public Python facade
├── config/rokae.yaml              # robot, init, mount and safety values
├── examples/                      # independent hardware tools
├── sdk/
│   ├── include/rokae/             # official 0.3.4 headers
│   ├── include/Eigen/             # required header-only dependency
│   └── lib/aarch64/libxCoreSDK.so.0.3.4
└── third_party/pybind11/          # pinned v2.13.6 headers and license
```

The copied SDK files come from
`/SSD-512G/Project/xCoreSDK-CPP-0.3.4`. Runtime and build do not depend on that
external directory. SDK license, upstream README and changelog are preserved
under `rokae/sdk/`.

## Build and import

```bash
cd /SSD-512G/Project/DexRetarget
conda activate /SSD-512G/conda_envs/anydex
export LD_LIBRARY_PATH=/SSD-512G/conda_envs/anydex/lib
bash rokae/build.sh

python -c 'import rokae; print(rokae.SDK_VERSION)'
```

Expected SDK version: `0.3.4`. The extension is emitted as
`rokae/python/_rokae_cpp.cpython-310-aarch64-linux-gnu.so`; its RUNPATH points
to the vendored `rokae/sdk/lib/aarch64` directory.

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

# Move smoothly to the forward/palm-down init. Dry-run first; this checks
# ER3 Pro joint-limit margins and J2/J4/J6 singularity guards.
python rokae/examples/05_move_to_init.py
python rokae/examples/05_move_to_init.py --execute

# Read and print copyable init YAML; never overwrites the config
python rokae/examples/save_current_as_init.py
```

Motion and realtime examples request SDK-controlled power-on by default. Use
`--no-power-on` only when the cell procedure explicitly requires external
power-on. The read-only example never powers on or changes motion mode.

## Official 0.3.4 APIs used

- connection/state: `xMateErProRobot`, `robotInfo`, `powerState`,
  `operateMode`, `operationState`, `jointPos`, `jointVel`, `cartPosture`;
- NRT: `MoveAbsJCommand`, `MoveLCommand`, `moveReset`, `moveAppend`,
  `moveStart`, `stop`;
- realtime: `setRtNetworkTolerance`, `startReceiveRobotState`,
  `getStateData`, `getRtMotionController`, `setFilterLimit`, `setControlLoop`,
  `startMove(cartesianPosition)`, `startLoop`, `stopMove`, `stopLoop`;
- teardown: `stopReceiveRobotState`, switch back to `NrtCommand`, then
  `disconnectFromRobot`.

`tcpPose_m` and realtime commands are row-major `base_T_tcp` matrices in metres.
NRT `MoveL` uses the active SDK external reference; the bridge compares
`endInRef` against `tcpPose_m` and rejects the command unless that reference is
aligned with the base (2 mm / 0.02 rad check). Configure the intended TCP/tool
and a base-aligned work object before the first Cartesian motion.
