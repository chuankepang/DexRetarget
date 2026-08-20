# Rokae Quest3 Teleoperation Implementation

## 1. Final Architecture

```text
Quest 3 Hand Tracking App (UDP/TCP)
                 │
                 ▼
       example/input/quest3.py
          │               │
          │ landmarks     │ wrist SE(3)
          ▼               ▼
   AnyDexRetarget    RelativePoseMapper
          │               │
          ▼               ▼
 InspireSerialOutput  world/base transform
          │               │
          ▼               ▼
   Inspire Hand      PoseSafetyLimiter
                          │
                          ▼
                  latest target buffer
                    │             │
                    ▼             ▼
             MockRokaeDriver  RokaeXCoreDriver
                    │             │ ctypes / C ABI
                    │             ▼
                    │       C++ xCoreSDK bridge
                    │             │ 1 kHz SDK callback
                    └─────────────▼
                      CartesianPosition
```

Quest acquisition/retargeting and ROKAE realtime command production are decoupled. The Python side replaces one latest target; it does not queue old VR frames. The C++ callback repeatedly returns the latest absolute Cartesian target at the xCoreSDK realtime rate.

This follows the same broad separation used by other official robot stacks: libfranka runs a robot-owned 1 kHz callback, UR RTDE synchronizes a latest data set, and Kinova separates normal API setup from its 1 ms cyclic channel. These references were used only for architecture comparison; the implemented ROKAE calls come exclusively from the local xCoreSDK headers/examples.

- libfranka control-loop overview: <https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html>
- Universal Robots RTDE guide: <https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html>
- Kinova official cyclic example: <https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_cpp/examples/108-Gen3_torque_control/01-torque_control_cyclic.cpp>

## 2. Files Added / Modified

Added:

- `anydexretarget/teleop/__init__.py`
- `anydexretarget/teleop/pose.py`
- `anydexretarget/teleop/arm.py`
- `example/config/rokae_teleop.yaml`
- `example/output/real/drivers_rokae.py`
- `example/output/real/rokae_cpp/CMakeLists.txt`
- `example/output/real/rokae_cpp/build_bridge.sh`
- `example/output/real/rokae_cpp/rokae_bridge.cpp`
- `example/test_rokae_teleop_sim.py`
- `tests/test_rokae_pose_mapping.py`
- `tests/test_rokae_base_transform.py`
- `tests/test_rokae_safety.py`
- `ROKAE_TELEOP_IMPLEMENTATION.md`

Modified:

- `example/teleop_arm_hand.py`
- `example/input/quest3.py`
- `example/output/real/__init__.py`
- `pyproject.toml`
- `README_QUEST3_INSPIRE_ROKAE.md`

The pre-existing local `requirements.txt` change enabling `pyserial` was preserved and was not created by this implementation pass.

## 3. Rokae SDK APIs Confirmed

The API source of truth is:

```text
/SSD-512G/Project/rokae-cpp/xCoreSDK-CPP
```

Inspected SDK revision:

```text
git commit: 028203f55099bb2a80d97fb8a0a01fe7da987ba6
SDK version: 0.3.4
xCore compatibility: v2.1.0.15+
```

Confirmed directly from `include/rokae/robot.h`, `include/rokae/motion_control_rt.h`, `include/rokae/data_types.h`, and `example/rt/*.cpp`:

- Robot construction/connection:
  - `rokae::xMateRobot(remoteIP, localIP)` for 6-axis collaborative models;
  - `rokae::xMateErProRobot(remoteIP, localIP)` for 7-axis ER Pro models;
  - `rokae::StandardRobot(remoteIP, localIP)` for standard industrial 6-axis models.
- Setup:
  - `setOperateMode(OperateMode::automatic, ec)`;
  - `setRtNetworkTolerance(percent, ec)` before realtime mode;
  - `setMotionControlMode(MotionControlMode::RtCommand, ec)`;
  - optional explicit `setPowerState(true, ec)`.
- State:
  - `startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tcpPose_m})`;
  - `updateRobotState(...)`;
  - `getStateData(RtSupportedFields::tcpPose_m, pose)`;
  - `tcpPose_m` is a row-major 4×4 TCP pose relative to the robot base.
- Realtime Cartesian control:
  - `getRtMotionController().lock()`;
  - `startMove(RtControllerMode::cartesianPosition)`;
  - `setControlLoop(std::function<CartesianPosition()>)`;
  - `startLoop(false)`;
  - `stopMove()` and `stopLoop()`.
- Cleanup:
  - `stopReceiveRobotState()`;
  - `setMotionControlMode(MotionControlMode::NrtCommand, ec)`;
  - `disconnectFromRobot(ec)`.

The inspected SDK exposes `CartesianPosition` as a realtime callback command. It does not expose a corresponding `CartesianVelocity` callback command type in `setControlLoop`; `tcpVel_c` is a state field, not a verified command object. Therefore this integration uses Cartesian pose servoing, not an invented velocity API.

## 4. Rokae Driver Design

`RokaeDriverBase` defines one interface for real and mock backends:

```python
connect()
start(power_on=False)
get_tcp_pose()
set_target_pose(base_T_tcp_target)
hold()
stop()
disconnect()
```

`MockRokaeDriver`:

- opens no network or device;
- runs its own configurable periodic thread;
- uses one thread-safe latest target instead of a FIFO;
- emulates Cartesian position hold after its command watchdog expires;
- exposes command count and timeout state for tests.

`RokaeXCoreDriver`:

- imports without loading xCoreSDK;
- loads `libanydex_rokae_bridge.so` only from `connect()`;
- sends contiguous row-major `float64[16]` targets through `ctypes`;
- supports `xmate-6`, `xmate-er-pro-7`, and `standard-6`.

The C++ bridge:

- is a small C ABI wrapper, not a copy of the SDK;
- links the official SDK shared library and headers in their original repository;
- uses atomic `shared_ptr<const std::array<double, 16>>` replacement for latest-value semantics;
- keeps Python/Quest work out of the 1 ms xCoreSDK callback;
- initializes the first target from measured `tcpPose_m` so starting the callback does not jump to a Quest absolute pose.

## 5. Quest → Rokae Pose Mapping

No Euler-angle subtraction is used. Quest wrist and robot targets remain rigid transforms.

At enable/reference time:

```text
T_vr_ref       = current Quest wrist pose
T_base_tcp_ref = current measured ROKAE TCP pose
T_world_tcp_ref = T_world_base @ T_base_tcp_ref
```

For each new Quest wrist frame:

```text
Delta_T_vr = inverse(T_vr_ref) @ T_vr_current
```

Let `C` be the configured proper rotation that maps Quest-reference incremental axes to robot-reference incremental axes:

```text
p_mapped = translation_scale * C @ p_delta_vr
R_mapped = C @ R_delta_vr @ C.T
R_mapped_scaled = Exp(rotation_scale * Log(R_mapped))
```

Then:

```text
T_world_tcp_target = T_world_tcp_ref @ Delta_T_mapped
T_base_tcp_target  = inverse(T_world_base) @ T_world_tcp_target
```

The result passed to xCoreSDK is an absolute base-frame target even though the human control input is reference-relative.

## 6. World / Base Transform Convention

The single mounting transform is configured in `example/config/rokae_teleop.yaml`:

```yaml
robot:
  base_transform:
    pose_xyzw: [X, Y, Z, QX, QY, QZ, QW]
```

Definition:

```text
T_world_base = pose of the ROKAE base in the shared world frame
T_base_world = inverse(T_world_base)
```

All world targets are converted in one place:

```text
T_base_tcp_target = T_base_world @ T_world_tcp_target
```

Upright, inverted, and tilted mounting use this transform. No mounting-specific `x=-y` or axis-sign logic exists in the driver.

The separate `quest_to_robot_rotation` is a proper rotation used only to define how Quest-relative control axes map to robot-reference incremental axes. It must be orthonormal with determinant `+1`.

## 7. Safety Limits

The Python safety path applies, in order:

1. translation and SO(3) rotation scaling;
2. SE(3) low-pass filtering (`low_pass_alpha`);
3. maximum Cartesian translation speed;
4. maximum Cartesian angular speed;
5. maximum translation delta per Quest frame;
6. maximum rotation delta per Quest frame;
7. base-frame Cartesian workspace clipping;
8. latest-target publication.

Additional state logic:

- arm enable/disable with hold;
- `SIGUSR1` re-center while preserving the last safe target;
- `SIGUSR2` arm enable toggle;
- Quest timeout enters `hold_timeout`;
- the first resumed sample after timeout is automatically re-referenced, preventing a catch-up jump;
- an initial measured TCP outside the configured workspace is rejected instead of being snapped to a boundary;
- real arm motion starts disabled unless `--arm-start-enabled` is explicitly supplied;
- power-on is never implicit and requires `--rokae-power-on`.

The software workspace is an additional limit, not a replacement for controller safety configuration, collision detection, hardware emergency stop, or model-specific limits.

## 8. Mock / Simulation Mode

Mock mode exercises the same mapper, world/base transform, safety limiter, watchdog, enable state, latest-value buffer, and output call used by the real backend:

```bash
python example/teleop_arm_hand.py \
  --enable-arm \
  --robot rokae \
  --mock-rokae \
  --quest3-port 9000
```

It neither loads xCoreSDK nor opens a ROKAE network connection.

The standalone numerical demo generates an 8-second Quest trajectory:

- 0–2 s: +X translation;
- 2–4 s: +Y translation;
- 4–6 s: +Z rotation;
- 6–8 s: combined XYZ and rotation.

It prints VR position, VR-relative translation, world target, raw base target, and filtered/limited base target. It can also write CSV.

## 9. Numerical Tests

Run:

```bash
python -m unittest discover -s tests -v
```

Result: 15 tests passed.

Covered cases:

- Test A: zero VR motion returns the robot reference;
- Test B: Quest X/Y/Z translation, axis permutation, and scaling;
- Test C: Rx/Ry/Rz and non-identity reference rotations through SO(3);
- Test D: upright `T_world_base = I`;
- Test E: upside-down base with 180-degree rotation;
- Test F: tilted base with yaw 30 degrees and pitch 20 degrees;
- Test G: 0.5 m translation jump is speed/delta limited;
- Test H: 180-degree rotation jump is angularly limited;
- Test I: low-pass filter reduces noisy trajectory variance;
- Test J: timeout holds and resumed tracking re-centers;
- workspace clipping and invalid initial workspace rejection;
- enable/disable hold and no-jump resume;
- Mock latest-value replacement and internal timeout hold.

All three standalone simulations passed:

```bash
python example/test_rokae_teleop_sim.py --base normal --csv /tmp/normal.csv
python example/test_rokae_teleop_sim.py --base upside-down --csv /tmp/upside.csv
python example/test_rokae_teleop_sim.py --base tilted --csv /tmp/tilted.csv
```

Each simulation generated 401 samples plus one CSV header at `dt=0.02 s`.

An end-to-end Mock integration test also passed using the real `teleop_arm_hand.py`, a local UDP Quest wrist stream, 80 input frames, target motion, timeout transition to `hold_timeout`, and SIGINT cleanup.

## 10. Build Result

The current environment has no `pybind11`, so the wrapper deliberately uses C++17 C ABI + Python `ctypes`; it adds no Python build dependency.

Build command:

```bash
bash example/output/real/rokae_cpp/build_bridge.sh
```

Build result:

```text
Architecture: aarch64
Compiler: GNU g++ 9.4.0
CMake: 3.16.3
Output: example/output/real/rokae_cpp/build/libanydex_rokae_bridge.so
Vendor library: libxCoreSDK.so.0.3.4
Result: compiled and linked successfully
```

`readelf` and `ldd` confirmed that the bridge RUNPATH resolves to:

```text
/SSD-512G/Project/rokae-cpp/xCoreSDK-CPP/lib/Linux/cpp/aarch64
```

The bridge was loaded through `ctypes`, and an `xmate-er-pro-7` session object was created/destroyed without making a robot connection.

Python syntax/import checks and Black formatting checks passed. xCoreSDK network/control execution was not attempted because no robot was required for this phase.

## 11. How to Run Without Hardware

Run all numerical tests:

```bash
cd /SSD-512G/Project/AnyDexRetarget
conda activate /SSD-512G/conda_envs/anydex
export LD_LIBRARY_PATH=/SSD-512G/conda_envs/anydex/lib
python -m unittest discover -s tests -v
```

Run the standalone simulation:

```bash
python example/test_rokae_teleop_sim.py \
  --base tilted \
  --csv /tmp/rokae_teleop_tilted.csv
```

Run Quest with Mock ROKAE, arm only:

```bash
python example/teleop_arm_hand.py \
  --enable-arm \
  --robot rokae \
  --mock-rokae \
  --quest3-port 9000 \
  --quest3-protocol udp \
  --arm-record /tmp/rokae_targets.jsonl
```

Run Quest with Mock ROKAE and real Inspire Hand:

```bash
python example/teleop_arm_hand.py \
  --enable-hand \
  --enable-arm \
  --robot rokae \
  --mock-rokae \
  --inspire-port /dev/ttyUSB0 \
  --quest3-port 9000
```

Run Inspire only without reading the ROKAE config or loading its bridge:

```bash
python example/teleop_arm_hand.py \
  --enable-hand \
  --inspire-port /dev/ttyUSB0 \
  --quest3-port 9000
```

## 12. How to Run With Hardware Later

First edit `example/config/rokae_teleop.yaml`:

- exact robot class: `xmate-6`, `xmate-er-pro-7`, or `standard-6`;
- robot IP;
- Jetson wired-interface IP;
- measured `T_world_base` mounting extrinsic;
- Quest axis mapping;
- conservative translation/rotation gains;
- a base-frame workspace that contains the current TCP.

Build the bridge:

```bash
bash example/output/real/rokae_cpp/build_bridge.sh
```

After the robot type, xCore version, SDK license, network, TCP/tool frame, safety area, and emergency-stop procedure are confirmed, start conservatively:

```bash
python example/teleop_arm_hand.py \
  --enable-arm \
  --robot rokae \
  --rokae-robot-type xmate-er-pro-7 \
  --rokae-robot-ip 192.168.0.160 \
  --rokae-local-ip 192.168.0.100 \
  --rokae-power-on \
  --arm-start-enabled \
  --arm-translation-scale 0.2 \
  --arm-rotation-scale 0.3 \
  --arm-max-translation-speed 0.05 \
  --arm-max-angular-speed 0.20
```

Omit `--rokae-power-on` if power is managed and verified externally. Omit `--arm-start-enabled` to start the realtime backend in hold-disabled state, then use the printed `SIGUSR2` command when the operator is ready.

## 13. Remaining Risks

- The physical ROKAE model is not yet specified; selecting the wrong SDK class is rejected by the controller or may prevent connection.
- The SDK requires xCore v2.1.0.15+ and an enabled xCoreSDK license.
- No real robot network, power, mode-switch, state-stream, Cartesian callback, emergency-stop, or disconnect behavior has been exercised.
- `T_world_base`, Quest axis mapping, TCP/end-effector frame, and units require physical calibration.
- Software Cartesian limits do not check joint limits, inverse-kinematic feasibility, singularities, self-collision, environment collision, or attached-hand geometry.
- The wrapper does not yet configure the SDK-side `setCartesianLimit`, collision behavior, or `setFilterLimit`; these must be chosen for the exact ROKAE model and site safety policy.
- Python-level watchdog causes a Cartesian hold. A completely stalled/dead Python process leaves the C++ position callback holding its last target until process teardown or controller/network safety stops it.
- Real-time performance requires a dedicated wired interface; Wi-Fi is unsuitable for the xCoreSDK realtime leg.
- The SDK runtime was built against the system C++ ABI. The ROKAE bridge should continue using the tested system compiler/runtime, while Python/Pinocchio may require the Conda `LD_LIBRARY_PATH` used by this project.

## 14. Next Step

At the robot, confirm in this order:

1. exact ROKAE model and corresponding wrapper class;
2. controller xCore version and xCoreSDK authorization;
3. robot IP, Jetson wired IP, subnet, and packet-loss tolerance;
4. configured TCP is the Inspire Hand control point expected by `tcpPose_m`;
5. `T_world_base` from physical mounting calibration;
6. Quest-to-robot control-axis rotation with the robot held disabled;
7. controller-side Cartesian safety area and collision settings;
8. emergency stop, deadman behavior, timeout hold, and SDK disconnect response;
9. low-speed empty-workspace trial with conservative gains;
10. only then enable combined Inspire + ROKAE operation and increase limits gradually.

The software architecture, Mock backend, coordinate chain, safety path, C++ build, and numerical validation are complete. The next phase is hardware-specific calibration and controlled acceptance testing, not an architectural rewrite.
