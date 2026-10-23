> 历史记录：只对应当时的机器与版本，不代表当前验证结果。当前状态见[重构与验证报告](../../docs/refactor_report.md)。

# ROKAE controller review

Reviewed after the aarch64 build and the offline tests, not only from the first compile.

## Architecture

- Quest publishes `EECommand`. `PositionController` or `ImpedanceController` owns IK, joint streaming, Cartesian position, or Cartesian impedance.
- The old `_publish_arm_target` branch in `teleop_arm_hand.py` was removed after the controller took that job, so the two paths cannot drift.
- Cup and box build `PathSegment`s and call `controller.follow`. Position follow is still MoveL. Impedance follow streams equilibria.
- `control.mode: pinocchio` with no `control.controller` still resolves to the position controller. Existing YAML behavior stays.

## Realtime

- The 1 kHz callback does not call Python.
- The success path of the impedance clip uses stack `Pose` values. It does not log or allocate.
- The callback's exception path still allocates an error string and sends `setFinished`. That path already existed for Cartesian position.
- The latest-command sample is the existing atomic publish. Python `set_target` also holds a mutex so teleop threads do not interleave the limiter and the sequence update.
- `get_tcp_pose` for the Python limiter runs at the Quest/task rate, outside the 1 kHz loop.

## Mathematics

- Equilibrium rotation uses the matrix logarithm, angle in `[0, pi]`. A 350 degree yaw is the same matrix as −10 degrees, so the clip follows the short arc.
- C++ and Python clips matched within 1e-6 on a pure translation and on a combined translation/rotation in `SharedRealtimeCoreTest`.
- Stiffness sign is the SDK factor, not a Franka `tau = J^T K e` law. The offline plant uses `F = K e - D v` with `e = equilibrium - position`, so a positive X error produces a positive X force. That plant is not the robot law.
- Units in `EECommand` are metres, radians, monotonic seconds.

## ARM64

- Built module is ELF AArch64.
- `ldd` resolves `libxCoreSDK.so.0.3.4` to `rokae/sdk/lib/aarch64/`. No x86_64 library is on that link line.
- `xMateModel` is absent. Torque impedance was not stubbed in.

## Safety

- Impedance gains are validated against the header maxima.
- Equilibrium error is limited in Python and again in the callback.
- Watchdog finish was checked on `RealtimeCore`, which impedance reuses.
- Controller switch while realtime is running is rejected.
- Collision thresholds were not increased.

## Fixes during this pass

- Dropped the unused teleop publisher so Quest cannot bypass the controller.
- Added the rotation case to the C++/Python limiter test after the translation case passed.

## Remaining risks

- `setFcCoor(world, identity)` is from the header, not from a hardware trace. If the force axes look tool-fixed on the robot, this matrix is the first thing to change.
- Stiffness `[300, 300, 300, 15, 15, 15]` is a placeholder. Too high will buzz; too low will not hold the cup.
- SDK collision detection does not run in force control. Do not treat impedance as having the same contact stop as position mode.
- At exactly 180 degrees the rotation axis is ambiguous. Normal teleop motion stays below the 0.30 rad limit, so this is not the operating region.
- Cup/box impedance completion uses 15 mm and about 4.6 degrees. A soft spring can hit the timeout before the TCP reaches the waypoint. The script prints the residual and continues.
- No hardware run was done.
