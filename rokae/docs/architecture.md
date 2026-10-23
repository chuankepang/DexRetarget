# ROKAE controller architecture

## Old architecture

Quest, cup, and box each talked to the driver in their own way.

```
Quest3 --wrist--> teleop_arm_hand.py
                     |  if pinocchio: host IK, set_target_joints
                     |  else:         set_target_pose
                     v
              RokaeXCoreDriver
                     |  jointPosition      (1 kHz C++ JointRealtimeCore)
                     |  cartesianPosition  (1 kHz C++ RealtimeCore)
                     v
              xCoreSDK-CPP 0.3.4

cup / box --> MoveAbsJ init, then MoveL waypoints
```

Both realtime routes already kept the 1 kHz loop in C++. Python only published the latest target. Non-realtime MoveL/MoveAbsJ stayed on the controller planner.

## New architecture

The high-level side publishes one end-effector command. A factory picks the low-level route. Quest, cup, and box do not branch on the SDK mode.

```
Quest3 / cup / box / later policy
            |
            v
       EECommand          base_T_tcp, monotonic time, sequence, source
            |
            v
   Reference conditioning
     C++ RealtimeCore: low-pass, velocity, acceleration, jerk, watchdog
     Python: equilibrium limiter, optional rate limit, workspace clip
            |
            v
     create_controller
        /            \
       v              v
 PositionController   ImpedanceController
       |                    |
       |                    v
       |            equilibrium pose
       |                    |
       v                    v
 cartesianPosition     cartesianImpedance
 or jointPosition      (native xCore)
       \                    /
        v                  v
              xCoreSDK 0.3.4
                    |
                    v
                  ROKAE
```

Reset is still joint position control. `move_to_init` runs MoveAbsJ before either realtime route starts. A running route is stopped before another one starts. The driver refuses to hot-switch Cartesian position and impedance.

## PositionController

`target_pose` is a position reference.

- `control.mode: pinocchio` (default): host differential IK, then `RtControllerMode::jointPosition`.
- `control.mode: cartesian`: `RtControllerMode::cartesianPosition`.
- Cup and box position execution still uses NRT MoveL. That path is unchanged.

## ImpedanceController

`target_pose` is the Cartesian impedance equilibrium, not a rigid pose demand.

`start` calls `RtControllerMode::cartesianImpedance` after `setCartesianImpedance`, `setCartesianImpedanceDesiredTorque`, and `setFcCoor(..., FrameType::world)`. The 1 kHz callback is the same pose smoother as Cartesian position, then `limit_equilibrium_pose` clips the sample toward the measured TCP before it is sent.

xCoreSDK 0.3.4 does not take a Cartesian damping argument. Damping stays inside the robot controller.

## Python / C++ boundary

```
Python producer (Quest rate or task rate)
    |  set_target_pose / set_target_joints
    v
atomic latest-command sample
    |
    v
C++ callback, 1 ms, no Python call
    |
    v
xCore startLoop
```

## Config flow

Highest precedence first:

1. CLI (`--arm-control`, `--controller`)
2. `tasks.<cup|box>.controller`
3. `control.controller`
4. position, with `control.mode` as the position backend

`pinocchio` and `cartesian` on the CLI select the position controller. `impedance` selects native Cartesian impedance. A file that only sets `control.mode` keeps the previous position behavior.

## State flow

Measured TCP and joints are stored from the 1 kHz state receiver with a sequence counter. Diagnostics report `realtime_route` as `cartesian`, `joint`, `cartesian_impedance`, or `idle`.

Stale commands use the existing watchdog in `RealtimeCore` and `JointRealtimeCore`:

- age under `hold_timeout` (0.15 s): track
- then hold the current command
- age at `stop_timeout` (0.60 s): finish the realtime motion

Position Cartesian and impedance share that watchdog because they share `RealtimeCore`.
