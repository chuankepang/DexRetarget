# ROKAE teleoperation implementation note

## Selected design

The application remains Python, while every xCoreSDK call and the fixed 1 kHz
Cartesian servo callback execute in C++. The binding is pybind11 and the only
vendor runtime is xCoreSDK-CPP 0.3.4, selected for the physical xCore 2.3.2
controller and Orin aarch64 host.

```text
Quest UDP (typically 60/72/90 Hz)
  -> Python RelativePoseMapper: spatial world deltas
       dp = p_current - p_reference
       dR = R_current @ R_reference.T
  -> T_world_tcp target -> inv(T_world_base) @ T_world_tcp
  -> pybind set_target_pose(matrix, timestamp, sequence)
  -> C++ atomic latest-value slot (no FIFO)
  -> C++ filter/limiter/watchdog
  -> xCoreSDK setControlLoop<CartesianPosition> (1 ms)
```

The approach follows the control-boundary pattern used by libfranka control
callbacks, UR reverse/servo interfaces and Kinova cyclic control: the network
and policy producer is asynchronous, the robot consumer is fixed-rate, old
commands are never replayed, and a watchdog owns dropout behavior.

## Timing behavior

- duplicate or decreasing sequence: reject;
- decreasing source timestamp: reject;
- source age beyond `max_source_age`: reject;
- age below `hold_timeout`: track newest target;
- age between hold and stop timeout: smoothly hold the last generated pose;
- age beyond `stop_timeout`: return a finished command and stop realtime motion;
- new input after explicit hold clears hold and replaces the slot;
- missed packets never produce a FIFO backlog.

Translation uses a continuous-time first-order low-pass coefficient derived
from actual callback `dt`. Rotation uses unit quaternions and SLERP; Euler angle
subtraction/averaging is not used. Velocity, acceleration and jerk are limited
independently for translation and SO(3) rotation.

## Coordinate frames

The current upright mount is represented once in YAML:

```text
R_world_base = I
T_world_base = [I, p_world_base; 0 0 0 1]
T_base_world = inverse(T_world_base)
T_base_tcp_target = T_base_world @ T_world_tcp_target
```

The rotation is known. The YAML XYZ values remain translation-calibration
placeholders unless the user defines the world origin at the robot base. No
scattered Y/Z sign flips are used; only `T_world_base` changes with the cell.

This project calibration is distinct from xCore's controller-side `baseFrame()`.
After changing the physical installation from inverted to upright, Robot Assist must
also report an upright installation/base orientation and use the correct load model;
changing this YAML alone does not update gravity compensation.

## Verification boundary

The C++ module compiles and imports on the Orin. The C++ realtime core is
covered by deterministic latest-value, stale-packet, watchdog, translation,
rotation, speed and acceleration tests, and by a 60/72/90 Hz Quest versus
1 kHz robot simulation with jitter, packet loss, delay spikes and an explicit
blocked-Python-producer interval.

Read-only connectivity of SDK 0.3.4 to controller 2.3.2 has been verified.
Joint motion, MoveL, the realtime callback, workspace, installed TCP/tool frame,
packet-loss tolerance and physical stop response still require the ordered
low-speed hardware acceptance described in `README_QUEST3_INSPIRE_ROKAE.md`.
