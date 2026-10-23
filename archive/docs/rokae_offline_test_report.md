> 历史记录：只对应当时的机器与版本，不代表当前验证结果。当前状态见[重构与验证报告](../../docs/refactor_report.md)。

# ROKAE offline test report

No robot connection. No `--execute` motion.

## Environment

- Machine: NVIDIA AGX Orin, `uname -m` = `aarch64`
- Python: `/SSD-512G/conda_envs/anydex/bin/python` (3.10)
- SDK: xCoreSDK-CPP 0.3.4, vendored `libxCoreSDK.so.0.3.4` identical to `xCoreSDK-CPP-0.3.4/lib/Linux/cpp/aarch64/libxCoreSDK.so.0.3.4`
- Build: `bash rokae/build.sh` with that Python. CMake configured and linked `_rokae_cpp.cpython-310-aarch64-linux-gnu.so`.
- Module: ELF 64-bit AArch64. `ldd` shows `libxCoreSDK.so.0.3.4` from `rokae/sdk/lib/aarch64/`.

## SDK libraries

| File | Inspection |
| --- | --- |
| aarch64 `libxCoreSDK.so.0.3.4` | ELF AArch64. Exports `setCartesianImpedance`, `setJointImpedance`, `setCartesianImpedanceDesiredTorque` for DoF 6 and 7, plus `Torque` send symbols. |
| aarch64 `libxCoreSDK.a` | Git LFS pointer, not used. |
| x86_64 `libxCoreSDK.so.0.3.4` | ELF x86-64. Not linked. |
| x86_64 `libxMateModel.a` | Git LFS pointer. No aarch64 counterpart. |
| aarch64 `.so` `xMateModel` / Jacobian | No symbols. |

## Tests

Command:

```
python -m unittest tests.test_rokae_controllers \
  tests.test_rokae_differential_ik \
  tests.test_rokae_realtime \
  tests.test_rokae_safety -v
```

Result: 38 tests, OK, about 29 s. A later rerun of `SharedRealtimeCoreTest` after adding the rotation comparison also passed.

| Test | Result |
| --- | --- |
| position config -> PositionController | pass |
| impedance config -> ImpedanceController | pass |
| CLI over task over YAML | pass |
| invalid controller / torque backend | pass, ValueError |
| translation clip, zero error | pass |
| rotation clip, quaternion sign | pass |
| NaN / Inf rejected | pass |
| rate limit first step <= 0.15 m/s * dt | pass |
| idle set_target rejected; start; clip; stop; set_target rejected | pass |
| four threads publishing | pass, all poses finite |
| mock spring force sign (offline plant only) | pass |
| RealtimeCore stale target sets should_finish | pass |
| C++ and Python limiter agree, translation and rotation | pass |
| existing differential IK, realtime, and safety tests | pass, 28 tests |

Also:

| Check | Result |
| --- | --- |
| `print_controller.py` | kind position, route pinocchio, stiffness 300/15, wrench 0 |
| `cup_flip.py` dry-run | exit 0, same 45 degree waypoints |
| `box_vertices.py` dry-run | exit 0 |
| `py_compile` teleop, cup, box, driver | OK |

## What the mock spring does not prove

`CompliantPlant` integrates `F = K e - D v` in Python. It checks gain plumbing and the sign of the spring. It does not reproduce xCore's Cartesian impedance, friction, or the robot's internal damping.

## Unresolved

No failure in the offline set. Hardware behavior of `cartesianImpedance`, the world force frame, and the placeholder stiffness is untested.
