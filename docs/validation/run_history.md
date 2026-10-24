# 项目修改与运行验证汇总

整理日期：2026-10-24。按要求将运行日志与参数试验结果归纳到本文，并删除原始 CSV、console、manifest、smoke 输出、根目录 logs/ 和独立 JSON 审查结果。以下历史结果不是当前参数的性能承诺。

## 当前配置与结论

- SDK 0.3.4，控制器 2.3.2，xMatePro3；不混用新版头文件与旧运行库。
- 关节传感器零点标定：控制器于 2026-10-24 15:57:35 记录 10006 成功。偏置值和精度未独立回读。
- 最新截图负载：质量约 0.96 kg，重心 [-51.43,8.23,8.40] mm。启用 SDK Toolset 结构体写入并读回，再同步 RT setLoad；保留 TCP/ref/惯量，不依赖 RL 工具名。
- 旧几何换轴、减质量以及跳过 RT setLoad 未构成可靠修复，不作为当前标定。
- hold_g2qx602c：5.00049 秒、5029 样本，定时完成并确认下电；最大偏移约 1.259 mm，最后一秒各轴峰峰值 <0.002 mm。仅验证该姿态的短时无触碰保持。
- hold_8dr6_xzi：19.395 秒触发 J3 行程 0.500062 rad；TCP 最终约 8.16 mm、最大 38.48 mm，最大关节速度约 0.3833 rad/s。用户确认仍在接触推动，不能据此判断松手回弹。
- 当前轻推阶段：关节行程 1.0 rad、速度 0.70 rad/s、TCP 位移 100 mm；启动关节偏移 0.04 rad。最新行程边界尚未硬件验证。
- 未验收：长时间静止、各方向轻推松手回弹、多姿态、接触遥操作及策略部署；零惯量仍未验证。

## 最近已执行的软件验证

- dexteleop 环境：285 项 pytest、9 项子测试通过；5/5 CTest 通过。
- 项目检查：191 个 Python 文件、42 个规范模块、194 个本地 Markdown 引用，无错误（清理前一次检查）。清理后再次执行相同检查，全部通过。
- 这些是软件检查结果，不替代硬件稳定性与刚度/阻尼验收。

## 独立试验运行索引

| 运行 | 停止原因 | 样本数 | 确认下电 |
|---|---|---:|---|
| hold_1bci3ms9 | startup_joint_drift | 291 | 是 |
| hold_1jjub90t | joint_speed | 114 | 是 |
| hold_1llhm5dr | startup_joint_drift | 275 | 是 |
| hold_1tfrkqhp | tcp_displacement | 24188 | 是 |
| hold_2vxrak3b | duration_completed | 30317 | 是 |
| hold_414zdzvg | startup_joint_drift | 272 | 是 |
| hold_529f8y0a | tcp_displacement | 10558 | 是 |
| hold_5ofpbhvm | joint_excursion | 22603 | 是 |
| hold_6d4c0v69 | tcp_displacement | 16565 | 是 |
| hold_6l1300us | startup_joint_drift | 305 | 是 |
| hold_7_nfvc_u | 未记录 | — | 未记录 |
| hold_81v3kce7 | startup_joint_drift | 274 | 是 |
| hold_8dr6_xzi | joint_excursion | 19536 | 是 |
| hold_9dden5nv | joint_speed | 13138 | 是 |
| hold_a8xmdp02 | duration_completed | 5034 | 是 |
| hold_ahk_5aty | joint_speed | 28302 | 是 |
| hold_ayth64bd | startup_joint_drift | 274 | 是 |
| hold_b__ykvxt | duration_completed | 5049 | 是 |
| hold_e34cng2c | joint_excursion | 7743 | 是 |
| hold_ehh9mfez | startup_joint_drift | 340 | 是 |
| hold_enrw1sbx | joint_speed | 7179 | 是 |
| hold_exu8x0sk | joint_speed | 17291 | 是 |
| hold_fqxo6uro | joint_speed | 11551 | 是 |
| hold_g2qx602c | duration_completed | 5029 | 是 |
| hold_g8jxfi8v | startup_joint_drift | 299 | 是 |
| hold_gstwpzxq | tcp_displacement | 15598 | 是 |
| hold_i_r1utbu | startup_joint_drift | 300 | 是 |
| hold_izsi2veb | joint_speed | 9405 | 是 |
| hold_j4rq31e_ | joint_speed | 21771 | 是 |
| hold_khm17w8f | startup_joint_drift | 300 | 是 |
| hold_kshpfdab | startup_joint_drift | 275 | 是 |
| hold_l0pwt3yr | duration_completed | 5055 | 是 |
| hold_letxbxvg | startup_joint_drift | 349 | 是 |
| hold_lvb_n9ou | tcp_displacement | 18241 | 是 |
| hold_mjqcaofa | startup_joint_drift | 320 | 是 |
| hold_njl46eko | startup_joint_drift | 296 | 是 |
| hold_o0p5ys90 | duration_completed | 5061 | 是 |
| hold_p03k9qz4 | startup_joint_drift | 371 | 是 |
| hold_pbbkqpew | 未记录 | — | 未记录 |
| hold_qj630rs6 | startup_joint_drift | 315 | 是 |
| hold_qon2i7b4 | joint_excursion | 15537 | 是 |
| hold_rl_6j3l1 | startup_joint_drift | 288 | 是 |
| hold_tl_ftst0 | startup_joint_drift | 269 | 是 |
| hold_tqbwgo5x | startup_joint_drift | 274 | 是 |
| hold_u16vt18r | duration_completed | 5052 | 是 |
| hold_uw43oags | tcp_displacement | 9766 | 是 |
| hold_wdz2jw6e | startup_joint_drift | 272 | 是 |
| hold_xlgsd92z | user_interrupt | 39505 | 是 |
| hold_ysb3lesz | startup_joint_drift | 304 | 是 |
| hold_z1bay73r | joint_excursion | 19703 | 是 |
| hold_zlgonasd | startup_joint_drift | 292 | 是 |
| home_fade2j2x | 未记录 | — | 是 |
| home_lndhjo30 | 未记录 | — | 是 |
| inspect_3qt3y_ix | 未记录 | — | 未记录 |
| inspect_cteoy1s9 | 未记录 | — | 未记录 |
| inspect_ifh4zq_e | 未记录 | — | 未记录 |
| prepare_yj_qmjzq | 未记录 | — | 是 |

## 已合并的审查结果索引

下列文件是历史结果来源名称，已删除；保留主题和简要结论用于追溯，不再链接不存在的原始结果。详细接口说明保留在 rokae/docs/。

### fixed_impedance_20261023

- pytest：{"passed": 237, "subtests_passed": 9, "seconds": 7.78}
- remaining：["Previous uncontrolled motion root cause unresolved", "Inertia remains zero/unverified", "No hardware validation of stopping/servo behavior", "Not an automatic readiness or safety approval"]

### frame_readback_20261023

- source：data/diagnostics/frame_readback_20261023_153315/result.json
- relation_translation_error_mm：0.00021664796096825374
- relation_rotation_error_deg：1.983990668017933e-05
- max_joint_change_during_reads_rad：2.3968449813782655e-07

### hold_1bci3ms9_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification

### hold_1jjub90t_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification
- conclusion：Unexpected motion with a constant reference. Software joint-speed guard stopped the test and requested power-off; root cause remains unconfirmed.

### hold_529f8y0a_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification

### hold_enrw1sbx_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification

### hold_izsi2veb_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification

### hold_letxbxvg_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification

### hold_o0p5ys90_analysis

- source：Saved runs only; no robot connection or motion performed
- user_confirmation：Full mounted assembly unchanged; only configured mass reduced
- current_global_config_not_overwritten：true

### hold_p03k9qz4_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification

### hold_qj630rs6_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification

### hold_ysb3lesz_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification

### hold_zlgonasd_analysis

- scope：Callback samples only; not controller servo timing, a stiffness measurement, or safety certification

### ik_audit_20261022

- scope：offline same fitted model; excludes RT filtering, robot tracking, calibration and official IK

### ik_review_entrypoint_20261022

- synthetic_cameras：true

### impedance_axis_spring_audit_20261023

- scope：Offline mathematics only; no firmware, actual damping, inertia, friction or stability validation

### impedance_entrypoint_20261022

- synthetic_cameras：true

### impedance_faults_20261023

- read_only：true
- query：latest 10 warning/error logs; not a complete controller archive

### impedance_history_20261024

- scope：Historical callback traces; no binary identity, exact hand release times or independent force ground truth in these trials. Not all pairs are controlled experiments.
- count：39

### impedance_home_baseline_20261024

- user_observation：home stationary: entire end assembly free in air, no touch or obvious cable tension

### impedance_mass_direction_audit_20261024

- scope：Offline comparison of eight saved experiments; no controller connection, motion or parameter changes

### impedance_postcalibration_hold_20261024

- conclusion：5s completion; ~1.26 mm offset, not 1cm; last second position near stationary. No external push/release evidence. SDK pos_c is identity so it is not useful command-execution confirmation in this run.

### impedance_repeated_drift_20261024

- scope：Offline saved-trace analysis; no robot connection or parameter change

### impedance_rereview_20261022

- synthetic_cameras：true

### impedance_return_audit_20261024

记录主题：hold_2vxrak3b、hold_tqbwgo5x、hold_1tfrkqhp、interpretation、offline_validation。

### impedance_rt_load_jump_20261024

- scope：Sequential controller-reported estimates; measured-minus-external is arithmetic, not independently read gravity torque. No impedance was started. Does not prove internal torque double compensation or physical payload validity.

### impedance_skip_rt_20261024

记录主题：trace、force_snapshots、gravity_geometry。

### impedance_startup_backport_20261023

- scope：Offline build/tests and review; no robot connection, power-on or motion performed
- checks：{"pytest_full": "261 passed, 9 subtests passed", "pytest_payload_startup_audit_after_config_change": "40 passed", "ctest": "3/3 passed", "project_check": {"python_files": 191, "canonical_modules": 42, "markdown_links": 181, "errors": []}, "git_diff_check": "passed"}

### impedance_subscription_fix_20261023

- conclusion：The added drain-until-empty precondition rejected startup. Old log cannot distinguish real backlog from a continuously serviced stream or receiver timing; not evidence of impedance drift.

### layout_capture_20261022

- synthetic_cameras：true

### layout_teleop_20261022

- teleop_only：true
- startup_pause_enable_pause_home_quit：true

### official_sdk_download_20261023

- date：2026-10-23
- runtime_in_use：rokae/sdk/lib/x86_64/libxCoreSDK.so.0.3.4
- controller_reported：2.3.2
- latest_repository_minimum_controller：3.2.1

### payload_axial_52mm_review_20261023

- remaining：["physical CoM hypothesis not measured", "controller inertia unverified", "successful RL loaded tool discrepancy unresolved", "firmware dynamics and transition behaviour not validated offline"]

### payload_candidate_review_20261023

- mass_kg：0.93938639463
- candidate_is_applied：false
- axis_relabel_determinant：-1.0
- axis_relabel_note：As an axis relabeling this reverses handedness, not a valid rotation between right-handed frames. A single vector could admit other rotations; none has been measured here.

### payload_coordinate_audit_20261023

- trial：data/diagnostics/impedance_test/hold_1jjub90t
- screenshot_to_config_axis_permutation：false
- screenshot_to_config_sign_change：false
- conversion：millimetres divided by 1000; no mounting-photo rotation applied

### payload_geometry_review_20261023

- conclusion：Original flange-frame CoM has a strong geometric inconsistency with the visible assembly. Axial Z dominance is plausible; an exact axis permutation, signs and z=52.71 mm are not established.

### refactor_20261022

- pytest：{"full": {"command": "python -m pytest -q -r s", "passed_items": 122, "passed_subtests": 9, "skipped": 10, "failed": 1, "deselected": 0, "junit_tests_including_subtests": 142, "duration_seconds": 7.41}, "available": {"command": "python -m pytest -q -k 'not init_nrt_speed_is_independent_and_conservative'", "passed_items": 122, "passed_subtests": 9, "skipped": 10, "failed": 0, "deselected": 1, "junit_tests_including_subtests": 141, "duration_seconds": 7.219}}
- unverified：["physical Quest tracking", "physical arm/hand control, feedback and home/cancel latency", "native realtime tests", "long-running capture plus GPU inference load", "hardware exposure sync/depth/IMU", "OpenPI training and autonomous physical deployment"]

### rokae_recalibration_audit_20261024

- checks：{"pytest": "75 passed", "ctest": "5/5 passed"}
- remaining：Saved HMI tools vs compiled RL definitions/SDK active Toolset mismatch unresolved; full precision mass and calibrated TCP not available.

### rokae_x86_20261022

- date：2026-10-22
- python：/data/miniconda3/envs/dexteleop/bin/python
- architecture：x86_64
- module：/data/Projects/DexRetarget/rokae/python/_rokae_cpp.cpython-310-x86_64-linux-gnu.so

### sdk_update_drift_comparison_20261023

- scope：Offline recorded trajectories and ideal rigid-payload gravity calculation; not a controller dynamics verification
