#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <string>
#include <vector>

namespace anydex::rokae_bridge {

using Pose = std::array<double, 16>;

struct ControllerLog {
  int id;
  std::string timestamp;
  std::string content;
  std::string repair;
};

// Maintenance only: at most one axis, no more than 1 degree outside its saved
// soft limit; clearance is 0.5..10 degrees inside. Never enlarges a limit.
std::vector<double> plan_disabled_soft_limit_recovery(
    const std::vector<double>& joints,
    const std::vector<std::array<double, 2>>& limits,
    double clearance_degrees = 0.5);

struct RealtimeConfig {
  // Applied only to non-realtime MoveAbsJ/MoveL trajectories.  xCoreSDK
  // multiplies the command speed by this online scale (0.01..1.0).
  double nrt_online_speed_scale{1.0};
  double translation_cutoff_hz{5.0};
  double rotation_cutoff_hz{5.0};
  double translation_deadband{0.001};
  double rotation_deadband{0.008};
  double max_translation_speed{0.15};
  double max_angular_speed{0.50};
  double max_translation_acceleration{0.40};
  double max_angular_acceleration{1.50};
  double max_translation_jerk{2.5};
  double max_angular_jerk{8.0};
  double max_target_translation_delta{0.25};
  double max_target_rotation_delta{1.0};
  std::array<double, 3> workspace_min{{-1.2, -1.2, -1.2}};
  std::array<double, 3> workspace_max{{1.2, 1.2, 1.2}};
  double hold_timeout{0.15};
  double stop_timeout{0.60};
  double max_source_age{0.20};
  double future_tolerance{0.05};
  bool controller_rate_limit{true};
  double controller_filter_cutoff_hz{30.0};
  unsigned rt_network_tolerance{20};
  // Cartesian impedance gains are sent only by the impedance route.
  // Stiffness units are N/m and Nm/rad. xCoreSDK 0.3.4 does not expose a
  // separate Cartesian damping argument. These defaults are conservative
  // placeholders below the documented ER Pro maxima, not a factory recipe.
  std::string impedance_force_frame{"world"};  // Legacy default; YAML selects tool/world explicitly.
  std::array<double, 6> cartesian_stiffness{{300.0, 300.0, 300.0, 15.0, 15.0, 15.0}};
  std::array<double, 6> cartesian_desired_wrench{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double max_equilibrium_translation_error{0.04};
  double max_equilibrium_rotation_error{0.30};

  // Zero disables matching for legacy callers; positive mass checks the active
  // controller tool before impedance power-on. Explicit opt-in applies mass/CoM
  // while preserving the controller TCP, reference frame and inertia.
  bool apply_load_on_impedance_start{false};
  double expected_load_mass{0.0};
  std::array<double, 3> expected_load_cog{{0.0, 0.0, 0.0}};
  void validate_impedance_load(double mass, const std::array<double, 3>& cog,
                               const std::array<double, 3>& inertia) const;
  void validate() const;
};

// Clip desired toward measured so the impedance equilibrium error stays inside
// the configured ball. Rotation uses the SO(3) logarithm, which is the short
// arc in [0, pi]. Position control does not call this.
Pose limit_equilibrium_pose(const Pose& desired, const Pose& measured,
                            double max_translation_error,
                            double max_rotation_error);

struct RobotState {
  bool connected{false};
  bool realtime_running{false};
  std::string sdk_version;
  std::string controller_version;
  std::string robot_type;
  int joint_count{0};
  std::string power_state;
  std::string operate_mode;
  std::string operation_state;
  std::vector<double> joint_position;
  std::vector<double> joint_velocity;
  Pose tcp_pose{};
  double timestamp{0.0};
};

struct Diagnostics {
  bool connected{false};
  bool realtime_running{false};
  std::string watchdog_state{"not_started"};
  std::uint64_t callback_count{0};
  std::uint64_t accepted{0};
  std::uint64_t rejected_sequence{0};
  std::uint64_t rejected_source_time{0};
  std::uint64_t rejected_source_age{0};
  std::uint64_t rejected_target_delta{0};
  std::int64_t latest_sequence{-1};
  double latest_command_age{0.0};
  std::array<double, 3> linear_velocity{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  std::array<double, 3> angular_acceleration{};
  std::vector<double> measured_joint_position;
  bool elbow_lock_enabled{false};
  double measured_elbow{0.0};
  double commanded_elbow{0.0};
  std::string callback_error;
  std::string realtime_route{"idle"};
  std::vector<double> commanded_joint_position;
  std::vector<double> commanded_joint_velocity;
};

struct JointRealtimeConfig {
  std::vector<double> max_velocity;
  std::vector<double> max_acceleration;
  std::vector<double> max_jerk;
  std::vector<double> position_min;
  std::vector<double> position_max;
  double reference_cutoff_hz{15.0};
  double deadband{1e-4};
  double max_target_delta{0.35};
  double hold_timeout{0.15};
  double stop_timeout{0.60};
  double max_source_age{0.20};
  double future_tolerance{0.05};

  std::size_t dof() const { return max_velocity.size(); }
  void validate() const;
};

class RealtimeCore {
 public:
  explicit RealtimeCore(const RealtimeConfig& config = {});
  ~RealtimeCore();
  RealtimeCore(RealtimeCore&&) noexcept;
  RealtimeCore& operator=(RealtimeCore&&) noexcept;
  RealtimeCore(const RealtimeCore&) = delete;
  RealtimeCore& operator=(const RealtimeCore&) = delete;

  void reset(const Pose& initial_pose, double timestamp);
  bool publish(const Pose& target, double source_timestamp,
               std::int64_t sequence, double receive_timestamp);
  Pose step(double timestamp);
  void hold();
  void request_stop();
  Diagnostics diagnostics(double timestamp) const;
  Pose command_pose() const;
  bool should_finish() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

class JointRealtimeCore {
 public:
  explicit JointRealtimeCore(const JointRealtimeConfig& config);
  ~JointRealtimeCore();
  JointRealtimeCore(JointRealtimeCore&&) noexcept;
  JointRealtimeCore& operator=(JointRealtimeCore&&) noexcept;
  JointRealtimeCore(const JointRealtimeCore&) = delete;
  JointRealtimeCore& operator=(const JointRealtimeCore&) = delete;

  void reset(const std::vector<double>& joints, double timestamp);
  bool publish(const std::vector<double>& target, double source_timestamp,
               std::int64_t sequence, double receive_timestamp);
  std::vector<double> step(double timestamp);
  void hold();
  void request_stop();
  std::vector<double> command_joints() const;
  Diagnostics diagnostics(double timestamp) const;
  bool should_finish() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

class RokaeDriver {
 public:
  RokaeDriver(std::string robot_ip, std::string local_ip,
              std::string robot_type = "xmate-er-pro-7",
              const RealtimeConfig& realtime_config = {});
  ~RokaeDriver();
  RokaeDriver(RokaeDriver&&) noexcept;
  RokaeDriver& operator=(RokaeDriver&&) noexcept;
  RokaeDriver(const RokaeDriver&) = delete;
  RokaeDriver& operator=(const RokaeDriver&) = delete;

  void connect();
  void disconnect();
  RobotState get_state();
  std::vector<double> get_joint_positions();
  Pose get_tcp_pose();
  Pose get_base_frame();
  Pose calculate_fk(const std::vector<double>& joints);
  std::pair<bool, std::vector<std::array<double, 2>>> get_soft_limits();
  std::vector<ControllerLog> get_controller_logs(unsigned count = 10, bool faults_only = false);
  Pose preview_nrt_cartesian_target(const Pose& base_T_tcp_target);

  void move_joint(const std::vector<double>& target, int speed,
                  double timeout, double max_joint_delta, bool power_on);
  void move_cartesian(const Pose& target, int speed, double timeout,
                      double max_translation_delta, double max_rotation_delta,
                      bool power_on);
  void move_to_init(const std::vector<double>& target, int speed,
                    double timeout, double max_joint_delta, bool power_on);
  void recover_from_joint_limits(const std::vector<double>& target, int speed,
                                 double timeout, double max_joint_delta);
  void recover_disabled_soft_limits(
      const std::vector<double>& expected_joints,
      const std::vector<std::array<double, 2>>& expected_limits,
      double clearance_degrees = 0.5,
      bool request_power_on = false,
      double jog_rate = 0.05,
      const std::function<void()>& check_cancel = {});

  void start_realtime_cartesian(bool power_on = true);
  void start_realtime_cartesian_impedance(bool power_on = true);
  void start_realtime_joint(const JointRealtimeConfig& config,
                            bool power_on = true);
  bool set_target_pose(const Pose& target, double source_timestamp,
                       std::int64_t sequence);
  bool set_target_joints(const std::vector<double>& target,
                         double source_timestamp, std::int64_t sequence);
  std::vector<double> get_command_joints() const;
  void hold();
  void stop_realtime();
  void stop();
  Pose get_command_pose() const;
  Diagnostics diagnostics() const;

  static double monotonic_time();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace anydex::rokae_bridge
