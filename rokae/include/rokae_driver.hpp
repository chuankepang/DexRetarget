#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace anydex::rokae_bridge {

using Pose = std::array<double, 16>;

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

  void validate() const;
};

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
  std::string callback_error;
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
  Pose preview_nrt_cartesian_target(const Pose& base_T_tcp_target);

  void move_joint(const std::vector<double>& target, int speed,
                  double timeout, double max_joint_delta, bool power_on);
  void move_cartesian(const Pose& target, int speed, double timeout,
                      double max_translation_delta, double max_rotation_delta,
                      bool power_on);
  void move_to_init(const std::vector<double>& target, int speed,
                    double timeout, double max_joint_delta, bool power_on);

  void start_realtime_cartesian(bool power_on = true);
  bool set_target_pose(const Pose& target, double source_timestamp,
                       std::int64_t sequence);
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
