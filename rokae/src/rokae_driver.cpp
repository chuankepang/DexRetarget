#include "rokae_driver.hpp"
#include "realtime_startup.hpp"

#include <algorithm>
#include <any>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <type_traits>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rokae/robot.h"
#include "rokae/utility.h"

namespace anydex::rokae_bridge {
namespace {

using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;

double now_seconds() {
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

void check_ec(const std::error_code& ec, const char* operation) {
  if (ec) {
    throw std::runtime_error(std::string(operation) + ": " + ec.message());
  }
}

Eigen::Matrix4d pose_matrix(const Pose& pose) {
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      matrix(row, col) = pose[static_cast<std::size_t>(row * 4 + col)];
    }
  }
  return matrix;
}

Pose pose_array(const Eigen::Matrix4d& matrix) {
  Pose pose{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      pose[static_cast<std::size_t>(row * 4 + col)] = matrix(row, col);
    }
  }
  return pose;
}

void validate_pose(const Pose& pose, const char* name) {
  const auto matrix = pose_matrix(pose);
  if (!matrix.allFinite()) {
    throw std::invalid_argument(std::string(name) + " contains non-finite values");
  }
  if ((matrix.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-8) {
    throw std::invalid_argument(std::string(name) + " has an invalid bottom row");
  }
  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() >
          2e-4 ||
      std::abs(rotation.determinant() - 1.0) > 2e-4) {
    throw std::invalid_argument(std::string(name) + " rotation is not SO(3)");
  }
}

Eigen::Vector3d limited_norm(const Eigen::Vector3d& value, double maximum) {
  const double norm = value.norm();
  if (norm <= maximum || norm < 1e-15) return value;
  return value * (maximum / norm);
}

Eigen::Vector3d rotation_vector(const Eigen::Matrix3d& rotation) {
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1e-15) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.axis() * angle_axis.angle();
}

Eigen::Matrix3d rotation_from_vector(const Eigen::Vector3d& vector) {
  const double angle = vector.norm();
  if (angle < 1e-15) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, vector / angle).toRotationMatrix();
}

double rotation_distance(const Pose& lhs, const Pose& rhs) {
  // Force evaluation into owning matrices.  Eigen Block expressions created
  // from pose_matrix(...) would otherwise reference destroyed temporaries.
  const Eigen::Matrix3d left = pose_matrix(lhs).topLeftCorner<3, 3>();
  const Eigen::Matrix3d right = pose_matrix(rhs).topLeftCorner<3, 3>();
  return rotation_vector(left.transpose() * right).norm();
}

std::string to_string(rokae::PowerState state) {
  switch (state) {
    case rokae::PowerState::on: return "on";
    case rokae::PowerState::off: return "off";
    case rokae::PowerState::estop: return "estop";
    case rokae::PowerState::gstop: return "gstop";
    default: return "unknown";
  }
}

std::string to_string(rokae::OperateMode mode) {
  switch (mode) {
    case rokae::OperateMode::manual: return "manual";
    case rokae::OperateMode::automatic: return "automatic";
    default: return "unknown";
  }
}

std::string to_string(rokae::OperationState state) {
  switch (state) {
    case rokae::OperationState::idle: return "idle";
    case rokae::OperationState::jog: return "jog";
    case rokae::OperationState::rtControlling: return "rt_controlling";
    case rokae::OperationState::drag: return "drag";
    case rokae::OperationState::rlProgram: return "rl_program";
    case rokae::OperationState::demo: return "demo";
    case rokae::OperationState::dynamicIdentify: return "dynamic_identify";
    case rokae::OperationState::frictionIdentify: return "friction_identify";
    case rokae::OperationState::loadIdentify: return "load_identify";
    case rokae::OperationState::moving: return "moving";
    case rokae::OperationState::jogging: return "jogging";
    default: return "unknown";
  }
}

}  // namespace

std::vector<double> plan_disabled_soft_limit_recovery(
    const std::vector<double>& joints,
    const std::vector<std::array<double, 2>>& limits, double clearance_degrees) {
  if (joints.empty() || joints.size() != limits.size()) {
    throw std::invalid_argument("recovery joints/limits have incompatible sizes");
  }
  auto target = joints;
  unsigned affected = 0;
  if (!std::isfinite(clearance_degrees) || clearance_degrees < 0.5 || clearance_degrees > 10.0) {
    throw std::invalid_argument("recovery clearance must be 0.5..10 degrees");
  }
  const double clearance = clearance_degrees * kPi / 180.0;
  constexpr double max_outside = kPi / 180.0;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const double q = joints[i], lo = limits[i][0], hi = limits[i][1];
    if (!std::isfinite(q) || !std::isfinite(lo) || !std::isfinite(hi) ||
        hi - lo <= 2 * clearance) {
      throw std::invalid_argument("invalid recovery joints/soft limits");
    }
    if (q <= lo || q >= hi) {
      if (++affected > 1) {
        throw std::invalid_argument("recovery supports only one out-of-range joint");
      }
      if (q < lo - max_outside || q > hi + max_outside) {
        throw std::invalid_argument("joint exceeds soft limit by more than 1 degree");
      }
      target[i] = q <= lo ? lo + clearance : hi - clearance;
    }
  }
  return target;
}

void RealtimeConfig::validate_impedance_load(
    double mass, const std::array<double, 3>& cog,
    const std::array<double, 3>& inertia) const {
  validate();
  if (!std::isfinite(mass) || mass <= 0.0) {
    throw std::runtime_error("impedance refused before power-on: active controller load mass must be positive; save and select the calibrated RobotAssist tool");
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(cog[axis]) || !std::isfinite(inertia[axis]) || inertia[axis] < 0.0) {
      throw std::runtime_error("impedance refused before power-on: invalid controller load cog/inertia");
    }
  }
  // Allow displayed-value rounding (5 g, 1 mm per axis), not a second calibration.
  if (expected_load_mass > 0.0) {
    bool matches = std::abs(mass - expected_load_mass) <= 0.005;
    for (int axis = 0; axis < 3; ++axis) {
      matches = matches && std::abs(cog[axis] - expected_load_cog[axis]) <= 0.001;
    }
    if (!matches) {
      throw std::runtime_error("impedance refused before power-on: active controller load differs from robot.payload; save/select the calibrated tool in RobotAssist (cog is metres, not millimetres)");
    }
  }
}

void RealtimeConfig::validate() const {
  const double positives[] = {
      translation_cutoff_hz, rotation_cutoff_hz, max_translation_speed,
      max_angular_speed, max_translation_acceleration,
      max_angular_acceleration, max_translation_jerk, max_angular_jerk,
      max_target_translation_delta, max_target_rotation_delta,
      hold_timeout, stop_timeout, max_source_age,
      controller_filter_cutoff_hz};
  for (double value : positives) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("realtime positive limits must be finite and > 0");
    }
  }
  if (!std::isfinite(nrt_online_speed_scale) ||
      nrt_online_speed_scale < 0.01 || nrt_online_speed_scale > 1.0) {
    throw std::invalid_argument("nrt_online_speed_scale must be in [0.01, 1.0]");
  }
  if (!std::isfinite(translation_deadband) || translation_deadband < 0.0 ||
      !std::isfinite(rotation_deadband) || rotation_deadband < 0.0 ||
      !std::isfinite(future_tolerance) || future_tolerance < 0.0) {
    throw std::invalid_argument("deadbands/tolerance must be finite and non-negative");
  }
  if (stop_timeout <= hold_timeout) {
    throw std::invalid_argument("stop_timeout must exceed hold_timeout");
  }
  if (rt_network_tolerance > 100) {
    throw std::invalid_argument("rt_network_tolerance must be in [0, 100]");
  }
  if (!std::isfinite(expected_load_mass) || expected_load_mass < 0.0) {
    throw std::invalid_argument("expected_load_mass must be finite and non-negative");
  }
  if (apply_load_on_impedance_start && expected_load_mass <= 0.0) {
    throw std::invalid_argument("applying impedance load requires a positive calibrated mass");
  }
  for (double value : expected_load_cog) {
    if (!std::isfinite(value)) throw std::invalid_argument("expected_load_cog must be finite");
  }
  if (impedance_force_frame != "world" && impedance_force_frame != "tool") {
    throw std::invalid_argument("impedance_force_frame must be world or tool");
  }
  constexpr double kMaxStiffnessTranslation = 1500.0;
  constexpr double kMaxStiffnessRotation = 100.0;
  constexpr double kMaxWrenchForce = 60.0;
  constexpr double kMaxWrenchTorque = 30.0;
  for (int axis = 0; axis < 6; ++axis) {
    const double stiffness = cartesian_stiffness[static_cast<std::size_t>(axis)];
    const double maximum = axis < 3 ? kMaxStiffnessTranslation : kMaxStiffnessRotation;
    if (!std::isfinite(stiffness) || stiffness < 0.0 || stiffness > maximum) {
      throw std::invalid_argument(
          "cartesian stiffness must be within the xCoreSDK 0.3.4 cobot maxima");
    }
    const double wrench = cartesian_desired_wrench[static_cast<std::size_t>(axis)];
    const double wrench_limit = axis < 3 ? kMaxWrenchForce : kMaxWrenchTorque;
    if (!std::isfinite(wrench) || std::abs(wrench) > wrench_limit) {
      throw std::invalid_argument(
          "cartesian impedance wrench exceeds the xCoreSDK 0.3.4 range");
    }
  }
  if (!std::isfinite(max_equilibrium_translation_error) ||
      max_equilibrium_translation_error <= 0.0 ||
      !std::isfinite(max_equilibrium_rotation_error) ||
      max_equilibrium_rotation_error <= 0.0 ||
      max_equilibrium_rotation_error > kPi) {
    throw std::invalid_argument(
        "equilibrium error limits must be finite, translation > 0, rotation in (0, pi]");
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(workspace_min[axis]) ||
        !std::isfinite(workspace_max[axis]) ||
        workspace_min[axis] >= workspace_max[axis]) {
      throw std::invalid_argument("workspace bounds must be finite and ordered");
    }
  }
}

Pose limit_equilibrium_pose(const Pose& desired, const Pose& measured,
                            double max_translation_error,
                            double max_rotation_error) {
  validate_pose(desired, "equilibrium pose");
  validate_pose(measured, "measured pose");
  if (!std::isfinite(max_translation_error) || max_translation_error <= 0.0 ||
      !std::isfinite(max_rotation_error) || max_rotation_error <= 0.0 ||
      max_rotation_error > kPi) {
    throw std::invalid_argument(
        "equilibrium error limits must be finite, translation > 0, rotation in (0, pi]");
  }
  const Eigen::Matrix4d desired_matrix = pose_matrix(desired);
  const Eigen::Matrix4d measured_matrix = pose_matrix(measured);
  const Eigen::Vector3d translation = limited_norm(
      desired_matrix.topRightCorner<3, 1>() -
          measured_matrix.topRightCorner<3, 1>(),
      max_translation_error);
  const Eigen::Matrix3d relative =
      measured_matrix.topLeftCorner<3, 3>().transpose() *
      desired_matrix.topLeftCorner<3, 3>();
  const Eigen::Vector3d rotation =
      limited_norm(rotation_vector(relative), max_rotation_error);
  Eigen::Matrix4d limited = Eigen::Matrix4d::Identity();
  limited.topLeftCorner<3, 3>() =
      measured_matrix.topLeftCorner<3, 3>() * rotation_from_vector(rotation);
  limited.topRightCorner<3, 1>() =
      measured_matrix.topRightCorner<3, 1>() + translation;
  return pose_array(limited);
}

class RealtimeCore::Impl {
 public:
  struct Sample {
    Pose pose;
    double source_timestamp;
    double receive_timestamp;
    std::int64_t sequence;
  };

  struct Dynamics {
    std::array<double, 3> linear_velocity{};
    std::array<double, 3> angular_velocity{};
    std::array<double, 3> linear_acceleration{};
    std::array<double, 3> angular_acceleration{};
  };

  explicit Impl(RealtimeConfig value) : config(std::move(value)) {
    config.validate();
  }

  void reset(const Pose& initial, double timestamp) {
    validate_pose(initial, "initial realtime pose");
    if (!std::isfinite(timestamp)) throw std::invalid_argument("invalid reset timestamp");
    const auto matrix = pose_matrix(initial);
    const auto p = matrix.topRightCorner<3, 1>();
    for (int axis = 0; axis < 3; ++axis) {
      if (p[axis] < config.workspace_min[axis] ||
          p[axis] > config.workspace_max[axis]) {
        throw std::invalid_argument("initial pose is outside workspace");
      }
    }
    pose = matrix;
    filtered = matrix;
    linear_velocity.setZero();
    angular_velocity.setZero();
    linear_acceleration.setZero();
    angular_acceleration.setZero();
    last_step_time = timestamp;
    initialized = true;
    hold_requested.store(false);
    stop_requested.store(false);
    should_finish_flag.store(false);
    watchdog.store(0);
    const auto initial_sample = std::make_shared<const Sample>(
        Sample{initial, timestamp, timestamp, -1});
    std::atomic_store(&latest, initial_sample);
    std::atomic_store(&command, std::make_shared<const Pose>(initial));
    std::atomic_store(&dynamics, std::make_shared<const Dynamics>());
    latest_sequence.store(-1);
    latest_source_time = timestamp;
  }

  bool publish(const Pose& target, double source_timestamp,
               std::int64_t sequence, double receive_timestamp) {
    validate_pose(target, "realtime target pose");
    if (!std::isfinite(source_timestamp) || !std::isfinite(receive_timestamp)) {
      throw std::invalid_argument("target timestamps must be finite");
    }
    const double source_age = receive_timestamp - source_timestamp;
    std::lock_guard<std::mutex> guard(publish_mutex);
    if (source_age > config.max_source_age ||
        source_age < -config.future_tolerance) {
      rejected_source_age.fetch_add(1);
      return false;
    }
    const auto current_command = std::atomic_load(&command);
    if (current_command) {
      // Materialize both matrices and the vector.  Keeping Eigen blocks from
      // temporary pose_matrix() results creates dangling expressions and can
      // turn an identical target into a spurious large-delta rejection.
      const Eigen::Matrix4d target_matrix = pose_matrix(target);
      const Eigen::Matrix4d command_matrix = pose_matrix(*current_command);
      const Eigen::Vector3d translation =
          target_matrix.topRightCorner<3, 1>() -
          command_matrix.topRightCorner<3, 1>();
      if (translation.norm() > config.max_target_translation_delta ||
          rotation_distance(*current_command, target) >
              config.max_target_rotation_delta) {
        rejected_target_delta.fetch_add(1);
        return false;
      }
    }
    const auto previous = std::atomic_load(&latest);
    if (previous && sequence <= previous->sequence) {
      rejected_sequence.fetch_add(1);
      return false;
    }
    if (previous && source_timestamp < previous->source_timestamp) {
      rejected_source_time.fetch_add(1);
      return false;
    }
    std::atomic_store(&latest, std::make_shared<const Sample>(
                                   Sample{target, source_timestamp,
                                          receive_timestamp, sequence}));
    latest_sequence.store(sequence);
    latest_source_time = source_timestamp;
    accepted.fetch_add(1);
    hold_requested.store(false);
    return true;
  }

  static double alpha(double cutoff, double dt) {
    return 1.0 - std::exp(-2.0 * kPi * cutoff * dt);
  }

  static void bounded_state(const Eigen::Vector3d& error,
                            Eigen::Vector3d& velocity,
                            Eigen::Vector3d& acceleration, double dt,
                            double max_velocity, double max_acceleration,
                            double max_jerk, Eigen::Vector3d& step) {
    // Overdamped position servo.  Merely limiting acceleration/jerk while
    // aiming at error/dt has no braking term and creates a sustained limit
    // cycle around a fixed Cartesian target.
    constexpr double kDampingRatio = 1.25;
    const double natural_frequency = std::min(
        2.0 * max_acceleration / max_velocity, 0.5 / dt);
    const auto desired_acceleration = limited_norm(
        natural_frequency * natural_frequency * error -
            2.0 * kDampingRatio * natural_frequency * velocity,
        max_acceleration);
    const auto jerk =
        limited_norm((desired_acceleration - acceleration) / dt, max_jerk);
    acceleration = limited_norm(acceleration + jerk * dt, max_acceleration);
    velocity = limited_norm(velocity + acceleration * dt, max_velocity);
    step = velocity * dt;
  }

  Pose step(double timestamp) {
    if (!initialized) throw std::runtime_error("realtime core is not initialized");
    if (!std::isfinite(timestamp)) throw std::invalid_argument("invalid step timestamp");
    double dt = timestamp - last_step_time;
    dt = std::clamp(dt, 0.00025, 0.004);
    last_step_time = timestamp;

    const auto sample = std::atomic_load(&latest);
    const double age = sample ? std::max(0.0, timestamp - sample->receive_timestamp)
                              : std::numeric_limits<double>::infinity();
    last_age.store(age);
    Eigen::Matrix4d desired = pose;
    if (stop_requested.load() || (!hold_requested.load() && age >= config.stop_timeout)) {
      watchdog.store(3);
      should_finish_flag.store(true);
    } else if (hold_requested.load()) {
      watchdog.store(2);
    } else if (!sample) {
      watchdog.store(0);
    } else if (age >= config.hold_timeout) {
      watchdog.store(2);
    } else {
      watchdog.store(1);
      desired = pose_matrix(sample->pose);
    }

    for (int axis = 0; axis < 3; ++axis) {
      desired(axis, 3) = std::clamp(desired(axis, 3),
                                    config.workspace_min[axis],
                                    config.workspace_max[axis]);
    }

    const Eigen::Vector3d position_error =
        desired.topRightCorner<3, 1>() - filtered.topRightCorner<3, 1>();
    if (position_error.norm() >= config.translation_deadband) {
      filtered.topRightCorner<3, 1>() +=
          alpha(config.translation_cutoff_hz, dt) * position_error;
    }

    const Eigen::Matrix3d filtered_rotation = filtered.topLeftCorner<3, 3>();
    const Eigen::Matrix3d desired_rotation = desired.topLeftCorner<3, 3>();
    const Eigen::Vector3d filter_rotation_error =
        rotation_vector(filtered_rotation.transpose() * desired_rotation);
    if (filter_rotation_error.norm() >= config.rotation_deadband) {
      Eigen::Quaterniond from(filtered_rotation);
      Eigen::Quaterniond to(desired_rotation);
      filtered.topLeftCorner<3, 3>() =
          from.slerp(alpha(config.rotation_cutoff_hz, dt), to)
              .normalized()
              .toRotationMatrix();
    }

    Eigen::Vector3d linear_step;
    bounded_state(filtered.topRightCorner<3, 1>() - pose.topRightCorner<3, 1>(),
                  linear_velocity, linear_acceleration, dt,
                  config.max_translation_speed,
                  config.max_translation_acceleration,
                  config.max_translation_jerk, linear_step);
    const Eigen::Vector3d angular_error = rotation_vector(
        pose.topLeftCorner<3, 3>().transpose() * filtered.topLeftCorner<3, 3>());
    Eigen::Vector3d angular_step;
    bounded_state(angular_error, angular_velocity, angular_acceleration, dt,
                  config.max_angular_speed, config.max_angular_acceleration,
                  config.max_angular_jerk, angular_step);

    pose.topRightCorner<3, 1>() += linear_step;
    pose.topLeftCorner<3, 3>() =
        pose.topLeftCorner<3, 3>() * rotation_from_vector(angular_step);
    for (int axis = 0; axis < 3; ++axis) {
      pose(axis, 3) = std::clamp(pose(axis, 3), config.workspace_min[axis],
                                 config.workspace_max[axis]);
    }
    pose.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
    const Pose output = pose_array(pose);
    std::atomic_store(&command, std::make_shared<const Pose>(output));
    auto snapshot = std::make_shared<Dynamics>();
    for (int axis = 0; axis < 3; ++axis) {
      snapshot->linear_velocity[axis] = linear_velocity[axis];
      snapshot->angular_velocity[axis] = angular_velocity[axis];
      snapshot->linear_acceleration[axis] = linear_acceleration[axis];
      snapshot->angular_acceleration[axis] = angular_acceleration[axis];
    }
    std::atomic_store(&dynamics,
                      std::shared_ptr<const Dynamics>(std::move(snapshot)));
    callback_count.fetch_add(1);
    return output;
  }

  Diagnostics diagnostics(double timestamp) const {
    Diagnostics result;
    result.watchdog_state = watchdog_name();
    result.callback_count = callback_count.load();
    result.accepted = accepted.load();
    result.rejected_sequence = rejected_sequence.load();
    result.rejected_source_time = rejected_source_time.load();
    result.rejected_source_age = rejected_source_age.load();
    result.rejected_target_delta = rejected_target_delta.load();
    result.latest_sequence = latest_sequence.load();
    const auto sample = std::atomic_load(&latest);
    result.latest_command_age = sample ? std::max(0.0, timestamp - sample->receive_timestamp)
                                       : std::numeric_limits<double>::infinity();
    const auto state = std::atomic_load(&dynamics);
    if (state) {
      result.linear_velocity = state->linear_velocity;
      result.angular_velocity = state->angular_velocity;
      result.linear_acceleration = state->linear_acceleration;
      result.angular_acceleration = state->angular_acceleration;
    }
    return result;
  }

  std::string watchdog_name() const {
    switch (watchdog.load()) {
      case 0: return "holding_no_input";
      case 1: return "tracking";
      case 2: return "holding_stale";
      case 3: return "safe_stop";
      default: return "not_started";
    }
  }

  RealtimeConfig config;
  std::shared_ptr<const Sample> latest;
  std::shared_ptr<const Pose> command;
  std::shared_ptr<const Dynamics> dynamics;
  mutable std::mutex publish_mutex;
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d filtered{Eigen::Matrix4d::Identity()};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
  double last_step_time{0.0};
  double latest_source_time{0.0};
  bool initialized{false};
  std::atomic<bool> hold_requested{false};
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> should_finish_flag{false};
  std::atomic<int> watchdog{-1};
  std::atomic<std::uint64_t> callback_count{0};
  std::atomic<std::uint64_t> accepted{0};
  std::atomic<std::uint64_t> rejected_sequence{0};
  std::atomic<std::uint64_t> rejected_source_time{0};
  std::atomic<std::uint64_t> rejected_source_age{0};
  std::atomic<std::uint64_t> rejected_target_delta{0};
  std::atomic<std::int64_t> latest_sequence{-1};
  std::atomic<double> last_age{0.0};
};

RealtimeCore::RealtimeCore(const RealtimeConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}
RealtimeCore::~RealtimeCore() = default;
RealtimeCore::RealtimeCore(RealtimeCore&&) noexcept = default;
RealtimeCore& RealtimeCore::operator=(RealtimeCore&&) noexcept = default;
void RealtimeCore::reset(const Pose& pose, double timestamp) { impl_->reset(pose, timestamp); }
bool RealtimeCore::publish(const Pose& pose, double source, std::int64_t sequence,
                           double received) {
  return impl_->publish(pose, source, sequence, received);
}
Pose RealtimeCore::step(double timestamp) { return impl_->step(timestamp); }
void RealtimeCore::hold() { impl_->hold_requested.store(true); }
void RealtimeCore::request_stop() { impl_->stop_requested.store(true); }
Diagnostics RealtimeCore::diagnostics(double timestamp) const {
  return impl_->diagnostics(timestamp);
}
Pose RealtimeCore::command_pose() const {
  const auto pose = std::atomic_load(&impl_->command);
  if (!pose) throw std::runtime_error("command pose is unavailable");
  return *pose;
}
bool RealtimeCore::should_finish() const { return impl_->should_finish_flag.load(); }

namespace {

class SessionBase {
 public:
  virtual ~SessionBase() = default;
  virtual void connect() = 0;
  virtual void disconnect() = 0;
  virtual RobotState state() = 0;
  virtual std::vector<double> joints() = 0;
  virtual Pose tcp_pose() = 0;
  virtual Pose base_frame() = 0;
  virtual Pose calculate_fk(const std::vector<double>&) = 0;
  virtual std::pair<bool, std::vector<std::array<double, 2>>> soft_limits() = 0;
  virtual std::vector<ControllerLog> controller_logs(unsigned, bool) = 0;
  virtual Pose preview_nrt_cartesian_target(const Pose&) = 0;
  virtual void move_joint(const std::vector<double>&, int, double, double, bool) = 0;
  virtual void recover_from_joint_limits(const std::vector<double>&, int,
                                         double, double) = 0;
  virtual void recover_disabled_soft_limits(
      const std::vector<double>&, const std::vector<std::array<double, 2>>&,
      double, bool, double, const std::function<void()>&) = 0;
  virtual void move_cartesian(const Pose&, int, double, double, double, bool) = 0;
  virtual void start_realtime(bool) = 0;
  virtual void start_realtime_impedance(bool) = 0;
  virtual void start_realtime_joint(const JointRealtimeConfig&, bool) = 0;
  virtual bool set_target(const Pose&, double, std::int64_t) = 0;
  virtual bool set_target_joints(const std::vector<double>&, double, std::int64_t) = 0;
  virtual std::vector<double> command_joints() const = 0;
  virtual void hold() = 0;
  virtual void stop_realtime() = 0;
  virtual void stop() = 0;
  virtual Pose command_pose() const = 0;
  virtual Diagnostics diagnostics() const = 0;
};

template <typename Robot, std::size_t DoF>
class Session final : public SessionBase {
 public:
  Session(std::string robot_ip, std::string local_ip, RealtimeConfig config)
      : robot_ip_(std::move(robot_ip)), local_ip_(std::move(local_ip)),
        config_(std::move(config)), core_(config_) {}

  ~Session() override {
    try { disconnect(); } catch (...) {}
  }

  void connect() override {
    if (robot_) return;
    robot_ = std::make_unique<Robot>(robot_ip_, local_ip_);
  }

  void disconnect() override {
    if (!robot_) return;
    std::exception_ptr first_error;
    if (realtime_ || controller_ || receiver_started_) {
      try {
        stop_realtime();
      } catch (...) {
        first_error = std::current_exception();
      }
    }
    std::error_code ec;
    if (entered_realtime_) {
      robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
      if (ec && !first_error) {
        first_error = std::make_exception_ptr(
            std::runtime_error("setMotionControlMode(NrtCommand): " + ec.message()));
      }
      entered_realtime_ = false;
    }
    ec.clear();
    robot_->disconnectFromRobot(ec);
    robot_.reset();
    if (ec && !first_error) {
      first_error = std::make_exception_ptr(
          std::runtime_error("disconnectFromRobot: " + ec.message()));
    }
    if (first_error) std::rethrow_exception(first_error);
  }

  RobotState state() override {
    require_connected();
    std::error_code ec;
    const auto info = robot_->robotInfo(ec); check_ec(ec, "robotInfo");
    RobotState result;
    result.connected = true;
    result.realtime_running = realtime_;
    result.sdk_version = robot_->sdkVersion();
    result.controller_version = info.version;
    result.robot_type = info.type;
    result.joint_count = info.joint_num;
    result.power_state = to_string(robot_->powerState(ec)); check_ec(ec, "powerState");
    result.operate_mode = to_string(robot_->operateMode(ec)); check_ec(ec, "operateMode");
    result.operation_state = to_string(robot_->operationState(ec)); check_ec(ec, "operationState");
    const auto q = robot_->jointPos(ec); check_ec(ec, "jointPos");
    const auto dq = robot_->jointVel(ec); check_ec(ec, "jointVel");
    result.joint_position.assign(q.begin(), q.end());
    result.joint_velocity.assign(dq.begin(), dq.end());
    result.tcp_pose = tcp_pose();
    result.timestamp = now_seconds();
    return result;
  }

  std::vector<double> joints() override {
    require_connected();
    std::error_code ec;
    const auto values = robot_->jointPos(ec); check_ec(ec, "jointPos");
    return {values.begin(), values.end()};
  }

  Pose tcp_pose() override {
    require_connected();
    if (realtime_) {
      Pose measured{};
      if (load_measured_pose(measured)) return measured;
    }
    // tcpPose_m is explicitly documented by SDK 0.3.4 as the end pose
    // relative to the robot base. endInRef is not equivalent when a non-base
    // work object/reference is active.
    bool temporary_receiver = false;
    try {
      if (!receiver_started_) {
        robot_->startReceiveRobotState(
            std::chrono::milliseconds(1),
            {rokae::RtSupportedFields::tcpPose_m});
        temporary_receiver = true;
      }
      robot_->updateRobotState(std::chrono::milliseconds(20));
      Pose measured{};
      if (robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, measured) != 0) {
        throw std::runtime_error("getStateData(tcpPose_m) failed");
      }
      validate_pose(measured, "base-relative TCP pose");
      if (temporary_receiver) robot_->stopReceiveRobotState();
      return measured;
    } catch (...) {
      if (temporary_receiver) {
        try { robot_->stopReceiveRobotState(); } catch (...) {}
      }
      throw;
    }
  }

  Pose base_frame() override {
    require_connected();
    std::error_code ec;
    const auto xyz_rpy = robot_->baseFrame(ec);
    check_ec(ec, "baseFrame");
    return pose_from_posture(xyz_rpy);
  }

  Pose calculate_fk(const std::vector<double>& joints) override {
    require_connected();
    if (joints.size() != DoF) {
      throw std::invalid_argument("FK joint target has wrong length");
    }
    std::array<double, DoF> q{};
    for (std::size_t index = 0; index < DoF; ++index) {
      if (!std::isfinite(joints[index])) {
        throw std::invalid_argument("FK joint target is non-finite");
      }
      q[index] = joints[index];
    }
    std::error_code ec;
    auto model = robot_->model();
    const auto result = model.calcFk(q, ec);
    check_ec(ec, "model.calcFk");
    const std::array<double, 6> xyz_rpy{{
        result.trans[0], result.trans[1], result.trans[2],
        result.rpy[0], result.rpy[1], result.rpy[2]}};
    return pose_from_posture(xyz_rpy);
  }

  std::pair<bool, std::vector<std::array<double, 2>>> soft_limits() override {
    require_connected();
    std::array<double[2], DoF> raw{};
    std::error_code ec;
    const bool enabled = robot_->getSoftLimit(raw, ec);
    check_ec(ec, "getSoftLimit");
    std::vector<std::array<double, 2>> limits;
    limits.reserve(DoF);
    for (std::size_t i = 0; i < DoF; ++i) {
      limits.push_back({raw[i][0], raw[i][1]});
    }
    return {enabled, limits};
  }

  std::vector<ControllerLog> controller_logs(unsigned count, bool faults_only) override {
    if (count < 1 || count > 10) throw std::invalid_argument("controller log count must be 1..10");
    require_connected();
    std::error_code ec;
    const std::set<rokae::LogInfo::Level> levels = faults_only
        ? std::set<rokae::LogInfo::Level>{rokae::LogInfo::warning, rokae::LogInfo::error}
        : std::set<rokae::LogInfo::Level>{};
    const auto logs = robot_->queryControllerLog(count, levels, ec);
    check_ec(ec, "queryControllerLog");
    std::vector<ControllerLog> result;
    for (const auto& log : logs) result.push_back({log.id, log.timestamp, log.content, log.repair});
    return result;
  }

  void prepare_motion(bool power_on, rokae::MotionControlMode mode,
                      bool mode_before_power = false) {
    require_connected();
    std::error_code ec;
    std::cout << "[ROKAE] requesting automatic operate mode ..." << std::endl;
    robot_->setOperateMode(rokae::OperateMode::automatic, ec);
    check_ec(ec, "setOperateMode(automatic)");
    // 阻抗遵循官方 0.3.4 顺序：automatic -> RtCommand -> power on。
    // 其余控制路线保留既有顺序，避免改变已使用的位置控制启动行为。
    if (mode_before_power) {
      robot_->setMotionControlMode(mode, ec);
      check_ec(ec, "setMotionControlMode(before power-on)");
    }
    if (power_on) {
      std::cout << "[ROKAE] requesting motor power-on ..." << std::endl;
      robot_->setPowerState(true, ec);
      check_ec(ec, "setPowerState(true)");
    }
    const auto power_deadline = Clock::now() + std::chrono::seconds(10);
    rokae::PowerState power = rokae::PowerState::unknown;
    do {
      power = robot_->powerState(ec);
      check_ec(ec, "powerState");
      if (power == rokae::PowerState::on) break;
      if (!power_on) {
        throw std::runtime_error(
            "robot power state is " + to_string(power) +
            "; automatic power-on was disabled, so power it externally or "
            "restart without --no-power-on/--no-rokae-power-on");
      }
      if (power == rokae::PowerState::estop ||
          power == rokae::PowerState::gstop) {
        throw std::runtime_error(
            "robot cannot power on because power state is " + to_string(power));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (Clock::now() < power_deadline);
    if (power != rokae::PowerState::on) {
      throw std::runtime_error(
          "robot did not reach power=on within 10 seconds; check enable "
          "switch, automatic mode, safety circuit and controller alarms");
    }
    if (!mode_before_power) {
      std::cout << "[ROKAE] power=on; selecting motion control mode ..." << std::endl;
      robot_->setMotionControlMode(mode, ec);
      check_ec(ec, "setMotionControlMode");
    } else {
      std::cout << "[ROKAE] power=on; realtime command mode selected before power-on."
                << std::endl;
    }
    if (mode == rokae::MotionControlMode::NrtCommand) {
      // Do not inherit an unknown low online scale from an earlier SDK client.
      // This scale is independent of MoveAbsJ/MoveL's integer speed setting.
      robot_->adjustSpeedOnline(config_.nrt_online_speed_scale, ec);
      check_ec(ec, "adjustSpeedOnline");
      std::cout << "[ROKAE] NRT online speed scale="
                << config_.nrt_online_speed_scale << std::endl;
    }
  }

  void move_joint(const std::vector<double>& target, int speed, double timeout,
                  double max_delta, bool power_on) override {
    if (target.size() != DoF) throw std::invalid_argument("joint target has wrong length");
    if (speed < 5 || speed > 1000) throw std::invalid_argument("joint speed must be 5..1000");
    if (!std::isfinite(max_delta) || max_delta <= 0.0) {
      throw std::invalid_argument("max_joint_delta must be positive");
    }
    const auto current = joints();
    double largest = 0.0;
    for (std::size_t i = 0; i < DoF; ++i) {
      if (!std::isfinite(target[i])) throw std::invalid_argument("joint target is non-finite");
      largest = std::max(largest, std::abs(target[i] - current[i]));
    }
    if (largest > max_delta) throw std::runtime_error("joint target exceeds max_joint_delta");
    prepare_motion(power_on, rokae::MotionControlMode::NrtCommand);
    std::error_code ec;
    std::cout << "[ROKAE] clearing NRT motion buffer ..." << std::endl;
    robot_->moveReset(ec); check_ec(ec, "moveReset");
    std::string id;
    std::cout << "[ROKAE] appending MoveAbsJ ..." << std::endl;
    robot_->moveAppend({rokae::MoveAbsJCommand(target, speed, 0)}, id, ec);
    check_ec(ec, "moveAppend(MoveAbsJ)");
    std::cout << "[ROKAE] starting trajectory id=" << id << " ..." << std::endl;
    robot_->moveStart(ec); check_ec(ec, "moveStart(MoveAbsJ)");
    wait_until(timeout, id, "MoveAbsJ", [&]() {
      const auto actual = joints();
      double error = 0.0;
      std::size_t max_error_joint = 0;
      for (std::size_t i = 0; i < DoF; ++i) {
        const double joint_error = std::abs(target[i] - actual[i]);
        if (joint_error > error) {
          error = joint_error;
          max_error_joint = i;
        }
      }
      std::ostringstream detail;
      detail << "max_joint_error=J" << (max_error_joint + 1) << ":" << error
             << " rad actual=" << actual[max_error_joint]
             << " target=" << target[max_error_joint] << " q=[";
      for (std::size_t i = 0; i < actual.size(); ++i) {
        if (i != 0) detail << ",";
        detail << actual[i];
      }
      detail << "]";
      return MotionProgress{error <= 0.01, detail.str()};
    });
    std::cout << "[ROKAE] MoveAbsJ reached target." << std::endl;
    robot_->moveReset(ec); check_ec(ec, "moveReset(after MoveAbsJ)");
  }

  void recover_disabled_soft_limits(
      const std::vector<double>& expected_joints,
      const std::vector<std::array<double, 2>>& expected_limits,
      double clearance_degrees,
      bool request_power_on,
      double jog_rate,
      const std::function<void()>& check_cancel) override {
    if (!std::isfinite(jog_rate) || jog_rate < 0.01 || jog_rate > 0.10) {
      throw std::invalid_argument("recovery jog_rate must be in 0.01..0.10");
    }
    require_connected();
    if (realtime_ || expected_joints.size() != DoF || expected_limits.size() != DoF) {
      throw std::invalid_argument("maintenance recovery requires matching DOF and no realtime control");
    }
    // Validate the operator-reviewed snapshot before changing any controller state.
    const auto target = plan_disabled_soft_limit_recovery(expected_joints, expected_limits, clearance_degrees);
    const auto before = joints();
    const auto saved = soft_limits();
    if (saved.first || saved.second != expected_limits) {
      throw std::runtime_error("soft-limit state changed since review; re-run inspection");
    }
    constexpr double tolerance = 0.05 * kPi / 180.0;
    for (std::size_t i = 0; i < DoF; ++i) {
      if (!std::isfinite(before[i]) || std::abs(before[i] - expected_joints[i]) > tolerance) {
        throw std::runtime_error("joint position changed since review; re-run inspection");
      }
    }
    const auto initial = state();
    if (initial.operation_state != "idle") {
      throw std::runtime_error("maintenance recovery requires an idle controller");
    }
    auto cancel = [&]() { if (check_cancel) check_cancel(); };
    auto power_off_manual = [&]() {
      std::error_code ec;
      robot_->stop(ec); check_ec(ec, "stop(recovery)");
      const auto before_off = robot_->powerState(ec); check_ec(ec, "powerState(before recovery off)");
      if (!request_power_on && before_off == rokae::PowerState::on) {
        std::cout << "[ROKAE] Jog stopped. Release the physical manual enable now; "
                     "waiting for power=off before restoring limits (10 s)." << std::endl;
      }
      robot_->setPowerState(false, ec); check_ec(ec, "power off(recovery)");
      const auto deadline = Clock::now() + std::chrono::seconds(10);
      for (;;) {
        const auto power = robot_->powerState(ec); check_ec(ec, "powerState(recovery)");
        if (power == rokae::PowerState::off) break;
        if (Clock::now() >= deadline) throw std::runtime_error("recovery power-off timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      robot_->setOperateMode(rokae::OperateMode::manual, ec);
      check_ec(ec, "setOperateMode(manual recovery)");
      const auto mode = robot_->operateMode(ec); check_ec(ec, "operateMode(recovery)");
      if (mode != rokae::OperateMode::manual) throw std::runtime_error("manual mode not accepted");
    };

    std::exception_ptr motion_error;
    bool jog_requested = false;
    try {
      cancel();
      power_off_manual();
      // Re-read after mode/power transitions. No stale position may become a jog.
      const auto start = joints();
      const auto current_limits = soft_limits();
      if (current_limits.first || current_limits.second != expected_limits) {
        throw std::runtime_error("soft-limit state changed before jog");
      }
      unsigned axis = DoF;
      for (std::size_t i = 0; i < DoF; ++i) {
        if (!std::isfinite(start[i]) || std::abs(start[i] - expected_joints[i]) > tolerance) {
          throw std::runtime_error("joint moved before jog; re-run inspection");
        }
        if (target[i] != expected_joints[i]) axis = static_cast<unsigned>(i);
      }
      if (axis != DoF) {
        cancel();
        std::error_code ec;
        if (request_power_on) {
          // SDK 0.3.4: manual power-on is supported ONLY without an external
          // enabling device/teach pendant. Never fall back to automatic mode.
          robot_->setPowerState(true, ec);
          if (ec) {
            throw std::runtime_error("power on(manual recovery): " + ec.message() +
                " [" + ec.category().name() + ":" + std::to_string(ec.value()) +
                "]. SDK manual power-on requires no external enable/teach pendant; "
                "use --manual-power-source operator when one is connected");
          }
        } else {
          std::cout << "[ROKAE] WAITING FOR MANUAL ENABLE (30 s): press and hold the "
                       "physical three-position enable at its middle position. "
                       "The reviewed single-axis Jog starts when power=on; keep "
                       "holding until the release prompt. No SDK power-on request is sent."
                    << std::endl;
        }
        const auto power_deadline = Clock::now() + std::chrono::seconds(30);
        for (;;) {
          cancel();
          const auto power = robot_->powerState(ec); check_ec(ec, "powerState(manual recovery)");
          if (power == rokae::PowerState::on) break;
          if (power == rokae::PowerState::estop || power == rokae::PowerState::gstop ||
              Clock::now() >= power_deadline) {
            throw std::runtime_error("manual power-on not observed; check physical enable/safety state. "
                                     "No recovery Jog command was sent");
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        const auto powered_joints = joints();
        const auto powered_limits = soft_limits();
        const auto powered_mode = robot_->operateMode(ec);
        check_ec(ec, "operateMode(before recovery jog)");
        if (powered_mode != rokae::OperateMode::manual || powered_limits.first ||
            powered_limits.second != expected_limits) {
          throw std::runtime_error("manual mode or soft-limit state changed during power-on");
        }
        for (std::size_t i = 0; i < DoF; ++i) {
          if (!std::isfinite(powered_joints[i]) || std::abs(powered_joints[i] - start[i]) > tolerance) {
            throw std::runtime_error("joint moved during power-on; refusing recovery jog");
          }
        }
        cancel();
        // Step is in degrees, joint index is zero based. No automatic mode,
        // MoveAbsJ, safety reset or disabling/widening any limit is used here.
        const double delta = target[axis] - start[axis];
        jog_requested = true;
        robot_->startJog(rokae::JogOpt::jointSpace, jog_rate,
                         std::abs(delta) * 180.0 / kPi, axis, delta > 0, ec);
        check_ec(ec, "startJog(single-axis recovery)");
        const auto deadline = Clock::now() + std::chrono::seconds(30);
        for (;;) {
          cancel();
          const auto actual = joints();
          for (std::size_t i = 0; i < DoF; ++i) {
            const double lower = std::min(start[i], target[i]) - tolerance;
            const double upper = std::max(start[i], target[i]) + tolerance;
            if (!std::isfinite(actual[i]) || actual[i] < lower || actual[i] > upper) {
              throw std::runtime_error("unexpected joint movement during recovery");
            }
          }
          const auto mode = robot_->operateMode(ec); check_ec(ec, "operateMode(jog)");
          const auto power = robot_->powerState(ec); check_ec(ec, "powerState(jog)");
          if (mode != rokae::OperateMode::manual || power != rokae::PowerState::on) {
            throw std::runtime_error("manual enable/power lost during recovery");
          }
          if (std::abs(actual[axis] - target[axis]) <= tolerance) break;
          if (Clock::now() >= deadline) throw std::runtime_error("single-axis jog recovery timed out");
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      }
    } catch (...) {
      motion_error = std::current_exception();
    }

    // Always stop Jog (SDK requirement) and request power-off, including on
    // signal, timeout or startJog failure. Never report success until read-back.
    try {
      power_off_manual();
      const auto final_q = joints();
      const auto final_limits = soft_limits();
      if (final_limits.second != expected_limits) {
        throw std::runtime_error("controller limits changed; will not overwrite them");
      }
      for (std::size_t i = 0; i < DoF; ++i) {
        if (!std::isfinite(final_q[i]) || final_q[i] <= expected_limits[i][0] ||
            final_q[i] >= expected_limits[i][1]) {
          throw std::runtime_error("joint remains outside limits; cannot re-enable them");
        }
      }
      std::error_code ec;
      // Default limits argument means enable existing values, never rewrite.
      robot_->setSoftLimit(true, ec); check_ec(ec, "setSoftLimit(true recovery)");
      const auto restored = soft_limits();
      if (!restored.first || restored.second != expected_limits) {
        throw std::runtime_error("original soft limits were not restored");
      }
      const auto final_power = robot_->powerState(ec); check_ec(ec, "powerState(after recovery)");
      const auto final_mode = robot_->operateMode(ec); check_ec(ec, "operateMode(after recovery)");
      if (final_power != rokae::PowerState::off || final_mode != rokae::OperateMode::manual) {
        throw std::runtime_error("final power-off/manual state not confirmed");
      }
    } catch (const std::exception& error) {
      // Even if stop/manual transition failed, make one separate power-off
      // request. A network failure can prevent confirmation; report explicitly.
      std::error_code off_ec;
      robot_->setPowerState(false, off_ec);
      std::string original;
      if (motion_error) {
        try { std::rethrow_exception(motion_error); }
        catch (const std::exception& exc) { original = std::string("; initial error: ") + exc.what(); }
        catch (...) { original = "; initial recovery interrupted"; }
      }
      off_ec.clear();
      const auto final_power = robot_->powerState(off_ec);
      const bool off_confirmed = !off_ec && final_power == rokae::PowerState::off;
      throw std::runtime_error(std::string("RECOVERY INCOMPLETE") + original +
          "; cleanup: " + error.what() + (jog_requested ? "; Jog was requested" : "; no Jog command sent") +
          (off_confirmed ? "; power=off read back" : "; power-off not confirmed") +
          "; soft-limit restoration not confirmed");
    }
    std::cout << "[ROKAE] original soft limits enabled and verified; powered off in manual mode."
              << std::endl;
    if (motion_error) std::rethrow_exception(motion_error);
  }

  void recover_from_joint_limits(const std::vector<double>& target, int speed,
                                 double timeout, double max_delta) override {
    if (target.size() != DoF) {
      throw std::invalid_argument("joint-limit recovery target has wrong length");
    }
    if (realtime_) {
      throw std::runtime_error("joint-limit recovery cannot run during realtime control");
    }
    const auto before = joints();
    std::array<double[2], DoF> raw_limits{};
    std::error_code ec;
    const bool limits_enabled = robot_->getSoftLimit(raw_limits, ec);
    check_ec(ec, "getSoftLimit(recovery)");
    if (!limits_enabled) {
      throw std::runtime_error(
          "controller soft limits are already disabled; refusing automatic "
          "init recovery. Inspect with python rokae/examples/06_recover_soft_limits.py first");
    }
    for (std::size_t i = 0; i < DoF; ++i) {
      if (!std::isfinite(target[i])) {
        throw std::invalid_argument("joint-limit recovery target is non-finite");
      }
      if (!(target[i] > raw_limits[i][0] && target[i] < raw_limits[i][1])) {
        throw std::invalid_argument(
            "joint-limit recovery target J" + std::to_string(i + 1) +
            " is not strictly inside the saved soft limits");
      }
      const bool near_lower = before[i] <= raw_limits[i][0] + 0.15;
      const bool near_upper = before[i] >= raw_limits[i][1] - 0.15;
      if (near_lower && target[i] <= before[i]) {
        throw std::invalid_argument("recovery J" + std::to_string(i + 1) +
                                    " does not move inward from its lower limit");
      }
      if (near_upper && target[i] >= before[i]) {
        throw std::invalid_argument("recovery J" + std::to_string(i + 1) +
                                    " does not move inward from its upper limit");
      }
      if (!near_lower && !near_upper && std::abs(target[i] - before[i]) > 1e-4) {
        throw std::invalid_argument(
            "recovery may move only joints near a configured soft limit");
      }
    }

    auto set_manual_power_off = [&]() {
      std::error_code state_ec;
      robot_->stop(state_ec);
      state_ec.clear();
      robot_->moveReset(state_ec);
      state_ec.clear();
      robot_->setPowerState(false, state_ec);
      check_ec(state_ec, "setPowerState(false) for soft-limit recovery");
      const auto deadline = Clock::now() + std::chrono::seconds(10);
      while (Clock::now() < deadline) {
        const auto power = robot_->powerState(state_ec);
        check_ec(state_ec, "powerState(recovery power-off)");
        if (power == rokae::PowerState::off) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (robot_->powerState(state_ec) != rokae::PowerState::off) {
        check_ec(state_ec, "powerState(recovery final)");
        throw std::runtime_error("robot did not power off for soft-limit recovery");
      }
      robot_->setOperateMode(rokae::OperateMode::manual, state_ec);
      check_ec(state_ec, "setOperateMode(manual) for soft-limit recovery");
    };

    std::cout << "[ROKAE] RECOVERY: powering off and entering manual mode ..." << std::endl;
    set_manual_power_off();
    robot_->setSoftLimit(false, ec);
    check_ec(ec, "setSoftLimit(false)");
    std::cout << "[ROKAE] RECOVERY: soft limits temporarily disabled; moving only "
                 "the near-limit axis/axes inward ..." << std::endl;

    std::exception_ptr motion_error;
    try {
      move_joint(target, speed, timeout, max_delta, true);
    } catch (...) {
      motion_error = std::current_exception();
    }

    std::exception_ptr restore_error;
    try {
      std::cout << "[ROKAE] RECOVERY: powering off before restoring soft limits ..."
                << std::endl;
      set_manual_power_off();
      ec.clear();
      robot_->setSoftLimit(true, ec, raw_limits);
      check_ec(ec, "setSoftLimit(true, saved limits)");
      std::array<double[2], DoF> verified{};
      const bool restored = robot_->getSoftLimit(verified, ec);
      check_ec(ec, "getSoftLimit(verify recovery restore)");
      if (!restored) throw std::runtime_error("soft limits remained disabled");
      std::cout << "[ROKAE] RECOVERY: original soft limits restored; robot remains "
                   "powered off in manual mode." << std::endl;
    } catch (...) {
      restore_error = std::current_exception();
    }
    if (restore_error) {
      try {
        std::rethrow_exception(restore_error);
      } catch (const std::exception& error) {
        throw std::runtime_error(
            std::string("CRITICAL: failed to restore controller soft limits: ") +
            error.what() + ". Keep the cell stopped and restore limits in Robot Assist");
      }
    }
    if (motion_error) std::rethrow_exception(motion_error);
  }

  void move_cartesian(const Pose& target, int speed, double timeout,
                      double max_translation_delta, double max_rotation_delta,
                      bool power_on) override {
    validate_pose(target, "Cartesian target");
    if (speed < 5 || speed > 1000) throw std::invalid_argument("Cartesian speed must be 5..1000 mm/s");
    if (!std::isfinite(max_translation_delta) || max_translation_delta <= 0.0 ||
        !std::isfinite(max_rotation_delta) || max_rotation_delta <= 0.0) {
      throw std::invalid_argument("Cartesian one-shot deltas must be positive");
    }
    const Pose current = tcp_pose();
    const Eigen::Matrix4d target_matrix = pose_matrix(target);
    const Eigen::Matrix4d current_matrix = pose_matrix(current);
    const Eigen::Vector3d delta =
        target_matrix.topRightCorner<3, 1>() -
        current_matrix.topRightCorner<3, 1>();
    const double translation_delta = delta.norm();
    const double rotation_delta = rotation_distance(current, target);
    // Targets are often generated exactly on the configured boundary.  The
    // TCP is sampled again after user confirmation, and both sensor noise and
    // binary floating-point can make 0.05 appear as 0.050000000x.  Permit only
    // numerical-scale slack; this is not a motion-limit relaxation.
    constexpr double kTranslationComparisonTolerance = 1e-6;  // 1 micrometre
    constexpr double kRotationComparisonTolerance = 1e-6;     // 1 microradian
    if (translation_delta >
            max_translation_delta + kTranslationComparisonTolerance ||
        rotation_delta > max_rotation_delta + kRotationComparisonTolerance) {
      std::ostringstream message;
      message << "Cartesian target exceeds configured one-shot delta: "
              << "translation=" << translation_delta
              << " m (limit=" << max_translation_delta << " m), rotation="
              << rotation_delta << " rad (limit=" << max_rotation_delta
              << " rad)";
      throw std::runtime_error(message.str());
    }

    const Pose nrt_target = convert_base_tcp_target(target, current);

    prepare_motion(power_on, rokae::MotionControlMode::NrtCommand);
    std::error_code ec;
    // The NRT planner consumes CartesianPosition::trans/rpy.  Constructing a
    // CartesianPosition from Array16 only fills pos and makes xCore 2.3.2
    // reject the waypoint as an invalid/zero posture.  Convert explicitly to
    // the six-value representation used by the official MoveL examples.
    std::array<double, 6> nrt_target_posture{};
    rokae::Utils::transArrayToPosture(nrt_target, nrt_target_posture);
    rokae::CartesianPosition nrt_command_target(nrt_target_posture);
    robot_->setDefaultConfOpt(false, ec);
    check_ec(ec, "setDefaultConfOpt(false)");
    std::cout << "[ROKAE] MoveL NRT xyz-rpy=[";
    for (std::size_t i = 0; i < nrt_target_posture.size(); ++i) {
      if (i != 0) std::cout << ",";
      std::cout << nrt_target_posture[i];
    }
    std::cout << "] conf=nearest-current" << std::endl;
    std::cout << "[ROKAE] clearing NRT motion buffer ..." << std::endl;
    robot_->moveReset(ec); check_ec(ec, "moveReset");
    std::string id;
    std::cout << "[ROKAE] appending MoveL ..." << std::endl;
    robot_->moveAppend({rokae::MoveLCommand(nrt_command_target, speed, 0)}, id, ec);
    check_ec(ec, "moveAppend(MoveL)");
    std::cout << "[ROKAE] starting trajectory id=" << id << " ..." << std::endl;
    robot_->moveStart(ec); check_ec(ec, "moveStart(MoveL)");
    wait_until(timeout, id, "MoveL", [&]() {
      const Pose actual = end_in_reference_pose();
      const double translation_error =
          (pose_matrix(actual).topRightCorner<3, 1>() -
           pose_matrix(nrt_target).topRightCorner<3, 1>()).norm();
      const double rotation_error = rotation_distance(actual, nrt_target);
      std::ostringstream detail;
      detail << "translation_error=" << translation_error
             << " m rotation_error=" << rotation_error << " rad";
      return MotionProgress{translation_error <= 0.002 &&
                                rotation_error <= 0.02,
                            detail.str()};
    });
    std::cout << "[ROKAE] MoveL reached target." << std::endl;
    robot_->moveReset(ec); check_ec(ec, "moveReset(after MoveL)");
  }

  Pose preview_nrt_cartesian_target(const Pose& target) override {
    validate_pose(target, "Cartesian preview target");
    return convert_base_tcp_target(target, tcp_pose());
  }

  void start_realtime(bool power_on) override {
    start_cartesian_realtime(power_on, false);
  }

  void start_realtime_impedance(bool power_on) override {
    start_cartesian_realtime(power_on, true);
  }

  void start_cartesian_realtime(bool power_on, bool impedance) {
    if (realtime_) {
      if (joint_realtime_) {
        throw std::runtime_error("realtime joint control is already running");
      }
      if (cartesian_impedance_ != impedance) {
        throw std::runtime_error(
            "refusing to hot-switch Cartesian position and impedance; stop realtime first");
      }
      return;
    }
    (void)tcp_pose();  // Validate that a controller TCP is available before mode switch.
    std::error_code ec;
    auto active_toolset = robot_->toolset(ec);
    check_ec(ec, "toolset");
    if (impedance && config_.apply_load_on_impedance_start) {
      // SDK Toolset is separate from the saved RL tool. Copy before editing:
      // never replace TCP/ref with identity or invent rotational inertia.
      auto requested = active_toolset;
      requested.load.mass = config_.expected_load_mass;
      requested.load.cog = config_.expected_load_cog;
      config_.validate_impedance_load(requested.load.mass, requested.load.cog,
                                     requested.load.inertia);
      robot_->setToolset(requested, ec);
      check_ec(ec, "setToolset(calibrated impedance payload)");
      active_toolset = robot_->toolset(ec);
      check_ec(ec, "toolset(readback calibrated payload)");
      config_.validate_impedance_load(active_toolset.load.mass,
                                     active_toolset.load.cog,
                                     active_toolset.load.inertia);
      const auto same_values = [](const auto& actual, const auto& expected) {
        return std::equal(actual.begin(), actual.end(), expected.begin(),
                          [](double a, double b) {
                            return std::isfinite(a) && std::isfinite(b) &&
                                   std::abs(a - b) <= 1e-9;
                          });
      };
      if (!same_values(active_toolset.end.pos, requested.end.pos) ||
          !same_values(active_toolset.ref.pos, requested.ref.pos) ||
          !same_values(active_toolset.end.trans, requested.end.trans) ||
          !same_values(active_toolset.end.rpy, requested.end.rpy) ||
          !same_values(active_toolset.ref.trans, requested.ref.trans) ||
          !same_values(active_toolset.ref.rpy, requested.ref.rpy) ||
          !same_values(active_toolset.load.inertia, requested.load.inertia)) {
        throw std::runtime_error("impedance refused before power-on: payload readback changed TCP/reference/inertia");
      }
      std::cout << "[ROKAE] calibrated payload applied to SDK Toolset and read back."
                << std::endl;
    }
    std::cout << "[ROKAE] active load: mass=" << active_toolset.load.mass
              << " kg cog=[" << active_toolset.load.cog[0] << ","
              << active_toolset.load.cog[1] << ","
              << active_toolset.load.cog[2] << "] m inertia=["
              << active_toolset.load.inertia[0] << ","
              << active_toolset.load.inertia[1] << ","
              << active_toolset.load.inertia[2] << "] kg*m^2" << std::endl;
    if (impedance) {
      config_.validate_impedance_load(active_toolset.load.mass,
                                     active_toolset.load.cog,
                                     active_toolset.load.inertia);
      std::cout << "[ROKAE] load check passed (not a calibration quality assessment)." << std::endl;
      std::cout << "[ROKAE] active TCP relative to flange: xyz=["
                << active_toolset.end.trans[0] << "," << active_toolset.end.trans[1]
                << "," << active_toolset.end.trans[2] << "] m rpy=["
                << active_toolset.end.rpy[0] << "," << active_toolset.end.rpy[1]
                << "," << active_toolset.end.rpy[2]
                << "] rad (preserved; payload CoM is not TCP)" << std::endl;
      if (std::all_of(active_toolset.load.inertia.begin(), active_toolset.load.inertia.end(),
                      [](double value) { return value == 0.0; })) {
        std::cout << "[ROKAE] WARNING: controller inertia is zero/unverified; mass/CoM identification does not validate rotational dynamics." << std::endl;
      }
    }
    robot_->setRtNetworkTolerance(config_.rt_network_tolerance, ec);
    check_ec(ec, "setRtNetworkTolerance");
    if (!impedance) {
      prepare_motion(power_on, rokae::MotionControlMode::RtCommand);
    }
    entered_realtime_ = true;
    std::atomic_store(&callback_error_, std::shared_ptr<const std::string>{});
    try {
      // 模式已在上电前切换，后续失败也必须经过下方的模式清理路径。
      if (impedance) {
        prepare_motion(power_on, rokae::MotionControlMode::RtCommand, true);
      }
      if (impedance) {
        controller_ = robot_->getRtMotionController().lock();
        if (!controller_) throw std::runtime_error("empty realtime controller");
        // 同步已保存的 TCP 到实时接口；必须在读取初始实时 TCP 之前执行。
        // 不是用重心替代 TCP，也不改变用户工具定义。
        controller_->setEndEffectorFrame(active_toolset.end.pos, ec);
        check_ec(ec, "RtMotionControl::setEndEffectorFrame(impedance)");
      }
      // 先完成会影响动力学的配置，再获取启动参考，避免以配置前的缓存作平衡点。
      if (impedance) {
        if constexpr (std::is_same_v<Robot, rokae::StandardRobot>) {
          throw std::runtime_error(
              "xCoreSDK 0.3.4 Cartesian impedance is only on cobot controllers");
        } else {
          // 先设置力控坐标，再负载/刚度/期望力，与官方示例及独立试验一致。
          // 这是力控任务坐标，不用于旋转或交换法兰坐标系中的负载重心。
          const std::array<double, 16> world_frame{
              1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
          controller_->setFcCoor(world_frame, config_.impedance_force_frame == "tool"
              ? rokae::FrameType::tool : rokae::FrameType::world, ec);
          check_ec(ec, "setFcCoor(impedance)");
          std::cout << "[ROKAE] impedance force frame=" << config_.impedance_force_frame
                    << "; target is equilibrium; damping is controller-internal" << std::endl;
          // Toolset readback proves the SDK tool configuration, not a separate
          // realtime dynamics readback. Explicitly pass the same validated load
          // through the RT API before startMove; never change protection limits.
          controller_->setLoad(active_toolset.load, ec);
          check_ec(ec, "RtMotionControl::setLoad(impedance)");
          std::cout << "[ROKAE] RT setLoad accepted: mass="
                    << active_toolset.load.mass
                    << " kg (API success, not independent dynamics verification)"
                    << std::endl;
          controller_->setCartesianImpedance(config_.cartesian_stiffness, ec);
          check_ec(ec, "setCartesianImpedance");
          controller_->setCartesianImpedanceDesiredTorque(
              config_.cartesian_desired_wrench, ec);
          check_ec(ec, "setCartesianImpedanceDesiredTorque");
          std::cout << "[ROKAE] cartesian impedance stiffness N/m, Nm/rad =";
          for (double value : config_.cartesian_stiffness) {
            std::cout << " " << value;
          }
          std::cout << std::endl;
        }
      }
      std::vector<std::string> realtime_fields{
          rokae::RtSupportedFields::tcpPose_m,
          rokae::RtSupportedFields::jointPos_m,
          rokae::RtSupportedFields::jointVel_m};
      if constexpr (DoF == 7) {
        realtime_fields.push_back(rokae::RtSupportedFields::elbow_m);
      }
      receiver_started_ = true; // 初始化失败也需要停止接收。
      if (impedance) {
        const auto packets = rokae_startup::receive_startup_state(*robot_, realtime_fields);
        std::cout << "[ROKAE] post-setup RT subscription ready; received packets="
                  << packets << std::endl;
      } else {
        robot_->startReceiveRobotState(std::chrono::milliseconds(1), realtime_fields);
        robot_->updateRobotState(std::chrono::milliseconds(10));
      }
      Pose measured{};
      if (robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, measured) != 0) {
        throw std::runtime_error("getStateData(tcpPose_m) failed during realtime initialization");
      }
      validate_pose(measured, "initial realtime measured TCP");
      store_measured_pose(measured);
      std::array<double, DoF> measured_joints{};
      if (robot_->getStateData(rokae::RtSupportedFields::jointPos_m,
                               measured_joints) != 0) {
        throw std::runtime_error(
            "getStateData(jointPos_m) failed during realtime initialization");
      }
      store_measured_joints(measured_joints);
      core_.reset(measured, now_seconds());
      // Enter HOLD before starting the SDK loop, not after returning to Python.
      // HOLD freezes the reference; it is not motor disable or joint position hold.
      if (impedance) core_.hold();
      // The official 0.3.4 impedance example omits hasElbow. This is not proof
      // that redundancy is stable for this payload/posture. Do not force an
      // unverified elbow target or confuse TCP stiffness with nullspace control.
      preserve_realtime_elbow_ = false;
      if constexpr (DoF == 7) {
        if (!impedance) {
          double measured_elbow = 0.0;
          if (robot_->getStateData(rokae::RtSupportedFields::elbow_m,
                                   measured_elbow) != 0 ||
              !std::isfinite(measured_elbow)) {
            throw std::runtime_error(
                "getStateData(elbow_m) failed during realtime initialization");
          }
          realtime_elbow_ = measured_elbow;
          measured_elbow_.store(measured_elbow);
          preserve_realtime_elbow_ = true;
          std::cout << "[ROKAE] locking 7-axis realtime elbow angle at "
                    << realtime_elbow_ << " rad" << std::endl;
        } else {
          std::cout << "[ROKAE] cartesian impedance leaves the arm angle free"
                    << std::endl;
        }
      }

      controller_ = robot_->getRtMotionController().lock();
      if (!controller_) throw std::runtime_error("getRtMotionController returned an empty handle");
      if (!controller_->setFilterLimit(config_.controller_rate_limit,
                                       config_.controller_filter_cutoff_hz)) {
        throw std::runtime_error("setFilterLimit rejected the configured values");
      }
      if constexpr (!std::is_same_v<Robot, rokae::StandardRobot>) {
        controller_->setFilterFrequency(
            config_.controller_filter_cutoff_hz,
            config_.controller_filter_cutoff_hz,
            config_.controller_filter_cutoff_hz, ec);
        check_ec(ec, "setFilterFrequency");
      }
      std::cout << "[ROKAE] realtime filters: rate_limit="
                << (config_.controller_rate_limit ? "on" : "off")
                << " cutoff=" << config_.controller_filter_cutoff_hz
                << " Hz" << std::endl;
      std::function<rokae::CartesianPosition()> callback = [this, impedance]() {
        try {
          Pose current{};
          const int tcp_status = robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, current);
          if (impedance && tcp_status != 0) {
            throw std::runtime_error("impedance TCP state read failed; refusing cached-state equilibrium limiting");
          }
          if (tcp_status == 0) {
            if (impedance) validate_pose(current, "impedance measured TCP");
            store_measured_pose(current);
          }
          std::array<double, DoF> joints{};
          if (robot_->getStateData(rokae::RtSupportedFields::jointPos_m,
                                   joints) == 0) {
            store_measured_joints(joints);
          }
          if constexpr (DoF == 7) {
            double elbow = 0.0;
            if (robot_->getStateData(rokae::RtSupportedFields::elbow_m,
                                     elbow) == 0 && std::isfinite(elbow)) {
              measured_elbow_.store(elbow);
            }
          }
          Pose stepped = core_.step(now_seconds());
          if (impedance) {
            Pose measured_now{};
            if (load_measured_pose(measured_now)) {
              stepped = limit_equilibrium_pose(
                  stepped, measured_now,
                  config_.max_equilibrium_translation_error,
                  config_.max_equilibrium_rotation_error);
            }
          }
          rokae::CartesianPosition command(stepped);
          if (preserve_realtime_elbow_) {
            command.elbow = realtime_elbow_;
            command.hasElbow = true;
          }
          if (core_.should_finish()) command.setFinished();
          return command;
        } catch (const std::exception& error) {
          std::atomic_store(
              &callback_error_,
              std::make_shared<const std::string>(error.what()));
          core_.request_stop();
          rokae::CartesianPosition command(core_.command_pose());
          command.setFinished();
          return command;
        } catch (...) {
          std::atomic_store(
              &callback_error_,
              std::make_shared<const std::string>("unknown C++ callback exception"));
          core_.request_stop();
          rokae::CartesianPosition command(core_.command_pose());
          command.setFinished();
          return command;
        }
      };
      callback_ = std::move(callback);
      controller_->setControlLoop(callback_, 0, true);
      controller_->startMove(impedance
                                 ? rokae::RtControllerMode::cartesianImpedance
                                 : rokae::RtControllerMode::cartesianPosition);
      controller_->startLoop(false);
    } catch (...) {
      if (controller_) {
        try { controller_->stopMove(); } catch (...) {}
        try { controller_->stopLoop(); } catch (...) {}
      }
      // 初始化断流等失败也可能发生在上电后、realtime_ 标志置位前。
      // 必须先请求阻抗路线下电，不能仅切换命令模式后抛出异常。
      if (impedance) {
        std::error_code power_ec;
        robot_->setPowerState(false, power_ec);
        if (power_ec) std::cerr << "[ROKAE] impedance startup power-off request failed: "
                                << power_ec.message() << std::endl;
      }
      if (receiver_started_) {
        robot_->stopReceiveRobotState();
        receiver_started_ = false;
      }
      controller_.reset();
      callback_ = {};
      robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
      entered_realtime_ = false;
      throw;
    }
    cartesian_impedance_ = impedance;
    realtime_ = true;
  }

  void start_realtime_joint(const JointRealtimeConfig& joint_config, bool power_on) override {
    if (realtime_) {
      if (!joint_realtime_) {
        throw std::runtime_error("realtime Cartesian control is already running");
      }
      return;
    }
    if (joint_config.dof() != DoF) {
      throw std::invalid_argument("joint stream dof does not match the robot");
    }
    (void)tcp_pose();
    std::error_code ec;
    robot_->setRtNetworkTolerance(config_.rt_network_tolerance, ec);
    check_ec(ec, "setRtNetworkTolerance");
    prepare_motion(power_on, rokae::MotionControlMode::RtCommand);
    entered_realtime_ = true;
    std::atomic_store(&callback_error_, std::shared_ptr<const std::string>{});
    joint_core_ = std::make_unique<JointRealtimeCore>(joint_config);
    try {
      robot_->startReceiveRobotState(
          std::chrono::milliseconds(1),
          {rokae::RtSupportedFields::tcpPose_m,
           rokae::RtSupportedFields::jointPos_m,
           rokae::RtSupportedFields::jointVel_m});
      receiver_started_ = true;
      robot_->updateRobotState(std::chrono::milliseconds(10));
      std::array<double, DoF> measured_joints{};
      if (robot_->getStateData(rokae::RtSupportedFields::jointPos_m,
                               measured_joints) != 0) {
        throw std::runtime_error(
            "getStateData(jointPos_m) failed during joint realtime initialization");
      }
      store_measured_joints(measured_joints);
      Pose measured{};
      if (robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, measured) == 0) {
        store_measured_pose(measured);
      }
      joint_core_->reset(std::vector<double>(measured_joints.begin(), measured_joints.end()),
                         now_seconds());
      controller_ = robot_->getRtMotionController().lock();
      if (!controller_) throw std::runtime_error("getRtMotionController returned an empty handle");
      if (!controller_->setFilterLimit(config_.controller_rate_limit,
                                       config_.controller_filter_cutoff_hz)) {
        throw std::runtime_error("setFilterLimit rejected the configured values");
      }
      if constexpr (!std::is_same_v<Robot, rokae::StandardRobot>) {
        controller_->setFilterFrequency(
            config_.controller_filter_cutoff_hz,
            config_.controller_filter_cutoff_hz,
            config_.controller_filter_cutoff_hz, ec);
        check_ec(ec, "setFilterFrequency");
      }
      std::cout << "[ROKAE] joint realtime stream: dof=" << DoF
                << " cutoff=" << joint_config.reference_cutoff_hz
                << " Hz" << std::endl;
      std::function<rokae::JointPosition()> callback = [this]() {
        try {
          std::array<double, DoF> joints{};
          if (robot_->getStateData(rokae::RtSupportedFields::jointPos_m,
                                   joints) == 0) {
            store_measured_joints(joints);
          }
          Pose current{};
          if (robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, current) == 0) {
            store_measured_pose(current);
          }
          rokae::JointPosition command(DoF);
          command.joints = joint_core_->step(now_seconds());
          if (joint_core_->should_finish()) command.setFinished();
          return command;
        } catch (const std::exception& error) {
          std::atomic_store(
              &callback_error_,
              std::make_shared<const std::string>(error.what()));
          joint_core_->request_stop();
          rokae::JointPosition command(joint_core_->command_joints());
          command.setFinished();
          return command;
        } catch (...) {
          std::atomic_store(
              &callback_error_,
              std::make_shared<const std::string>("unknown C++ callback exception"));
          joint_core_->request_stop();
          rokae::JointPosition command(joint_core_->command_joints());
          command.setFinished();
          return command;
        }
      };
      joint_callback_ = std::move(callback);
      controller_->setControlLoop(joint_callback_, 0, true);
      controller_->startMove(rokae::RtControllerMode::jointPosition);
      controller_->startLoop(false);
    } catch (...) {
      if (controller_) {
        try { controller_->stopMove(); } catch (...) {}
        try { controller_->stopLoop(); } catch (...) {}
      }
      if (receiver_started_) {
        robot_->stopReceiveRobotState();
        receiver_started_ = false;
      }
      controller_.reset();
      joint_callback_ = {};
      joint_core_.reset();
      robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
      entered_realtime_ = false;
      throw;
    }
    joint_realtime_ = true;
    realtime_ = true;
  }

  bool set_target(const Pose& target, double source, std::int64_t sequence) override {
    if (joint_realtime_) {
      throw std::runtime_error("realtime joint route does not accept Cartesian targets");
    }
    if (!realtime_) throw std::runtime_error("realtime Cartesian control is not running");
    return core_.publish(target, source, sequence, now_seconds());
  }

  bool set_target_joints(const std::vector<double>& target, double source,
                         std::int64_t sequence) override {
    if (!joint_realtime_ || !joint_core_) {
      throw std::runtime_error("realtime joint control is not running");
    }
    return joint_core_->publish(target, source, sequence, now_seconds());
  }

  std::vector<double> command_joints() const override {
    if (!joint_core_) throw std::runtime_error("joint command is unavailable");
    return joint_core_->command_joints();
  }

  void hold() override {
    if (!realtime_) return;
    if (joint_realtime_ && joint_core_) joint_core_->hold();
    else core_.hold();
  }

  void stop_realtime() override {
    if (!realtime_ && !controller_ && !receiver_started_) return;
    core_.request_stop();
    if (joint_core_) joint_core_->request_stop();
    std::exception_ptr first;
    if (controller_) {
      try { controller_->stopMove(); } catch (...) { first = std::current_exception(); }
      try { controller_->stopLoop(); } catch (...) { if (!first) first = std::current_exception(); }
    }
    if (receiver_started_) {
      robot_->stopReceiveRobotState();
      receiver_started_ = false;
    }
    controller_.reset();
    callback_ = {};
    joint_callback_ = {};
    realtime_ = false;
    joint_realtime_ = false;
    cartesian_impedance_ = false;
    if (first) std::rethrow_exception(first);
  }

  void stop() override {
    require_connected();
    if (realtime_ || controller_) {
      stop_realtime();
      return;
    }
    std::error_code ec;
    robot_->stop(ec); check_ec(ec, "stop");
    robot_->moveReset(ec); check_ec(ec, "moveReset(stop)");
  }

  Pose command_pose() const override { return core_.command_pose(); }

  Diagnostics diagnostics() const override {
    auto result = (joint_realtime_ && joint_core_)
                      ? joint_core_->diagnostics(now_seconds())
                      : core_.diagnostics(now_seconds());
    result.connected = static_cast<bool>(robot_);
    result.realtime_running = realtime_;
    result.realtime_route = joint_realtime_        ? "joint"
                            : cartesian_impedance_ ? "cartesian_impedance"
                            : realtime_            ? "cartesian"
                                                   : "idle";
    result.measured_joint_position = load_measured_joints();
    if (joint_core_) {
      result.commanded_joint_position = joint_core_->command_joints();
      if (joint_realtime_) {
        result.commanded_joint_velocity =
            joint_core_->diagnostics(now_seconds()).commanded_joint_velocity;
      }
    }
    result.elbow_lock_enabled = preserve_realtime_elbow_;
    result.measured_elbow = measured_elbow_.load();
    result.commanded_elbow = preserve_realtime_elbow_ ? realtime_elbow_ : 0.0;
    const auto callback_error = std::atomic_load(&callback_error_);
    if (callback_error) result.callback_error = *callback_error;
    return result;
  }

 private:
  void store_measured_pose(const Pose& pose) noexcept {
    measured_pose_sequence_.fetch_add(1, std::memory_order_acq_rel);
    for (std::size_t index = 0; index < pose.size(); ++index) {
      measured_pose_[index].store(pose[index], std::memory_order_relaxed);
    }
    measured_pose_sequence_.fetch_add(1, std::memory_order_release);
    measured_pose_available_.store(true, std::memory_order_release);
  }

  bool load_measured_pose(Pose& pose) const noexcept {
    if (!measured_pose_available_.load(std::memory_order_acquire)) return false;
    for (;;) {
      const auto before = measured_pose_sequence_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) continue;
      for (std::size_t index = 0; index < pose.size(); ++index) {
        pose[index] = measured_pose_[index].load(std::memory_order_relaxed);
      }
      const auto after = measured_pose_sequence_.load(std::memory_order_acquire);
      if (before == after) return true;
    }
  }

  void store_measured_joints(const std::array<double, DoF>& joints) noexcept {
    measured_joints_sequence_.fetch_add(1, std::memory_order_acq_rel);
    for (std::size_t index = 0; index < DoF; ++index) {
      measured_joints_[index].store(joints[index], std::memory_order_relaxed);
    }
    measured_joints_sequence_.fetch_add(1, std::memory_order_release);
    measured_joints_available_.store(true, std::memory_order_release);
  }

  std::vector<double> load_measured_joints() const {
    if (!measured_joints_available_.load(std::memory_order_acquire)) return {};
    std::array<double, DoF> snapshot{};
    for (;;) {
      const auto before = measured_joints_sequence_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) continue;
      for (std::size_t index = 0; index < DoF; ++index) {
        snapshot[index] = measured_joints_[index].load(std::memory_order_relaxed);
      }
      const auto after = measured_joints_sequence_.load(std::memory_order_acquire);
      if (before == after) break;
    }
    return {snapshot.begin(), snapshot.end()};
  }

  struct MotionProgress {
    bool reached;
    std::string detail;
  };

  Pose convert_base_tcp_target(const Pose& target, const Pose& current) {
    // NRT MoveL targets are ref_T_end in the active SDK Toolset, whereas this
    // driver exposes base_T_tcp, matching tcpPose_m and realtime control.
    std::error_code ec;
    const auto base_xyz_rpy = robot_->baseFrame(ec);
    check_ec(ec, "baseFrame");
    const Pose world_T_base = pose_from_posture(base_xyz_rpy);

    const auto active_toolset = robot_->toolset(ec);
    check_ec(ec, "toolset");
    const auto print_six = [](const char* name,
                              const std::array<double, 6>& values) {
      std::cout << "[ROKAE] " << name << " xyz-rpy=[";
      for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) std::cout << ",";
        std::cout << values[i];
      }
      std::cout << "]" << std::endl;
    };
    const std::array<double, 6> ref_values{{
        active_toolset.ref.trans[0], active_toolset.ref.trans[1],
        active_toolset.ref.trans[2], active_toolset.ref.rpy[0],
        active_toolset.ref.rpy[1], active_toolset.ref.rpy[2]}};
    const std::array<double, 6> end_values{{
        active_toolset.end.trans[0], active_toolset.end.trans[1],
        active_toolset.end.trans[2], active_toolset.end.rpy[0],
        active_toolset.end.rpy[1], active_toolset.end.rpy[2]}};
    std::error_code posture_ec;
    const auto flange_values =
        robot_->posture(rokae::CoordinateType::flangeInBase, posture_ec);
    check_ec(posture_ec, "posture(flangeInBase)");
    const auto end_in_ref_values =
        robot_->posture(rokae::CoordinateType::endInRef, posture_ec);
    check_ec(posture_ec, "posture(endInRef)");
    print_six("baseFrame", base_xyz_rpy);
    print_six("toolset.ref", ref_values);
    print_six("toolset.end", end_values);
    print_six("flangeInBase", flange_values);
    print_six("endInRef", end_in_ref_values);
    std::cout << "[ROKAE] toolset pos bottom rows: ref=["
              << active_toolset.ref.pos[12] << ","
              << active_toolset.ref.pos[13] << ","
              << active_toolset.ref.pos[14] << ","
              << active_toolset.ref.pos[15] << "] end=["
              << active_toolset.end.pos[12] << ","
              << active_toolset.end.pos[13] << ","
              << active_toolset.end.pos[14] << ","
              << active_toolset.end.pos[15] << "]" << std::endl;
    const Pose world_T_ref = pose_from_frame(active_toolset.ref);
    const Pose flange_T_end = pose_from_frame(active_toolset.end);
    const Pose base_T_flange = pose_from_posture(flange_values);
    const Pose actual_ref_T_end = pose_from_posture(end_in_ref_values);

    const Eigen::Matrix4d flange_T_tcp =
        pose_matrix(base_T_flange).inverse() * pose_matrix(current);
    const Eigen::Matrix4d predicted_ref_T_end =
        pose_matrix(world_T_ref).inverse() * pose_matrix(world_T_base) *
        pose_matrix(base_T_flange) * pose_matrix(flange_T_end);
    const Pose predicted_current = pose_array(predicted_ref_T_end);
    const double frame_translation_error =
        (predicted_ref_T_end.topRightCorner<3, 1>() -
         pose_matrix(actual_ref_T_end).topRightCorner<3, 1>()).norm();
    const double frame_rotation_error =
        rotation_distance(predicted_current, actual_ref_T_end);
    std::cout << "[ROKAE] MoveL frame check: translation_error="
              << frame_translation_error << " m rotation_error="
              << frame_rotation_error << " rad" << std::endl;
    if (frame_translation_error > 0.002 || frame_rotation_error > 0.02) {
      std::ostringstream message;
      message << "MoveL frame conversion is inconsistent with controller state: "
              << "translation_error=" << frame_translation_error
              << " m rotation_error=" << frame_rotation_error
              << " rad; check baseFrame/toolset definitions";
      throw std::runtime_error(message.str());
    }

    const Eigen::Matrix4d base_T_flange_target =
        pose_matrix(target) * flange_T_tcp.inverse();
    const Pose nrt_target = pose_array(
        pose_matrix(world_T_ref).inverse() * pose_matrix(world_T_base) *
        base_T_flange_target * pose_matrix(flange_T_end));
    validate_pose(nrt_target, "NRT ref_T_end target");
    return nrt_target;
  }

  Pose end_in_reference_pose() {
    return posture_pose(rokae::CoordinateType::endInRef);
  }

  Pose posture_pose(rokae::CoordinateType coordinate) {
    std::error_code ec;
    const auto xyz_rpy = robot_->posture(coordinate, ec);
    check_ec(ec, coordinate == rokae::CoordinateType::endInRef
                     ? "posture(endInRef)"
                     : "posture(flangeInBase)");
    return pose_from_posture(xyz_rpy);
  }

  static Pose pose_from_posture(const std::array<double, 6>& xyz_rpy) {
    Pose pose{};
    rokae::Utils::postureToTransArray(xyz_rpy, pose);
    validate_pose(pose, "SDK posture pose");
    return pose;
  }

  static Pose pose_from_frame(const rokae::Frame& frame) {
    const std::array<double, 6> xyz_rpy{{
        frame.trans[0], frame.trans[1], frame.trans[2],
        frame.rpy[0], frame.rpy[1], frame.rpy[2]}};
    return pose_from_posture(xyz_rpy);
  }

  void require_connected() const {
    if (!robot_) throw std::runtime_error("ROKAE robot is not connected");
  }

  std::string recent_controller_errors() {
    std::error_code ec;
    const auto logs = robot_->queryControllerLog(
        10, {rokae::LogInfo::warning, rokae::LogInfo::error}, ec);
    if (ec) return "controller log query failed: " + ec.message();
    if (logs.empty()) return "no recent warning/error controller log";
    std::ostringstream result;
    for (const auto& log : logs) {
      result << "\n  [" << log.timestamp << "] " << log.content;
      if (!log.repair.empty()) result << " (repair: " << log.repair << ")";
    }
    return result.str();
  }

  void abort_nrt_motion() noexcept {
    std::error_code ec;
    robot_->stop(ec);
    ec.clear();
    robot_->moveReset(ec);
  }

  template <typename ProgressFn>
  void wait_until(double timeout, const std::string& trajectory_id,
                  const char* motion_name, ProgressFn progress_fn) {
    if (!std::isfinite(timeout) || timeout <= 0.0) throw std::invalid_argument("timeout must be positive");
    const auto start = Clock::now();
    const auto deadline = Clock::now() + std::chrono::duration<double>(timeout);
    auto next_report = Clock::now();
    bool saw_moving = false;
    std::string event_remark;
    while (Clock::now() < deadline) {
      std::error_code ec;
      const auto state = robot_->operationState(ec); check_ec(ec, "operationState");
      const auto power = robot_->powerState(ec); check_ec(ec, "powerState(wait motion)");
      const MotionProgress progress = progress_fn();
      if (state == rokae::OperationState::moving) saw_moving = true;
      if (power != rokae::PowerState::on) {
        abort_nrt_motion();
        throw std::runtime_error(
            std::string(motion_name) + " aborted because power state became " +
            to_string(power) + "\nRecent controller logs: " +
            recent_controller_errors());
      }

      bool event_reached = false;
      std::error_code event_query_ec;
      const auto info = robot_->queryEventInfo(
          rokae::Event::moveExecution, event_query_ec);
      if (!event_query_ec && !info.empty()) {
        try {
          using namespace rokae::EventInfoKey::MoveExecution;
          const auto id_it = info.find(ID);
          if (id_it != info.end() &&
              std::any_cast<std::string>(id_it->second) == trajectory_id) {
            const auto error_it = info.find(Error);
            if (error_it != info.end()) {
              const auto motion_error =
                  std::any_cast<std::error_code>(error_it->second);
              if (motion_error) {
                abort_nrt_motion();
                throw std::runtime_error(
                    std::string(motion_name) + " controller execution error: " +
                    motion_error.message() + "\nRecent controller logs: " +
                    recent_controller_errors());
              }
            }
            const auto reached_it = info.find(ReachTarget);
            if (reached_it != info.end()) {
              event_reached = std::any_cast<bool>(reached_it->second);
            }
            const auto remark_it = info.find(Remark);
            if (remark_it != info.end()) {
              event_remark = std::any_cast<std::string>(remark_it->second);
            }
          }
        } catch (const std::bad_any_cast&) {
          event_remark = "controller returned malformed moveExecution event";
        }
      }

      if (state == rokae::OperationState::unknown) {
        abort_nrt_motion();
        throw std::runtime_error(
            std::string(motion_name) +
            " operation state became unknown; motion was stopped\nRecent "
            "controller logs: " + recent_controller_errors());
      }
      if ((event_reached || state == rokae::OperationState::idle) &&
          progress.reached) {
        return;
      }

      const auto now = Clock::now();
      if (now >= next_report) {
        std::cout << "[ROKAE] " << motion_name
                  << " waiting: state=" << to_string(state)
                  << " power=" << to_string(power) << " " << progress.detail;
        if (!event_remark.empty()) std::cout << " remark=" << event_remark;
        std::cout << std::endl;
        next_report = now + std::chrono::seconds(1);
      }
      if (!saw_moving && !progress.reached &&
          now - start >= std::chrono::seconds(5)) {
        abort_nrt_motion();
        throw std::runtime_error(
            std::string(motion_name) +
            " did not enter moving state within 5 seconds; check automatic "
            "mode, enable switch, motion permission, safety state and target. "
            "Last event remark: " +
            (event_remark.empty() ? "none" : event_remark) +
            "\nRecent controller logs: " + recent_controller_errors());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    abort_nrt_motion();
    throw std::runtime_error(
        std::string(motion_name) + " timed out after " +
        std::to_string(timeout) +
        " seconds and was stopped\nRecent controller logs: " +
        recent_controller_errors());
  }

  using Controller = typename decltype(std::declval<Robot>().getRtMotionController())::element_type;
  std::string robot_ip_;
  std::string local_ip_;
  RealtimeConfig config_;
  RealtimeCore core_;
  std::unique_ptr<Robot> robot_;
  std::shared_ptr<Controller> controller_;
  std::function<rokae::CartesianPosition()> callback_;
  std::function<rokae::JointPosition()> joint_callback_;
  std::unique_ptr<JointRealtimeCore> joint_core_;
  bool joint_realtime_{false};
  bool cartesian_impedance_{false};
  std::array<std::atomic<double>, 16> measured_pose_{};
  std::atomic<std::uint64_t> measured_pose_sequence_{0};
  std::atomic<bool> measured_pose_available_{false};
  std::array<std::atomic<double>, DoF> measured_joints_{};
  std::atomic<std::uint64_t> measured_joints_sequence_{0};
  std::atomic<bool> measured_joints_available_{false};
  std::shared_ptr<const std::string> callback_error_;
  bool realtime_{false};
  bool entered_realtime_{false};
  bool receiver_started_{false};
  bool preserve_realtime_elbow_{false};
  double realtime_elbow_{0.0};
  std::atomic<double> measured_elbow_{0.0};
};

}  // namespace

class RokaeDriver::Impl {
 public:
  Impl(std::string robot_ip, std::string local_ip, std::string robot_type,
       const RealtimeConfig& config) {
    config.validate();
    if (robot_type == "xmate-6") {
      session = std::make_unique<Session<rokae::xMateRobot, 6>>(
          std::move(robot_ip), std::move(local_ip), config);
    } else if (robot_type == "xmate-er-pro-7") {
      session = std::make_unique<Session<rokae::xMateErProRobot, 7>>(
          std::move(robot_ip), std::move(local_ip), config);
    } else if (robot_type == "standard-6") {
      session = std::make_unique<Session<rokae::StandardRobot, 6>>(
          std::move(robot_ip), std::move(local_ip), config);
    } else {
      throw std::invalid_argument("unsupported robot_type: " + robot_type);
    }
  }
  std::unique_ptr<SessionBase> session;
};

RokaeDriver::RokaeDriver(std::string robot_ip, std::string local_ip,
                         std::string robot_type,
                         const RealtimeConfig& realtime_config)
    : impl_(std::make_unique<Impl>(std::move(robot_ip), std::move(local_ip),
                                  std::move(robot_type), realtime_config)) {}
RokaeDriver::~RokaeDriver() = default;
RokaeDriver::RokaeDriver(RokaeDriver&&) noexcept = default;
RokaeDriver& RokaeDriver::operator=(RokaeDriver&&) noexcept = default;
void RokaeDriver::connect() { impl_->session->connect(); }
void RokaeDriver::disconnect() { impl_->session->disconnect(); }
RobotState RokaeDriver::get_state() { return impl_->session->state(); }
std::vector<double> RokaeDriver::get_joint_positions() { return impl_->session->joints(); }
Pose RokaeDriver::get_tcp_pose() { return impl_->session->tcp_pose(); }
Pose RokaeDriver::get_base_frame() { return impl_->session->base_frame(); }
Pose RokaeDriver::calculate_fk(const std::vector<double>& joints) {
  return impl_->session->calculate_fk(joints);
}
std::pair<bool, std::vector<std::array<double, 2>>> RokaeDriver::get_soft_limits() {
  return impl_->session->soft_limits();
}
std::vector<ControllerLog> RokaeDriver::get_controller_logs(unsigned count, bool faults_only) {
  return impl_->session->controller_logs(count, faults_only);
}
Pose RokaeDriver::preview_nrt_cartesian_target(const Pose& pose) {
  return impl_->session->preview_nrt_cartesian_target(pose);
}
void RokaeDriver::move_joint(const std::vector<double>& q, int speed, double timeout,
                             double max_delta, bool power_on) {
  impl_->session->move_joint(q, speed, timeout, max_delta, power_on);
}
void RokaeDriver::move_cartesian(const Pose& pose, int speed, double timeout,
                                 double max_translation_delta,
                                 double max_rotation_delta, bool power_on) {
  impl_->session->move_cartesian(pose, speed, timeout, max_translation_delta,
                                 max_rotation_delta, power_on);
}
void RokaeDriver::move_to_init(const std::vector<double>& q, int speed,
                               double timeout, double max_delta, bool power_on) {
  move_joint(q, speed, timeout, max_delta, power_on);
}
void RokaeDriver::recover_from_joint_limits(const std::vector<double>& q,
                                            int speed, double timeout,
                                            double max_delta) {
  impl_->session->recover_from_joint_limits(q, speed, timeout, max_delta);
}
void RokaeDriver::recover_disabled_soft_limits(
    const std::vector<double>& q,
    const std::vector<std::array<double, 2>>& limits,
    double clearance_degrees,
    bool request_power_on,
    double jog_rate,
    const std::function<void()>& check_cancel) {
  impl_->session->recover_disabled_soft_limits(q, limits, clearance_degrees, request_power_on, jog_rate, check_cancel);
}
void RokaeDriver::start_realtime_cartesian(bool power_on) { impl_->session->start_realtime(power_on); }
void RokaeDriver::start_realtime_cartesian_impedance(bool power_on) {
  impl_->session->start_realtime_impedance(power_on);
}
void RokaeDriver::start_realtime_joint(const JointRealtimeConfig& config, bool power_on) {
  impl_->session->start_realtime_joint(config, power_on);
}
bool RokaeDriver::set_target_pose(const Pose& pose, double source, std::int64_t sequence) {
  return impl_->session->set_target(pose, source, sequence);
}
bool RokaeDriver::set_target_joints(const std::vector<double>& joints, double source,
                                    std::int64_t sequence) {
  return impl_->session->set_target_joints(joints, source, sequence);
}
std::vector<double> RokaeDriver::get_command_joints() const {
  return impl_->session->command_joints();
}
void RokaeDriver::hold() { impl_->session->hold(); }
void RokaeDriver::stop_realtime() { impl_->session->stop_realtime(); }
void RokaeDriver::stop() { impl_->session->stop(); }
Pose RokaeDriver::get_command_pose() const { return impl_->session->command_pose(); }
Diagnostics RokaeDriver::diagnostics() const { return impl_->session->diagnostics(); }
double RokaeDriver::monotonic_time() { return now_seconds(); }

}  // namespace anydex::rokae_bridge
