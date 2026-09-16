#include "rokae_driver.hpp"

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
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(workspace_min[axis]) ||
        !std::isfinite(workspace_max[axis]) ||
        workspace_min[axis] >= workspace_max[axis]) {
      throw std::invalid_argument("workspace bounds must be finite and ordered");
    }
  }
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
  virtual Pose preview_nrt_cartesian_target(const Pose&) = 0;
  virtual void move_joint(const std::vector<double>&, int, double, double, bool) = 0;
  virtual void move_cartesian(const Pose&, int, double, double, double, bool) = 0;
  virtual void start_realtime(bool) = 0;
  virtual bool set_target(const Pose&, double, std::int64_t) = 0;
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
      const auto measured = std::atomic_load(&measured_pose_);
      if (measured) return *measured;
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

  void prepare_motion(bool power_on, rokae::MotionControlMode mode) {
    require_connected();
    std::error_code ec;
    std::cout << "[ROKAE] requesting automatic operate mode ..." << std::endl;
    robot_->setOperateMode(rokae::OperateMode::automatic, ec);
    check_ec(ec, "setOperateMode(automatic)");
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
    std::cout << "[ROKAE] power=on; selecting motion control mode ..." << std::endl;
    robot_->setMotionControlMode(mode, ec);
    check_ec(ec, "setMotionControlMode");
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
    if (realtime_) return;
    (void)tcp_pose();  // Validate that a controller TCP is available before mode switch.
    std::error_code ec;
    const auto active_toolset = robot_->toolset(ec);
    check_ec(ec, "toolset");
    std::cout << "[ROKAE] active load: mass=" << active_toolset.load.mass
              << " kg cog=[" << active_toolset.load.cog[0] << ","
              << active_toolset.load.cog[1] << ","
              << active_toolset.load.cog[2] << "] m inertia=["
              << active_toolset.load.inertia[0] << ","
              << active_toolset.load.inertia[1] << ","
              << active_toolset.load.inertia[2] << "] kg*m^2" << std::endl;
    robot_->setRtNetworkTolerance(config_.rt_network_tolerance, ec);
    check_ec(ec, "setRtNetworkTolerance");
    prepare_motion(power_on, rokae::MotionControlMode::RtCommand);
    entered_realtime_ = true;
    std::atomic_store(&callback_error_, std::shared_ptr<const std::string>{});
    try {
      robot_->startReceiveRobotState(
          std::chrono::milliseconds(1),
          {rokae::RtSupportedFields::tcpPose_m,
           rokae::RtSupportedFields::jointPos_m,
           rokae::RtSupportedFields::jointVel_m});
      receiver_started_ = true;
      robot_->updateRobotState(std::chrono::milliseconds(10));
      Pose measured{};
      if (robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, measured) != 0) {
        throw std::runtime_error("getStateData(tcpPose_m) failed during realtime initialization");
      }
      validate_pose(measured, "initial realtime measured TCP");
      std::atomic_store(&measured_pose_, std::make_shared<const Pose>(measured));
      core_.reset(measured, now_seconds());

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
      std::function<rokae::CartesianPosition()> callback = [this]() {
        try {
          Pose current{};
          if (robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, current) == 0) {
            std::atomic_store(&measured_pose_, std::make_shared<const Pose>(current));
          }
          rokae::CartesianPosition command(core_.step(now_seconds()));
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
      controller_->startMove(rokae::RtControllerMode::cartesianPosition);
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
      callback_ = {};
      robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
      entered_realtime_ = false;
      throw;
    }
    realtime_ = true;
  }

  bool set_target(const Pose& target, double source, std::int64_t sequence) override {
    if (!realtime_) throw std::runtime_error("realtime Cartesian control is not running");
    return core_.publish(target, source, sequence, now_seconds());
  }

  void hold() override { if (realtime_) core_.hold(); }

  void stop_realtime() override {
    if (!realtime_ && !controller_ && !receiver_started_) return;
    core_.request_stop();
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
    realtime_ = false;
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
    auto result = core_.diagnostics(now_seconds());
    result.connected = static_cast<bool>(robot_);
    result.realtime_running = realtime_;
    const auto callback_error = std::atomic_load(&callback_error_);
    if (callback_error) result.callback_error = *callback_error;
    return result;
  }

 private:
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
  std::shared_ptr<const Pose> measured_pose_;
  std::shared_ptr<const std::string> callback_error_;
  bool realtime_{false};
  bool entered_realtime_{false};
  bool receiver_started_{false};
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
void RokaeDriver::start_realtime_cartesian(bool power_on) { impl_->session->start_realtime(power_on); }
bool RokaeDriver::set_target_pose(const Pose& pose, double source, std::int64_t sequence) {
  return impl_->session->set_target(pose, source, sequence);
}
void RokaeDriver::hold() { impl_->session->hold(); }
void RokaeDriver::stop_realtime() { impl_->session->stop_realtime(); }
void RokaeDriver::stop() { impl_->session->stop(); }
Pose RokaeDriver::get_command_pose() const { return impl_->session->command_pose(); }
Diagnostics RokaeDriver::diagnostics() const { return impl_->session->diagnostics(); }
double RokaeDriver::monotonic_time() { return now_seconds(); }

}  // namespace anydex::rokae_bridge
