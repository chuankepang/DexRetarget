#include "rokae_driver.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace anydex::rokae_bridge {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDampingRatio = 1.25;

double clamp_scalar(double value, double lower, double upper) {
  return std::clamp(value, lower, upper);
}

std::vector<double> checked_joints(const std::vector<double>& joints,
                                   std::size_t dof, const char* name) {
  if (joints.size() != dof) {
    throw std::invalid_argument(std::string(name) + " has the wrong length");
  }
  for (double value : joints) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string(name) + " contains a non-finite value");
    }
  }
  return joints;
}

}  // namespace

void JointRealtimeConfig::validate() const {
  const std::size_t count = max_velocity.size();
  if (count < 1 || max_acceleration.size() != count || max_jerk.size() != count ||
      position_min.size() != count || position_max.size() != count) {
    throw std::invalid_argument("joint stream vectors must share one positive length");
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (!std::isfinite(max_velocity[index]) || max_velocity[index] <= 0.0 ||
        !std::isfinite(max_acceleration[index]) || max_acceleration[index] <= 0.0 ||
        !std::isfinite(max_jerk[index]) || max_jerk[index] <= 0.0) {
      throw std::invalid_argument("joint velocity, acceleration and jerk limits must be positive");
    }
    if (!std::isfinite(position_min[index]) || !std::isfinite(position_max[index]) ||
        position_min[index] >= position_max[index]) {
      throw std::invalid_argument("joint position limits must be finite and ordered");
    }
  }
  if (!std::isfinite(reference_cutoff_hz) || reference_cutoff_hz <= 0.0 ||
      !std::isfinite(max_target_delta) || max_target_delta <= 0.0 ||
      !std::isfinite(hold_timeout) || hold_timeout <= 0.0 ||
      !std::isfinite(stop_timeout) || !std::isfinite(max_source_age) ||
      max_source_age <= 0.0) {
    throw std::invalid_argument("joint stream timing limits must be positive");
  }
  if (!std::isfinite(deadband) || deadband < 0.0 ||
      !std::isfinite(future_tolerance) || future_tolerance < 0.0) {
    throw std::invalid_argument("joint deadband and future tolerance must be non-negative");
  }
  if (stop_timeout <= hold_timeout) {
    throw std::invalid_argument("joint stop_timeout must exceed hold_timeout");
  }
}

class JointRealtimeCore::Impl {
 public:
  struct Sample {
    std::vector<double> joints;
    double source_timestamp{0.0};
    double receive_timestamp{0.0};
    std::int64_t sequence{0};
  };

  explicit Impl(JointRealtimeConfig value) : config(std::move(value)) {
    config.validate();
    const std::size_t count = config.dof();
    position.assign(count, 0.0);
    filtered.assign(count, 0.0);
    velocity.assign(count, 0.0);
    acceleration.assign(count, 0.0);
  }

  void reset(const std::vector<double>& joints, double timestamp) {
    auto initial = checked_joints(joints, config.dof(), "initial joints");
    if (!std::isfinite(timestamp)) {
      throw std::invalid_argument("invalid joint reset timestamp");
    }
    for (std::size_t index = 0; index < initial.size(); ++index) {
      if (initial[index] < config.position_min[index] ||
          initial[index] > config.position_max[index]) {
        throw std::invalid_argument("initial joints are outside the stream limits");
      }
    }
    position = initial;
    filtered = initial;
    std::fill(velocity.begin(), velocity.end(), 0.0);
    std::fill(acceleration.begin(), acceleration.end(), 0.0);
    last_step_time = timestamp;
    initialized = true;
    hold_requested.store(false);
    stop_requested.store(false);
    should_finish_flag.store(false);
    watchdog.store(0);
    auto sample = std::make_shared<Sample>();
    sample->joints = initial;
    sample->source_timestamp = timestamp;
    sample->receive_timestamp = timestamp;
    sample->sequence = -1;
    std::atomic_store(&latest, std::shared_ptr<const Sample>(sample));
    publish_state(initial, velocity);
    latest_sequence.store(-1);
    latest_source_time = timestamp;
  }

  bool publish(const std::vector<double>& target, double source_timestamp,
               std::int64_t sequence, double receive_timestamp) {
    if (!initialized) throw std::runtime_error("joint stream is not initialized");
    auto joints = checked_joints(target, config.dof(), "joint target");
    if (!std::isfinite(source_timestamp) || !std::isfinite(receive_timestamp)) {
      throw std::invalid_argument("joint target timestamps must be finite");
    }
    const double source_age = receive_timestamp - source_timestamp;
    std::lock_guard<std::mutex> guard(publish_mutex);
    if (source_age > config.max_source_age || source_age < -config.future_tolerance) {
      rejected_source_age.fetch_add(1);
      return false;
    }
    const auto current = std::atomic_load(&command);
    if (current) {
      double peak = 0.0;
      for (std::size_t index = 0; index < joints.size(); ++index) {
        peak = std::max(peak, std::abs(joints[index] - (*current)[index]));
      }
      if (peak > config.max_target_delta) {
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
    auto sample = std::make_shared<Sample>();
    sample->joints = std::move(joints);
    sample->source_timestamp = source_timestamp;
    sample->receive_timestamp = receive_timestamp;
    sample->sequence = sequence;
    std::atomic_store(&latest, std::shared_ptr<const Sample>(std::move(sample)));
    latest_sequence.store(sequence);
    latest_source_time = source_timestamp;
    accepted.fetch_add(1);
    hold_requested.store(false);
    return true;
  }

  std::vector<double> step(double timestamp) {
    if (!initialized) throw std::runtime_error("joint stream is not initialized");
    if (!std::isfinite(timestamp)) throw std::invalid_argument("invalid joint step timestamp");
    double dt = timestamp - last_step_time;
    dt = std::clamp(dt, 0.00025, 0.004);
    last_step_time = timestamp;
    const auto sample = std::atomic_load(&latest);
    const double age = sample ? std::max(0.0, timestamp - sample->receive_timestamp)
                              : std::numeric_limits<double>::infinity();
    last_age.store(age);
    std::vector<double> desired = position;
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
      desired = sample->joints;
    }

    const double blend = 1.0 - std::exp(-2.0 * kPi * config.reference_cutoff_hz * dt);
    for (std::size_t index = 0; index < position.size(); ++index) {
      const double filter_error = desired[index] - filtered[index];
      if (std::abs(filter_error) >= config.deadband) {
        filtered[index] += blend * filter_error;
      }
      double error = filtered[index] - position[index];
      if (std::abs(error) < config.deadband) error = 0.0;
      const double natural = std::min(
          2.0 * config.max_acceleration[index] / config.max_velocity[index],
          0.5 / dt);
      const double desired_acceleration = clamp_scalar(
          natural * natural * error - 2.0 * kDampingRatio * natural * velocity[index],
          -config.max_acceleration[index], config.max_acceleration[index]);
      const double jerk = clamp_scalar(
          (desired_acceleration - acceleration[index]) / dt,
          -config.max_jerk[index], config.max_jerk[index]);
      acceleration[index] = clamp_scalar(
          acceleration[index] + jerk * dt,
          -config.max_acceleration[index], config.max_acceleration[index]);
      velocity[index] = clamp_scalar(
          velocity[index] + acceleration[index] * dt,
          -config.max_velocity[index], config.max_velocity[index]);
      position[index] += velocity[index] * dt;
      const bool low = position[index] < config.position_min[index];
      const bool high = position[index] > config.position_max[index];
      position[index] = clamp_scalar(
          position[index], config.position_min[index], config.position_max[index]);
      if (low || high) {
        velocity[index] = 0.0;
        acceleration[index] = 0.0;
      }
    }
    publish_state(position, velocity);
    callback_count.fetch_add(1);
    return position;
  }

  void publish_state(const std::vector<double>& joints,
                     const std::vector<double>& joint_velocity) {
    std::atomic_store(&command, std::make_shared<const std::vector<double>>(joints));
    auto snapshot = std::make_shared<State>();
    snapshot->position = joints;
    snapshot->velocity = joint_velocity;
    std::atomic_store(&state, std::shared_ptr<const State>(std::move(snapshot)));
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
    result.latest_command_age =
        sample ? std::max(0.0, timestamp - sample->receive_timestamp)
               : std::numeric_limits<double>::infinity();
    result.realtime_route = "joint";
    const auto snapshot = std::atomic_load(&state);
    if (snapshot) {
      result.commanded_joint_position = snapshot->position;
      result.commanded_joint_velocity = snapshot->velocity;
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

  JointRealtimeConfig config;
  struct State {
    std::vector<double> position;
    std::vector<double> velocity;
  };

  std::shared_ptr<const Sample> latest;
  std::shared_ptr<const std::vector<double>> command;
  std::shared_ptr<const State> state;
  mutable std::mutex publish_mutex;
  std::vector<double> position;
  std::vector<double> filtered;
  std::vector<double> velocity;
  std::vector<double> acceleration;
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

JointRealtimeCore::JointRealtimeCore(const JointRealtimeConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}
JointRealtimeCore::~JointRealtimeCore() = default;
JointRealtimeCore::JointRealtimeCore(JointRealtimeCore&&) noexcept = default;
JointRealtimeCore& JointRealtimeCore::operator=(JointRealtimeCore&&) noexcept = default;

void JointRealtimeCore::reset(const std::vector<double>& joints, double timestamp) {
  impl_->reset(joints, timestamp);
}
bool JointRealtimeCore::publish(const std::vector<double>& target, double source,
                                std::int64_t sequence, double received) {
  return impl_->publish(target, source, sequence, received);
}
std::vector<double> JointRealtimeCore::step(double timestamp) {
  return impl_->step(timestamp);
}
void JointRealtimeCore::hold() { impl_->hold_requested.store(true); }
void JointRealtimeCore::request_stop() { impl_->stop_requested.store(true); }
std::vector<double> JointRealtimeCore::command_joints() const {
  const auto joints = std::atomic_load(&impl_->command);
  if (!joints) throw std::runtime_error("joint command is unavailable");
  return *joints;
}
Diagnostics JointRealtimeCore::diagnostics(double timestamp) const {
  return impl_->diagnostics(timestamp);
}
bool JointRealtimeCore::should_finish() const {
  return impl_->should_finish_flag.load();
}

}  // namespace anydex::rokae_bridge
