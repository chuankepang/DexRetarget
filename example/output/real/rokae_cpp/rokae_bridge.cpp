#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "rokae/robot.h"

namespace {

using Pose = std::array<double, 16>;

void copy_error(const std::string &message, char *buffer, std::size_t size) {
  if (buffer == nullptr || size == 0) {
    return;
  }
  std::strncpy(buffer, message.c_str(), size - 1);
  buffer[size - 1] = '\0';
}

void check_error(const std::error_code &ec, const char *operation) {
  if (ec) {
    throw std::runtime_error(std::string(operation) + ": " + ec.message());
  }
}

class SessionBase {
 public:
  virtual ~SessionBase() = default;
  virtual void connect(const std::string &robot_ip, const std::string &local_ip,
                       unsigned network_tolerance) = 0;
  virtual void start(bool power_on) = 0;
  virtual Pose tcp_pose() = 0;
  virtual void set_target(const Pose &target) = 0;
  virtual void hold() = 0;
  virtual void stop() = 0;
  virtual void disconnect() = 0;
};

template <typename Robot>
class Session final : public SessionBase {
 public:
  ~Session() override {
    try {
      disconnect();
    } catch (...) {
    }
  }

  void connect(const std::string &robot_ip, const std::string &local_ip,
               unsigned network_tolerance) override {
    if (robot_) {
      return;
    }
    robot_ = std::make_unique<Robot>(robot_ip, local_ip);
    network_tolerance_ = network_tolerance;
  }

  void start(bool power_on) override {
    if (!robot_) {
      throw std::runtime_error("robot is not connected");
    }
    if (started_) {
      return;
    }

    std::error_code ec;
    robot_->setOperateMode(rokae::OperateMode::automatic, ec);
    check_error(ec, "setOperateMode(automatic)");
    robot_->setRtNetworkTolerance(network_tolerance_, ec);
    check_error(ec, "setRtNetworkTolerance");
    robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
    check_error(ec, "setMotionControlMode(RtCommand)");
    if (power_on) {
      robot_->setPowerState(true, ec);
      check_error(ec, "setPowerState(true)");
    }
    if (robot_->powerState(ec) != rokae::PowerState::on) {
      check_error(ec, "powerState");
      throw std::runtime_error(
          "robot is not powered; power it externally or request power_on");
    }

    robot_->startReceiveRobotState(
        std::chrono::milliseconds(1), {rokae::RtSupportedFields::tcpPose_m});
    robot_->updateRobotState(std::chrono::milliseconds(10));
    Pose initial{};
    if (robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, initial) != 0) {
      throw std::runtime_error("getStateData(tcpPose_m) failed");
    }
    std::atomic_store(&latest_target_, std::make_shared<const Pose>(initial));
    std::atomic_store(&latest_measured_, std::make_shared<const Pose>(initial));

    controller_ = robot_->getRtMotionController().lock();
    if (!controller_) {
      throw std::runtime_error("getRtMotionController returned an empty handle");
    }
    controller_->startMove(rokae::RtControllerMode::cartesianPosition);
    std::function<rokae::CartesianPosition()> callback = [this]() {
      auto target = std::atomic_load(&latest_target_);
      rokae::CartesianPosition command;
      command.pos = *target;
      return command;
    };
    controller_->setControlLoop(callback);
    controller_->startLoop(false);
    started_ = true;
  }

  Pose tcp_pose() override {
    if (!robot_) {
      throw std::runtime_error("robot is not connected");
    }
    if (started_) {
      robot_->updateRobotState(std::chrono::milliseconds(10));
      Pose measured{};
      if (robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, measured) != 0) {
        throw std::runtime_error("getStateData(tcpPose_m) failed");
      }
      std::atomic_store(&latest_measured_, std::make_shared<const Pose>(measured));
    }
    auto measured = std::atomic_load(&latest_measured_);
    if (!measured) {
      throw std::runtime_error("TCP pose is unavailable before realtime start");
    }
    return *measured;
  }

  void set_target(const Pose &target) override {
    if (!started_) {
      throw std::runtime_error("realtime Cartesian loop is not started");
    }
    std::atomic_store(&latest_target_, std::make_shared<const Pose>(target));
  }

  void hold() override {
    auto target = std::atomic_load(&latest_target_);
    if (target) {
      std::atomic_store(&latest_target_, std::make_shared<const Pose>(*target));
    }
  }

  void stop() override {
    if (!started_) {
      return;
    }
    std::exception_ptr first_error;
    if (controller_) {
      try {
        // Tell the controller to hold/stop before removing the client loop.
        controller_->stopMove();
      } catch (...) {
        first_error = std::current_exception();
      }
      try {
        controller_->stopLoop();
      } catch (...) {
        if (!first_error) {
          first_error = std::current_exception();
        }
      }
    }
    robot_->stopReceiveRobotState();
    started_ = false;
    controller_.reset();
    if (first_error) {
      std::rethrow_exception(first_error);
    }
  }

  void disconnect() override {
    if (!robot_) {
      return;
    }
    if (started_) {
      stop();
    }
    std::error_code ec;
    robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot_->disconnectFromRobot(ec);
    check_error(ec, "disconnectFromRobot");
    robot_.reset();
  }

 private:
  using Controller = typename decltype(
      std::declval<Robot>().getRtMotionController())::element_type;

  std::unique_ptr<Robot> robot_;
  std::shared_ptr<Controller> controller_;
  std::shared_ptr<const Pose> latest_target_;
  std::shared_ptr<const Pose> latest_measured_;
  unsigned network_tolerance_{20};
  bool started_{false};
};

struct Handle {
  std::unique_ptr<SessionBase> session;
  std::string last_error;
};

template <typename Function>
int invoke(Handle *handle, Function &&function) noexcept {
  if (handle == nullptr || !handle->session) {
    return -1;
  }
  try {
    function();
    handle->last_error.clear();
    return 0;
  } catch (const std::exception &exception) {
    handle->last_error = exception.what();
  } catch (...) {
    handle->last_error = "unknown C++ exception";
  }
  return -1;
}

}  // namespace

extern "C" {

void *anydex_rokae_create(const char *robot_type, char *error_buffer,
                          std::size_t error_buffer_size) noexcept {
  try {
    if (robot_type == nullptr) {
      throw std::invalid_argument("robot_type is null");
    }
    auto handle = std::make_unique<Handle>();
    const std::string type(robot_type);
    if (type == "xmate-6") {
      handle->session = std::make_unique<Session<rokae::xMateRobot>>();
    } else if (type == "xmate-er-pro-7") {
      handle->session = std::make_unique<Session<rokae::xMateErProRobot>>();
    } else if (type == "standard-6") {
      handle->session = std::make_unique<Session<rokae::StandardRobot>>();
    } else {
      throw std::invalid_argument("unsupported robot_type: " + type);
    }
    return handle.release();
  } catch (const std::exception &exception) {
    copy_error(exception.what(), error_buffer, error_buffer_size);
    return nullptr;
  }
}

int anydex_rokae_connect(void *raw_handle, const char *robot_ip,
                         const char *local_ip,
                         unsigned network_tolerance) noexcept {
  auto *handle = static_cast<Handle *>(raw_handle);
  return invoke(handle, [&]() {
    if (robot_ip == nullptr || local_ip == nullptr) {
      throw std::invalid_argument("robot_ip/local_ip is null");
    }
    handle->session->connect(robot_ip, local_ip, network_tolerance);
  });
}

int anydex_rokae_start(void *raw_handle, int power_on) noexcept {
  auto *handle = static_cast<Handle *>(raw_handle);
  return invoke(handle, [&]() { handle->session->start(power_on != 0); });
}

int anydex_rokae_get_tcp_pose(void *raw_handle, double *output) noexcept {
  auto *handle = static_cast<Handle *>(raw_handle);
  return invoke(handle, [&]() {
    if (output == nullptr) {
      throw std::invalid_argument("output pose is null");
    }
    const Pose pose = handle->session->tcp_pose();
    std::copy(pose.begin(), pose.end(), output);
  });
}

int anydex_rokae_set_target_pose(void *raw_handle, const double *input) noexcept {
  auto *handle = static_cast<Handle *>(raw_handle);
  return invoke(handle, [&]() {
    if (input == nullptr) {
      throw std::invalid_argument("target pose is null");
    }
    Pose target{};
    std::copy(input, input + target.size(), target.begin());
    handle->session->set_target(target);
  });
}

int anydex_rokae_hold(void *raw_handle) noexcept {
  auto *handle = static_cast<Handle *>(raw_handle);
  return invoke(handle, [&]() { handle->session->hold(); });
}

int anydex_rokae_stop(void *raw_handle) noexcept {
  auto *handle = static_cast<Handle *>(raw_handle);
  return invoke(handle, [&]() { handle->session->stop(); });
}

int anydex_rokae_disconnect(void *raw_handle) noexcept {
  auto *handle = static_cast<Handle *>(raw_handle);
  return invoke(handle, [&]() { handle->session->disconnect(); });
}

const char *anydex_rokae_last_error(void *raw_handle) noexcept {
  auto *handle = static_cast<Handle *>(raw_handle);
  return handle == nullptr ? "invalid ROKAE handle" : handle->last_error.c_str();
}

void anydex_rokae_destroy(void *raw_handle) noexcept {
  delete static_cast<Handle *>(raw_handle);
}

}  // extern "C"
