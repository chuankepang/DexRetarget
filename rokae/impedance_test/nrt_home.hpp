#pragma once
#include "guard.hpp"
#include "rokae/robot.h"
#include <any>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace fixed_impedance {
// 仅位置控制回 home：在线倍率由 5% 提至 50%；阻抗阶段参数不使用此值。
inline constexpr double kHomeSpeedScale = .50;
inline constexpr int kHomeSdkSpeed = 100;
inline void nrt_check(std::error_code ec, const char* name) {
  if (ec) throw std::runtime_error(std::string(name)+": "+ec.message());
}

// 使用当前 SDK 0.3.4 已有的 NRT 接口移植新版回位路径；不调用旧 RT MoveJ。
// 不以 idle 单独判定完成：同时要求目标关节和静止反馈，拒绝 unknown/下电/执行错误。
template<class Robot, class Cancelled>
bool nrt_home(Robot& robot, const Joints& home, Cancelled cancelled,
              std::chrono::milliseconds timeout = std::chrono::seconds(60)) {
  std::error_code ec;
  try {
    if (cancelled()) throw std::runtime_error("home interrupted");
    robot.moveReset(ec); nrt_check(ec,"NRT moveReset before home");
    robot.adjustSpeedOnline(kHomeSpeedScale,ec); nrt_check(ec,"NRT home online scale");
    const auto start = robot.jointPos(ec); nrt_check(ec,"NRT home initial joints");
    bool needed = false;
    for (std::size_t i=0;i<7;++i) {
      if (!std::isfinite(start[i])) throw std::runtime_error("invalid NRT initial joints");
      needed = needed || std::abs(start[i]-home[i])>1e-4;
    }
    std::string id;
    if (needed) {
      const std::vector<double> target(home.begin(),home.end());
      robot.moveAppend(std::vector<rokae::MoveAbsJCommand>{rokae::MoveAbsJCommand(target,kHomeSdkSpeed,0)},id,ec);
      nrt_check(ec,"NRT MoveAbsJ append");
      if (id.empty()) throw std::runtime_error("NRT home missing trajectory ID");
      robot.moveStart(ec); nrt_check(ec,"NRT MoveAbsJ start");
    }
    const auto deadline=std::chrono::steady_clock::now()+timeout;
    while (std::chrono::steady_clock::now()<deadline) {
      if (cancelled()) throw std::runtime_error("home interrupted");
      const auto state=robot.operationState(ec); nrt_check(ec,"NRT home operation");
      if (state==rokae::OperationState::unknown) throw std::runtime_error("unknown operation during home");
      const auto power=robot.powerState(ec); nrt_check(ec,"NRT home power");
      if (power!=rokae::PowerState::on) throw std::runtime_error("power lost during home");
      const auto q=robot.jointPos(ec); nrt_check(ec,"NRT home joints");
      const auto dq=robot.jointVel(ec); nrt_check(ec,"NRT home velocity");
      for (std::size_t i=0;i<7;++i)
        if (!std::isfinite(q[i]) || !std::isfinite(dq[i]))
          throw std::runtime_error("invalid NRT home feedback");
      if (needed) {
        const auto info=robot.queryEventInfo(rokae::Event::moveExecution,ec);
        nrt_check(ec,"NRT home execution event");
        using namespace rokae::EventInfoKey::MoveExecution;
        const auto key=info.find(ID);
        if (key!=info.end() && std::any_cast<std::string>(key->second)==id) {
          const auto err=info.find(Error);
          if (err!=info.end()) nrt_check(std::any_cast<std::error_code>(err->second),"NRT home execution");
        }
      }
      if (state==rokae::OperationState::idle && home_stationary(q,home,dq)) {
        robot.moveReset(ec); nrt_check(ec,"NRT moveReset after home");
        return needed;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("NRT home timed out");
  } catch (...) {
    // 请求停止并清空队列，随后由外层 Session 请求下电；不重试运动。
    std::error_code stop_ec;
    robot.stop(stop_ec);
    robot.moveReset(stop_ec);
    throw;
  }
}
}  // namespace fixed_impedance
