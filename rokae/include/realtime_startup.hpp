#pragma once
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace rokae_startup {
// 仅在 startMove 前调用：配置完负载/TCP 后重新建立订阅，避免沿用旧订阅队列。
// 1 kHz 持续流不以“读到空队列”作为成功条件；有限次正超时接收均须成功。
// SDK 0.3.4 无公开帧序号/源时间戳校验，这不是绝对报文年龄的证明。
template<class Robot, class Clock = std::chrono::steady_clock>
std::size_t receive_startup_state(Robot& robot, const std::vector<std::string>& fields) {
  using namespace std::chrono;
  robot.stopReceiveRobotState();
  // 官方接口会等待首帧；失败由调用方清理订阅及上电状态，不退回缓存位姿。
  robot.startReceiveRobotState(milliseconds(1), fields);
  const auto started = Clock::now();
  constexpr std::size_t packets = 3;
  for (std::size_t i = 0; i < packets; ++i) {
    if (!robot.updateRobotState(milliseconds(20)))
      throw std::runtime_error("missing realtime startup packet " + std::to_string(i+1));
    if (Clock::now() - started > milliseconds(60))
      throw std::runtime_error("realtime startup sampling exceeded 60 ms");
  }
  return packets;
}
}  // namespace rokae_startup
