#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>

namespace fixed_impedance {
using Pose = std::array<double, 16>;
using Joints = std::array<double, 7>;
// 回位必须同时满足位置与速度，调用方要求连续满足 0.5 秒。
inline bool home_stationary(const Joints& q,const Joints& home,const Joints& dq) {
  for(std::size_t i=0;i<q.size();++i)
    if(!std::isfinite(q[i])||!std::isfinite(home[i])||!std::isfinite(dq[i])||
       std::abs(q[i]-home[i])>.002||std::abs(dq[i])>=.01)return false;
  return true;
}
struct Limits {
  double startup_seconds{2.0};
  double startup_joint_delta{0.04};  // rad; initial phase: do not touch the robot.
  double joint_delta{0.50};         // rad from initial, including redundancy motion.
  double joint_speed{0.30};         // rad/s; deliberately small push test.
  double translation{0.06};         // m from the fixed equilibrium.
  double rotation{0.30};            // rad from the fixed equilibrium.
};
// 显式轻推试验仅在初始无接触阶段结束后扩大速度/位移实验边界。
// 不是额定安全范围；启动、姿态及控制器保护不变。
inline Limits phase_limits(double seconds, bool light_push) {
  Limits result;
  if(light_push && seconds>=result.startup_seconds) {
    result.joint_delta=1.0; // rad；容纳轻推时的冗余姿态变化，仍限制关节总行程。
    result.joint_speed=.70; // rad/s；轻推阶段实验值，不改变启动检查或 SDK 限幅。
    result.translation=.10;
  }
  return result;
}
enum class Stop { none, invalid_state, startup_drift, joint_excursion,
                  joint_speed, translation, rotation, duration, interrupted,
                  buffer_full, callback_exception, controller_fault };
// 计时结束不等于无条件成功：启动、运行异常及清理结果都要参与退出判定。
inline bool successful_exit(Stop stop, bool started, bool cleaned, bool runtime_failed) {
  return started && cleaned && !runtime_failed &&
         (stop==Stop::duration || stop==Stop::interrupted);
}
inline const char* reason(Stop value) {
  switch(value) {
    case Stop::none: return "none";
    case Stop::invalid_state: return "invalid_state";
    case Stop::startup_drift: return "startup_joint_drift";
    case Stop::joint_excursion: return "joint_excursion";
    case Stop::joint_speed: return "joint_speed";
    case Stop::translation: return "tcp_displacement";
    case Stop::rotation: return "tcp_rotation";
    case Stop::duration: return "duration_completed";
    case Stop::interrupted: return "user_interrupt";
    case Stop::buffer_full: return "trace_buffer_full";
    case Stop::callback_exception: return "callback_exception";
    case Stop::controller_fault: return "controller_power_or_mode_fault";
  }
  return "unknown";
}
inline bool valid_pose(const Pose& p) {
  for (double v:p) if (!std::isfinite(v)) return false;
  using Matrix = Eigen::Matrix<double,4,4,Eigen::RowMajor>;
  const Eigen::Map<const Matrix> t(p.data());
  const Eigen::Matrix3d r=t.topLeftCorner<3,3>();
  return (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()<1e-5 &&
         std::abs(r.determinant()-1)<1e-5 &&
         std::abs(p[12])+std::abs(p[13])+std::abs(p[14])+std::abs(p[15]-1)<1e-8;
}
inline double translation_error(const Pose& a, const Pose& b) {
  return std::sqrt(std::pow(a[3]-b[3],2)+std::pow(a[7]-b[7],2)+std::pow(a[11]-b[11],2));
}
inline double rotation_error(const Pose& a, const Pose& b) {
  using Matrix=Eigen::Matrix<double,4,4,Eigen::RowMajor>;
  const Eigen::Map<const Matrix> x(a.data()), y(b.data());
  const Eigen::Matrix3d r=x.topLeftCorner<3,3>().transpose()*y.topLeftCorner<3,3>();
  return std::acos(std::clamp((r.trace()-1.0)/2.0,-1.0,1.0));
}
inline Stop evaluate(double seconds, const Pose& measured, const Pose& initial,
                     const Joints& q, const Joints& q0, const Joints& dq,
                     const Limits& limits={}) {
  if (!std::isfinite(seconds)||seconds<0||!valid_pose(measured)||!valid_pose(initial))
    return Stop::invalid_state;
  for (std::size_t i=0;i<q.size();++i) {
    if (!std::isfinite(q[i])||!std::isfinite(q0[i])||!std::isfinite(dq[i])) return Stop::invalid_state;
    if (std::abs(dq[i])>limits.joint_speed) return Stop::joint_speed;
    if (seconds<limits.startup_seconds && std::abs(q[i]-q0[i])>limits.startup_joint_delta)
      return Stop::startup_drift;
    if (std::abs(q[i]-q0[i])>limits.joint_delta) return Stop::joint_excursion;
  }
  if (translation_error(measured,initial)>limits.translation) return Stop::translation;
  if (rotation_error(measured,initial)>limits.rotation) return Stop::rotation;
  return Stop::none;
}
}
