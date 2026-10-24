#pragma once
#include "rokae/robot.h"
#include <cmath>
#include <limits>
#include <ostream>

namespace fixed_impedance {
// 这些是控制器模型估计/传感器反馈，不是独立的外置测力仪读数。
// 诊断失败输出 unavailable，绝不把未赋值数组伪装成“外力为零”。
template<class Robot>
void write_force_snapshot(Robot& robot, std::ostream& out, const char* phase) {
  const double missing=std::numeric_limits<double>::quiet_NaN();
  std::array<double,7> joint,external;
  std::array<double,3> torque,force;
  joint.fill(missing);external.fill(missing);torque.fill(missing);force.fill(missing);
  std::error_code ec;
  out<<"force_snapshot_phase="<<phase<<'\n';
  try {
    robot.getEndTorque(rokae::FrameType::world,joint,external,torque,force,ec);
    const auto finite=[](const auto& values){for(double x:values)if(!std::isfinite(x))return false;return true;};
    if(ec||!finite(joint)||!finite(external)||!finite(torque)||!finite(force)) {
      out<<"force_snapshot_available=0\nforce_snapshot_error="<<(ec?ec.message():"nonfinite or incomplete feedback")<<'\n';
      return;
    }
    out<<"force_snapshot_available=1\n";
    const auto emit=[&](const char* name,const auto& values){out<<name<<'=';for(double x:values)out<<' '<<x;out<<'\n';};
    emit("joint_torque_measured_Nm",joint);emit("joint_external_estimate_Nm",external);
    emit("world_cart_torque_estimate_Nm",torque);emit("world_cart_force_estimate_N",force);
  } catch(const std::exception& e) {
    out<<"force_snapshot_available=0\nforce_snapshot_error="<<e.what()<<'\n';
  }
}
}
