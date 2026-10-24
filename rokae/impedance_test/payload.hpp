#pragma once
#include "rokae/robot.h"
#include <cmath>
#include <stdexcept>

namespace fixed_impedance {
// 仅独立诊断：skip 不写零，也不清除/验证控制器原有 RT 负载。
template<class Rt>
bool configure_rt_load(Rt& rt, const rokae::Load& load, const std::string& mode) {
  if(mode=="skip")return false;
  if(mode!="explicit")throw std::runtime_error("invalid RT load mode");
  std::error_code ec;
  rt.setLoad(load,ec);
  if(ec)throw std::runtime_error("RT setLoad: "+ec.message());
  return true;
}

// false 只跳过 SDK Toolset 写入；RT 负载策略由 configure_rt_load 单独决定。
// YAML 在此时是期望值，防止误用上次试验留下的活动负载。
template<class Robot>
rokae::Toolset prepare_payload(Robot& robot, const rokae::Toolset& active,
                              bool apply, double mass, const std::array<double,3>& cog) {
  auto result=active;
  if(apply) {
    result.load.mass=mass;result.load.cog=cog;
    std::error_code ec;
    robot.setToolset(result,ec);
    if(ec)throw std::runtime_error("setToolset requested payload: "+ec.message());
    result=robot.toolset(ec);
    if(ec)throw std::runtime_error("toolset readback: "+ec.message());
  }
  const double mass_tolerance=apply?1e-6:.005;
  const double cog_tolerance=apply?1e-8:.001;
  if(!std::isfinite(result.load.mass)||result.load.mass<=0||result.load.mass>3||
     std::abs(result.load.mass-mass)>mass_tolerance)
    throw std::runtime_error("active payload mass invalid/mismatched; inspect/select the intended SDK Toolset before power-on");
  for(std::size_t i=0;i<3;++i) {
    if(!std::isfinite(result.load.cog[i])||std::abs(result.load.cog[i]-cog[i])>cog_tolerance)
      throw std::runtime_error("active payload CoM mismatched; YAML is expected flange-frame metres");
    if(!std::isfinite(result.load.inertia[i])||result.load.inertia[i]<0||
       std::abs(result.load.inertia[i]-active.load.inertia[i])>1e-9)
      throw std::runtime_error("invalid/changed inertia readback");
  }
  // 保存完整坐标表示；Toolset 设置只能改变质量/重心。
  const auto same=[](const auto& a,const auto& b){
    for(std::size_t i=0;i<a.size();++i)
      if(!std::isfinite(a[i])||!std::isfinite(b[i])||std::abs(a[i]-b[i])>1e-9)return false;
    return true;
  };
  if(!same(result.end.pos,active.end.pos)||!same(result.end.trans,active.end.trans)||
     !same(result.end.rpy,active.end.rpy)||!same(result.ref.pos,active.ref.pos)||
     !same(result.ref.trans,active.ref.trans)||!same(result.ref.rpy,active.ref.rpy))
    throw std::runtime_error("TCP/reference readback changed");
  return result;
}
}
