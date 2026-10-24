#include "guard.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace fixed_impedance;
void expect(bool ok) { if(!ok) throw std::runtime_error("guard test failed"); }
int main() {
  expect(successful_exit(Stop::duration,true,true,false));
  expect(successful_exit(Stop::interrupted,true,true,false));
  expect(!successful_exit(Stop::duration,false,true,false));
  expect(!successful_exit(Stop::duration,true,false,false));
  expect(!successful_exit(Stop::duration,true,true,true));
  expect(!successful_exit(Stop::controller_fault,true,true,false));
  expect(!successful_exit(Stop::startup_drift,true,true,false));
  const Pose initial{1,0,0,.4,0,1,0,0,0,0,1,.5,0,0,0,1};
  Joints q{},q0{},dq{};
  dq[6]=.301; // 新速度边界：baseline 拒绝，轻推阶段允许。
  expect(evaluate(9.299,initial,initial,q,q0,dq)==Stop::joint_speed);
  expect(evaluate(1.999,initial,initial,q,q0,dq,phase_limits(1.999,true))==Stop::joint_speed);
  expect(evaluate(2.0,initial,initial,q,q0,dq,phase_limits(2.0,true))==Stop::none);
  dq[6]=.701;
  expect(evaluate(3,initial,initial,q,q0,dq,phase_limits(3,true))==Stop::joint_speed);
  dq[6]=0;
  // 轻推阶段与启动阶段边界分开验证。
  dq[5]=.451327412287;
  auto contact=initial;contact[3]-=.014100165936;contact[7]+=.002915753662;contact[11]+=.022224144402;
  expect(evaluate(7.103,contact,initial,q,q0,dq,phase_limits(7.103,true))==Stop::none);
  expect(evaluate(1.999,contact,initial,q,q0,dq,phase_limits(1.999,true))==Stop::joint_speed);
  expect(evaluate(7.103,contact,initial,q,q0,dq,phase_limits(7.103,false))==Stop::joint_speed);
  dq[5]=-.701;
  expect(evaluate(7.103,contact,initial,q,q0,dq,phase_limits(7.103,true))==Stop::joint_speed);
  dq[5]=0;
  auto pushed=initial;pushed[7]+=.061;
  expect(evaluate(3,pushed,initial,q,q0,dq,phase_limits(3,true))==Stop::none);
  expect(evaluate(3,pushed,initial,q,q0,dq,phase_limits(3,false))==Stop::translation);
  expect(evaluate(1.999,pushed,initial,q,q0,dq,phase_limits(1.999,true))==Stop::translation);
  pushed[7]=initial[7]+.1001;
  expect(evaluate(3,pushed,initial,q,q0,dq,phase_limits(3,true))==Stop::translation);
  expect(phase_limits(3,true).joint_delta==1.0);
  q[2]=.500061677989;
  expect(evaluate(19.4,initial,initial,q,q0,dq,phase_limits(19.4,true))==Stop::none);
  expect(evaluate(19.4,initial,initial,q,q0,dq,phase_limits(19.4,false))==Stop::joint_excursion);
  q[2]=1.001;
  expect(evaluate(19.4,initial,initial,q,q0,dq,phase_limits(19.4,true))==Stop::joint_excursion);
  q[2]=0;
  expect(phase_limits(3,true).rotation==Limits{}.rotation);
  expect(home_stationary(q,q0,dq));
  q[3]=.003;expect(!home_stationary(q,q0,dq));q[3]=0;
  dq[2]=.02;expect(!home_stationary(q,q0,dq));dq[2]=0;
  q[0]=std::numeric_limits<double>::quiet_NaN();expect(!home_stationary(q,q0,dq));q[0]=0;
  for(int i=0;i<2000;++i) expect(evaluate(i*.001,initial,initial,q,q0,dq)==Stop::none);
  q[0]=.05;
  expect(evaluate(1,initial,initial,q,q0,dq)==Stop::startup_drift);
  expect(evaluate(3,initial,initial,q,q0,dq)==Stop::none);
  q[0]=.51;
  expect(evaluate(3,initial,initial,q,q0,dq)==Stop::joint_excursion);
  q[0]=0;dq[2]=-.31;
  expect(evaluate(3,initial,initial,q,q0,dq)==Stop::joint_speed);
  dq[2]=0;auto measured=initial;measured[11]+=.061;
  expect(evaluate(3,measured,initial,q,q0,dq)==Stop::translation);
  measured=initial;double a=.31;measured[0]=std::cos(a);measured[1]=-std::sin(a);
  measured[4]=std::sin(a);measured[5]=std::cos(a);
  expect(evaluate(3,measured,initial,q,q0,dq)==Stop::rotation);
  measured=initial;measured[15]=0;
  expect(evaluate(3,measured,initial,q,q0,dq)==Stop::invalid_state);
  measured=initial;q[5]=std::numeric_limits<double>::quiet_NaN();
  expect(evaluate(3,measured,initial,q,q0,dq)==Stop::invalid_state);
  std::cout<<"Guard: fixed hold, redundancy drift, speed, TCP/rotation and invalid state passed\n";
}
