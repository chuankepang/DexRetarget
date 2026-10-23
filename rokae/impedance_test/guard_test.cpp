#include "guard.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace fixed_impedance;
void expect(bool ok) { if(!ok) throw std::runtime_error("guard test failed"); }
int main() {
  const Pose initial{1,0,0,.4,0,1,0,0,0,0,1,.5,0,0,0,1};
  Joints q{},q0{},dq{};
  for(int i=0;i<2000;++i) expect(evaluate(i*.001,initial,initial,q,q0,dq)==Stop::none);
  q[0]=.03;
  expect(evaluate(1,initial,initial,q,q0,dq)==Stop::startup_drift);
  expect(evaluate(3,initial,initial,q,q0,dq)==Stop::none);
  q[0]=.26;
  expect(evaluate(3,initial,initial,q,q0,dq)==Stop::joint_excursion);
  q[0]=0;dq[2]=-.16;
  expect(evaluate(3,initial,initial,q,q0,dq)==Stop::joint_speed);
  dq[2]=0;auto measured=initial;measured[11]+=.031;
  expect(evaluate(3,measured,initial,q,q0,dq)==Stop::translation);
  measured=initial;double a=.16;measured[0]=std::cos(a);measured[1]=-std::sin(a);
  measured[4]=std::sin(a);measured[5]=std::cos(a);
  expect(evaluate(3,measured,initial,q,q0,dq)==Stop::rotation);
  measured=initial;measured[15]=0;
  expect(evaluate(3,measured,initial,q,q0,dq)==Stop::invalid_state);
  measured=initial;q[5]=std::numeric_limits<double>::quiet_NaN();
  expect(evaluate(3,measured,initial,q,q0,dq)==Stop::invalid_state);
  std::cout<<"Guard: fixed hold, redundancy drift, speed, TCP/rotation and invalid state passed\n";
}
