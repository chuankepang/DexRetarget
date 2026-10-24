#include "diagnostics.hpp"
#include <sstream>
#include <stdexcept>

struct FakeRobot {
  int calls=0;bool fail=false,incomplete=false,throws=false;
  void getEndTorque(rokae::FrameType frame,std::array<double,7>& joint,std::array<double,7>& external,
                    std::array<double,3>& torque,std::array<double,3>& force,std::error_code& ec) {
    ++calls;
    if(frame!=rokae::FrameType::world)throw std::runtime_error("wrong frame");
    if(throws)throw std::runtime_error("fake read failure");
    if(fail){ec=std::make_error_code(std::errc::io_error);return;}
    joint.fill(2);external.fill(.2);torque.fill(.1);force.fill(0);
    if(incomplete)force[2]=std::numeric_limits<double>::quiet_NaN();
  }
};
int main() {
  for(int mode=0;mode<4;++mode) {
    FakeRobot robot;robot.fail=mode==1;robot.incomplete=mode==2;robot.throws=mode==3;
    std::ostringstream out;
    fixed_impedance::write_force_snapshot(robot,out,"inspect");
    const auto text=out.str();
    if(robot.calls!=1||text.find(mode?"force_snapshot_available=0":"force_snapshot_available=1")==std::string::npos)
      throw std::runtime_error("snapshot availability failed");
    if((text.find("world_cart_force_estimate_N=")!=std::string::npos)!=(mode==0))
      throw std::runtime_error("invalid feedback interpreted as zero force");
  }
}
