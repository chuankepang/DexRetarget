// Offline checks against the installed official SDK types; never constructs a robot.
#include "rokae/data_types.h"
#include "rokae/utility.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
void require(bool ok) { if(!ok) throw std::runtime_error("SDK coordinate layout check failed"); }
int main() {
  const std::array<double,3> expected_cog{
      -52.71043598571/1000.0,12.17669684527/1000.0,8.60984186271/1000.0};
  const rokae::Load load(.93938639463,expected_cog,{0,0,0});
  require(load.cog==expected_cog); // Constructor must not permute, rotate or negate CoM.
  std::array<double,16> pose{};
  rokae::Utils::postureToTransArray({.123,-.234,.345,.2,-.4,.6},pose);
  require(pose[3]==.123&&pose[7]==-.234&&pose[11]==.345);
  require(pose[12]==0&&pose[13]==0&&pose[14]==0&&pose[15]==1);
  const rokae::CartesianPosition command(pose);
  require(command.pos==pose); // This is the exact constructor used by the hold callback.
  require(!command.hasElbow);
  Eigen::Matrix3d rotation;Eigen::Vector3d translation;
  rokae::Utils::arrayToTransMatrix(command.pos,rotation,translation);
  require((translation-Eigen::Vector3d(.123,-.234,.345)).norm()<1e-12);
  require(std::abs(rotation.determinant()-1)<1e-12);
  std::array<double,16> round_trip{};
  rokae::Utils::transMatrixToArray(rotation,translation,round_trip);
  require(round_trip==pose);
  std::cout<<"Offline SDK: CoM XYZ/signs, row-major pose, command constructor and rotation passed\n";
}
