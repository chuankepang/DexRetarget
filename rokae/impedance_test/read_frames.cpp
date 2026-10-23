// Read current coordinate frames only: no mode, tool, load or power setters.
#include "rokae/robot.h"
#include "rokae/utility.h"
#include <iomanip>
#include <iostream>
#include <stdexcept>

void check(std::error_code ec, const char* operation) {
  if (ec) throw std::runtime_error(std::string(operation) + ": " + ec.message());
}
template<std::size_t N> void array(const std::array<double,N>& values) {
  std::cout << '[';
  for (std::size_t i=0;i<N;++i) std::cout << (i ? "," : "") << values[i];
  std::cout << ']';
}
void pose(const char* name, const std::array<double,6>& values) {
  std::array<double,16> matrix{};
  rokae::Utils::postureToTransArray(values,matrix);
  std::cout << ",\n\"" << name << "\":{\"xyz_m_rpy_rad\":";
  array(values); std::cout << ",\"matrix_row_major\":"; array(matrix); std::cout << '}';
}
std::array<double,6> coordinates(const rokae::Frame& frame) {
  return {frame.trans[0],frame.trans[1],frame.trans[2],frame.rpy[0],frame.rpy[1],frame.rpy[2]};
}
int main(int argc,char** argv) {
  if(argc!=3) { std::cerr << "Usage: read_frames robot_ip local_ip\n"; return 2; }
  try {
    rokae::xMateErProRobot robot(argv[1],argv[2]); std::error_code ec;
    const auto power=robot.powerState(ec);check(ec,"powerState");
    const auto operation=robot.operationState(ec);check(ec,"operationState");
    const auto q_before=robot.jointPos(ec);check(ec,"jointPos before");
    const auto base=robot.baseFrame(ec);check(ec,"baseFrame");
    const auto flange=robot.posture(rokae::CoordinateType::flangeInBase,ec);check(ec,"flangeInBase");
    const auto tools=robot.toolset(ec);check(ec,"toolset");
    const auto end=robot.posture(rokae::CoordinateType::endInRef,ec);check(ec,"endInRef");
    const auto q_after=robot.jointPos(ec);check(ec,"jointPos after");
    const auto power_after=robot.powerState(ec);check(ec,"powerState after");
    std::cout << std::setprecision(16) << "{\"power_enum\":" << static_cast<int>(power)
      << ",\"power_after_enum\":" << static_cast<int>(power_after)
      << ",\"operation_enum\":" << static_cast<int>(operation) << ",\"q_before_rad\":";
    array(q_before);std::cout << ",\"q_after_rad\":";array(q_after);
    pose("world_T_base",base);pose("base_T_flange",flange);
    pose("flange_T_end",coordinates(tools.end));pose("world_T_ref",coordinates(tools.ref));
    pose("ref_T_end",end);
    std::cout << ",\"mass_kg\":" << tools.load.mass << ",\"cog_m\":";
    array(tools.load.cog);std::cout << ",\"inertia_kg_m2\":";array(tools.load.inertia);
    std::cout << "}\n";
  } catch(const std::exception& error) { std::cerr << error.what() << '\n';return 1; }
}
