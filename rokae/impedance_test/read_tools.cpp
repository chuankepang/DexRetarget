// Read saved tools and active Toolset. Optional RL query mode only while off/idle.
// Explicit --project loads definitions only while off/idle. Never runs a task.
// No tool/power setters or motion calls.
#include "rokae/robot.h"
#include <iomanip>
#include <iostream>
#include <stdexcept>
void check(std::error_code ec,const char* operation) {
  if(ec)throw std::runtime_error(std::string(operation)+": "+ec.message());
}
void print_load(const rokae::Load& load) {
  std::cout<<"  mass_kg="<<load.mass<<"\n  cog_m=["<<load.cog[0]<<", "<<load.cog[1]<<", "<<load.cog[2]
           <<"]\n  inertia_kg_m2=["<<load.inertia[0]<<", "<<load.inertia[1]<<", "<<load.inertia[2]<<"]\n";
}
void print_frame(const rokae::Frame& frame) {
  std::cout<<"  TCP_translation_m=["<<frame.trans[0]<<", "<<frame.trans[1]<<", "<<frame.trans[2]
           <<"]\n  TCP_rpy_rad=["<<frame.rpy[0]<<", "<<frame.rpy[1]<<", "<<frame.rpy[2]<<"]\n";
}
struct QueryMode {
  rokae::xMateErProRobot& robot;bool changed=false;
  void restore() {
    if(!changed)return;
    std::error_code ec;robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand,ec);
    check(ec,"restore NrtCommand");changed=false;
    std::cout<<"Restored SDK NrtCommand mode (motor power unchanged).\n";
  }
  ~QueryMode(){if(changed)try{restore();}catch(const std::exception& e){std::cerr<<"RESTORE FAILED: "<<e.what()<<'\n';}}
};
int main(int argc,char** argv) {
  if(argc<4){std::cerr<<"Usage: read_tools robot_ip local_ip tool_name [--query-project-mode] [--project name] [--parse-project] [--project name]\n";return 2;}
  try {
    bool query_mode=false,parse_project=false;std::string project_name;
    for(int i=4;i<argc;++i) {
      const std::string flag=argv[i];
      if(flag=="--query-project-mode")query_mode=true;
      else if(flag=="--parse-project")parse_project=true;
      else if(flag=="--project"&&i+1<argc){project_name=argv[++i];query_mode=true;}
      else throw std::runtime_error("unknown/incomplete option: "+flag);
    }
    if(parse_project&&project_name.empty())throw std::runtime_error("--parse-project requires --project");
    std::cout<<std::setprecision(14)<<std::unitbuf;
    rokae::xMateErProRobot robot(argv[1],argv[2]);std::error_code ec;
    const auto info=robot.robotInfo(ec);check(ec,"robotInfo");
    std::cout<<"robot="<<info.type<<" controller="<<info.version<<" SDK="<<robot.sdkVersion()<<'\n';
    const auto power=robot.powerState(ec);check(ec,"powerState");
    const auto operation=robot.operationState(ec);check(ec,"operationState");
    std::cout<<"power_enum="<<static_cast<int>(power)<<" operation_enum="<<static_cast<int>(operation)<<'\n';
    const auto active=robot.toolset(ec);check(ec,"toolset");
    std::cout<<"ACTIVE SDK TOOLSET (name not returned by this API)\n";print_load(active.load);print_frame(active.end);
    QueryMode mode{robot};
    if(query_mode) {
      if(power!=rokae::PowerState::off||operation!=rokae::OperationState::idle)
        throw std::runtime_error("RL query mode requires power=off and operation=idle");
      std::cout<<"Temporarily selecting NrtRLTask for saved-tool queries; tasks will never be run.\n";
      robot.setMotionControlMode(rokae::MotionControlMode::NrtRLTask,ec);check(ec,"select NrtRLTask");
      mode.changed=true;
    }
    if(!project_name.empty()) {
      const auto projects=robot.projectsInfo(ec);check(ec,"projectsInfo before explicit load");
      bool located=false;
      for(const auto& project:projects)if(project.name==project_name) {
        // An empty project is still queried: with no selected tasks it cannot
        // be run. The controller may reject the load; report that error intact.
        // Load only; optional explicit ppToMain parses. runProject is never called.
        std::cout<<"Loading definitions for project="<<project.name<<" tasks=";
        for(const auto& task:project.taskList)std::cout<<task<<' ';
        std::cout<<"(NO execution)\n";
        robot.loadProject(project.name,project.taskList,ec);check(ec,"loadProject definitions only");
        located=true;break;
      }
      if(!located)throw std::runtime_error("requested project was not found");
    }
    if(parse_project) {
      std::cout<<"Parsing loaded project with ppToMain (no runProject, still powered off).\n";
      robot.ppToMain(ec);check(ec,"ppToMain parse only");
    }
    const auto tools=robot.toolsInfo(ec);
    if(ec) {
      std::cerr<<"SAVED TOOL LIST UNAVAILABLE: "<<ec.message()<<"\nNo tool selection or task execution was attempted. This is not proof that the saved tool does not exist.\n";
      return 3;
    }
    bool found=false;
    std::cout<<"LOADED-PROJECT TOOLS (not all projects/global tools): "<<tools.size()<<'\n';
    for(const auto& tool:tools) {
      std::cout<<"name="<<tool.name<<" alias="<<tool.alias<<" robotHeld="<<tool.robotHeld<<'\n';
      print_load(tool.load);print_frame(tool.pos);
      if(tool.name==argv[3])found=true;
    }
    if(query_mode) {
      const auto projects=robot.projectsInfo(ec);
      if(ec)std::cerr<<"Project-name query unavailable: "<<ec.message()<<'\n';
      else {
        std::cout<<"PROJECT NAMES (not loaded or run by this query):\n";
        for(const auto& project:projects)std::cout<<"  "<<project.name<<'\n';
      }
    }
    mode.restore();
    const auto final_power=robot.powerState(ec);check(ec,"final powerState");
    std::cout<<"power_after_enum="<<static_cast<int>(final_power)<<'\n';
    if(!found){std::cerr<<"Requested name was not in the currently loaded-project list; other projects/global tools were not searched.\n";return 4;}
    if(!project_name.empty())std::cout<<"Project left loaded (not running): "<<project_name<<'\n';
    std::cout<<"FOUND requested tool "<<argv[3]<<"; no tool selection or parameter write performed.\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<"READ FAILED: "<<error.what()<<'\n';return 1;}
}
