// Replays recorded feedback through the actual guard; no SDK/robot connection.
#include "guard.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
using namespace fixed_impedance;
std::vector<std::string> split(const std::string& line) {
  std::istringstream in(line);std::vector<std::string> out;std::string item;
  while(std::getline(in,item,','))out.push_back(item);
  return out;
}
int main(int argc,char** argv) {
  try {
    if(argc!=3)throw std::runtime_error("Usage: replay_guard trace.csv summary.txt (requires initial_impedance_q)");
    std::ifstream summary(argv[2]);std::string line;Joints q0{};bool have_origin=false;
    while(std::getline(summary,line))if(line.rfind("initial_impedance_q=",0)==0) {
      std::istringstream in(line.substr(line.find('=')+1));have_origin=true;
      for(double& x:q0)if(!(in>>x))throw std::runtime_error("invalid initial joint reference");
    }
    if(!have_origin)throw std::runtime_error("exact pre-start joint reference missing; cannot faithfully replay startup drift");
    std::ifstream trace(argv[1]);if(!std::getline(trace,line))throw std::runtime_error("empty trace");
    const auto header=split(line);
    auto index=[&](const std::string& name){
      auto it=std::find(header.begin(),header.end(),name);
      if(it==header.end())throw std::runtime_error("missing trace column");
      return static_cast<std::size_t>(it-header.begin());
    };
    std::size_t count=0;Stop last=Stop::none;
    while(std::getline(trace,line)) {
      const auto row=split(line);if(row.size()!=header.size())throw std::runtime_error("bad row");
      auto value=[&](const std::string& name){return std::stod(row[index(name)]);};
      Pose measured{},reference{};Joints q{},dq{};
      for(int i=0;i<16;++i){measured[i]=value("tcp_m_"+std::to_string(i));reference[i]=value("reference_"+std::to_string(i));}
      for(int i=0;i<7;++i){q[i]=value("q_m_"+std::to_string(i));dq[i]=value("dq_m_"+std::to_string(i));}
      last=evaluate(value("seconds"),measured,reference,q,q0,dq);++count;
      if(row[index("stop")]!=reason(last))throw std::runtime_error("guard replay differs from recorded stop");
    }
    if(!count)throw std::runtime_error("no samples");
    std::cout<<"Replay matched "<<count<<" samples; final="<<reason(last)<<'\n';return 0;
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
