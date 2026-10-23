// Fixed-reference Cartesian impedance, using the official xCoreSDK 0.3.4 API.
// No Quest, Python realtime loop, host IK, reference filter or desired force.
#include "guard.hpp"
#include "rokae/robot.h"
#include <atomic>
#include <csignal>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace fixed_impedance;
using Clock=std::chrono::steady_clock;
using Robot=rokae::xMateErProRobot;
using Rt=rokae::RtMotionControlCobot<7>;
volatile std::sig_atomic_t interrupted=0;
void signal_stop(int) { interrupted=1; }
void check(std::error_code ec,const char* operation) {
  if(ec) throw std::runtime_error(std::string(operation)+": "+ec.message());
}
void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
template<std::size_t N> void read_values(std::istream& in,std::array<double,N>& values) {
  for(double& v:values) { require(bool(in>>v)&&std::isfinite(v),"invalid/non-finite parameter"); }
}
template<class Values> void print_values(std::ostream& out,const Values& values) {
  for(double v:values) out<<' '<<v;
}
struct Options {
  std::string ip,local,output;
  double mass{},duration{};
  std::array<double,3> cog{};
  Joints home{};
  std::array<double,6> stiffness{};
};
Options read_options(const char* path) {
  std::ifstream in(path);require(bool(in),"cannot open parameters");
  Options o;std::string version;
  std::getline(in,version);require(version=="rokae.fixed_impedance.v1","unsupported parameters");
  std::getline(in,o.ip);std::getline(in,o.local);std::getline(in,o.output);
  require(!o.ip.empty()&&!o.local.empty()&&!o.output.empty(),"missing connection/output");
  require(bool(in>>o.mass)&&std::isfinite(o.mass)&&o.mass>0&&o.mass<=3,"mass must be in (0,3] kg for this xMatePro3 test");
  read_values(in,o.cog);read_values(in,o.home);read_values(in,o.stiffness);
  for(double x:o.cog) require(std::abs(x)<=.3,"CoM must be metres within 0.3 m per axis");
  for(std::size_t i=0;i<6;++i) require(o.stiffness[i]>0&&o.stiffness[i]<=(i<3?1500:100),"invalid stiffness for SDK 0.3.4 ER Pro");
  require(bool(in>>o.duration)&&std::isfinite(o.duration)&&o.duration>=3&&o.duration<=60,"duration must be 3..60 seconds");
  std::string extra;require(!(in>>extra),"unexpected trailing parameters");return o;
}
void print_logs(Robot& robot,std::ostream& out) {
  std::error_code ec;
  const auto logs=robot.queryControllerLog(10,{rokae::LogInfo::warning,rokae::LogInfo::error},ec);
  if(ec) {out<<"log_query_error="<<ec.message()<<'\n';return;}
  out<<"Latest 10 warnings/errors; may include earlier trials:\n";
  for(const auto& row:logs) out<<row.timestamp<<" id="<<row.id<<" "<<row.content<<'\n';
}
struct Session {
  Robot& robot;std::shared_ptr<Rt> rt;
  bool control_owned=false,receiver=false,loop_attempted=false,loop_joined=true;
  bool clean(std::ostream& out) noexcept {
    bool ok=true;
    // Stop the motion before stopping its loop or requesting power-off.
    if(control_owned&&rt) {
      try {rt->stopMove();} catch(const std::exception& e) {out<<"stopMove: "<<e.what()<<'\n';ok=false;}
      if(loop_attempted) {
        try {rt->stopLoop();loop_joined=true;} catch(const std::exception& e) {out<<"stopLoop: "<<e.what()<<'\n';ok=false;}
      }
    }
    if(receiver&&loop_joined) {try{robot.stopReceiveRobotState();}catch(...){ok=false;}receiver=false;}
    if(control_owned) {
      std::error_code ec;robot.setPowerState(false,ec);
      if(ec){out<<"power_off_request_error="<<ec.message()<<'\n';ok=false;}
      ec.clear();const auto power=robot.powerState(ec);
      out<<"power_off_readback="<<(!ec&&power==rokae::PowerState::off)<<'\n';
      if(ec||power!=rokae::PowerState::off)ok=false;
      if(loop_joined&&!ec&&power==rokae::PowerState::off) {
        ec.clear();robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand,ec);
        if(ec){out<<"restore_NrtCommand_error="<<ec.message()<<'\n';ok=false;}
      } else {out<<"NrtCommand restoration skipped: stop/power-off not confirmed\n";ok=false;}
      control_owned=false;
    }
    return ok;
  }
  ~Session(){if(control_owned||receiver)clean(std::cerr);}
};
struct Sample {
  double seconds{},period{};
  Pose measured{};
  Joints q{},dq{},tau{},q_command{},tau_command{};
  Stop stop{Stop::none};
};
struct TraceState {
  Pose reference{};Joints q0{};double duration{};
  std::vector<Sample> samples;
  std::atomic<bool> done{false};std::atomic<Stop> stop{Stop::none};
  Clock::time_point start{},previous{};bool first=true;
};
void write_trace(const std::string& path,const std::vector<Sample>& samples,const Pose& reference) {
  std::ofstream out(path);require(bool(out),"cannot write trace");out<<std::setprecision(12);
  out<<"seconds,callback_period_s,stop";
  for(const char* prefix:{"tcp_m","reference","q_m","dq_m","tau_m","q_c","tau_c"}) {
    int count=(std::string(prefix)=="tcp_m"||std::string(prefix)=="reference")?16:7;
    for(int i=0;i<count;++i)out<<','<<prefix<<'_'<<i;
  }
  out<<'\n';
  for(const auto& row:samples) {
    out<<row.seconds<<','<<row.period<<','<<reason(row.stop);
    auto emit=[&out](const auto& values){for(double v:values)out<<','<<v;};
    emit(row.measured);emit(reference);emit(row.q);emit(row.dq);emit(row.tau);emit(row.q_command);emit(row.tau_command);out<<'\n';
  }
  require(bool(out),"trace write failed");
}
int main(int argc,char** argv) {
  if(argc!=4||std::string(argv[3])!="--execute") {
    std::cerr<<"Use run.py (default dry-run). Native usage: fixed_impedance inspect|home|hold parameters --execute\n";return 2;
  }
  try {
    std::cout.setf(std::ios::unitbuf);
    const std::string stage=argv[1];
    require(stage=="inspect"||stage=="home"||stage=="hold","unknown stage");
    const auto o=read_options(argv[2]);
    // Open files before touching the controller. Launcher creates a fresh directory.
    std::ofstream summary(o.output+"/summary.txt");require(bool(summary),"cannot open summary output");
    summary<<std::setprecision(12)<<"stage="<<stage<<"\n";
    Robot robot(o.ip,o.local);Session session{robot,{}};std::error_code ec;
    const auto info=robot.robotInfo(ec);check(ec,"robotInfo");
    summary<<"robot="<<info.type<<" controller="<<info.version<<" sdk="<<robot.sdkVersion()<<'\n';
    std::cout<<"Robot="<<info.type<<" controller="<<info.version<<" SDK="<<robot.sdkVersion()<<'\n';
    if(stage!="inspect")require(info.type=="xMatePro3"&&info.joint_num==7,"this test requires the identified xMatePro3 seven-axis robot");
    const auto power_before=robot.powerState(ec);check(ec,"initial powerState");
    summary<<"power_before_enum="<<static_cast<int>(power_before)<<'\n';
    auto active=robot.toolset(ec);check(ec,"toolset");
    summary<<"active_mass_before="<<active.load.mass<<"\nactive_cog_before=";print_values(summary,active.load.cog);summary<<'\n';
    std::cout<<"Active controller mass="<<active.load.mass<<" kg; requested="<<o.mass<<" kg\n";
    const auto q=robot.jointPos(ec);check(ec,"jointPos");
    const auto dq=robot.jointVel(ec);check(ec,"jointVel");
    summary<<"q_before=";print_values(summary,q);summary<<"\ndq_before=";print_values(summary,dq);summary<<'\n';
    std::array<double[2],7> limits{};const bool limits_on=robot.getSoftLimit(limits,ec);check(ec,"getSoftLimit");
    summary<<"soft_limits_enabled="<<limits_on<<'\n';
    if(stage=="inspect") {print_logs(robot,summary);std::cout<<"READ ONLY: saved state/tool summary and latest logs.\n";return 0;}
    require(power_before==rokae::PowerState::off||power_before==rokae::PowerState::on,"controller safety stop/unknown power state");
    require(limits_on,"soft limits disabled; repair separately before this test");
    const auto operation=robot.operationState(ec);check(ec,"operationState");
    require(operation==rokae::OperationState::idle,"controller must be idle");
    for(std::size_t i=0;i<7;++i) {
      require(std::isfinite(q[i])&&std::isfinite(dq[i])&&std::abs(dq[i])<.02,"robot is moving or state invalid");
      require(q[i]>limits[i][0]+.01&&q[i]<limits[i][1]-.01,"current joint too close to/outside soft limit");
      require(o.home[i]>limits[i][0]+.01&&o.home[i]<limits[i][1]-.01,"home violates soft-limit margin");
      if(stage=="hold")require(std::abs(q[i]-o.home[i])<=.02,"not at requested home; execute home stage first");
    }
    // Test the current flange-centre control point. Do not guess or overwrite TCP.
    for(double x:active.end.trans)require(std::isfinite(x)&&std::abs(x)<1e-8,"nonzero TCP: this isolated flange-reference test refuses to change it");
    for(double x:active.end.rpy)require(std::isfinite(x)&&std::abs(x)<1e-8,"nonzero TCP rotation: inspect before this test");
    for(double x:active.load.inertia)require(std::isfinite(x)&&x>=0,"invalid controller inertia");
    active.load.mass=o.mass;active.load.cog=o.cog;
    robot.setToolset(active,ec);check(ec,"setToolset calibrated payload");
    auto readback=robot.toolset(ec);check(ec,"toolset readback");
    require(std::abs(readback.load.mass-o.mass)<1e-6,"mass readback mismatch");
    for(std::size_t i=0;i<3;++i) {
      require(std::abs(readback.load.cog[i]-o.cog[i])<1e-8,"CoM readback mismatch");
      require(std::abs(readback.load.inertia[i]-active.load.inertia[i])<1e-9,"inertia readback changed");
      require(std::abs(readback.end.trans[i]-active.end.trans[i])<1e-9&&std::abs(readback.end.rpy[i]-active.end.rpy[i])<1e-9,"TCP readback changed");
    }
    summary<<"applied_mass="<<readback.load.mass<<"\napplied_cog=";print_values(summary,readback.load.cog);
    summary<<"\ncontroller_inertia_unverified=";print_values(summary,readback.load.inertia);summary<<'\n';
    std::cout<<"Applying calibrated mass/CoM; preserving controller inertia (zero is unverified).\n";
    std::signal(SIGINT,signal_stop);std::signal(SIGTERM,signal_stop);
    robot.setRtNetworkTolerance(20,ec);check(ec,"setRtNetworkTolerance");
    // Official v0.3.4 ordering: automatic -> RtCommand -> power on.
    session.control_owned=true;
    robot.setOperateMode(rokae::OperateMode::automatic,ec);check(ec,"automatic");
    robot.setMotionControlMode(rokae::MotionControlMode::RtCommand,ec);check(ec,"RtCommand");
    robot.setPowerState(true,ec);check(ec,"power on");
    require(robot.powerState(ec)==rokae::PowerState::on,"power-on readback not on");check(ec,"powerState");
    session.rt=robot.getRtMotionController().lock();require(bool(session.rt),"empty RT controller");
    auto rt=session.rt;
    if(stage=="home") {
      std::cout<<"Official RT MoveJ at speed scale 0.05 to requested home. No impedance is started.\n";
      const auto start_q=robot.jointPos(ec);check(ec,"fresh MoveJ start joints");
      for(std::size_t i=0;i<7;++i)require(std::isfinite(start_q[i])&&std::abs(start_q[i]-q[i])<.02,"unexpected joint drift after power-on");
      std::atomic<bool> finished{false},timed_out{false};
      const auto move_start=Clock::now();
      std::thread cancel([&]{while(!finished.load()){
        const bool timeout=Clock::now()-move_start>std::chrono::seconds(60);
        if(interrupted||timeout){timed_out.store(timeout);try{rt->stopMove();}catch(...){}return;}
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }});
      try{rt->MoveJ(.05,start_q,o.home);}catch(...){finished.store(true);cancel.join();throw;}
      finished.store(true);cancel.join();
      const auto actual=robot.jointPos(ec);check(ec,"home readback");
      for(std::size_t i=0;i<7;++i)require(std::abs(actual[i]-o.home[i])<=.02,"home was not reached");
      require(!timed_out.load(),"home timed out");
      require(!interrupted,"home interrupted");summary<<"home_reached=true\n";
      const bool cleaned=session.clean(summary);std::cout<<"Home complete; power-off requested and checked.\n";
      return cleaned?0:1;
    }
    // Official force-frame -> load -> gains -> zero desired force, all before startMove.
    const Pose identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    rt->setFcCoor(identity,rokae::FrameType::tool,ec);check(ec,"setFcCoor(tool/flange)");
    rt->setLoad(readback.load,ec);check(ec,"RT setLoad");
    rt->setCartesianImpedance(o.stiffness,ec);check(ec,"setCartesianImpedance");
    rt->setCartesianImpedanceDesiredTorque({0,0,0,0,0,0},ec);check(ec,"zero desired force");
    require(rt->setFilterLimit(true,30),"setFilterLimit rejected");
    rt->setFilterFrequency(30,30,30,ec);check(ec,"setFilterFrequency");
    robot.startReceiveRobotState(std::chrono::milliseconds(1),{
      rokae::RtSupportedFields::tcpPose_m,rokae::RtSupportedFields::jointPos_m,
      rokae::RtSupportedFields::jointVel_m,rokae::RtSupportedFields::tau_m,
      rokae::RtSupportedFields::jointPos_c,rokae::RtSupportedFields::tau_c});
    session.receiver=true;
    robot.updateRobotState(std::chrono::milliseconds(20));
    Pose reference{};Joints q0{},dq0{};
    require(robot.getStateData(rokae::RtSupportedFields::tcpPose_m,reference)==0&&valid_pose(reference),"invalid initial TCP");
    require(robot.getStateData(rokae::RtSupportedFields::jointPos_m,q0)==0&&
            robot.getStateData(rokae::RtSupportedFields::jointVel_m,dq0)==0,"missing initial joint state");
    for(std::size_t i=0;i<7;++i)require(std::isfinite(q0[i])&&std::isfinite(dq0[i])&&std::abs(q0[i]-o.home[i])<=.02&&std::abs(dq0[i])<.02,"joint drift before impedance start");
    summary<<"reference_row_major=";print_values(summary,reference);summary<<"\nstiffness_tool_axes=";print_values(summary,o.stiffness);summary<<'\n';summary.flush();
    // Shared ownership keeps callback storage alive even if an SDK stop reports
    // failure. No callback-owned vector is read until stopLoop returns successfully.
    auto trace=std::make_shared<TraceState>();
    trace->reference=reference;trace->q0=q0;trace->duration=o.duration;
    trace->samples.reserve(static_cast<std::size_t>(o.duration*2000)+4096);
    auto& stop=trace->stop;auto& done=trace->done;
    std::function<rokae::CartesianPosition()> callback=[trace, &robot] {
      auto& reference=trace->reference;auto& q0=trace->q0;auto& samples=trace->samples;
      auto& first=trace->first;auto& start=trace->start;auto& previous=trace->previous;
      auto& stop=trace->stop;auto& done=trace->done;
      rokae::CartesianPosition command(reference); // Immutable equilibrium: no reference follower.
      try {
        const auto now=Clock::now();if(first){start=previous=now;first=false;}
        Sample row;row.seconds=std::chrono::duration<double>(now-start).count();
        row.period=std::chrono::duration<double>(now-previous).count();previous=now;
        bool good=robot.getStateData(rokae::RtSupportedFields::tcpPose_m,row.measured)==0;
        good=robot.getStateData(rokae::RtSupportedFields::jointPos_m,row.q)==0&&good;
        good=robot.getStateData(rokae::RtSupportedFields::jointVel_m,row.dq)==0&&good;
        good=robot.getStateData(rokae::RtSupportedFields::tau_m,row.tau)==0&&good;
        good=robot.getStateData(rokae::RtSupportedFields::jointPos_c,row.q_command)==0&&good;
        good=robot.getStateData(rokae::RtSupportedFields::tau_c,row.tau_command)==0&&good;
        row.stop=good?evaluate(row.seconds,row.measured,reference,row.q,q0,row.dq):Stop::invalid_state;
        for(const auto* values:{&row.tau,&row.q_command,&row.tau_command})
          for(double x:*values)if(!std::isfinite(x))row.stop=Stop::invalid_state;
        if(interrupted)row.stop=Stop::interrupted;
        if(row.seconds>=trace->duration&&row.stop==Stop::none)row.stop=Stop::duration;
        if(samples.size()==samples.capacity())row.stop=Stop::buffer_full;
        else samples.push_back(row); // Preallocated, single-writer; exported only after loop joins.
        if(row.stop!=Stop::none){Stop expected=Stop::none;stop.compare_exchange_strong(expected,row.stop);command.setFinished();done.store(true);}
      } catch(...) {stop.store(Stop::callback_exception);command.setFinished();done.store(true);}
      return command;
    };
    std::cout<<"Fixed equilibrium; first 2 seconds DO NOT TOUCH. Ctrl+C requests stop.\n";
    bool started=false,cleaned=false;int result=1;
    try {
      rt->startMove(rokae::RtControllerMode::cartesianImpedance);started=true;
      rt->setControlLoop(callback,0,true);session.loop_attempted=true;session.loop_joined=false;rt->startLoop(false);
      const auto wall_start=Clock::now();bool announced=false;
      while(!done.load()) {
        const double age=std::chrono::duration<double>(Clock::now()-wall_start).count();
        if(age>2&&!announced){std::cout<<"Observation window ended; only gentle small pushes within guard limits.\n";announced=true;}
        if(age>o.duration+2){stop.store(Stop::callback_exception);break;}
        if(interrupted)break;
        const auto power=robot.powerState(ec);check(ec,"monitor powerState");
        if(power!=rokae::PowerState::on){stop.store(Stop::controller_fault);break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } catch(const std::exception& e) {summary<<"runtime_error="<<e.what()<<'\n';std::cerr<<e.what()<<'\n';}
    // Always terminate the RT loop before reading the callback-owned samples.
    cleaned=session.clean(summary);
    if(interrupted&&stop.load()==Stop::none)stop.store(Stop::interrupted);
    summary<<"startMove_returned="<<started<<"\nstop_reason="<<reason(stop.load())<<"\nloop_joined="<<session.loop_joined<<"\nsamples="<<(session.loop_joined?trace->samples.size():0)<<'\n';
    print_logs(robot,summary);
    if(!session.loop_joined) {std::cerr<<"SDK loop join failed; trace not exported to avoid concurrent access. Use physical stop.\n";return 1;}
    write_trace(o.output+"/trace.csv",trace->samples,reference);
    if(stop.load()==Stop::joint_speed||stop.load()==Stop::startup_drift||
       stop.load()==Stop::joint_excursion||stop.load()==Stop::translation||
       stop.load()==Stop::rotation||stop.load()==Stop::invalid_state) {
      std::cout<<"Software guard triggered; this program requested SDK stop and power-off. "
               <<"Check summary.txt for power-off confirmation and controller faults.\n";
    }
    std::cout<<"Stopped: "<<reason(stop.load())<<"; samples="<<trace->samples.size()<<"; trace="<<o.output<<"/trace.csv\n";
    if(cleaned&&(stop.load()==Stop::duration||stop.load()==Stop::interrupted))result=0;
    return result;
  } catch(const std::exception& e) {std::cerr<<"FAILED: "<<e.what()<<'\n';return 1;}
}
