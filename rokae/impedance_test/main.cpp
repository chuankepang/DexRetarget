// Fixed-reference Cartesian impedance, using the official xCoreSDK 0.3.4 API.
// No Quest, Python realtime loop, host IK, reference filter or desired force.
#include "guard.hpp"
#include "diagnostics.hpp"
#include "nrt_home.hpp"
#include "payload.hpp"
#include "realtime_startup.hpp"
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
  std::string force_frame="tool",startup="legacy-rt",interaction="baseline";
  double mass{},duration{};
  bool apply_payload=true; // v1-v4 历史协议始终写 Toolset；v5 显式携带开关。
  std::string rt_load_mode="explicit"; // 独立试验 CLI；正式遥操作不受此诊断选项影响。
  std::array<double,3> cog{};
  Joints home{};
  std::array<double,6> stiffness{};
};
Options read_options(const char* path) {
  std::ifstream in(path);require(bool(in),"cannot open parameters");
  Options o;std::string version;
  std::getline(in,version);require(version=="rokae.fixed_impedance.v1"||version=="rokae.fixed_impedance.v2"||version=="rokae.fixed_impedance.v3"||version=="rokae.fixed_impedance.v4"||version=="rokae.fixed_impedance.v5","unsupported parameters");
  std::getline(in,o.ip);std::getline(in,o.local);std::getline(in,o.output);
  require(!o.ip.empty()&&!o.local.empty()&&!o.output.empty(),"missing connection/output");
  require(bool(in>>o.mass)&&std::isfinite(o.mass)&&o.mass>0&&o.mass<=3,"mass must be in (0,3] kg for this xMatePro3 test");
  read_values(in,o.cog);read_values(in,o.home);read_values(in,o.stiffness);
  for(double x:o.cog) require(std::abs(x)<=.3,"CoM must be metres within 0.3 m per axis");
  for(std::size_t i=0;i<6;++i) require(o.stiffness[i]>0&&o.stiffness[i]<=(i<3?1500:100),"invalid stiffness for SDK 0.3.4 ER Pro");
  require(bool(in>>o.duration)&&std::isfinite(o.duration)&&o.duration>=3&&o.duration<=60,"duration must be 3..60 seconds");
  if(version!="rokae.fixed_impedance.v1") {
    require(bool(in>>o.force_frame),"missing force frame");
    require(o.force_frame=="tool"||o.force_frame=="world","unsupported force frame");
  }
  if(version=="rokae.fixed_impedance.v3"||version=="rokae.fixed_impedance.v4"||version=="rokae.fixed_impedance.v5") {
    require(bool(in>>o.startup),"missing startup path");
    require(o.startup=="latest-compatible"||o.startup=="legacy-rt","unsupported startup path");
  }
  if(version=="rokae.fixed_impedance.v4"||version=="rokae.fixed_impedance.v5") {
    require(bool(in>>o.interaction),"missing interaction mode");
    require(o.interaction=="baseline"||o.interaction=="light-push","unsupported interaction mode");
  }
  if(version=="rokae.fixed_impedance.v5") {
    std::string apply;require(bool(in>>apply)&&(apply=="true"||apply=="false"),"invalid apply_payload flag");
    o.apply_payload=apply=="true";
  }
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
  Pose sent{},controller_command{};
  bool controller_command_valid=false;
  Joints q{},dq{},tau{},q_command{},tau_command{};
  Stop stop{Stop::none};
};
struct TraceState {
  Pose reference{};Joints q0{};double duration{};
  bool light_push=false;
  std::atomic<bool> interaction_ready{false};
  std::vector<Sample> samples;
  std::atomic<bool> done{false};std::atomic<Stop> stop{Stop::none};
  Clock::time_point start{},previous{};bool first=true;
};
void write_trace(const std::string& path,const std::vector<Sample>& samples,const Pose& reference) {
  std::ofstream out(path);require(bool(out),"cannot write trace");out<<std::setprecision(12);
  out<<"seconds,callback_period_s,stop";
  for(const char* prefix:{"tcp_m","reference","q_m","dq_m","tau_m","q_c","tau_c","sent_tcp","tcp_c"}) {
    const std::string name(prefix);
    int count=(name=="tcp_m"||name=="reference"||name=="sent_tcp"||name=="tcp_c")?16:7;
    for(int i=0;i<count;++i)out<<','<<prefix<<'_'<<i;
  }
  out<<",tcp_c_valid\n";
  for(const auto& row:samples) {
    out<<row.seconds<<','<<row.period<<','<<reason(row.stop);
    auto emit=[&out](const auto& values){for(double v:values)out<<','<<v;};
    emit(row.measured);emit(reference);emit(row.q);emit(row.dq);emit(row.tau);emit(row.q_command);emit(row.tau_command);
    emit(row.sent);emit(row.controller_command);out<<','<<row.controller_command_valid<<'\n';
  }
  require(bool(out),"trace write failed");
}
int main(int argc,char** argv) {
  if((argc!=4&&argc!=5)||std::string(argv[3])!="--execute") {
    std::cerr<<"Use run.py (default dry-run). Native usage: fixed_impedance inspect|home|prepare|hold parameters --execute [--rt-load-mode=explicit|skip]\n";return 2;
  }
  try {
    std::cout.setf(std::ios::unitbuf);
    const std::string stage=argv[1];
    require(stage=="inspect"||stage=="home"||stage=="prepare"||stage=="hold","unknown stage");
    auto o=read_options(argv[2]);
    if(argc==5) {
      const std::string mode=argv[4];
      require(mode=="--rt-load-mode=explicit"||mode=="--rt-load-mode=skip","invalid RT load comparison option");
      o.rt_load_mode=mode=="--rt-load-mode=skip"?"skip":"explicit";
    }
    require(o.rt_load_mode!="skip"||stage=="prepare"||stage=="hold","RT load skip requires prepare or hold stage");
    require(o.interaction=="baseline"||stage=="hold","light-push requires hold stage");
    // Open files before touching the controller. Launcher creates a fresh directory.
    std::ofstream summary(o.output+"/summary.txt");require(bool(summary),"cannot open summary output");
    summary<<std::setprecision(12)<<"stage="<<stage<<"\n";
    summary<<"rt_load_mode="<<o.rt_load_mode<<'\n';
    Robot robot(o.ip,o.local);Session session{robot,{}};std::error_code ec;
    const auto info=robot.robotInfo(ec);check(ec,"robotInfo");
    summary<<"robot="<<info.type<<" controller="<<info.version<<" sdk="<<robot.sdkVersion()<<'\n';
    std::cout<<"Robot="<<info.type<<" controller="<<info.version<<" SDK="<<robot.sdkVersion()<<'\n';
    if(stage!="inspect")require(info.type=="xMatePro3"&&info.joint_num==7,"this test requires the identified xMatePro3 seven-axis robot");
    if(stage!="inspect") {
      require(robot.sdkVersion()=="0.3.4","this binary requires matched SDK 0.3.4 headers/runtime");
      require(info.version=="2.3.2","this isolated experiment is scoped to controller 2.3.2; review compatibility before another controller version");
    }
    summary<<"startup_path="<<o.startup<<"\ninteraction="<<o.interaction<<'\n';
    const auto power_before=robot.powerState(ec);check(ec,"initial powerState");
    summary<<"power_before_enum="<<static_cast<int>(power_before)<<'\n';
    const auto operation_before=robot.operationState(ec);check(ec,"initial operationState");
    summary<<"operation_before_enum="<<static_cast<int>(operation_before)<<'\n';
    auto active=robot.toolset(ec);check(ec,"toolset");
    summary<<"active_mass_before="<<active.load.mass<<"\nactive_cog_before=";print_values(summary,active.load.cog);summary<<'\n';
    summary<<"active_inertia_before=";print_values(summary,active.load.inertia);
    summary<<"\nactive_tcp_xyz_before=";print_values(summary,active.end.trans);
    summary<<"\nactive_tcp_rpy_before=";print_values(summary,active.end.rpy);
    summary<<"\nactive_ref_xyz_before=";print_values(summary,active.ref.trans);
    summary<<"\nactive_ref_rpy_before=";print_values(summary,active.ref.rpy);summary<<'\n';
    const auto base_frame=robot.baseFrame(ec);check(ec,"baseFrame snapshot");
    summary<<"world_T_base_xyz_rpy=";print_values(summary,base_frame);summary<<'\n';
    std::cout<<"Active controller mass="<<active.load.mass<<" kg; requested="<<o.mass<<" kg\n";
    const auto q=robot.jointPos(ec);check(ec,"jointPos");
    const auto dq=robot.jointVel(ec);check(ec,"jointVel");
    summary<<"q_before=";print_values(summary,q);summary<<"\ndq_before=";print_values(summary,dq);summary<<'\n';
    std::array<double[2],7> limits{};const bool limits_on=robot.getSoftLimit(limits,ec);check(ec,"getSoftLimit");
    summary<<"soft_limits_enabled="<<limits_on<<'\n';
    summary<<"Pre-start controller warnings/errors:\n";print_logs(robot,summary);
    if(stage=="inspect") {
      write_force_snapshot(robot,summary,"inspect_current_power_state");
      const auto power_after=robot.powerState(ec);check(ec,"inspect powerState after");
      summary<<"power_after_enum="<<static_cast<int>(power_after)<<'\n';
      print_logs(robot,summary);
      std::cout<<"READ ONLY: saved state/tool/force-estimate summary and latest logs.\n";
      return 0;
    }
    require(power_before==rokae::PowerState::off||power_before==rokae::PowerState::on,"controller safety stop/unknown power state");
    require(limits_on,"soft limits disabled; repair separately before this test");
    const auto operation=robot.operationState(ec);check(ec,"operationState");
    require(operation==rokae::OperationState::idle,"controller must be idle");
    for(std::size_t i=0;i<7;++i) {
      require(std::isfinite(q[i])&&std::isfinite(dq[i])&&std::abs(dq[i])<.02,"robot is moving or state invalid");
      require(q[i]>limits[i][0]+.01&&q[i]<limits[i][1]-.01,"current joint too close to/outside soft limit");
      require(o.home[i]>limits[i][0]+.01&&o.home[i]<limits[i][1]-.01,"home violates soft-limit margin");
    }
    // Test the current flange-centre control point. Do not guess or overwrite TCP.
    for(double x:active.end.trans)require(std::isfinite(x)&&std::abs(x)<1e-8,"nonzero TCP: this isolated flange-reference test refuses to change it");
    for(double x:active.end.rpy)require(std::isfinite(x)&&std::abs(x)<1e-8,"nonzero TCP rotation: inspect before this test");
    for(double x:active.load.inertia)require(std::isfinite(x)&&x>=0,"invalid controller inertia");
    summary<<"apply_on_impedance_start="<<o.apply_payload<<'\n';
    const auto readback=prepare_payload(robot,active,o.apply_payload,o.mass,o.cog);
    summary<<"sdk_toolset_write_performed="<<o.apply_payload<<'\n';
    summary<<"applied_mass="<<readback.load.mass<<"\napplied_cog=";print_values(summary,readback.load.cog);
    summary<<"\ncontroller_inertia_unverified=";print_values(summary,readback.load.inertia);summary<<'\n';
    std::cout<<(o.apply_payload?"Applied requested mass/CoM to SDK Toolset. ":"SDK Toolset write SKIPPED; using validated active load. ")
             <<"Preserving controller inertia (zero is unverified).\n";
    std::cout<<"Applied CoM [x,y,z] m=";print_values(std::cout,readback.load.cog);std::cout<<'\n';
    std::signal(SIGINT,signal_stop);std::signal(SIGTERM,signal_stop);
    robot.setRtNetworkTolerance(20,ec);check(ec,"setRtNetworkTolerance");
    // 新版启动思路使用旧 SDK 已有接口：先 NRT 回位，再进入 RT。
    session.control_owned=true;
    robot.setOperateMode(rokae::OperateMode::automatic,ec);check(ec,"automatic");
    const bool latest=o.startup=="latest-compatible";
    robot.setMotionControlMode(latest?rokae::MotionControlMode::NrtCommand:rokae::MotionControlMode::RtCommand,ec);check(ec,"home motion mode");
    robot.setPowerState(true,ec);check(ec,"power on");
    require(robot.powerState(ec)==rokae::PowerState::on,"power-on readback not on");check(ec,"powerState");
    if(!latest) {session.rt=robot.getRtMotionController().lock();require(bool(session.rt),"empty RT controller");}
    auto rt=session.rt;
    {
      // 所有运动试验先回同一 home；阻抗只在回位并持续静止后启动。
      bool move_needed=false;
      summary<<"home_speed_scale="<<kHomeSpeedScale<<'\n';
      if(latest) {
        const auto before_home=robot.jointPos(ec);check(ec,"fresh NRT home start joints");
        for(std::size_t i=0;i<7;++i)
          require(std::isfinite(before_home[i])&&std::abs(before_home[i]-q[i])<.02,
                  "unexpected joint drift after power-on");
        std::cout<<"Startup latest-compatible: NRT MoveAbsJ home (SDK speed="<<kHomeSdkSpeed
                 <<", online_scale="<<kHomeSpeedScale<<"); RT not active.\n";
        move_needed=nrt_home(robot,o.home,[]{return bool(interrupted);});
        summary<<"home_reached=true\n";
      } else {
        std::cout<<"Position-control home preparation at speed scale "<<kHomeSpeedScale
                 <<"; impedance is not active.\n";
        const auto start_q=robot.jointPos(ec);check(ec,"fresh MoveJ start joints");
        for(std::size_t i=0;i<7;++i)require(std::isfinite(start_q[i])&&std::abs(start_q[i]-q[i])<.02,"unexpected joint drift after power-on");
        std::atomic<bool> finished{false},timed_out{false};
        const auto move_start=Clock::now();
        std::thread cancel([&]{while(!finished.load()){
          const bool timeout=Clock::now()-move_start>std::chrono::seconds(60);
          if(interrupted||timeout){timed_out.store(timeout);try{rt->stopMove();}catch(...){}return;}
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }});
        move_needed=false;
        for(std::size_t i=0;i<7;++i)move_needed=move_needed||std::abs(start_q[i]-o.home[i])>1e-4;
        // 已到位时不发送近乎相同的目标，避免 SDK 拒绝零长度轨迹。
        try{if(move_needed)rt->MoveJ(kHomeSpeedScale,start_q,o.home);}catch(...){finished.store(true);cancel.join();throw;}
        finished.store(true);cancel.join();
        const auto actual=robot.jointPos(ec);check(ec,"home readback");
        for(std::size_t i=0;i<7;++i)require(std::abs(actual[i]-o.home[i])<=.02,"home was not reached");
        require(!timed_out.load(),"home timed out");
        require(!interrupted,"home interrupted");summary<<"home_reached=true\n";
      }

      const auto settle_start=Clock::now();auto stable_since=settle_start;
      bool settled=false;
      while(Clock::now()-settle_start<std::chrono::seconds(5)) {
        require(!interrupted,"home settling interrupted");
        const auto position=robot.jointPos(ec);check(ec,"settle jointPos");
        const auto velocity=robot.jointVel(ec);check(ec,"settle jointVel");
        const auto power=robot.powerState(ec);check(ec,"settle powerState");
        require(power==rokae::PowerState::on,"power lost while settling at home");
        const auto now=Clock::now();
        if(!home_stationary(position,o.home,velocity))stable_since=now;
        if(now-stable_since>=std::chrono::milliseconds(500)){settled=true;break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      require(settled,"home did not settle within 5 seconds; impedance not started");
      summary<<"home_move_executed="<<move_needed<<"\nhome_settled=true\n";summary.flush();
      std::cout<<"Home verified stationary for 0.5 seconds.\n";
      // home 单阶段也采集上电静止估计；与抱闸下的 inspect 读数区分。
      write_force_snapshot(robot,summary,latest?"nrt_home_powered_stationary":"legacy_rt_home_powered_stationary");
      if(stage=="home") {
        const bool cleaned=session.clean(summary);std::cout<<"Home complete; power-off requested and checked.\n";
        return cleaned?0:1;
      }
    }
    if(latest) {
      robot.setMotionControlMode(rokae::MotionControlMode::RtCommand,ec);check(ec,"RtCommand after home");
      robot.setOperateMode(rokae::OperateMode::automatic,ec);check(ec,"RT automatic");
      robot.setPowerState(true,ec);check(ec,"RT power on");
      require(robot.powerState(ec)==rokae::PowerState::on,"power lost entering RT");check(ec,"RT powerState");
      session.rt=robot.getRtMotionController().lock();require(bool(session.rt),"empty RT controller");
      rt=session.rt;
    }
    // Official force-frame -> load -> gains -> zero desired force, all before startMove.
    const Pose identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    // SDK Toolset 与实时 TCP 配置分开，显式使用已核对的法兰中心，避免残留 RT TCP。
    rt->setEndEffectorFrame(identity,ec);check(ec,"RT setEndEffectorFrame(flange)");
    summary<<"rt_tcp_explicit_identity=true\n";
    const auto force_type=o.force_frame=="world"?rokae::FrameType::world:rokae::FrameType::tool;
    rt->setFcCoor(identity,force_type,ec);check(ec,"setFcCoor");
    summary<<"force_frame="<<o.force_frame<<'\n';
    if(stage=="prepare"||o.rt_load_mode=="skip")write_force_snapshot(robot,summary,"rt_before_setLoad");
    std::cout<<"Force frame="<<o.force_frame<<"; stiffness [N/m, Nm/rad]=";
    print_values(std::cout,o.stiffness);std::cout<<'\n';
    if(configure_rt_load(*rt,readback.load,o.rt_load_mode)) {
      summary<<"rt_load_write_performed=1\nrt_requested_mass="<<readback.load.mass
             <<"\nrt_requested_cog=";print_values(summary,readback.load.cog);summary<<'\n';
      std::cout<<"RT setLoad synchronized the active load (API accepted; internal compensation not independently read back).\n";
      if(stage=="prepare")write_force_snapshot(robot,summary,"rt_after_setLoad");
    } else {
      summary<<"rt_load_write_performed=0\nrt_existing_load_independently_verified=0\n";
      std::cout<<"EXPERIMENTAL: RT setLoad SKIPPED; existing RT load/compensation is NOT independently verified.\n";
      write_force_snapshot(robot,summary,"rt_setLoad_skipped");
    }
    rt->setCartesianImpedance(o.stiffness,ec);check(ec,"setCartesianImpedance");
    rt->setCartesianImpedanceDesiredTorque({0,0,0,0,0,0},ec);check(ec,"zero desired force");
    require(rt->setFilterLimit(true,30),"setFilterLimit rejected");
    rt->setFilterFrequency(30,30,30,ec);check(ec,"setFilterFrequency");
    // 非实时查询仅在启动回调之前；之后重新接收状态，避免将诊断延迟带入平衡点。
    write_force_snapshot(robot,summary,"rt_configured_before_startMove");
    const std::vector<std::string> fields{
      rokae::RtSupportedFields::tcpPose_m,rokae::RtSupportedFields::jointPos_m,
      rokae::RtSupportedFields::jointVel_m,rokae::RtSupportedFields::tau_m,
      rokae::RtSupportedFields::jointPos_c,rokae::RtSupportedFields::tau_c,
      rokae::RtSupportedFields::tcpPose_c};
    session.receiver=true; // 包括订阅初始化中途失败的清理。
    if(latest) {
      const auto packets=rokae_startup::receive_startup_state(robot,fields);
      summary<<"startup_subscription_recreated=true\nstartup_received_packets="<<packets<<'\n';
      std::cout<<"Post-setup RT subscription ready; received packets="<<packets<<'\n';
    } else {
      robot.startReceiveRobotState(std::chrono::milliseconds(1),fields);
      require(robot.updateRobotState(std::chrono::milliseconds(20))>0,"missing initial RT state");
    }
    Pose reference{};Joints q0{},dq0{};
    require(robot.getStateData(rokae::RtSupportedFields::tcpPose_m,reference)==0&&valid_pose(reference),"invalid initial TCP");
    require(robot.getStateData(rokae::RtSupportedFields::jointPos_m,q0)==0&&
            robot.getStateData(rokae::RtSupportedFields::jointVel_m,dq0)==0,"missing initial joint state");
    for(std::size_t i=0;i<7;++i)require(std::isfinite(q0[i])&&std::isfinite(dq0[i])&&std::abs(q0[i]-o.home[i])<=.02&&std::abs(dq0[i])<.02,"joint drift before impedance start");
    summary<<"initial_impedance_q=";print_values(summary,q0);summary<<'\n';
    summary<<"reference_row_major=";print_values(summary,reference);summary<<"\nstiffness_force_axes=";print_values(summary,o.stiffness);summary<<'\n';summary.flush();
    if(stage=="prepare") {
      // 只隔离参数配置，不进入原生阻抗器；必须在注册回调/startMove 之前退出。
      summary<<"impedance_start_attempted=0\n";
      const bool cleaned=session.clean(summary);
      summary<<"cleanup_ok="<<cleaned<<"\nexit_code="<<(cleaned?0:1)<<'\n';
      std::cout<<"RT preparation complete; impedance was NOT started. Cleanup "
               <<(cleaned?"confirmed power-off.":"incomplete; inspect summary.txt.")<<'\n';
      return cleaned?0:1;
    }
    // Shared ownership keeps callback storage alive even if an SDK stop reports
    // failure. No callback-owned vector is read until stopLoop returns successfully.
    auto trace=std::make_shared<TraceState>();
    trace->reference=reference;trace->q0=q0;trace->duration=o.duration;
    trace->light_push=o.interaction=="light-push";
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
        row.sent=command.pos; // 记录实际交给 SDK 的位姿，不用事后复制参考点代替发送证据。
        // pos_c 是 SDK 公开的指令反馈，零矩阵/无效值单独标记，不能解释为零目标。
        row.controller_command_valid=robot.getStateData(rokae::RtSupportedFields::tcpPose_c,row.controller_command)==0
                                     &&valid_pose(row.controller_command);
        row.period=std::chrono::duration<double>(now-previous).count();previous=now;
        bool good=robot.getStateData(rokae::RtSupportedFields::tcpPose_m,row.measured)==0;
        good=robot.getStateData(rokae::RtSupportedFields::jointPos_m,row.q)==0&&good;
        good=robot.getStateData(rokae::RtSupportedFields::jointVel_m,row.dq)==0&&good;
        good=robot.getStateData(rokae::RtSupportedFields::tau_m,row.tau)==0&&good;
        good=robot.getStateData(rokae::RtSupportedFields::jointPos_c,row.q_command)==0&&good;
        good=robot.getStateData(rokae::RtSupportedFields::tau_c,row.tau_command)==0&&good;
        row.stop=good?evaluate(row.seconds,row.measured,reference,row.q,q0,row.dq,
                              phase_limits(row.seconds,trace->light_push)):Stop::invalid_state;
        for(const auto* values:{&row.tau,&row.q_command,&row.tau_command})
          for(double x:*values)if(!std::isfinite(x))row.stop=Stop::invalid_state;
        if(interrupted)row.stop=Stop::interrupted;
        if(row.seconds>=trace->duration&&row.stop==Stop::none)row.stop=Stop::duration;
        if(samples.size()==samples.capacity())row.stop=Stop::buffer_full;
        else samples.push_back(row); // Preallocated, single-writer; exported only after loop joins.
        if(row.stop==Stop::none && row.seconds>=Limits{}.startup_seconds)
          trace->interaction_ready.store(true);
        if(row.stop!=Stop::none){Stop expected=Stop::none;stop.compare_exchange_strong(expected,row.stop);command.setFinished();done.store(true);}
      } catch(...) {stop.store(Stop::callback_exception);command.setFinished();done.store(true);}
      return command;
    };
    std::cout<<"Fixed equilibrium; first 2 seconds DO NOT TOUCH. Ctrl+C requests stop.\n"
             <<"定时观察 "<<o.duration<<" 秒；到时程序主动结束阻抗并下电，不是启动失败。\n";
    bool started=false,cleaned=false,runtime_failed=false;int result=1;
    try {
      // 与新版官方示例一致：先注册固定参考回调，再切换阻抗并启动循环。
      rt->setControlLoop(callback,0,true);
      rt->startMove(rokae::RtControllerMode::cartesianImpedance);started=true;
      summary<<"requested_rt_mode=cartesianImpedance\nmode_start_api_success=1\n"
             <<"desired_wrench=0 0 0 0 0 0\nsdk_rate_limit=1\nsdk_filter_hz=30\n";
      summary.flush();
      session.loop_attempted=true;session.loop_joined=false;rt->startLoop(false);
      const auto wall_start=Clock::now();bool announced=false;
      while(!done.load()) {
        const double age=std::chrono::duration<double>(Clock::now()-wall_start).count();
        if(!announced && (trace->light_push ? trace->interaction_ready.load() : age>2)){
          if(trace->light_push)std::cout<<"轻推阶段：先缓慢偏离约 10–20 mm 后松手观察回弹；速度护栏 0.70 rad/s，TCP 偏移上限 100 mm。\n";
          else std::cout<<"前 2 秒观察完成，继续保持不触碰；总计 "<<o.duration<<" 秒后自动结束并下电。\n";
          announced=true;
        }
        if(age>o.duration+2){stop.store(Stop::callback_exception);break;}
        if(interrupted)break;
        const auto power=robot.powerState(ec);check(ec,"monitor powerState");
        if(power!=rokae::PowerState::on){stop.store(Stop::controller_fault);break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } catch(const std::exception& e) {runtime_failed=true;summary<<"runtime_error="<<e.what()<<'\n';std::cerr<<e.what()<<'\n';}
    // Always terminate the RT loop before reading the callback-owned samples.
    cleaned=session.clean(summary);
    if(interrupted&&stop.load()==Stop::none)stop.store(Stop::interrupted);
    summary<<"startMove_returned="<<started<<"\nstop_reason="<<reason(stop.load())<<"\nloop_joined="<<session.loop_joined<<"\nsamples="<<(session.loop_joined?trace->samples.size():0)<<'\n';
    print_logs(robot,summary);
    if(!session.loop_joined) {std::cerr<<"SDK loop join failed; trace not exported to avoid concurrent access. Use physical stop.\n";return 1;}
    write_trace(o.output+"/trace.csv",trace->samples,reference);
    if(stop.load()==Stop::joint_excursion && !trace->samples.empty()) {
      const auto& last=trace->samples.back();
      const auto limit=phase_limits(last.seconds,trace->light_push).joint_delta;
      for(std::size_t i=0;i<last.q.size();++i) {
        const double delta=last.q[i]-q0[i];
        if(std::abs(delta)>limit) {
          summary<<"guard_joint_1based="<<i+1<<"\nguard_joint_delta_rad="<<delta
                 <<"\nguard_joint_delta_limit_rad="<<limit<<'\n';
          std::cout<<"本程序关节行程护栏：J"<<i+1<<" 相对起点偏移="<<delta
                   <<" rad，上限="<<limit<<" rad；不是速度或加速度超限。\n";
          break;
        }
      }
    }
    if(stop.load()==Stop::translation && !trace->samples.empty()) {
      const auto& last=trace->samples.back();
      const auto limit=phase_limits(last.seconds,trace->light_push).translation;
      const auto error=translation_error(last.measured,reference);
      summary<<"guard_tcp_displacement_m="<<error<<"\nguard_tcp_displacement_limit_m="<<limit<<'\n';
      std::cout<<"本程序位移边界：TCP 距固定参考点 "<<1000*error
               <<" mm，上限 "<<1000*limit<<" mm；已请求停止，未自动移动参考点。\n";
    }
    // 回调停止并 join 后输出触发细节，不在 1 ms 控制循环内打印。
    if(stop.load()==Stop::joint_speed && !trace->samples.empty()) {
      const auto& last=trace->samples.back();
      const auto limit=phase_limits(last.seconds,trace->light_push).joint_speed;
      for(std::size_t i=0;i<last.dq.size();++i) {
        if(std::abs(last.dq[i])>limit) {
          summary<<"guard_joint_1based="<<i+1<<"\nguard_joint_speed_rad_s="<<last.dq[i]
                 <<"\nguard_joint_speed_limit_rad_s="<<limit<<'\n';
          std::cout<<"本程序速度护栏：J"<<i+1<<" 测量速度="<<last.dq[i]
                   <<" rad/s，允许绝对值 <= "<<limit
                   <<" rad/s；TCP 偏移="<<1000*translation_error(last.measured,reference)<<" mm。\n";
          break;
        }
      }
    }
    if(stop.load()==Stop::joint_speed||stop.load()==Stop::startup_drift||
       stop.load()==Stop::joint_excursion||stop.load()==Stop::translation||
       stop.load()==Stop::rotation||stop.load()==Stop::invalid_state) {
      std::cout<<"Software guard triggered; this program requested SDK stop and power-off. "
               <<"Check summary.txt for power-off confirmation and controller faults.\n";
    }
    std::cout<<"Stopped: "<<reason(stop.load())<<"; samples="<<trace->samples.size()<<"; trace="<<o.output<<"/trace.csv\n";
    if(successful_exit(stop.load(),started,cleaned,runtime_failed))result=0;
    summary<<"cleanup_ok="<<cleaned<<"\nexit_code="<<result<<'\n';
    if(result==0 && stop.load()==Stop::duration) {
      std::cout<<"正常完成：阻抗定时观察已运行 "<<o.duration
               <<" 秒，程序主动停止并已确认下电。此结果不代表负载标定或阻抗性能验收通过。\n";
    } else if(result==0) {
      std::cout<<"用户停止：程序已结束阻抗并确认下电。\n";
    } else if(cleaned && !runtime_failed && stop.load()!=Stop::none && stop.load()!=Stop::duration) {
      std::cout<<"试验因 "<<reason(stop.load())<<" 停止；停止/清理成功，已确认下电。\n";
    } else {
      std::cout<<"试验异常退出或清理未完成；检查 summary.txt 中的 stop_reason、runtime_error 和下电确认。\n";
    }
    return result;
  } catch(const std::exception& e) {std::cerr<<"FAILED: "<<e.what()<<'\n';return 1;}
}
