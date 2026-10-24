// 不连接机器人：验证启动前置条件及失败清理，不能验证真实阻抗动力学。
#include "nrt_home.hpp"
#include "realtime_startup.hpp"
#include <deque>
#include <iostream>
#include <limits>
using namespace fixed_impedance;
void expect(bool ok) { if(!ok) throw std::runtime_error("startup test failed"); }
template<class F> void fails(F f,const std::string& message) {
  try {f();} catch(const std::exception& e) {
    if(std::string(e.what()).find(message)!=std::string::npos)return;
    throw;
  }
  throw std::runtime_error("expected failure: "+message);
}
struct Stream {
  std::deque<int> results; std::vector<bool> waits;
  bool started=false, stopped=false, fail_start=false;
  void stopReceiveRobotState(){stopped=true;started=false;}
  void startReceiveRobotState(std::chrono::milliseconds interval,const std::vector<std::string>& fields){
    expect(stopped && interval.count()==1 && fields==std::vector<std::string>{"tcp"});
    if(fail_start)throw std::runtime_error("subscription failed");
    started=true;
  }
  int updateRobotState(std::chrono::steady_clock::duration wait) {
    expect(started && wait==std::chrono::milliseconds(20));
    waits.push_back(wait.count()>0);
    if(results.empty()) throw std::runtime_error("unexpected read");
    int r=results.front();results.pop_front();
    if(r<0)throw std::runtime_error("SDK disconnected");
    return r;
  }
};
struct SlowClock {
  static inline int calls=0;
  static auto now(){return std::chrono::steady_clock::time_point{}+std::chrono::milliseconds(61*calls++);}
};
struct HomeRobot {
  Joints target{},position{},velocity{};
  rokae::OperationState state=rokae::OperationState::idle;
  rokae::PowerState power=rokae::PowerState::on;
  bool arrive=true,execution_error=false,append_error=false;
  int resets=0,stops=0,starts=0,appends=0;
  double scale=0;
  void moveReset(std::error_code& ec){ec.clear();++resets;}
  void stop(std::error_code& ec){ec.clear();++stops;}
  void adjustSpeedOnline(double value,std::error_code& ec){ec.clear();scale=value;}
  Joints jointPos(std::error_code& ec){ec.clear();return position;}
  Joints jointVel(std::error_code& ec){ec.clear();return velocity;}
  auto operationState(std::error_code& ec){ec.clear();return state;}
  auto powerState(std::error_code& ec){ec.clear();return power;}
  void moveAppend(const std::vector<rokae::MoveAbsJCommand>& cmds,std::string& id,std::error_code& ec){
    expect(cmds.size()==1);++appends;id="home";
    ec=append_error?std::make_error_code(std::errc::io_error):std::error_code{};
  }
  void moveStart(std::error_code& ec){ec.clear();++starts;if(arrive)position=target;}
  rokae::EventInfo queryEventInfo(rokae::Event,std::error_code& ec){
    ec.clear();using namespace rokae::EventInfoKey::MoveExecution;
    return {{ID,std::string("home")},{Error,execution_error?
      std::make_error_code(std::errc::io_error):std::error_code{}}};
  }
};
int main(){
  // 连续非空流正常完成，不要求 0 返回，也不读第四次；不会绕过断流。
  Stream fresh{{1,1,1,1},{}};
  expect(rokae_startup::receive_startup_state(fresh,{"tcp"})==3);
  expect(fresh.results.size()==1 && fresh.waits.size()==3);
  Stream absent{{0},{}};fails([&]{rokae_startup::receive_startup_state(absent,{"tcp"});},"missing realtime startup packet 1");
  Stream lost{{1,0},{}};fails([&]{rokae_startup::receive_startup_state(lost,{"tcp"});},"packet 2");
  Stream broken{{1,-1},{}};fails([&]{rokae_startup::receive_startup_state(broken,{"tcp"});},"disconnected");
  Stream failed{{},{}};failed.fail_start=true;
  fails([&]{rokae_startup::receive_startup_state(failed,{"tcp"});},"subscription failed");
  expect(failed.waits.empty());
  Stream slow{{1},{}};SlowClock::calls=0;
  fails([&]{rokae_startup::receive_startup_state<Stream,SlowClock>(slow,{"tcp"});},"exceeded 60 ms");
  const auto never=[] {return false;};
  HomeRobot home;home.position[0]=.1;
  expect(nrt_home(home,home.target,never));
  expect(home.starts==1&&home.resets==2&&home.stops==0&&home.scale==.50);
  HomeRobot there;expect(!nrt_home(there,there.target,never));expect(there.appends==0);
  HomeRobot early;early.position[0]=.1;early.arrive=false;
  fails([&]{nrt_home(early,early.target,never,std::chrono::milliseconds(1));},"timed out");
  expect(early.stops==1&&early.resets==2); // idle 但未到位不能当作成功。
  HomeRobot off;off.power=rokae::PowerState::off;
  fails([&]{nrt_home(off,off.target,never);},"power lost");expect(off.stops==1);
  HomeRobot unknown;unknown.state=rokae::OperationState::unknown;
  fails([&]{nrt_home(unknown,unknown.target,never);},"unknown operation");
  HomeRobot event;event.position[0]=.1;event.execution_error=true;
  fails([&]{nrt_home(event,event.target,never);},"execution");expect(event.stops==1);
  HomeRobot append;append.position[0]=.1;append.append_error=true;
  fails([&]{nrt_home(append,append.target,never);},"append");expect(append.starts==0&&append.stops==1);
  HomeRobot invalid;invalid.velocity[0]=std::numeric_limits<double>::quiet_NaN();
  fails([&]{nrt_home(invalid,invalid.target,never);},"invalid NRT home feedback");
  HomeRobot cancel;cancel.position[0]=.1;int polls=0;
  fails([&]{nrt_home(cancel,cancel.target,[&]{return ++polls>1;});},"interrupted");
  expect(cancel.starts==1&&cancel.stops==1&&cancel.resets==2);
  std::cout<<"Startup: bounded subscription restart/sampling, NRT arrival, idle/off/error/invalid/cancel cleanup passed\n";
}
