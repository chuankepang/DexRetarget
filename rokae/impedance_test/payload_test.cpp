#include "payload.hpp"
#include <iostream>
#include <limits>
using namespace fixed_impedance;
void expect(bool ok){if(!ok)throw std::runtime_error("payload test failed");}
template<class F> void fails(F f){bool failed=false;try{f();}catch(const std::runtime_error&){failed=true;}expect(failed);}
struct FakeRobot {
  rokae::Toolset state;
  int writes=0,reads=0;
  bool fail_write=false,fail_read=false,mutate_frame=false;
  void setToolset(const rokae::Toolset& value,std::error_code& ec){
    ++writes;ec=fail_write?std::make_error_code(std::errc::io_error):std::error_code{};
    if(!ec)state=value;
  }
  rokae::Toolset toolset(std::error_code& ec){
    ++reads;ec=fail_read?std::make_error_code(std::errc::io_error):std::error_code{};
    auto result=state;if(mutate_frame)result.ref.trans[0]+=.1;return result;
  }
};
struct FakeRt {
  int writes=0;bool fail=false;rokae::Load last;
  void setLoad(const rokae::Load& value,std::error_code& ec) {
    ++writes;last=value;ec=fail?std::make_error_code(std::errc::io_error):std::error_code{};
  }
};
int main(){
  rokae::Toolset active;active.load.mass=.9642;active.load.cog={.008,.012,.052};
  active.load.inertia={.001,.002,.003};
  FakeRobot unchanged;
  auto kept=prepare_payload(unchanged,active,false,.9642,active.load.cog);
  expect(unchanged.writes==0&&unchanged.reads==0&&kept.load.mass==active.load.mass);
  fails([&]{prepare_payload(unchanged,active,false,.0642,active.load.cog);});
  expect(unchanged.writes==0);
  auto zero=active;zero.load.mass=0;
  fails([&]{prepare_payload(unchanged,zero,false,.9642,active.load.cog);});
  auto invalid=active;invalid.load.cog[0]=std::numeric_limits<double>::quiet_NaN();
  fails([&]{prepare_payload(unchanged,invalid,false,.9642,active.load.cog);});
  auto wrong_cog=active;wrong_cog.load.cog[2]+=.002;
  fails([&]{prepare_payload(unchanged,wrong_cog,false,.9642,active.load.cog);});
  FakeRobot written;
  auto applied=prepare_payload(written,active,true,.5,active.load.cog);
  expect(written.writes==1&&written.reads==1&&applied.load.mass==.5);
  expect(applied.load.inertia==active.load.inertia); // replacement, no application-side addition.
  FakeRobot corrupt;corrupt.mutate_frame=true;
  fails([&]{prepare_payload(corrupt,active,true,.5,active.load.cog);});
  FakeRobot rejected;rejected.fail_write=true;
  fails([&]{prepare_payload(rejected,active,true,.5,active.load.cog);});expect(rejected.reads==0);
  FakeRobot unreadable;unreadable.fail_read=true;
  fails([&]{prepare_payload(unreadable,active,true,.5,active.load.cog);});
  FakeRt rt;
  expect(!configure_rt_load(rt,active.load,"skip")&&rt.writes==0);
  expect(configure_rt_load(rt,active.load,"explicit")&&rt.writes==1);
  expect(rt.last.mass==active.load.mass&&rt.last.cog==active.load.cog&&rt.last.inertia==active.load.inertia);
  expect(!configure_rt_load(rt,active.load,"skip")&&rt.writes==1); // skip does not reset existing state.
  fails([&]{configure_rt_load(rt,active.load,"typo");});expect(rt.writes==1);
  rt.fail=true;fails([&]{configure_rt_load(rt,active.load,"explicit");});
  std::cout<<"Payload: skip/write counts, expected active load, preservation and failures passed; no robot connected\n";
}
