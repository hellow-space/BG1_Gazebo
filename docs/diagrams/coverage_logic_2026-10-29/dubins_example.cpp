// Teaching geometry only: one native F2C turn, never sent to the robot.
#include <fields2cover.h>
#include <fstream>
int main(int argc,char**argv) {
  if(argc!=2)return 2;
  F2CRobot robot;robot.robot_width=.76;robot.op_width=.6;robot.setMinRadius(.6);
  f2c::pp::DubinsCurves turn;
  auto path=turn.createTurn(robot,F2CPoint(5.8,5.),0.,F2CPoint(14.5,5.8),3.141592653589793);
  std::ofstream out(argv[1]);
  for(const auto& state:path.states)out<<state.point.getX()<<","<<state.point.getY()<<"\n";
  return path.states.empty()?3:0;
}
