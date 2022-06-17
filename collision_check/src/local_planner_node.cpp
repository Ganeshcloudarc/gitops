#include "collision_check/local_planner_core.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_planner_autopilot");
  LocalPlanner local_planner;
  local_planner.init();
  local_planner.run();
  ros::spin();

  return 0;
}
