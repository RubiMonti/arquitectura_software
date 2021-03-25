#include "ros/ros.h"
#include "ball_and_goal/FindYellowGoal.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_yellow_node");
  ros::NodeHandle n;

  ball_and_goal_bica::FindYellowGoal go_yellow_node;

  ros::Rate loop_rate(5);

  int count = 0;

  while (go_yellow_node.ok())
  {
    go_yellow_node.step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
  
}