#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"
class BumperRobot
{
  public:
    BumperRobot(){
      bumper_pressed = false;

      vel_pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
      bumper_sub_ = n_.subscribe("/mobile_base/events/bumper", 1, &BumperRobot::messageCallback, this);

    }

    void messageCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    bumper_pressed = msg->bumper;
    if(bumper_pressed){bumper_pressed= true;}
    else if(bumper_pressed = false){
      bumper_pressed = false;
    }
  }
  void actions(){

    geometry_msgs::Twist vel;
    if(bumper_pressed){
      vel.linear.x = 0.0;
    }
    else{
      vel.linear.x = 0.3;
    }
    vel_pub_.publish(vel);
  }


  private:
    bool bumper_pressed;
    ros::NodeHandle n_;
    ros::Subscriber bumper_sub_;
    ros::Publisher vel_pub_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bumper_robot");
  BumperRobot bumperrobot;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {

    bumperrobot.actions();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

