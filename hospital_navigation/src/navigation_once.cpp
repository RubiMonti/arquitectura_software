// Copyright 2021 ROScon de Reyes
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



void set_goal(move_base_msgs::MoveBaseGoal& goal, char* arg)
{
    ROS_INFO("ARG = %s\n", arg);
    if (!(strcasecmp(arg,"habitacion1")))
    {
        ROS_INFO("Mandando a habitacion 1\n");
        goal.target_pose.pose.position.x = 1.26;
        goal.target_pose.pose.position.y = -8.79;
    }
    else if (!(strcasecmp(arg,"habitacion2")))
    {
        ROS_INFO("Mandando a habitacion 2\n");
        goal.target_pose.pose.position.x = 8.23;
        goal.target_pose.pose.position.y = 5.84;
    }
    else if (!(strcasecmp(arg,"spawn")))
    {
        ROS_INFO("Mandando a spawn\n");
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 2.0;
    }
    else if (!(strcasecmp(arg,"consulta1")))
    {
        ROS_INFO("Mandando a consulta 1\n");
        goal.target_pose.pose.position.x = 1;
        goal.target_pose.pose.position.y = 1;
        goal.target_pose.pose.orientation.w = 0.0013;

    }
    else if (!(strcasecmp(arg,"consulta2")))
    {
        ROS_INFO("Mandando a consulta 2\n");
        goal.target_pose.pose.position.x = 3.15;
        goal.target_pose.pose.position.y = -2.18;
    }
    else if (!(strcasecmp(arg,"almacen1")))
    {
        ROS_INFO("Mandando a almacen 1\n");
        goal.target_pose.pose.position.x = -15.17;
        goal.target_pose.pose.position.y = 0.93;
    }
    else if (!(strcasecmp(arg,"almacen2")))
    {
        ROS_INFO("Mandando a almacen 2\n");
        goal.target_pose.pose.position.x = -9.07;
        goal.target_pose.pose.position.y = 12.97;
    }
    else
        ROS_INFO("No se ha detectado nada\n");
    
    goal.target_pose.pose.orientation.w = 0.0013;

}

void doneCb(const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/*
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    double goal_x = goal.target_pose.pose.position.x;
    double goal_y = goal.target_pose.pose.position.y;
    double current_x = feedback->base_position.pose.position.x;
    double current_y = feedback->base_position.pose.position.y;

    double diff_x = goal_x - current_x;
    double diff_y = goal_y - current_y;

    double dist = sqrt(diff_x * diff_x + diff_y * diff_y);

    ROS_INFO("Distance to goal: %lf", dist);
}
*/

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation");

    move_base_msgs::MoveBaseGoal goal;

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){

        ROS_INFO("Waiting for the move_base action server to come up");

    }

    goal.target_pose.header.frame_id = "map";

    set_goal(goal, argv[argc - 1]);
    // -------- COORDENADAS PARA LA HABITACION 1 ---------- //
    goal.target_pose.pose.position.x = 3.83;
    goal.target_pose.pose.position.y = 0.416;
    goal.target_pose.pose.orientation.w = 0.0013;

  
    goal.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("Sending goal");
    ac.sendGoal(goal, doneCb);//, MoveBaseClient::SimpleActiveCallback(), feedbackCb);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)

        ROS_INFO("Hooray, the base moved 1 meter forward");

    else

        ROS_INFO("The base failed to move towards the desired point for some reason");



    return 0;

}