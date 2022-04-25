#include <ros/ros.h>
#include <common.hpp>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/RobotServiceAction.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_client");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<spark_backend::RobotServiceAction> rc("robot", true);
    ROS_INFO("Waiting for server to connect...");
    rc.waitForServer();
    spark_backend::RobotServiceGoal goal;
    goal.move_type = static_cast<int>(MoveType::MOVE_JOINT_ANGLE);
    goal.use_joint = true;
    goal.set_fraction = true;
    goal.move_rate = 0.1;
    goal.move_joint_pos[0] = 0;
    goal.move_joint_pos[1] = 0;
    goal.move_joint_pos[2] = 0;
    goal.move_joint_pos[3] = 0;
    goal.move_joint_pos[4] = 0;
    goal.move_joint_pos[5] = 0;

    rc.sendGoal(goal);
    rc.waitForResult();
    auto result = rc.getResult();

    return EXIT_SUCCESS;
}