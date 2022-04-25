#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/RobotServiceAction.h"
#include <thread>
#include <vector>
#include "common.hpp"
#include <cmath>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_client");
    actionlib::SimpleActionClient<spark_backend::RobotServiceAction> ac("robot", true);
    ROS_INFO("Waiting for the server to start.");
    ac.waitForServer();
    ROS_INFO("Server connected, now sending goal.");
    spark_backend::RobotServiceGoal goal;
    goal.move_joint_pos[0] = 14.45 * M_PI / 180.0;
    goal.move_joint_pos[1] = -53.97 * M_PI / 180.0;
    goal.move_joint_pos[2] = 58.04 * M_PI / 180.0;
    goal.move_joint_pos[3] = -10.61 * M_PI / 180.0;
    goal.move_joint_pos[4] = 90.15 * M_PI / 180.0;
    goal.move_joint_pos[5] = 174.949 * M_PI / 180.0;
    goal.use_joint = true;
    goal.move_type = static_cast<int>(MoveType::MOVE_J);
    ac.sendGoal(goal);
    sleep(1);
    std::cout << "The server status is: ";
    std::cout << ac.getState().toString() << std::endl;
    sleep(2);
    printf("Sending goal to pause the robot.\n");
    goal.move_type = static_cast<int>(MoveType::PAUSE);
    ac.sendGoal(goal);
    std::cout << "The server status is: ";
    std::cout << ac.getState().toString() << std::endl;
    sleep(3);
    printf("Sending goal to resume the robot.\n");
    goal.move_type = static_cast<int>(MoveType::RESUME);
    ac.sendGoal(goal);
    sleep(3);
    std::cout << "The server status is: ";
    std::cout << ac.getState().toString() << std::endl;
    return 0;
}