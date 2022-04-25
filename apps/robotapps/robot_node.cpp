#include <ros/ros.h>
#include "robot_services.hpp"

int main(int argc, char** argv) {
    ROS_INFO("Initiating the aubo robot server node...");
    ros::init(argc, argv, "robot_server");
    spark_robot::AuboServer as ("robot","aubo_info");  // Add "aubo_info" by Hudi 2021.05.21
    ROS_INFO("Robot server node initialized successfully, and the node is waiting for client instructions...");
    ros::spin();
    return EXIT_SUCCESS;
}