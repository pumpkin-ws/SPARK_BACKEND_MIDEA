#include "ros/ros.h"
#include "vision_services.hpp"

int main(int argc, char** argv) {
    ROS_INFO("Initiating the vision server node....");
    ros::init(argc, argv, "vision_server");
    spark_vision::VisionServer vs ("vision");
    ROS_INFO("Vision server node initialized successfully, and the node is waiting for client instructions...");
    ros::spin();
    return EXIT_SUCCESS;
}