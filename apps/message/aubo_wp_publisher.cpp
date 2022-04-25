#include <ros/ros.h>
#include "robot/aubo/include/aubo_move_utils.hpp"
#include "spark_backend/AuboInfo.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

int main(int argc, char** argv) {
    ros::init(argc, argv, "aubo_wp_publisher");
    ros::NodeHandle nh;
    ros::Publisher aubo_pub = nh.advertise<spark_backend::AuboInfo>("aubo_wp_info", 10);
    spark_robot::AuboUtils aubo;
    aubo.startJointStreaming();
    aubo.startWayPointStreaming();
    spark_backend::AuboInfo aubo_info;
    ros::Rate loop_rate(60);

    while(ros::ok()) {
        // auto start = std::chrono::steady_clock::now();
        aubo_info.joint_values.clear();
        aubo_info.waypoint_values.clear();
        aubo_info.joint_values = aubo.get_m_joints();
        aubo_info.waypoint_values = aubo.get_m_waypoints();
// if (aubo_info.waypoint_values.size() != 0)
// printf("The current waypoints are: %.4f, %.4f, %.4f\n", aubo_info.waypoint_values[0], aubo_info.waypoint_values[1], aubo_info.waypoint_values[2]);
        aubo_pub.publish(aubo_info);
        loop_rate.sleep();
        ros::spinOnce();
    }

}
