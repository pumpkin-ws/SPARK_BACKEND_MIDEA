#include <ros/ros.h>
#include "robot/aubo/include/aubo_move_utils.hpp"
#include "spark_backend/AuboInfo.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"


int main(int argc, char** argv) {
    ros::init(argc, argv, "aubo_io_publisher");
    ros::NodeHandle nh;
    ros::Publisher aubo_pub = nh.advertise<spark_backend::AuboInfo>("aubo_io_info", 10);
    spark_robot::AuboUtils aubo;
    spark_backend::AuboInfo aubo_info;
    ros::Rate loop_rate(60);

    // start acquiring and publishing
    std::vector<double> pin_number{0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17};

    while(ros::ok()) { // ros::ok() will ensure ROS will be able to terminate the program

        // auto start = std::chrono::steady_clock::now();    
        aubo_info.DI_pin_ID.clear();
        aubo_info.DI_pin_status.clear();
        aubo_info.DO_pin_ID.clear();
        aubo_info.DO_pin_status.clear();

        for (auto elem : pin_number) {
            float DI_state = aubo.getSingleIOStatus(spark_robot::AuboMover::IOType::DI, elem);
            float DO_state = aubo.getSingleIOStatus(spark_robot::AuboMover::IOType::DO, elem);
            aubo_info.DI_pin_ID.push_back(elem);
            aubo_info.DI_pin_status.push_back(DI_state);
            aubo_info.DO_pin_ID.push_back(elem);
            aubo_info.DO_pin_status.push_back(DO_state);
        }

        aubo_pub.publish(aubo_info);
        ros::spinOnce();
        loop_rate.sleep();
        // auto end = std::chrono::steady_clock::now();
        // std::chrono::duration<double, std::milli> time_passed = end - start;
        // std::cout << "Time spent on getting robot DO/DI in one pass is " << time_passed.count() << " ms.\n";
    }

}