#include <ros/ros.h>
#include "robot/aubo/include/aubo_move_utils.hpp"
#include "spark_backend/AuboInfo.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

int main(int argc, char** argv) {
    ros::init(argc, argv, "aubo_publisher");
    ros::NodeHandle nh;
    ros::Publisher aubo_pub = nh.advertise<spark_backend::AuboInfo>("aubo_info", 100);
    spark_robot::AuboUtils aubo;
    aubo.startJointStreaming();
    aubo.startWayPointStreaming();
    spark_backend::AuboInfo aubo_info;
    ros::Rate loop_rate(60);

    std::vector<double> pin_number{0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17};

    // FIXME: either change the joint angle and waypoint publisher to a separate thread, or make the io publisher faster.
    while (ros::ok()) {
        auto start = std::chrono::steady_clock::now();
        aubo_info.joint_values.clear();
        aubo_info.joint_values = aubo.get_m_joints();
        aubo_info.waypoint_values = aubo.get_m_waypoints();
        printf("------------------------------------\n");
        if (aubo_info.waypoint_values.size() != 0) {
            std::cout << "x: " << aubo_info.waypoint_values[0] << std::endl;
            std::cout << "y: " << aubo_info.waypoint_values[1] << std::endl;
            std::cout << "z: " << aubo_info.waypoint_values[2] << std::endl;

            Eigen::Quaternionf rot(
                aubo_info.waypoint_values[4], 
                aubo_info.waypoint_values[5], 
                aubo_info.waypoint_values[6],
                aubo_info.waypoint_values[3]
            );
            rot.w() = aubo_info.waypoint_values[3];
            rot.x() = aubo_info.waypoint_values[4];
            rot.y() = aubo_info.waypoint_values[5];
            rot.z() = aubo_info.waypoint_values[6];
            rot.normalize();
            std::cout << "The quaternion values are :" << std::endl;
            std::cout << rot.w() << std::endl;
            std::cout << rot.x() << std::endl;
            std::cout << rot.y() << std::endl;
            std::cout << rot.z() << std::endl;

            printf("-------------------------------------\n");

            std::cout << "rx: " << rot.matrix().eulerAngles(2, 1, 0)[2] * 180.0 / M_PI<< std::endl; 
            std::cout << "ry: " << rot.matrix().eulerAngles(2, 1, 0)[1] * 180.0 / M_PI << std::endl; 
            std::cout << "rz: " << rot.matrix().eulerAngles(2, 1, 0)[0] * 180.0 / M_PI << std::endl; 
            
        }

        printf("#*************current point**************#\n");

        /**
         * @brief get all the DIs and DOs
         * 
         */
        aubo_info.DI_pin_ID.clear();
        aubo_info.DI_pin_status.clear();
        aubo_info.DO_pin_ID.clear();
        aubo_info.DO_pin_status.clear();

        auto start_get_pin = std::chrono::steady_clock::now();

        for(auto elem : pin_number) {
            float DI_state = aubo.getSingleIOStatus(spark_robot::AuboMover::IOType::DI, elem);
            float DO_state = aubo.getSingleIOStatus(spark_robot::AuboMover::IOType::DO, elem);
            aubo_info.DI_pin_ID.push_back(elem);
            aubo_info.DI_pin_status.push_back(DI_state);
            aubo_info.DO_pin_ID.push_back(elem);
            aubo_info.DO_pin_status.push_back(DO_state);
        };


        auto end_get_pin = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> time_get_pin = end_get_pin - start_get_pin;
        std::cout << "Time getting pins is " << time_get_pin.count() << " ms." << std::endl;
        
        aubo_pub.publish(aubo_info);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> time_publishing_one_message =  end - start;
        std::cout << "Time spent on publishing one message is " << time_publishing_one_message.count() << " ms." << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
}
