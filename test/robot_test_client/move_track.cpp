#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/RobotServiceAction.h"
#include <thread>
#include <vector>
#include "common.hpp"
#include <cmath>

int main(int argc, char** argv) {
    {
        // a test of storing multiple vectors into one long vector
        std::vector<double> total;
        std::vector<double> a{1, 2, 3, 4};
        std::vector<double> b{5, 6, 7, 8};
        total.insert(total.end(), a.begin(), a.end());
        total.insert(total.end(), b.begin(), b.end());
        for(auto elem : total) {
            std::cout << elem << std::endl;
        }
    }
    ros::init(argc, argv, "robot_client");
    actionlib::SimpleActionClient<spark_backend::RobotServiceAction> ac("robot", true);
    ROS_INFO("Waiting for the server to start");
    ac.waitForServer();
    spark_backend::RobotServiceGoal goal;
    goal.num_of_track_points = 4;
    std::vector<double> point1{
        14.45 * M_PI / 180.0,
        -53.97 * M_PI / 180.0,
        58.04 * M_PI / 180.0,
        -10.61 * M_PI / 180.0,
        90.15 * M_PI / 180.0,
        174.949 * M_PI / 180.0
    };
    std::vector<double> point2{
        14.45 * M_PI / 180.0,
        -45.97 * M_PI / 180.0,
        58.04 * M_PI / 180.0,
        -10.61 * M_PI / 180.0,
        90.15 * M_PI / 180.0,
        174.949 * M_PI / 180.0
    };
    std::vector<double> point3{
        14.45 * M_PI / 180.0,
        -21.97 * M_PI / 180.0,
        58.04 * M_PI / 180.0,
        -10.61 * M_PI / 180.0,
        90.15 * M_PI / 180.0,
        174.949 * M_PI / 180.0
    };
    std::vector<double> point4{
        4.45 * M_PI / 180.0,
        -11.97 * M_PI / 180.0,
        58.04 * M_PI / 180.0,
        -10.61 * M_PI / 180.0,
        80.15 * M_PI / 180.0,
        154.949 * M_PI / 180.0
    };
    goal.move_track_pos.insert(goal.move_track_pos.end(), point1.begin(), point1.end());
    goal.move_track_pos.insert(goal.move_track_pos.end(), point2.begin(), point2.end());
    goal.move_track_pos.insert(goal.move_track_pos.end(), point3.begin(), point3.end());
    goal.move_track_pos.insert(goal.move_track_pos.end(), point4.begin(), point4.end());
    goal.use_joint = true;
    goal.move_type = static_cast<int>(MoveType::TRACK_JOINT);
    ac.sendGoal(goal);
    sleep(2);
    // get the current server status
    std::cout << "The server status is: ";
    std::cout << ac.getState().toString() << std::endl; 

    return EXIT_SUCCESS;
}