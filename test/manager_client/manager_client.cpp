#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/ServiceManagerAction.h"
#include "common.hpp"
#include <chrono>
#include "math/include/coord_transform.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "manager_client");
    // automatically spins the client in the background 
    actionlib::SimpleActionClient<spark_backend::ServiceManagerAction> ac("spark_manager", true);
    ROS_INFO("Waiting for server to connect!");
    ac.waitForServer();
    ROS_INFO("Server connected");
    spark_backend::ServiceManagerGoal goal;
    goal.manager_task_type = static_cast<int>(ManagerTasks::GET_AUBO_INFO);
    ac.sendGoal(goal);
    // TODO: do a chrono here to record the time, see how fast does ros returns a result.
    auto before_time = std::chrono::steady_clock::now();
    bool has_result = ac.waitForResult();
    auto after_time = std::chrono::steady_clock::now();
    double duration_milli = std::chrono::duration<double, std::milli>(after_time - before_time).count();
    printf("The time spent when calling the service is %f ms.\n", duration_milli);
    
    // get result if wait result is successful
    if(has_result) {
        ROS_INFO("Execution result has been returned from the server.");
        auto result = ac.getResult();
        for (int i = 0; i < result->current_joints.size(); i++) {
            std::cout << result->current_joints[i] * 180 / M_PI << std::endl;
        }
        printf("The current position of the robot is:\n");
        printf("x: %f, y: %f, z: %f\n", result->current_pos[0], result->current_pos[1], result->current_pos[2]);
        printf("The current rotation pose is: \n");
        Eigen::Quaternionf pose;
        pose.w() = result->current_pos[3];
        pose.x() = result->current_pos[4];
        pose.y() = result->current_pos[5];
        pose.z() = result->current_pos[6];
        Eigen::Vector3f euler_angles = spark_math::quaternionToEuler(pose);
        std::cout << euler_angles[0] * 180 / M_PI << std::endl;
        std::cout << euler_angles[1] * 180 / M_PI << std::endl;
        std::cout << euler_angles[2] * 180 / M_PI << std::endl;

        std::cout << std::endl;
    };
    
    // goal.manager_task_type = static_cast<int>(ManagerTasks::MIDEA_WIFI);
    // ac.sendGoal(goal);
    // before_time = std::chrono::steady_clock::now();
    // ac.waitForServer();
    // after_time = std::chrono::steady_clock::now();
    // duration_milli = std::chrono::duration<double, std::milli>(after_time - before_time).count();
    // printf("The time for calling an empty service is %f ms.\n", duration_milli);
    return EXIT_SUCCESS;
}