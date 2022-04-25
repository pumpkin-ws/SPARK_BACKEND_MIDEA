#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "spark_backend/VisionServiceAction.h"
#include "common.hpp"
#include "geometry_msgs/Point.h"

int main(int argc, char** argv) {
    std::cout << "This is the client for calling the object tracking service." << std::endl;
    ros::init(argc, argv, "vision_client");
    actionlib::SimpleActionClient<spark_backend::VisionServiceAction> ac("vision");
    ROS_INFO("Waiting for the vision server to start...");
    ac.waitForServer();
    ROS_INFO("Server started, ready to send goal...");
    spark_backend::VisionServiceGoal goal;
    goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_3D_OBJECT);
    goal.track_target_name_3d = "wifi_2021_12_06";
    ac.sendGoal(goal);
    ac.waitForResult();
    // unparse and print the result here
    auto result = ac.getResult();
    std::cout << "The number of results is : " << result->number_of_tracked_object << std::endl;
    //for(int i = 0; i < result->number_of_tracked_object; i++) {
        //printf("The track center for object %d is:\n", i);
        std::cout << result->PCB_center[0].x << std::endl;
        std::cout << result->PCB_center[0].y << std::endl;
        std::cout << result->PCB_center[0].z << std::endl;
        std::cout << result->PCB_rotation[0].x << std::endl;
        std::cout << result->PCB_rotation[0].y << std::endl;
        std::cout << result->PCB_rotation[0].z << std::endl;
    //    printf("The track major vec for object %d is:\n", i);
     //   std::cout << result->wifi_major_axis[i].x << std::endl;
     //   std::cout << result->wifi_major_axis[i].y << std::endl;
     //   std::cout << result->wifi_major_axis[i].z << std::endl;
     //   printf("The track middle vec for object %d is:\n", i);
    //    std::cout << result->wifi_middle_axis[i].x << std::endl;
     //   std::cout << result->wifi_middle_axis[i].y << std::endl;
    //    std::cout << result->wifi_middle_axis[i].z << std::endl;

    //}
    return EXIT_SUCCESS;
}
