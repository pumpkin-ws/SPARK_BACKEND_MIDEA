#include "spark_service_manager.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ServiceManager");
    ROS_INFO("Initiating the service manager!");
    // TODO: start the manager action server here
    spark_backend::ServiceManager as("spark_manager");
    // TODO: spin the ros node to start service manager node
    ros::spin();
    return EXIT_SUCCESS;
}