#include <ros/ros.h>
#include <spark_backend/AuboInfo.h>

void msgCallback(const spark_backend::AuboInfo::ConstPtr &msg, std::string* info) { // need to pass in as a pointer
    // can only change the content if the variable is passed by a pointer
    // use the pointer's 
    *info = msg->msg.c_str();
    std::cout << msg->msg.c_str() << std::endl;
    ROS_INFO("Updating subscriber...");
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_msg_subscriber");
    ros::NodeHandle n;
    std::string info;
    // pass in the variable as an pointer address so the content can be changed
    ros::Subscriber sub = n.subscribe<spark_backend::AuboInfo>("aubo_info", 1000, boost::bind(&msgCallback, _1, &info));
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        if(info.empty()){
            printf("No message detected!\n");
        }
        if(!info.empty())
            std::cout << info;
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}