#include <ros/ros.h>
#include <spark_backend/AuboInfo.h>

void msgCallback(const spark_backend::AuboInfo::ConstPtr &msg, std::string* info) { // need to pass in as a pointer
    // can only change the content if the variable is passed by a pointer
    // use the pointer's 
    *info = msg->msg.c_str();
    for(int i = 0; i < msg->joint_values.size(); i++) {
        std::cout << i << ":" << msg->joint_values[i] << std::endl;
    }

};

void msgCB(const spark_backend::AuboInfo::ConstPtr &msg) {
    ROS_INFO("Able to enter callback");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "custom_msg_subscriber");
    ros::NodeHandle n;
    std::string info;
    ros::Subscriber sub = n.subscribe<spark_backend::AuboInfo>("aubo_info", 1, boost::bind(&msgCallback, _1, &info));
    // ros::Subscriber sub = n.subscribe<spark_backend::AuboInfo>("aubo_info", 1000, msgCB);
    ros::Rate loop_rate(200);
    while(ros::ok()) {
        ROS_INFO("Waiting for message");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}