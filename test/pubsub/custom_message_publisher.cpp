#include <ros/ros.h>
#include <spark_backend/AuboInfo.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_msg_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<spark_backend::AuboInfo>("test", 1000);
    ros::Rate loop_rate{10};
    spark_backend::AuboInfo aubo;
    aubo.msg = "Hello there!\n";
    while(ros::ok()) {
        pub.publish(aubo);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}