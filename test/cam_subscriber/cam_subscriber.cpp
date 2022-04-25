/**
 * @file cam_subscriber.cpp
 * @author Sheng Wei (you@domain.com)
 * @brief This program tests the synchronized image and point cloud subscriber
 * @version 0.1
 * @date 2021-04-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ros/ros.h"
#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <opencv2/opencv.hpp>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy2;


void callback(const sensor_msgs::ImageConstPtr &image_ptr, const sensor_msgs::PointCloud2ConstPtr &cloud_ptr) {
    ROS_INFO("Able to receive image and point cloud information.");
    // display the image and the point cloud here
    cv::Mat image = cv_bridge::toCvShare(image_ptr, "bgr8")->image.clone();
    cv::imshow("current image", image);
    cv::waitKey(10);
}

void callback2(const sensor_msgs::ImageConstPtr &image_ptr, const sensor_msgs::ImageConstPtr &image_ptr2) {
    ROS_INFO("In the image synchronizer of the same topic.");
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cam_subscriber");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_subscriber(nh, "eihstream/image", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_subscriber(nh, "eihstream/pointcloud2", 10);
    message_filters::Subscriber<sensor_msgs::Image> image_subscriber2(nh, "eihstream/image", 10);

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_subscriber, pointcloud_subscriber);
    message_filters::Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(10), image_subscriber, image_subscriber2);
    // sync.setMaxIntervalDuration(ros::Duration(10));
    sync.registerCallback(boost::bind(&callback, _1, _2));
    // sync.setMaxIntervalDuration(ros::Duration(10));
    // sync2.registerCallback(boost::bind(&callback2, _1, _2));
    ros::Rate loop_rate(90);
    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}