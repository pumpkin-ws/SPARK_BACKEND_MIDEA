#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "vision/camera/include/realsense_driver.hpp"
#include "common.hpp"
#include <stdlib.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> cloud;

void rs2pcl(rs2::points rs2_points, rs2::frame color, cloud::Ptr pcl_cloud);

int main(int argc, char** argv) {
    std::printf("Publish images and point clouds for the eye-in-hand camera.\n");
    /**
     * @brief initialize ros node and create related topics and messages
     * 
     */
    ros::init(argc, argv, "ethstream");
    ros::NodeHandle nh;
    // initialize the image transporter topic
    image_transport::ImageTransport it(nh);
    image_transport::Publisher eih_image_publisher = it.advertise("ethstream/image", 10);
    sensor_msgs::ImagePtr img_msg;
    // initialize the point cloud transporter topic
    ros::Publisher eih_cloud_publisher = nh.advertise<cloud>("ethstream/cloud", 1);
    cloud::Ptr cloud_msg(new cloud());
    cloud_msg->header.frame_id = "ethstream_frame";
    ros::Rate loop_rate{60}; // TODO: Is 60 a good loop rate to use? What is the relation between loop rate and signal rate?
    /**
     * @brief get images and point clouds from the camera driver clas
     * 
     */
    spark_cameras::SparkRealsense eih_camera;
    eih_camera.init();
    eih_camera.setParam(EYE_TO_HAND_SERIAL, FRAME_WIDTH, FRAME_HEIGHT, FRAME_RATE);
    eih_camera.startStream();

    cv::Mat color;
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::frameset fs;
    rs2::frame rs_color;
    rs2::frame rs_depth;
    rs2::pointcloud pc;
    rs2::points rs_points;

    while(nh.ok()) {
        fs = eih_camera.m_pipeline.wait_for_frames();
        fs = align_to.process(fs);
        rs_color = fs.get_color_frame();
        rs_depth = fs.get_depth_frame();
        rs_points = pc.calculate(rs_depth);
        
        color = cv::Mat(
                    cv::Size(FRAME_WIDTH, FRAME_HEIGHT), 
                    CV_8UC3, 
                    (void*)rs_color.get_data(), 
                    cv::Mat::AUTO_STEP
        );

        rs2pcl(rs_points, rs_color, cloud_msg);

        /**
         * @brief publish the image and the point cloud here
         * 
         */
        if(!color.empty()) {
            img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
            eih_image_publisher.publish(img_msg);
        }
        sensor_msgs::PointCloud2 output;
        output.header.frame_id = "ethstream_pointcloud2";
        
        if(!cloud_msg->empty()) {
            pcl::toROSMsg(*cloud_msg, output);
            output.header.stamp = ros::Time::now();
            pcl_conversions::toPCL(ros::Time::now(), cloud_msg->header.stamp);
            eih_cloud_publisher.publish(output);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;    
}

void rs2pcl(rs2::points rs2_points, rs2::frame color, cloud::Ptr pcl_cloud) {
    auto stream_profile = rs2_points.get_profile().as<rs2::video_stream_profile>();
    pcl_cloud->width = static_cast<uint32_t>(stream_profile.width());
    pcl_cloud->height = static_cast<uint32_t>(stream_profile.height());
    pcl_cloud->is_dense = false;
    pcl_cloud->points.resize(rs2_points.size());

    auto vertices = rs2_points.get_vertices();
    auto textures = rs2_points.get_texture_coordinates();

    for(int i = 0; i < rs2_points.size(); i++) {
        auto rgb_value = spark_cameras::SparkRealsense::getTexColor(color, textures[i]);
        pcl_cloud->points[i].x = vertices[i].x;
        pcl_cloud->points[i].y = vertices[i].y;
        pcl_cloud->points[i].z = vertices[i].z;
        pcl_cloud->points[i].r = (std::get<2>(rgb_value));
        pcl_cloud->points[i].g = (std::get<1>(rgb_value));
        pcl_cloud->points[i].b = (std::get<0>(rgb_value));
    }
    return;
};