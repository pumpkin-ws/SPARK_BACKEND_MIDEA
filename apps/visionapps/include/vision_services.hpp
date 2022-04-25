#ifndef VISION_SERVICES_HPP_
#define VISION_SERVICES_HPP_

#include "ros/ros.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "actionlib/server/simple_action_server.h"
#include "spark_backend/VisionServiceAction.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "pcl_ros/point_cloud.h"

#include "pcl/point_types.h"
#include "pcl/visualization/pcl_visualizer.h"

#include <thread>
#include "common.hpp"

#include "opencv2/opencv.hpp"
#include "filesystem/include/file_system.hpp"

#include "HalconCpp.h"

#include "vision/calibration/include/halcon_cam_calib_2d.hpp"
#include "vision/calibration/include/halcon_projection_transform_2d.hpp"
#include "vision/calibration/include/halcon_hand_eye_calib_2d.hpp"
#include "vision/calibration/include/halcon_cam_calib_2d.hpp"
#include "vision/calibration/include/halcon_eye_in_hand_3d.hpp"
#include "vision/tracker/include/halcon_tracker.hpp"
#include "math/include/coord_transform.hpp"

/*
        for PCBA by tuomasi begin
    */
#include "spark_backend/AuboInfo.h"
/*
        for PCBA by tuomasi end
*/

namespace spark_vision {

// type defin all clouds
typedef pcl::PointXYZ point_type;
typedef pcl::PointCloud<point_type> pcl_Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> pclCloud;

class VisionServer {
private:
    // TODO: delete the copy constructor here
    ros::NodeHandle m_nh;
    actionlib::SimpleActionServer<spark_backend::VisionServiceAction> m_as;
    spark_backend::VisionServiceFeedback m_feedback;
    spark_backend::VisionServiceResult m_result;
    std::string m_action_name;
    VisionTasks m_vision_task;

    void goalCB();
    void preemptCB();

    // pcl::visualization::PCLVisualizer m_cloud_viewer;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLCloudPtr;
    struct MediaData {
        PCLCloud cloud; 
        cv::Mat image;
        std::mutex mtx;
    };
    MediaData m_eih_data;
    MediaData m_eth_data;
    ros::Time m_prev_stamp{ros::Time::now()}; // initialize the previous time stamp with the time when the program is started
    ros::Time m_current_stamp;

    // need to write an image subscriber to get the images from the eye-in-hand stream and the eye-to-hand stream
    void eyeInHandSubscriber();
void eyeInHandSyncCallback(const sensor_msgs::ImageConstPtr &image_msg_ptr, const sensor_msgs::PointCloud2ConstPtr &cloud_msg_ptr) {
        // lock the mutex
        std::lock_guard<std::mutex> lock_guard(this->m_eih_data.mtx);
        // copy the image to the m_eih_data structure
        // the image quality is good here
        this->m_eih_data.image = cv_bridge::toCvShare(image_msg_ptr, "bgr8")->image.clone();
        m_current_stamp = image_msg_ptr->header.stamp;
        std::string vs_restart_attempted{""};
        m_nh.getParam(VS_RESTART_ATTEMPT, vs_restart_attempted);

        if (m_current_stamp.toSec() == m_prev_stamp.toSec() && vs_restart_attempted == "FALSE") {
            m_nh.setParam(CAMERA_STATUS, "DISCONNECTED");
            // throw warning message to front end and try to restart camera
            m_nh.setParam(BACKEND_WARNING, "视频数据停止更新，正在尝试重启");
            for (int i{0}; i < 5; i++) {
                // kill all possible leftover eihstream
                system("killall -9 eihstream");
                // FIXME: this is a very customary way of starting a program, need the file structure to be fixed
                system("../media/eihstream");
                system("sleep 5");
                // start the process and wait
                // check if the process is still running
                std::string cam_status{""};
                m_nh.getParam(CAMERA_STATUS, cam_status);
                if (cam_status == "CONNECTED") {
                    break;
                }
                if (cam_status == "DISCONNECTED") {
                    continue;
                }
            }
            m_nh.setParam(BACKEND_WARNING, "无法重启相机，请插拔相机接口重试启动相机");
            m_nh.setParam(VS_RESTART_ATTEMPT, "TRUE");
        }

        // convert the point cloud message to the point cloud structure
        pcl::fromROSMsg(*cloud_msg_ptr, this->m_eih_data.cloud);
    };

    void getCurrentEIHFrame(cv::Mat& image, PCLCloud& cloud_ptr);

    /**
     * @brief Get the Wifi2021 Geometric Prop object
     * 
     * @param track_result the track result obtained from the multi object tracking algorithm
     * @return std::map<int, std::map<std::string, cv::Point>> cv::Point will contain the "center" of the wifi board,  "coner1" to "corner4", "major" vector and "middle" vector
     * The minor vector is defined as the vertically up vector, so will not be displayed here. 
     */
    std::map<int, std::map<std::string, cv::Point2d>> getWifi2021GeometricProp(std::map<int, std::map<std::string, double>> track_result); 
    
    std::map<int, std::map<std::string, cv::Point2d>> getWifi2021RobotCordProp(std::map<int, std::map<std::string, cv::Point2d>> geometric_props, const Eigen::Matrix3d eye_hand_matrix);

    std::vector<double> track3DObject(std::string template_name);

    /**
     * @brief this function tracks the uv coordinates of the templates
     * 
     * @param template_name 
     * @return std::vector<double> returns the row at [0], the col at [1]
     */
    std::vector<double> track2DObject(std::string template_name);

    void auboMsgSubscriber();
    void auboMsgCallback(const spark_backend::AuboInfo::ConstPtr &msg);
    struct AuboMessage {
        std::vector<float> way_points;
        std::vector<float> current_joints;
        std::mutex mtx;
    };
    AuboMessage m_aubo_msg;
    void getAuboMsg(std::vector<double> &joint_vals, std::vector<double> &current_pos);

public:
    // The constructor
    VisionServer(std::string name) :
    m_as(m_nh, name, false),
    m_action_name(name) {
        m_as.registerGoalCallback(boost::bind(&VisionServer::goalCB, this));
        m_as.registerPreemptCallback(boost::bind(&VisionServer::preemptCB, this));
        std::thread eye_in_hand_media_subscriber(&VisionServer::eyeInHandSubscriber, this);
        eye_in_hand_media_subscriber.detach();
        std::thread t_aubo_subscriber(&VisionServer::auboMsgSubscriber, this); // need "this" at the end of starting a thread in a member function
        t_aubo_subscriber.detach();
        ROS_INFO("Started subscriber for eye in hand media.");
        m_as.start();
    }
    
    // The destructor
    ~VisionServer(void) {
        ROS_INFO("Killing the vision server...");
        m_nh.shutdown();
    }

};

std::string getSparkDir();



}

#endif
