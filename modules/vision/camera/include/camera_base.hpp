/**
 * @file camera_base.hpp
 * @author Hu Di (you@domain.com)
 * @brief camera driver for starting realsense
 * @version 0.1
 * @date 2021-02-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef CAMERA_BASE_HPP_
#define CAMERA_BASE_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

namespace spark_cameras {

    class SparkCameras {
    public:
        SparkCameras(){}
        virtual ~SparkCameras(){}
        
        virtual int init() = 0;
        virtual int setParam(std::string serial, int width, int height, int fps) = 0;
        virtual int startStream() = 0;
        virtual int getPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc) = 0;
        virtual int getRGBFrame(cv::Mat *cv_mat_ptr) = 0;
        virtual int storePCDFile(std::string filename) = 0;
        virtual int store2DImage(std::string filename) = 0;
        virtual int showPointcloud() = 0;
        virtual int showRGBLive() = 0;
        virtual int release() = 0;
    protected:
        std::string m_serial = "1";
        int m_width = 640;
        int m_height = 360;
        int m_fps = 30;

    private:
        SparkCameras(const SparkCameras&) = delete;
        SparkCameras& operator=(const SparkCameras&) = delete;
    };
}
#endif