#ifndef REALSENSE_DRIVER_HPP_
#define REALSENSE_DRIVER_HPP_

#include "camera_base.hpp"

#include <librealsense2/rs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace spark_cameras {

    class SparkRealsense : public SparkCameras {
    public:
        SparkRealsense();
        void help();
        explicit SparkRealsense(int frame_mode);
        ~SparkRealsense();
        int init() override;
        int setParam(std::string serial, int width, int height, int fps) override;
        int startStream() override;
        int getPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc) override;
        int getRGBFrame(cv::Mat *cv_mat_ptr) override;
        int storePCDFile(std::string filename) override;
        int store2DImage(std::string filename) override;
        int showPointcloud() override;
        int showRGBLive() override;
        int release() override;
        rs2::pipeline m_pipeline;
        static std::tuple<uint8_t, uint8_t, uint8_t> getTexColor(
            rs2::video_frame texture, 
            rs2::texture_coordinate tex_coords);

    private:
        const int M_FRAME_MODE; // uninitialized private constant object, can only be initialized once
        cv::Mat frame_to_mat(const rs2::frame& f);
        SparkRealsense(const SparkRealsense&) = delete;
        SparkRealsense& operator=(const SparkRealsense&) = delete;
        int m_ret;
        rs2::config m_config;
        rs2::frameset m_frameset;
        rs2::frame m_color_frame;
        rs2::frame m_depth_frame;
        rs2::pointcloud m_rs_pc;
    };

}


#endif