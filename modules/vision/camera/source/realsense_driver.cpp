#include "realsense_driver.hpp"

namespace spark_cameras{

SparkRealsense::SparkRealsense():
    m_ret(0),
    M_FRAME_MODE(1){
        printf("SPARK Realsense: Create a realsense object.\n");
    };

SparkRealsense::SparkRealsense(int frame_mode):
    m_ret(0),
    M_FRAME_MODE(frame_mode){
        printf("SPARK Realsense: Create a realsense object.\n");
    };

SparkRealsense::~SparkRealsense(){
    printf("SPARK Realsense: Delete a SparkRealsense object.\n");
}

int SparkRealsense::init(){
    printf("SPARK Warning: No need to call init() function for realsense.\n");
    return 0;
}
int SparkRealsense::setParam(std::string serial, int width, int height, int fps) {
    if (width <= 0 || height <= 0 || fps <=0) {
        printf("SPARK Warning: Unvalid parameters! Have used the default values.\n");
        return 0;
    }
    for (int i = 0; i< serial.size(); i++) {
        if (serial.at(i) >'9' || serial.at(i) < '0') {
            printf("SPARK Warning: Unvalid parameters! Using default serial number.\n");
            return 0;
        }  
    }
    m_serial = serial;
    m_width = width;
    m_height = height;
    m_fps = fps;
    return 0;
}

int SparkRealsense::startStream(){
    switch (M_FRAME_MODE) {
    case 1:
        m_config.enable_stream(RS2_STREAM_COLOR, m_width, m_height, RS2_FORMAT_RGB8, m_fps);
        m_config.enable_stream(RS2_STREAM_DEPTH, m_width, m_height, RS2_FORMAT_Z16, m_fps);
        m_pipeline.start(m_config); 
        break;
    case 2:
        m_config.enable_stream(RS2_STREAM_COLOR, m_width, m_height, RS2_FORMAT_RGB8, m_fps);
        m_pipeline.start(m_config);
        break;
    default:
        printf("SPARK FAILED: Wrong frame_mode. Please choose frame_mode [1-2].\n");
        return 1;
    }
    printf("SPARK Realsense: Stream start getting frames.\n");
    return 0;
}

int SparkRealsense::getPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc){
    if ( M_FRAME_MODE != 1 ) {
        printf("SPARK FAILED: Wrong frame_mode. Please choose frame_mode 1.\n");
        return 1;
    }
    // get rs_points
    m_frameset = m_pipeline.wait_for_frames();
    rs2::align align_to(RS2_STREAM_COLOR);
    m_frameset = align_to.process(m_frameset);
    m_color_frame = m_frameset.get_color_frame();
    m_depth_frame = m_frameset.get_depth_frame();
    m_rs_pc.map_to(m_color_frame);
    rs2::points rs_points;
    rs_points = m_rs_pc.calculate(m_depth_frame);

    //convert rs_points to pcl-point-cloud
    auto video_sp = rs_points.get_profile().as<rs2::video_stream_profile>();
    pcl_pc->width = static_cast<uint32_t>(video_sp.width());
	pcl_pc->height = static_cast<uint32_t>(video_sp.height());
	pcl_pc->is_dense = false;
	pcl_pc->points.resize(rs_points.size());
    auto vertices = rs_points.get_vertices();
	auto textures = rs_points.get_texture_coordinates();

	for (int i = 0; i < rs_points.size(); i++){
		auto rgb_value = getTexColor(m_color_frame, textures[i]);
		pcl_pc->points[i].x = vertices[i].x;
		pcl_pc->points[i].y = vertices[i].y;
		pcl_pc->points[i].z = vertices[i].z;
		pcl_pc->points[i].r = (std::get<2>(rgb_value));
		pcl_pc->points[i].g = (std::get<1>(rgb_value));
		pcl_pc->points[i].b = (std::get<0>(rgb_value));
	}
    printf("SPARK Realsense: Get pcl point cloud succeed.\n");
    return 0;
}

int SparkRealsense::getRGBFrame(cv::Mat *cv_mat_ptr){
    if (M_FRAME_MODE != 1 && M_FRAME_MODE != 2) {
        printf("SPARK FAILED: Wrong frame_mode. Please choose frame_mode 1 or 2.\n");
        return 1;
    }
    if (cv_mat_ptr == nullptr){
        printf("SPARK FAILED: The input cv_mat_ptr is nullptr.\n");
        return 1;
    }
    m_frameset = m_pipeline.wait_for_frames();
    rs2::align align_to(RS2_STREAM_COLOR);
    m_frameset = align_to.process(m_frameset);
    m_color_frame = m_frameset.get_color_frame();

    const int w = m_color_frame.as<rs2::video_frame>().get_width();
    const int h = m_color_frame.as<rs2::video_frame>().get_height();
    *cv_mat_ptr = cv::Mat(cv::Size(w, h), CV_8UC3, 
                          (void*)m_color_frame.get_data(), cv::Mat::AUTO_STEP);
    return 0;
}

int SparkRealsense::storePCDFile(std::string filename){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    m_ret = SparkRealsense::getPointCloud(pcl_pc);
    if (m_ret != 0) {
        printf("SPARK FAILED: function getPointCloud failed.\n");
        return 1;
    }
    pcl::io::savePCDFileASCII(filename, *pcl_pc);
    printf("SPARK Realsense: Store pcl point cloud in PCD file succeed.\n");
    return 0;
}

int SparkRealsense::store2DImage(std::string filename){
    cv::Mat cv_mat;
    m_ret = getRGBFrame(&cv_mat);
    if (m_ret != 0) {
        printf("SPARK FAILED: function getRGBFrame failed.\n");
        return 1;
    }
    cv::imwrite(filename, cv_mat);

    printf("SPARK Realsense: Store 2D Image succeed.\n");
    return 0;
}

int SparkRealsense::showPointcloud(){
    printf("SPARK Realsense: Enter Pointcloud Visualization.\n");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    m_ret = SparkRealsense::getPointCloud(pcl_pc);
    if (m_ret != 0) {
        printf("SPARK FAILED: function getPointCloud failed.\n");
        return 1;
    }

    pcl::visualization::PCLVisualizer viewer("Realsense Pointcloud");
	viewer.addPointCloud(pcl_pc, "Realsense Pointcloud");

    while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
        usleep(100000);
	}
    viewer.close();
    printf("SPARK Realsense: Exit Pointcloud Visualization.\n");
    return 0;
}

int SparkRealsense::showRGBLive(){
    printf("SPARK Realsense: Enter RGB live visualization");
    printf("--press 'q'/'Q' to quit--\n");
    while(1){
        cv::Mat cv_mat;
        m_ret = getRGBFrame(&cv_mat);
        if (m_ret != 0) {
            printf("SPARK FAILED: function getRGBFrame failed.\n");
            return 1;
        }
        char key = cv::waitKey(1);
        cv::imshow("Deptrum RGB Live", cv_mat);
        if (key == 'q' || key == 'Q') {
            break;
        }
        usleep(1000);
    }
    printf("SPARK Realsense: Exit RGB live visualization\n");
    return 0;
}

int SparkRealsense::release(){
    m_pipeline.stop();
    printf("SPARK Realsense: Release succeed.\n");
    return 0;
}

//use std::get<index> to index a tuple object representing the RGB values
std::tuple<uint8_t, uint8_t, uint8_t> SparkRealsense::getTexColor(
    rs2::video_frame texture, 
    rs2::texture_coordinate tex_coords)
{
    const int w = texture.get_width();
    const int h = texture.get_height();
    int x = std::min(std::max(int(tex_coords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(tex_coords.v*h + .5f), 0), h - 1);
    int idx = x * texture.get_bytes_per_pixel() + 
              y * texture.get_stride_in_bytes();
    //this line converts the image data
    const auto texture_data = 
        reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(
        texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

//this function is copied from realsense-sdk-helpers
cv::Mat SparkRealsense::frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

}