/**
 * @file vision_tracker.hpp
 * @author Sheng Wei (pumpkin_wang@foxmail.com)
 * @brief contains code used for 2D tracking, this version is still mainly dependent on Halcon,
 * eventually will need to get rid of halcon
 * @version 0.1
 * @date 2021-02-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef HALCON_TRACKER3D_HPP_
#define HALCON_TRACKER3D_HPP_
#include "spark_vision.hpp"

#include <map>
#include <opencv2/opencv.hpp>

#include "pcl/point_types.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "common.hpp"

/**
 * @brief The name space for vision related utilities
 * 
 */
namespace spark_vision {
    typedef pcl::PointCloud<pcl::PointXYZRGB> pclCloud;
    std::string getSparkDir() {
        const char* home = getenv("HOME");
        std::string home_dir(home);
        home_dir += "/Documents/spark";
        return home_dir;
    };
    typedef pcl::PointXYZ point_type;
    typedef pcl::PointCloud<point_type> pcl_Cloud;
    /**
     * @brief The name space for 2d trackers, GET RID OF HALCON, USE OPENCV!!
     * 
     */
    namespace tracker_3d {
        /**
         * @brief Different tracking algorithms using halcon, shape based,
         * light tracking, or others
         * 
         */
        class HalconTemplateTracker3d {
        public:
            /**
             * @brief Construct a new HalconTemplateTracker3d object
             * 
             * 
             * @param TemplateImage 
             */
            explicit HalconTemplateTracker3d(){

            };

            ~HalconTemplateTracker3d(){
                
            }

            /**
             * @brief track object based on templates already generated
             * 
             * @param image : the image may or may not contain the 
             * @return std::map<std::string, double> 
             */
            typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
            std::vector<double> track3dObject(cv::Mat current_image, PCLCloud current_cloud, std::string template_name);

            //create 3D image template
            bool modelGeneration(std::string model_name, cv::Mat current_image, PCLCloud current_cloud);
            //convert from HObject image to 
            HalconCpp::HObject cvimg2hcimg(cv::Mat);
            //display the 2D image from HObject
            void displayImage(HalconCpp::HObject);
            HalconCpp::HWindow create_window();
            void SelectTemplate(HalconCpp::HObject);
            void GenerateTemplateFromFile(std::string template_name);
            void closeAllWindow();
            /**
             * @brief Perform anisotropic tracking of the object
             * 
             * @return std::vector<double> stores the rotation angle at [0], the row of the track center at [1], and the col of the track center at [2]
             */
            std::vector<double> TrackObject(HalconCpp::HObject);

            double get_ROI_phi () const;
            double get_ROI_L1 () const;
            double get_ROI_L2 () const;
            cv::Point get_ROI_center () const;  

            HalconCpp::HTuple hv_Score;

        private:
            // delete the copy constructors to avoid any default copy behaviors
            HalconTemplateTracker3d(const HalconTemplateTracker3d&) = delete;
            HalconTemplateTracker3d& operator=(const HalconTemplateTracker3d&) = delete;

            HalconCpp::HObject Mat2HObject(cv::Mat& image);
            void get_hom_mat2d_from_matching_result (HalconCpp::HTuple hv_Row, HalconCpp::HTuple hv_Column, HalconCpp::HTuple hv_Angle, 
					 HalconCpp::HTuple hv_ScaleR, HalconCpp::HTuple hv_ScaleC, HalconCpp::HTuple *hv_HomMat2D);
            void set_display_font (HalconCpp::HTuple hv_WindowHandle, HalconCpp::HTuple hv_Size, HalconCpp::HTuple hv_Font, HalconCpp::HTuple hv_Bold, 
		       HalconCpp::HTuple hv_Slant);
            std::vector<double> readTemplateParameterFromFile(std::string filename);
            void dev_display_shape_matching_results (HalconCpp::HTuple hv_ModelID, HalconCpp::HTuple hv_Color, HalconCpp::HTuple hv_Row, 
					 HalconCpp::HTuple hv_Column, HalconCpp::HTuple hv_Angle, HalconCpp::HTuple hv_ScaleR, HalconCpp::HTuple hv_ScaleC, HalconCpp::HTuple hv_Model);
            void dev_open_window_fit_image (HalconCpp::HObject ho_Image, HalconCpp::HTuple hv_Row, HalconCpp::HTuple hv_Column, 
				HalconCpp::HTuple hv_WidthLimit, HalconCpp::HTuple hv_HeightLimit, HalconCpp::HTuple *hv_WindowHandle);
            HalconCpp::HObject ho_ModelImage, ho_ROI, ho_ImageGauss, ho_ImageReduced, ho_ModelROI, ho_ImagergbTest;
            HalconCpp::HObject ho_ImageEmphasis, ho_ImageIlluminate;
            HalconCpp::HTuple  hv_WindowHandle;
            HalconCpp::HTuple hv_Row1, hv_Column1, hv_Phi1, hv_Length11, hv_Length21, hv_ModelID;
            HalconCpp::HTuple  hv_ModelROIArea, hv_ModelROIRow, hv_ModelROIColumn;
            HalconCpp::HTuple  hv_NumLevels, hv_AngleStart, hv_AngleExtent, hv_AngleStep;
            HalconCpp::HTuple  hv_ScaleMin, hv_ScaleMax, hv_ScaleStep, hv_Metric;
            HalconCpp::HObject  ho_ModelContours, ho_ModelContours1, ho_ContoursAffineTrans;
            HalconCpp::HTuple  hv_MinContrast, hv_Row, hv_Column, hv_Angle, hv_ScaleR;
            HalconCpp::HTuple  hv_ScaleC, hv_Model, hv_OutAngle, hv_HomMat2D;
            HalconCpp::HTuple  hv_HomMat2DScale;

            //parameters for creating a display wwindow;
            Hlong m_WindowRow = 0;
            Hlong m_WindowColumn = 0;
            Hlong m_WindowWidth = RS_IMG_WIDTH;
            Hlong m_WindowHeight = RS_IMG_HEIGHT;
            HalconCpp::HWindow m_Window;

            double MinMatchingScore = 0.3, greediness = 0.9;

            const double A  =  6.44e-3,      B  = -2.17e-1,         C  =  5.153e2,
                         D  =  2.197e-1,     E  = 7.113e-3,         F  = -2.63e2,
                         k1 =  -3.21e-6,     k2 = 3.167e-6,         k3 =  5.75e-1 ;

            const double X = 512.5, Y = -313.1; //the X and Y position when taking picture
            bool m_has_template = false;
        };
    }
}


#endif