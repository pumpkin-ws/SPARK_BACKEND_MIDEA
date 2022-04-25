#include "halcon_tracker3d.hpp"
#include "yaml-cpp/yaml.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/io/pcd_io.h>
#include "pcl/point_types.h"
using namespace spark_vision::tracker_3d;

std::vector<double> HalconTemplateTracker3d::track3dObject(cv::Mat img, spark_vision::pclCloud cloud, std::string template_name) {
    std::vector<double> DOF4{0, 0, 0, 0, 0};
    /*
        Generate tracking model based on the stored rgb and the bounding box parameters
    */
    Eigen::Isometry3f obj_pose = Eigen::Isometry3f::Identity();
    // TODO: this should specify a string name as the file input
    GenerateTemplateFromFile(template_name);
    //get the current image and point cloud
    std::string home_dir = spark_vision::getSparkDir();
    std::string template_dir = home_dir + "/vision/track_template/" + template_name + "/";
    std::string template_parameters_path = template_dir + "track_param.yml"; 

    //convert from cv image to halcon image, perform tracking in 2D and crop out the bounding box
    HalconCpp::HObject H_Current_Img = Mat2HObject(img);
    std::vector<double> obj2DPose = TrackObject(H_Current_Img);
    cv::Point template_center, corner1, corner2, corner3, corner4;
    // TODO: it is better for this to be read from a yaml file.
    // The tempalte parameters 
    std::vector<double> Template_Parameters = readTemplateParameterFromFile(template_parameters_path);
    YAML::Node template_reader = YAML::LoadFile(template_parameters_path);
    cv::Point center;
    center.x = template_reader["x"].as<double>();
    center.y = template_reader["y"].as<double>();
    double phi = template_reader["phi"].as<double>();
    double length1 = template_reader["l1"].as<double>();
    double length2 = template_reader["l2"].as<double>();

    double diag = std::sqrt(length1 * length1 + length2 * length2);
    double alpha  = std::asin(length2 / diag) - M_PI/2;  

    //if either one of the pose comes out to be non zero, then tracking is successful, if all comes out to be zero, then tracking falied
    if ((obj2DPose[0] != 0) || (obj2DPose[1] != 0) || (obj2DPose[2] != 0)) {
        double rotate_angle = obj2DPose[0] * M_PI / 180.0;
        
        template_center.x = obj2DPose[2];
        template_center.y = obj2DPose[1];

        std::cout << template_center.x << " " << template_center.y << std::endl; 
 
        corner1.x = template_center.x + 0.5*std::sin(phi + rotate_angle + alpha)*diag;
        corner1.y = template_center.y + 0.5*std::cos(phi + rotate_angle + alpha)*diag;

        corner2.x = template_center.x + 0.5*std::sin(phi + rotate_angle - alpha) * diag;
        corner2.y = template_center.y + 0.5*std::cos(phi + rotate_angle - alpha) * diag;
        
        corner3.x = template_center.x - 0.5*std::sin(phi + rotate_angle + alpha) * diag;
        corner3.y = template_center.y - 0.5*std::cos(phi + rotate_angle + alpha) * diag;
        
        corner4.x = template_center.x - 0.5*std::sin(phi + rotate_angle - alpha) * diag;
        corner4.y = template_center.y - 0.5*std::cos(phi + rotate_angle - alpha) * diag;   

        //generate a mask based on the tracked corner points
        cv::Mat mask = cv::Mat(RS_IMG_HEIGHT, RS_IMG_WIDTH, CV_8UC1, cv::Scalar(0));
        std::vector<cv::Point> points {corner1, corner2, corner3, corner4};
        std::vector<std::vector<cv::Point>> line_coordinates{points};

        // trying to draw line
        std::cout << "Trying to draw the boundary lines" << std::endl;

        cv::Mat track_result = img.clone();
        cv::line(track_result, corner1, corner2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        cv::line(track_result, corner2, corner3, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        cv::line(track_result, corner3, corner4, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        cv::line(track_result, corner4, corner1, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

        const char *home = getenv("HOME");
        std::string home_dir(home);
        std::string cv_template_save_path = home_dir + "/Documents/spark/vision/track_template/" + template_name + "/ROI_tracked_image_"+ ".jpg"; // + time_info 
        cv::imwrite(cv_template_save_path, track_result);

        cv::drawContours(mask, line_coordinates, 0, cv::Scalar(255), cv::FILLED, 8);
        cv::imshow("mask", mask);
        cv::waitKey(50);
        cv::destroyWindow("mask");

        spark_vision::pcl_Cloud::Ptr cropped_cloud (new spark_vision::pcl_Cloud());
        for (int u = 0; u < RS_IMG_WIDTH; u++) {
            for (int v = 0; v < RS_IMG_HEIGHT; v++) {
                
                if (int(mask.at<uchar>(v, u)) != 0) {
                    //calculating point cloud
                    double x = cloud.at(u, v).x;
                    double y = cloud.at(u, v).y;
                    double z = cloud.at(u, v).z;
                    pcl::PointXYZ ap;
                    ap.x = x;
                    ap.y = y;
                    ap.z = z;
                    if(z < 0.5 && z > 0.2) cropped_cloud->points.push_back(ap); //this is equivalent to a z direction pass through filter
                    // cropped_cloud->points.push_back(ap); //this is equivalent to a z direction pass through filter
                }
            }
        } 
        std::cout << "The number of points in the captured point cloud is " << cropped_cloud->size() << std::endl;
        // cropped_cloud->width = 1;
        // cropped_cloud->height = cropped_cloud->points.size();

        //post process and clean up the point cloud here     
        pcl::RadiusOutlierRemoval<spark_vision::point_type> ror;
        ror.setRadiusSearch(0.005);
        ror.setMinNeighborsInRadius(50);
        ror.setInputCloud(cropped_cloud);
        ror.filter(*cropped_cloud);

        pcl::StatisticalOutlierRemoval<spark_vision::point_type> sor;
        sor.setMeanK(50);
        sor.setStddevMulThresh(0.5);
        sor.setInputCloud(cropped_cloud);
        sor.filter(*cropped_cloud);

        //estimate the orientation
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cropped_cloud);
        feature_extractor.compute();

        std::vector<float> moment_of_inertia;
        std::vector<float> eccentricity;
        
        pcl::PointXYZ min_point_AABB;
        pcl::PointXYZ max_point_AABB;
        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;      
        Eigen::Vector3f mass_center;
    
        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        feature_extractor.getAABB(min_point_AABB, max_point_AABB);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues(major_value, middle_value, minor_value);
        feature_extractor.getMassCenter(mass_center);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);

        /*
            This part need to be fixed and checked when testing the system
        */
        Eigen::Vector3f minor_pose_direction {0, 0, 1};  
        Eigen::Matrix3f major_rotation = Eigen::Matrix3f::Identity();

        major_rotation << std::cos(-rotate_angle + phi), -std::sin(-rotate_angle + phi),  0.0,
                          std::sin(-rotate_angle + phi),  std::cos(-rotate_angle + phi),  0.0, 
                          0.0,                            0.0,                            1.0;
        Eigen::Vector3f major_pose_direction = major_rotation * Eigen::Vector3f(1, 0, 0);
        Eigen::Vector3f middle_pose_direction = minor_pose_direction.cross(major_pose_direction);
        
        Eigen::Vector4f major_pose_col {-major_pose_direction(0), -major_pose_direction(1), -major_pose_direction(2), 0};
        Eigen::Vector4f middle_pose_col {-middle_pose_direction(0), -middle_pose_direction(1), -middle_pose_direction(2), 0};
        Eigen::Vector4f minor_pose_col{-minor_pose_direction(0), -minor_pose_direction(1), -minor_pose_direction(2), 0};
        Eigen::Vector4f trans_col{position_OBB.x, position_OBB.y, position_OBB.z, 1};

        Eigen::Matrix4f obj_pos = Eigen::Matrix4f::Identity();
        obj_pos << middle_pose_col, major_pose_col, minor_pose_col, trans_col;
        obj_pose = obj_pos;


        
        Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat (rotational_matrix_OBB);
        { //uncomment visualization for viewing pose of the object
        // pcl::visualization::PCLVisualizer viewer("cropped viewer");
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cropped_color(cropped_cloud, 0, 255, 0);
        // viewer.addPointCloud(cropped_cloud, cropped_color, "cropped");
        // viewer.addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
        // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
        // viewer.addSphere(position_OBB, 0.005, 255, 255, 0, "pos OBB");
        // viewer.addCoordinateSystem(0.1);

        // //add arrow for minor pos direction
        // double scale = 0.5;
        // pcl::PointXYZ minor_point;
        // minor_point.x = minor_pose_direction[0] * scale + position_OBB.x;
        // minor_point.y = minor_pose_direction[1] * scale + position_OBB.y;
        // minor_point.z = minor_pose_direction[2] * scale + position_OBB.z;
        // viewer.addArrow(minor_point, position_OBB, 0.0f, 0.0f, 1.0f, false, "minor");

        // pcl::PointXYZ major_point;
        // major_point.x =  major_pose_direction[0] * scale + position_OBB.x;
        // major_point.y =  major_pose_direction[1] * scale + position_OBB.y;
        // major_point.z =  major_pose_direction[2] * scale + position_OBB.z;

        // viewer.addArrow(major_point, position_OBB, 1.0f, 0.0f, 0.0f, false, "major");

        // pcl::PointXYZ middle_point;
        // middle_point.x =  middle_pose_direction[0] * scale + position_OBB.x;
        // middle_point.y =  middle_pose_direction[1] * scale + position_OBB.y;
        // middle_point.z =  middle_pose_direction[2] * scale + position_OBB.z;

        // viewer.addArrow(middle_point, position_OBB, 0.0f, 1.0f, 0.0f, false, "middle");
        
        // for (int i = 0; i < 100; i++) {
        //   std::this_thread::sleep_for(std::chrono::milliseconds(5));
        //   viewer.spinOnce();
        // }
        // viewer.close();     
        }
        std::cout << "The rotate angel in radian:" << std::endl;
        std::cout << obj2DPose[0] * M_PI / 180.0 << std::endl;

        DOF4[0] = (obj2DPose[0] * M_PI / 180.0);
        DOF4[1] = phi;
        DOF4[2] = position_OBB.x;
        DOF4[3] = position_OBB.y;
        DOF4[4] = position_OBB.z;
        return DOF4;
    } else {
        return DOF4;
    }
};

HalconCpp::HObject HalconTemplateTracker3d::cvimg2hcimg(cv::Mat cvimg){
    return Mat2HObject(cvimg);
}

void HalconTemplateTracker3d::displayImage(HalconCpp::HObject img){
    if(HalconCpp::HDevWindowStack::IsOpen())
        CloseWindow(HalconCpp::HDevWindowStack::Pop());
    dev_open_window_fit_image(img, 0, 0, RS_IMG_WIDTH, RS_IMG_HEIGHT, &this->hv_WindowHandle);
    set_display_font(this->hv_WindowHandle, 16, "mono", "true", "false");
    if(HalconCpp::HDevWindowStack::IsOpen())
        ClearWindow(HalconCpp::HDevWindowStack::GetActive());
    if(HalconCpp::HDevWindowStack::IsOpen())
        DispObj(img, HalconCpp::HDevWindowStack::GetActive());
    if(HalconCpp::HDevWindowStack::IsOpen())
        SetLineWidth(HalconCpp::HDevWindowStack::GetActive(), 2);
}

void HalconTemplateTracker3d::dev_open_window_fit_image (HalconCpp::HObject ho_Image, HalconCpp::HTuple hv_Row, HalconCpp::HTuple hv_Column, 
				HalconCpp::HTuple hv_WidthLimit, HalconCpp::HTuple hv_HeightLimit, HalconCpp::HTuple *hv_WindowHandle)
{
  
  // Local iconic variables
  
  // Local control variables
  HalconCpp::HTuple  hv_MinWidth, hv_MaxWidth, hv_MinHeight;
  HalconCpp::HTuple  hv_MaxHeight, hv_ResizeFactor, hv_ImageWidth, hv_ImageHeight;
  HalconCpp::HTuple  hv_TempWidth, hv_TempHeight, hv_WindowWidth, hv_WindowHeight;
  
  //This procedure opens a new graphics window and adjusts the size
  //such that it fits into the limits specified by WidthLimit
  //and HeightLimit, but also maintains the correct image aspect ratio.
  //
  //If it is impossible to match the minimum and maximum extent requirements
  //at the same time (f.e. if the image is very long but narrow),
  //the maximum value gets a higher priority,
  //
  //Parse input tuple WidthLimit
  if (0 != (HalconCpp::HTuple(int((hv_WidthLimit.TupleLength())==0)).TupleOr(int(hv_WidthLimit<0))))
  {
    hv_MinWidth = 500;
    hv_MaxWidth = 800;
  }
  else if (0 != (int((hv_WidthLimit.TupleLength())==1)))
  {
    hv_MinWidth = 0;
    hv_MaxWidth = hv_WidthLimit;
  }
  else
  {
    hv_MinWidth = ((const HalconCpp::HTuple&)hv_WidthLimit)[0];
    hv_MaxWidth = ((const HalconCpp::HTuple&)hv_WidthLimit)[1];
  }
  //Parse input tuple HeightLimit
  if (0 != (HalconCpp::HTuple(int((hv_HeightLimit.TupleLength())==0)).TupleOr(int(hv_HeightLimit<0))))
  {
    hv_MinHeight = 400;
    hv_MaxHeight = 600;
  }
  else if (0 != (int((hv_HeightLimit.TupleLength())==1)))
  {
    hv_MinHeight = 0;
    hv_MaxHeight = hv_HeightLimit;
  }
  else
  {
    hv_MinHeight = ((const HalconCpp::HTuple&)hv_HeightLimit)[0];
    hv_MaxHeight = ((const HalconCpp::HTuple&)hv_HeightLimit)[1];
  }
  //
  //Test, if window size has to be changed.
  hv_ResizeFactor = 1;
  GetImageSize(ho_Image, &hv_ImageWidth, &hv_ImageHeight);
  //First, expand window to the minimum extents (if necessary).
  if (0 != (HalconCpp::HTuple(int(hv_MinWidth>hv_ImageWidth)).TupleOr(int(hv_MinHeight>hv_ImageHeight))))
  {
    hv_ResizeFactor = (((hv_MinWidth.TupleReal())/hv_ImageWidth).TupleConcat((hv_MinHeight.TupleReal())/hv_ImageHeight)).TupleMax();
  }
  hv_TempWidth = hv_ImageWidth*hv_ResizeFactor;
  hv_TempHeight = hv_ImageHeight*hv_ResizeFactor;
  //Then, shrink window to maximum extents (if necessary).
  if (0 != (HalconCpp::HTuple(int(hv_MaxWidth<hv_TempWidth)).TupleOr(int(hv_MaxHeight<hv_TempHeight))))
  {
    hv_ResizeFactor = hv_ResizeFactor*((((hv_MaxWidth.TupleReal())/hv_TempWidth).TupleConcat((hv_MaxHeight.TupleReal())/hv_TempHeight)).TupleMin());
  }
  hv_WindowWidth = hv_ImageWidth*hv_ResizeFactor;
  hv_WindowHeight = hv_ImageHeight*hv_ResizeFactor;
  //Resize window
  HalconCpp::SetWindowAttr("background_color","black");
  HalconCpp::OpenWindow(hv_Row,hv_Column,hv_WindowWidth,hv_WindowHeight,0,"visible","",&(*hv_WindowHandle));
  HalconCpp::HDevWindowStack::Push((*hv_WindowHandle));
  if (HalconCpp::HDevWindowStack::IsOpen())
    SetPart(HalconCpp::HDevWindowStack::GetActive(),0, 0, hv_ImageHeight-1, hv_ImageWidth-1);
  return;
}

void HalconTemplateTracker3d::set_display_font(HalconCpp::HTuple hv_WindowHandle, HalconCpp::HTuple hv_Size, HalconCpp::HTuple hv_Font, HalconCpp::HTuple hv_Bold, 
		       HalconCpp::HTuple hv_Slant) {
  
  // Local iconic variables
  
  // Local control variables
  HalconCpp::HTuple  hv_OS, hv_Fonts, hv_Style, hv_Exception;
  HalconCpp::HTuple  hv_AvailableFonts, hv_Fdx, hv_Indices;
  
  //This procedure sets the text font of the current window with
  //the specified attributes.
  //
  //Input parameters:
  //WindowHandle: The graphics window for which the font will be set
  //Size: The font size. If Size=-1, the default of 16 is used.
  //Bold: If set to 'true', a bold font is used
  //Slant: If set to 'true', a slanted font is used
  //
  GetSystem("operating_system", &hv_OS);
  if (0 != (HalconCpp::HTuple(int(hv_Size==HalconCpp::HTuple())).TupleOr(int(hv_Size==-1))))
  {
    hv_Size = 16;
  }
  if (0 != (int((hv_OS.TupleSubstr(0,2))==HalconCpp::HTuple("Win"))))
  {
    //Restore previous behaviour
    hv_Size = (1.13677*hv_Size).TupleInt();
  }
  else
  {
    hv_Size = hv_Size.TupleInt();
  }
  if (0 != (int(hv_Font==HalconCpp::HTuple("Courier"))))
  {
    hv_Fonts.Clear();
    hv_Fonts[0] = "Courier";
    hv_Fonts[1] = "Courier 10 Pitch";
    hv_Fonts[2] = "Courier New";
    hv_Fonts[3] = "CourierNew";
    hv_Fonts[4] = "Liberation Mono";
  }
  else if (0 != (int(hv_Font==HalconCpp::HTuple("mono"))))
  {
    hv_Fonts.Clear();
    hv_Fonts[0] = "Consolas";
    hv_Fonts[1] = "Menlo";
    hv_Fonts[2] = "Courier";
    hv_Fonts[3] = "Courier 10 Pitch";
    hv_Fonts[4] = "FreeMono";
    hv_Fonts[5] = "Liberation Mono";
  }
  else if (0 != (int(hv_Font==HalconCpp::HTuple("sans"))))
  {
    hv_Fonts.Clear();
    hv_Fonts[0] = "Luxi Sans";
    hv_Fonts[1] = "DejaVu Sans";
    hv_Fonts[2] = "FreeSans";
    hv_Fonts[3] = "Arial";
    hv_Fonts[4] = "Liberation Sans";
  }
  else if (0 != (int(hv_Font==HalconCpp::HTuple("serif"))))
  {
    hv_Fonts.Clear();
    hv_Fonts[0] = "Times New Roman";
    hv_Fonts[1] = "Luxi Serif";
    hv_Fonts[2] = "DejaVu Serif";
    hv_Fonts[3] = "FreeSerif";
    hv_Fonts[4] = "Utopia";
    hv_Fonts[5] = "Liberation Serif";
  }
  else
  {
    hv_Fonts = hv_Font;
  }
  hv_Style = "";
  if (0 != (int(hv_Bold==HalconCpp::HTuple("true"))))
  {
    hv_Style += HalconCpp::HTuple("Bold");
  }
  else if (0 != (int(hv_Bold!=HalconCpp::HTuple("false"))))
  {
    hv_Exception = "Wrong value of control parameter Bold";
    throw HalconCpp::HException(hv_Exception);
  }
  if (0 != (int(hv_Slant==HalconCpp::HTuple("true"))))
  {
    hv_Style += HalconCpp::HTuple("Italic");
  }
  else if (0 != (int(hv_Slant!=HalconCpp::HTuple("false"))))
  {
    hv_Exception = "Wrong value of control parameter Slant";
    throw HalconCpp::HException(hv_Exception);
  }
  if (0 != (int(hv_Style==HalconCpp::HTuple(""))))
  {
    hv_Style = "Normal";
  }
  QueryFont(hv_WindowHandle, &hv_AvailableFonts);
  hv_Font = "";
  {
    HalconCpp::HTuple end_val48 = (hv_Fonts.TupleLength())-1;
    HalconCpp::HTuple step_val48 = 1;
    for (hv_Fdx=0; hv_Fdx.Continue(end_val48, step_val48); hv_Fdx += step_val48)
    {
      hv_Indices = hv_AvailableFonts.TupleFind(HalconCpp::HTuple(hv_Fonts[hv_Fdx]));
      if (0 != (int((hv_Indices.TupleLength())>0)))
      {
	if (0 != (int(HalconCpp::HTuple(hv_Indices[0])>=0)))
	{
	  hv_Font = HalconCpp::HTuple(hv_Fonts[hv_Fdx]);
	  break;
	}
      }
    }
  }
  if (0 != (int(hv_Font==HalconCpp::HTuple(""))))
  {
    throw HalconCpp::HException("Wrong value of control parameter Font");
  }
  hv_Font = (((hv_Font+"-")+hv_Style)+"-")+hv_Size;
  SetFont(hv_WindowHandle, hv_Font);
  return;
}

void HalconTemplateTracker3d::SelectTemplate(HalconCpp::HObject img) {
    //open a window to display the current image
    displayImage(img);
    SetWindowParam(this->hv_WindowHandle, "window_title", "Template Selection");
    this->ho_ModelImage = img;
    HalconCpp::DrawRectangle2(this->hv_WindowHandle, &this->hv_Row1, &this->hv_Column1, &this->hv_Phi1, &this->hv_Length11, &this->hv_Length21);
    GenRectangle2(&this->ho_ROI, this->hv_Row1, this->hv_Column1, this->hv_Phi1, this->hv_Length11, this->hv_Length21);
    HalconCpp::GaussFilter(this->ho_ModelImage, &this->ho_ImageGauss, 5);
    HalconCpp::ReduceDomain(this->ho_ModelImage, this->ho_ROI, &this->ho_ImageReduced);
    HalconCpp::GetDomain(this->ho_ImageReduced, &this->ho_ModelROI);
    HalconCpp::CreateAnisoShapeModel(this->ho_ImageReduced, "auto", HalconCpp::HTuple(0).TupleRad(), HalconCpp::HTuple(360).TupleRad(), "auto", 0.5, 1.5, "auto", 0.5, 1.5, "auto", "auto", "use_polarity", "auto", "auto", &this->hv_ModelID);
    HalconCpp::AreaCenter(this->ho_ModelROI, &this->hv_ModelROIArea, &this->hv_ModelROIRow, &this->hv_ModelROIColumn);
    HalconCpp::DispCross(this->hv_WindowHandle, this->hv_ModelROIRow, this->hv_ModelROIColumn, 26, 0);
    HalconCpp::GetShapeModelParams(this->hv_ModelID, &this->hv_NumLevels, &this->hv_AngleStart, &this->hv_AngleExtent, &this->hv_AngleStep, &this->hv_ScaleMin, &this->hv_ScaleMax, &this->hv_ScaleStep, &this->hv_Metric, &this->hv_MinContrast);
    HalconCpp::GetShapeModelContours(&this->ho_ModelContours, this->hv_ModelID, 1);
    dev_display_shape_matching_results(this->hv_ModelID, "green", this->hv_ModelROIRow, this->hv_ModelROIColumn, 0, 1, 1, 0);
    //close windows
    if (HalconCpp::HDevWindowStack::IsOpen())
        CloseWindow(HalconCpp::HDevWindowStack::Pop());
}

void HalconTemplateTracker3d::closeAllWindow() {
    if(HalconCpp::HDevWindowStack::IsOpen())
        CloseWindow(HalconCpp::HDevWindowStack::Pop());
    return;
}

std::vector<double> HalconTemplateTracker3d::TrackObject(HalconCpp::HObject img) {
    //find the matching of the template
    std::cout << "finding anisotropic shape model" << std::endl;
    HalconCpp::GaussFilter(img, &this->ho_ImageGauss, 5);
    HalconCpp::Emphasize(this->ho_ImageGauss, &this->ho_ImageEmphasis, 7, 7, 1);
    HalconCpp::Illuminate(this->ho_ImageEmphasis, &this->ho_ImageIlluminate, 20, 20, 0.55);
    try{
        HalconCpp::FindAnisoShapeModels(this->ho_ImageEmphasis, this->hv_ModelID, HalconCpp::HTuple(-45).TupleRad(), HalconCpp::HTuple(45).TupleRad(), 
              0.5, 1.5, 0.5, 1.5, this->MinMatchingScore, 1, 0.5, "least_squares", 5, this->greediness, &this->hv_Row, &this->hv_Column, 
              &this->hv_Angle, &this->hv_ScaleR, &this->hv_ScaleC, &this->hv_Score, &this->hv_Model);
    } catch (HalconCpp::HException& e) {
        std::cout << e.ErrorMessage() << std::endl;
    }

    // displayImage(img);
    std::vector<double> obj2DPose(3);
    //check if the obtained model has a correct matching
    if(0 != (int((hv_Row.TupleLength() < 1)))){
        // ClearWindow(HDevWindowStack::GetActive());
        // DispObj(img, HDevWindowStack::GetActive());       
        // disp_message(hv_WindowHandle, "No objects found", "window", 12, 12, "black", "true");
        LOG_ERROR("unable to find anisotropic shape mdoel");
    }else{
        //show the image and display the matching result
        LOG_DEBUG("able to find anisotropic shape mdoel");
        this->hv_OutAngle = (this->hv_Angle * 180) / 3.14;

        HalconCpp::GetShapeModelContours(&this->ho_ModelContours1, this->hv_ModelID, 1);
        HalconCpp::VectorAngleToRigid(0, 0, 0, this->hv_Row, this->hv_Column, this->hv_Angle, &this->hv_HomMat2D);
        HalconCpp::HomMat2dScale(this->hv_HomMat2D, this->hv_ScaleR, this->hv_ScaleC, this->hv_Row, this->hv_Column, &this->hv_HomMat2DScale);
        HalconCpp::AffineTransContourXld(this->ho_ModelContours1, &this->ho_ContoursAffineTrans, this->hv_HomMat2DScale);
        
        double X_obj, Y_obj;
        double X1 = 0, Y1 = 0; //the current robot pose offset in X and Y
        double k = this->k1 * double(this->hv_Row) + this->k2 * double(this->hv_Column) + this->k3;
        X_obj =  (this->A * double(this->hv_Row) + this->B * double(this->hv_Column) + this->C) / k;
        Y_obj = (this->D * double(this->hv_Row) + this->E * double(this->hv_Column) + this->F) / k;
 
        obj2DPose[0] = double(this->hv_OutAngle);
        obj2DPose[1] = this->hv_Row;
        obj2DPose[2] = this->hv_Column;
    }
    return obj2DPose;
}

void HalconTemplateTracker3d::GenerateTemplateFromFile(std::string template_name) {
    std::string home_dir = spark_vision::getSparkDir();
    std::string template_dir = home_dir + "/vision/track_template/" + template_name + "/";
    std::string cv_template_save_path = template_dir + template_name + "_color.jpg";
    std::cout << cv_template_save_path << std::endl;
    cv::Mat rgb_template = cv::imread(cv_template_save_path);

    std::string template_parameters_path = template_dir + "/track_param.yml"; 

    YAML::Node template_reader = YAML::LoadFile(template_parameters_path);
    cv::Point center;
    center.x = template_reader["x"].as<double>();
    center.y = template_reader["y"].as<double>();
    double phi = template_reader["phi"].as<double>();
    double length1 = template_reader["l1"].as<double>();
    double length2 = template_reader["l2"].as<double>();

    this->hv_Row1 = center.y;
    this->hv_Column1 = center.x;
    this->hv_Phi1 = phi;
    this->hv_Length11 = length1;
    this->hv_Length21 = length2;
    std::cout << "Generating ROI.." << std::endl;
    this->ho_ModelImage = Mat2HObject(rgb_template);
    GenRectangle2(&this->ho_ROI, this->hv_Row1, this->hv_Column1, this->hv_Phi1, this->hv_Length11, this->hv_Length21);
    HalconCpp::GaussFilter(this->ho_ModelImage, &this->ho_ImageGauss, 5);
    HalconCpp::ReduceDomain(this->ho_ModelImage, this->ho_ROI, &this->ho_ImageReduced);
    HalconCpp::GetDomain(this->ho_ImageReduced, &this->ho_ModelROI);
    // FIXME: too many unexplained variables in this function
    HalconCpp::CreateAnisoShapeModel(this->ho_ImageReduced, "auto", HalconCpp::HTuple(0).TupleRad(), HalconCpp::HTuple(360).TupleRad(), "auto", 0.5, 1.5, "auto", 0.5, 1.5, "auto", "auto", "use_polarity", "auto", "auto", &this->hv_ModelID);
    HalconCpp::AreaCenter(this->ho_ModelROI, &this->hv_ModelROIArea, &this->hv_ModelROIRow, &this->hv_ModelROIColumn);
    HalconCpp::GetShapeModelParams(this->hv_ModelID, &this->hv_NumLevels, &this->hv_AngleStart, &this->hv_AngleExtent, &this->hv_AngleStep, &this->hv_ScaleMin, &this->hv_ScaleMax, &this->hv_ScaleStep, &this->hv_Metric, &this->hv_MinContrast);
    HalconCpp::GetShapeModelContours(&this->ho_ModelContours, this->hv_ModelID, 1);
    std::cout << "Model generated from prestored files successfully" << std::endl;
    
};

HalconCpp::HObject HalconTemplateTracker3d::Mat2HObject(cv::Mat& image)
{
  HalconCpp::HObject Hobj=HalconCpp::HObject();
  int hgt=image.rows;
  int wid=image.cols;
  int i;
  //	CV_8UC3
  if(image.type() == CV_8UC3)
  {
    std::vector<cv::Mat> imgchannel;
    split(image,imgchannel);
    cv::Mat imgB=imgchannel[0];
    cv::Mat imgG=imgchannel[1];
    cv::Mat imgR=imgchannel[2];
    uchar* dataR=new uchar[hgt*wid];
    uchar* dataG=new uchar[hgt*wid];
    uchar* dataB=new uchar[hgt*wid];
    for(i=0;i<hgt;i++)
    {
      memcpy(dataR+wid*i,imgR.data+imgR.step*i,wid);
      memcpy(dataG+wid*i,imgG.data+imgG.step*i,wid);
      memcpy(dataB+wid*i,imgB.data+imgB.step*i,wid);
    }
    HalconCpp::GenImage3(&Hobj,"byte",wid,hgt,(Hlong)dataR,(Hlong)dataG,(Hlong)dataB);
    delete []dataR;
    delete []dataG;
    delete []dataB;
    dataR=NULL;
    dataG=NULL;
    dataB=NULL;
  }
  //	CV_8UCU1
  else if(image.type() == CV_8UC1)
  {
    uchar* data=new uchar[hgt*wid];
    for(i=0;i<hgt;i++)
      memcpy(data+wid*i,image.data+image.step*i,wid);
    HalconCpp::GenImage1(&Hobj,"byte",wid,hgt,(Hlong)data);
    delete[] data;
    data=NULL;
  }
  return Hobj;
}

std::vector<double> HalconTemplateTracker3d::readTemplateParameterFromFile(std::string filename) {
    std::ifstream infile;
    //std::cout << "trying to open file" << std::endl;
    infile.open(filename);
    //std::cout << "open file successfully" << std::endl;
    std::string s;
    const char *split = ",";
    std::vector<double> line;
    double num;
    std::getline(infile, s);
    char *s_input = (char *)s.c_str();
    char *p = strtok(s_input, split);
    while (p != NULL) {
        num = atof(p);
        line.push_back(num);
        p = strtok(NULL, split);
    }

    infile.close();
    return line;
}

void HalconTemplateTracker3d::dev_display_shape_matching_results (HalconCpp::HTuple hv_ModelID, HalconCpp::HTuple hv_Color, HalconCpp::HTuple hv_Row, 
					 HalconCpp::HTuple hv_Column, HalconCpp::HTuple hv_Angle, HalconCpp::HTuple hv_ScaleR, HalconCpp::HTuple hv_ScaleC, HalconCpp::HTuple hv_Model)
{
  
  // Local iconic variables
  HalconCpp::HObject  ho_ClutterRegion, ho_ModelContours, ho_ContoursAffinTrans;
  HalconCpp::HObject  ho_RegionAffineTrans;
  
  // Local control variables
  HalconCpp::HTuple  hv_WindowHandle, hv_UseClutter, hv_UseClutter0;
  HalconCpp::HTuple  hv_HomMat2D, hv_ClutterContrast, hv_Index, hv_Exception;
  HalconCpp::HTuple  hv_NumMatches, hv_GenParamValue, hv_HomMat2DInvert;
  HalconCpp::HTuple  hv_Match, hv_HomMat2DTranslate, hv_HomMat2DCompose;
  
  //This procedure displays the results of Shape-Based Matching.
  //
  //Ensure that the different models have the same use_clutter value.
  //
  //This procedure displays the results on the active graphics window.
  if (HalconCpp::HDevWindowStack::IsOpen())
    hv_WindowHandle = HalconCpp::HDevWindowStack::GetActive();
  //If no graphics window is currently open, nothing can be displayed.
  if (0 != (int(hv_WindowHandle==-1)))
  {
    return;
  }
  //
  hv_UseClutter = "false";
  try
  {
    GetShapeModelClutter(&ho_ClutterRegion, HalconCpp::HTuple(hv_ModelID[0]), "use_clutter", 
			 &hv_UseClutter0, &hv_HomMat2D, &hv_ClutterContrast);
    {
      HalconCpp::HTuple end_val14 = (hv_ModelID.TupleLength())-1;
      HalconCpp::HTuple step_val14 = 1;
      for (hv_Index=0; hv_Index.Continue(end_val14, step_val14); hv_Index += step_val14)
      {
	GetShapeModelClutter(&ho_ClutterRegion, HalconCpp::HTuple(hv_ModelID[hv_Index]), "use_clutter", 
			     &hv_UseClutter, &hv_HomMat2D, &hv_ClutterContrast);
	if (0 != (int(hv_UseClutter!=hv_UseClutter0)))
	{
	  throw HalconCpp::HException("Shape models are not of the same clutter type");
	}
      }
    }
  }
  // catch (Exception) 
  catch (HalconCpp::HException &HDevExpDefaultException)
  {
    HDevExpDefaultException.ToHTuple(&hv_Exception);
  }
  if (0 != (int(hv_UseClutter==HalconCpp::HTuple("true"))))
  {
    if (HalconCpp::HDevWindowStack::IsOpen())
      SetDraw(HalconCpp::HDevWindowStack::GetActive(),"margin");
    //For clutter-enabled models, the Color tuple should have either
    //exactly 2 entries, or 2* the number of models. The first color
    //is used for the match and the second for the clutter region,
    //respectively.
    if (0 != (HalconCpp::HTuple(int((hv_Color.TupleLength())!=(2*(hv_ModelID.TupleLength())))).TupleAnd(int((hv_Color.TupleLength())!=2))))
    {
      throw HalconCpp::HException("Length of Color does not correspond to models with enabled clutter parameters");
    }
  }
  
  hv_NumMatches = hv_Row.TupleLength();
  if (0 != (int(hv_NumMatches>0)))
  {
    if (0 != (int((hv_ScaleR.TupleLength())==1)))
    {
      TupleGenConst(hv_NumMatches, hv_ScaleR, &hv_ScaleR);
    }
    if (0 != (int((hv_ScaleC.TupleLength())==1)))
    {
      TupleGenConst(hv_NumMatches, hv_ScaleC, &hv_ScaleC);
    }
    if (0 != (int((hv_Model.TupleLength())==0)))
    {
      TupleGenConst(hv_NumMatches, 0, &hv_Model);
    }
    else if (0 != (int((hv_Model.TupleLength())==1)))
    {
      TupleGenConst(hv_NumMatches, hv_Model, &hv_Model);
    }
    //Redirect all display calls to a buffer window and update the
    //graphics window only at the end, to speed up the visualization.
    SetWindowParam(hv_WindowHandle, "flush", "false");
    {
      HalconCpp::HTuple end_val49 = (hv_ModelID.TupleLength())-1;
      HalconCpp::HTuple step_val49 = 1;
      for (hv_Index=0; hv_Index.Continue(end_val49, step_val49); hv_Index += step_val49)
      {
	GetShapeModelContours(&ho_ModelContours, HalconCpp::HTuple(hv_ModelID[hv_Index]), 1);
	if (0 != (int(hv_UseClutter==HalconCpp::HTuple("true"))))
	{
	  GetShapeModelClutter(&ho_ClutterRegion, HalconCpp::HTuple(hv_ModelID[hv_Index]), HalconCpp::HTuple(), 
			       &hv_GenParamValue, &hv_HomMat2D, &hv_ClutterContrast);
	  HomMat2dInvert(hv_HomMat2D, &hv_HomMat2DInvert);
	}
	if (HalconCpp::HDevWindowStack::IsOpen())
	  SetColor(HalconCpp::HDevWindowStack::GetActive(),HalconCpp::HTuple(hv_Color[hv_Index%(hv_Color.TupleLength())]));
	{
	  HalconCpp::HTuple end_val56 = hv_NumMatches-1;
	  HalconCpp::HTuple step_val56 = 1;
	  for (hv_Match=0; hv_Match.Continue(end_val56, step_val56); hv_Match += step_val56)
	  {
	    if (0 != (int(hv_Index==HalconCpp::HTuple(hv_Model[hv_Match]))))
	    {
	      get_hom_mat2d_from_matching_result(HalconCpp::HTuple(hv_Row[hv_Match]), HalconCpp::HTuple(hv_Column[hv_Match]), 
						 HalconCpp::HTuple(hv_Angle[hv_Match]), HalconCpp::HTuple(hv_ScaleR[hv_Match]), HalconCpp::HTuple(hv_ScaleC[hv_Match]), 
						 &hv_HomMat2DTranslate);
	      AffineTransContourXld(ho_ModelContours, &ho_ContoursAffinTrans, hv_HomMat2DTranslate);
	      if (0 != (int(hv_UseClutter==HalconCpp::HTuple("true"))))
	      {
		HomMat2dCompose(hv_HomMat2DTranslate, hv_HomMat2DInvert, &hv_HomMat2DCompose);
		AffineTransRegion(ho_ClutterRegion, &ho_RegionAffineTrans, hv_HomMat2DCompose, 
				  "constant");
		if (0 != (int((hv_Color.TupleLength())==2)))
		{
		  if (HalconCpp::HDevWindowStack::IsOpen())
		    SetColor(HalconCpp::HDevWindowStack::GetActive(),HalconCpp::HTuple(hv_Color[1]));
		  if (HalconCpp::HDevWindowStack::IsOpen())
		    DispObj(ho_RegionAffineTrans, HalconCpp::HDevWindowStack::GetActive());
		  if (HalconCpp::HDevWindowStack::IsOpen())
		    SetColor(HalconCpp::HDevWindowStack::GetActive(),HalconCpp::HTuple(hv_Color[0]));
		}
		else
		{
		  if (HalconCpp::HDevWindowStack::IsOpen())
		    SetColor(HalconCpp::HDevWindowStack::GetActive(),HalconCpp::HTuple(hv_Color[(hv_Index*2)+1]));
		  if (HalconCpp::HDevWindowStack::IsOpen())
		    DispObj(ho_RegionAffineTrans, HalconCpp::HDevWindowStack::GetActive());
		  if (HalconCpp::HDevWindowStack::IsOpen())
		    SetColor(HalconCpp::HDevWindowStack::GetActive(),HalconCpp::HTuple(hv_Color[hv_Index*2]));
		}
	      }
	      if (HalconCpp::HDevWindowStack::IsOpen())
		DispObj(ho_ContoursAffinTrans, HalconCpp::HDevWindowStack::GetActive());
	    }
	  }
	}
      }
    }
    //Copy the content of the buffer window to the graphics window.
    SetWindowParam(hv_WindowHandle, "flush", "true");
    FlushBuffer(hv_WindowHandle);
  }
  return;
}

void HalconTemplateTracker3d::get_hom_mat2d_from_matching_result (HalconCpp::HTuple hv_Row, HalconCpp::HTuple hv_Column, HalconCpp::HTuple hv_Angle, 
					 HalconCpp::HTuple hv_ScaleR, HalconCpp::HTuple hv_ScaleC, HalconCpp::HTuple *hv_HomMat2D)
{
  
  // Local control variables
  HalconCpp::HTuple  hv_HomMat2DIdentity, hv_HomMat2DScale;
  HalconCpp::HTuple  hv_HomMat2DRotate;
  
  //This procedure calculates the transformation matrix for the model contours
  //from the results of Shape-Based Matching.
  //
  HalconCpp::HomMat2dIdentity(&hv_HomMat2DIdentity);
  HalconCpp::HomMat2dScale(hv_HomMat2DIdentity, hv_ScaleR, hv_ScaleC, 0, 0, &hv_HomMat2DScale);
  HalconCpp::HomMat2dRotate(hv_HomMat2DScale, hv_Angle, 0, 0, &hv_HomMat2DRotate);
  HalconCpp::HomMat2dTranslate(hv_HomMat2DRotate, hv_Row, hv_Column, &(*hv_HomMat2D));
  return;
  
  
}

bool HalconTemplateTracker3d::modelGeneration(std::string model_name, cv::Mat current_image,
    PCLCloud current_cloud) {
    pcl_Cloud::Ptr newCloud(new pcl_Cloud());
    // const char *home = getenv("HOME");
    // std::string home = getSparkDir();
    std::string home_dir = getSparkDir();
    std::cout << "model_name is: " << model_name << std::endl;
    // check if the directory exists, if not, then generate the template directory
    std::string model_dir = home_dir + "/vision/track_template/" + model_name + "/";
    if(!spark_filesystem::checkDirectoryExists(model_dir)) { // create directory if directory not already exists
      spark_filesystem::createDirectory(model_dir);
    }

    std::string model_name_rgb = model_name + "_color.jpg";
    std::string model_name_cloud_full = model_name + "_cloud.pcd"; //+  time_info
    
    cv::imwrite(model_dir + model_name_rgb, current_image); //save the image to a model 
    pcl::io::savePCDFile(model_dir + model_name_cloud_full, current_cloud);

    cv::Mat tempo = current_image.clone();
    HalconCpp::HObject H_Template_Img = Mat2HObject(current_image);
    SelectTemplate(H_Template_Img);
    double phi_original, phi, length1, length2;
    cv::Point template_center;
    length1 = 2 * get_ROI_L1();
    length2 = 2 * get_ROI_L2();
    phi_original = get_ROI_phi();
    phi = get_ROI_phi() - M_PI / 2.0;
    template_center = get_ROI_center();

    cv::Point corner1, corner2, corner3, corner4;
    double diag = std::sqrt(length1 * length1 + length2 * length2);
    double alpha = std::asin(length2 / diag);

    corner1.x = template_center.x + 0.5 * std::sin(phi + alpha) * diag;
    corner1.y = template_center.y + 0.5 * std::cos(phi + alpha) * diag;

    corner2.x = template_center.x + 0.5 * std::sin(phi - alpha) * diag;
    corner2.y = template_center.y + 0.5 * std::cos(phi - alpha) * diag;

    corner3.x = template_center.x - 0.5 * std::sin(phi + alpha) * diag;
    corner3.y = template_center.y - 0.5 * std::cos(phi + alpha) * diag;

    corner4.x = template_center.x - 0.5 * std::sin(phi - alpha) * diag;
    corner4.y = template_center.y - 0.5 * std::cos(phi - alpha) * diag;

    cv::line(tempo, corner1, corner2, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(tempo, corner2, corner3, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(tempo, corner3, corner4, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(tempo, corner4, corner1, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    
    //store the image in the Documnets directory, get the home directory, then put the file in it
    std::string cv_template_save_path = model_dir + "ROI_image_"+ ".jpg";
    std::string model_name_cloud_cropped = model_dir + "ROI_cloud_"  + ".pcd";
    cv::imwrite(cv_template_save_path, tempo); //an image with the bounding box, indicating the template
    // cv::imwrite(cv_original_save_path, original);

    //create mask
    cv::Mat mask = cv::Mat(RS_IMG_HEIGHT, RS_IMG_WIDTH, CV_8UC1, cv::Scalar(0));
    std::vector<cv::Point> points{corner1, corner2, corner3, corner4};
    std::vector<std::vector<cv::Point>> line_coordinates{points};
    cv::drawContours(mask, line_coordinates, 0, cv::Scalar(255), cv::FILLED, 8);
    pcl_Cloud::Ptr cropped_cloud (new pcl_Cloud());
    for (int u = 0; u < RS_IMG_WIDTH; u++) {
        for (int v = 0; v < RS_IMG_HEIGHT; v++) {
            if (int(mask.at<uchar>(v, u)) != 0) {
                //calculating point cloud
                double x = current_cloud.at(u, v).x;
                double y = current_cloud.at(u, v).y;
                double z = current_cloud.at(u, v).z;
                pcl::PointXYZ ap;
                ap.x = x;
                ap.y = y;
                ap.z = z;
                if(z < 0.5 && z > 0.2) cropped_cloud->points.push_back(ap); //this is equivalent to a z direction pass through filter
                // cropped_cloud->points.push_back(ap); //this is equivalent to a z direction pass through filter
            }
        }
    }

    std::cout << "The number of points in the captured point cloud is " << cropped_cloud->size() << std::endl;

    cropped_cloud->width = 1;
    cropped_cloud->height = cropped_cloud->points.size();
    std::cout << cropped_cloud->height << std::endl;
    //crop out the point cloud, and store cropped point cloud
    if (cropped_cloud->size() > 0) {
      pcl::io::savePCDFile(model_name_cloud_cropped, *cropped_cloud);
    }
    
    //store the template parameters
    std::string template_parameters_path = model_dir + "/track_param.yml"; 
    // use yaml node so the structure is much more readable
    YAML::Node model_saver;
    model_saver["x"] =  template_center.x;
    model_saver["y"] = template_center.y;
    model_saver["phi"] = phi_original; 
    model_saver["l1"] = length1;
    model_saver["l2"] = length2;
    std::ofstream fout (template_parameters_path);
    fout << model_saver;
    fout.close();

    return true;
}

cv::Point HalconTemplateTracker3d::get_ROI_center() const
{
    return cv::Point(double(hv_Column1), double(hv_Row1));
}

double HalconTemplateTracker3d::get_ROI_L1() const
{
    return double(this->hv_Length11);
}

double HalconTemplateTracker3d::get_ROI_L2() const
{
    return double(this->hv_Length21);
}

double HalconTemplateTracker3d::get_ROI_phi() const
{
    return double(this->hv_Phi1);
}