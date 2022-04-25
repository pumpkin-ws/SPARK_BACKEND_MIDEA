#include "vision_services.hpp"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include "vision/tracker/include/halcon_tracker3d.hpp"
// vs stands for vision server
using vs = spark_vision::VisionServer;

// std::string spark_vision::getSparkDir() {
//     const char* home = getenv("HOME");
//     std::string home_dir(home);
//     home_dir += "/Documents/spark";
//     return home_dir;
// };

void vs::goalCB() {
    int goal;
    if (this->m_as.isNewGoalAvailable()) {
        auto new_goal = this->m_as.acceptNewGoal();
        this->m_vision_task = static_cast<VisionTasks>(new_goal->vision_task_type);
        std::string home_dir(spark_vision::getSparkDir());
        switch(this->m_vision_task) {
            case VisionTasks::CALIB_2D_EYE_IN_HAND : {
                // get the current image and then perform distortion correction and projection correction
                std::string image_data_dir = home_dir + "/vision/" + "calib2d/" + "eye_in_hand/" + "data/";
                Eigen::Matrix3d internal_param;
                std::vector<double> distortion_coeff;
                spark_vision::cam_calib_2d::halconCamParamToEigen(
                    image_data_dir + "cam_param.data",
                    image_data_dir + "internal_param.txt",
                    image_data_dir + "distortion.txt",
                    internal_param,
                    distortion_coeff
                );                
                cv::Mat current_image; 
                PCLCloud current_cloud; 
                this->getCurrentEIHFrame(current_image, current_cloud);
                // 1 - undistort image
                spark_vision::cam_calib_2d::halconDistortionCorrection(internal_param, distortion_coeff, current_image);
                // 2 - anti-project image
                Eigen::Matrix3d projection_matrix;
                spark_vision::projection_transform_2d::readProjectionMatrix(
                    image_data_dir + "projection_matrix.txt", 
                    projection_matrix
                );
                spark_vision::projection_transform_2d::halconAntiProjection(projection_matrix, current_image);
                // 3 - calculate the hand-eye affine transformation matrix
                Eigen::Matrix3d eye_hand_matrix = Eigen::Matrix3d::Identity();
                double threshold_value = 80;
                spark_vision::hand_eye_calib_2d::halconHandEyeAffine(
                    current_image, 
                    threshold_value, 
                    image_data_dir + "input_robot_coordinate.txt",
                    eye_hand_matrix,
                    image_data_dir + "eye2hand_trans.txt"
                );
                std::cout << "The eye to hand 2D affine transformation matrix is : " << std::endl;
                std::cout << eye_hand_matrix << std::endl; 
                break;
            }
            case VisionTasks::CALIB_3D_EYE_IN_HAND : {
                // TODO: The output eye-hand calibration matrix should be stored in a file and returned to the user
                printf("Reading the camera parameters from file: %s.\n", new_goal->cam_parameter_path.c_str());
                printf("Reading calibration images from: %s.\n", new_goal->image_path.c_str());
                printf("Reading tool pose from: %s.\n", new_goal->eih_calib_tool_pose_path.c_str());
                printf("Saving calibration result to: %s.\n", new_goal->eih_calib_result_path.c_str());

                spark_vision::CalibrateEyeInHand3D_Halcon calib_eih;
                calib_eih.readCalibParametersFromYAML(new_goal->cam_parameter_path);
                calib_eih.calculateCalibrationPlatePoses(new_goal->image_path);
                calib_eih.readRobotPoseFromYAML(new_goal->eih_calib_tool_pose_path);
                calib_eih.calibrateAndSaveResult(new_goal->eih_calib_result_path);

                // return the result to the client
                Eigen::Matrix4d calib_result = calib_eih.readCalibrateResult(new_goal->eih_calib_result_path);
                std::cout << "The eye-in-hand calibration matrix is: " << std::endl;
                std::cout << calib_result << std::endl;
                std::cout << "------------------------------------" << std::endl;

                Eigen::Isometry3d calib_result_iso(calib_result);
                this->m_result.eih_calib_result.position.x = calib_result_iso.translation()[0];
                this->m_result.eih_calib_result.position.y = calib_result_iso.translation()[1];
                this->m_result.eih_calib_result.position.z = calib_result_iso.translation()[2];
                Eigen::Quaterniond pose_rotation(calib_result_iso.rotation());
                this->m_result.eih_calib_result.orientation.w = pose_rotation.w();
                this->m_result.eih_calib_result.orientation.x = pose_rotation.x();
                this->m_result.eih_calib_result.orientation.y = pose_rotation.y();
                this->m_result.eih_calib_result.orientation.z = pose_rotation.z();

                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case VisionTasks::CALIB_3D_EYE_TO_HAND : {
                break;
            }
            case VisionTasks::CALIB_INTRINSICS : {
                // perform camera intrinsics calibration and save the result 
                // load in the images
                // TODO: intrinsic calibration will require the location of 
                // TODO: should have an acquire image and save image step
                std::string image_load_dir = home_dir + "/vision/" + "calib2d/" + "eye_in_hand/" + "images/";
                std::vector<cv::Mat> image_vec;
                std::vector<std::string> filenames = spark_filesystem::getAllFileName(image_load_dir, ".png");
                // std::sort(filenames.begin(), filenames.end());
                for (int i = 0; i < filenames.size(); i++) {
                    image_vec.push_back(cv::imread(image_load_dir + filenames[i], cv::IMREAD_COLOR));
                }
                // perform camera calibration and save the result
                std::string image_data_dir = home_dir + "/vision/" + "calib2d/" + "eye_in_hand/" + "data/";
                HalconCpp::HTuple cam_param;
                spark_vision::cam_calib_2d::halconCamCalib( image_vec, 
                                                            image_data_dir + "caltab.descr", 
                                                            cam_param, 
                                                            image_data_dir + "cam_param.data");
                // convert halcon parameters to eigen parameters and save the result to a text file
                Eigen::Matrix3d internal_param;
                std::vector<double> distortion_coeff;
                spark_vision::cam_calib_2d::halconCamParamToEigen(
                    image_data_dir + "cam_param.data",
                    image_data_dir + "internal_param.txt",
                    image_data_dir + "distortion.txt",
                    internal_param,
                    distortion_coeff
                );
                this->m_result.intrinsics_calibrated = true;
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case VisionTasks::CALIB_PROJECTION : {
                // get the current images and perform distortion correction
                // 1 - undistort image
                std::string image_data_dir = home_dir + "/vision/" + "calib2d/" + "eye_in_hand/" + "data/";
                Eigen::Matrix3d internal_param;
                std::vector<double> distortion_coeff;
                spark_vision::cam_calib_2d::halconCamParamToEigen(
                    image_data_dir + "cam_param.data",
                    image_data_dir + "internal_param.txt",
                    image_data_dir + "distortion.txt",
                    internal_param,
                    distortion_coeff
                );
                // get current image and point cloud
                cv::Mat current_image;
                PCLCloud current_cloud;
                this->getCurrentEIHFrame(current_image, current_cloud);
                // first undistort the image
                spark_vision::cam_calib_2d::halconDistortionCorrection(internal_param, distortion_coeff, current_image);
                Eigen::Matrix3d projection_matrix = Eigen::Matrix3d::Identity();
                double threshold_value = 80;
                spark_vision::projection_transform_2d::halconProjectionCalib(
                    current_image,
                    threshold_value,
                    image_data_dir + "projection_pixels.txt",
                    projection_matrix,
                    image_data_dir + "projection_matrix.txt"
                );
                if (HalconCpp::HDevWindowStack::IsOpen()) {
                    HalconCpp::CloseWindow(
                        HalconCpp::HDevWindowStack::Pop()
                    );
                }
                std::cout << "The projection matrix is :\n" << std::endl;
                std::cout << projection_matrix << std::endl;
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case VisionTasks::GET_EIH_FRAME_DATA : {
                cv::Mat image;
                PCLCloud cloud;
                this->getCurrentEIHFrame(image, cloud);
                PCLCloudPtr cloud_ptr = cloud.makeShared();
                cv::imshow("image", image);
                cv::waitKey(0);
                cv::destroyAllWindows();
                pcl::visualization::PCLVisualizer viewer("cloud viewer");
                viewer.addPointCloud(cloud_ptr);
                viewer.spin();
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case VisionTasks::GENERATE_3D_TEMPLATE : {
                std::string model_name = new_goal->PCB_template_name;
                cv::Mat current_image;
                PCLCloud current_cloud;
    
                while(current_image.empty() || current_cloud.empty()) {
                    this->getCurrentEIHFrame(current_image, current_cloud);
                }
                spark_vision::tracker_3d::HalconTemplateTracker3d tracker3d;
                tracker3d.modelGeneration(model_name, current_image, current_cloud);
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            // by tuomasi
            case VisionTasks::TRACK_3D_OBJECT : {
              std::vector<double> res = track3DObject(new_goal->PCB_template_name);
              this->m_result.PCB_rotation.clear();
              this->m_result.PCB_center.clear();
              this->m_result.number_of_tracked_object = 0;
              if (res.size() == 6) {
                  geometry_msgs::Point p_center;
                  p_center.x = res[0];
                  p_center.y = res[1];
                  p_center.z = res[2];
                  this->m_result.PCB_center.push_back(p_center);

                  geometry_msgs::Point p_rotation;
                  p_rotation.x = res[3];
                  p_rotation.y = res[4];
                  p_rotation.z = res[5];
                  std::cout << res[0] << "  "<< res[1] << "  "<< res[2] << "  "<< res[3] << "  "<< res[4] << "  "<< res[5] << std::endl;
                  this->m_result.PCB_rotation.push_back(p_rotation);
                  this->m_result.number_of_tracked_object = 1;
              }
              this->m_as.setSucceeded(this->m_result);
              break;
            }
            case VisionTasks::TRACK_2D_OBJECT : {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                std::string template_name = new_goal->PCB_template_name;
                std::vector<double> uv_result = track2DObject(template_name);
                this->m_result.uv_center.x = uv_result[0];
                this->m_result.uv_center.y = uv_result[1];
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case VisionTasks::TRACK_BARCODE : {
              // track barcode and identify barcode content on the rig
              this->m_as.setSucceeded(this->m_result);
              break;
            }
            case VisionTasks::SAVE_CURRENT_FRAME : {
              cv::Mat current_img;
              PCLCloud current_cloud;
              while(current_img.empty()) {
                this->getCurrentEIHFrame(current_img, current_cloud);    
              }
              std::string img_path = new_goal->image_path + new_goal->image_name;
              cv::imwrite(img_path, current_img);
              this->m_as.setSucceeded(this->m_result);
            }
            default : {
                break;
            }
        }
    }
};

void vs::preemptCB() {
    ROS_INFO("The program is preempted.");
};

HalconCpp::HObject Mat2HObject(cv::Mat& image)
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

void vs::eyeInHandSubscriber() {
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> image_subscriber(this->m_nh, "eihstream/image", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_subscriber(this->m_nh, "eihstream/pointcloud2", 10);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_subscriber, cloud_subscriber);
    sync.registerCallback(boost::bind(&VisionServer::eyeInHandSyncCallback, this, _1, _2));
    ros::Rate loop_rate(15);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
};

void vs::getCurrentEIHFrame(cv::Mat& image, PCLCloud& cloud) {
    std::lock_guard<std::mutex> lock_guard(this->m_eih_data.mtx);
    image = this->m_eih_data.image.clone();
    cloud = this->m_eih_data.cloud;
};

void spark_vision::VisionServer::getAuboMsg(std::vector<double> &joint_vals, std::vector<double> &current_pos) {
    // lock the aubo message
    std::lock_guard<std::mutex> lock(this->m_aubo_msg.mtx);
    joint_vals.clear();
    joint_vals.insert(
        joint_vals.end(),
        this->m_aubo_msg.current_joints.begin(),
        this->m_aubo_msg.current_joints.end()
    );
    current_pos.clear();
    current_pos.insert(
        current_pos.end(),
        this->m_aubo_msg.way_points.begin(),
        this->m_aubo_msg.way_points.end()
    );
};

std::vector<double> spark_vision::VisionServer::track2DObject(std::string template_name) {
    cv::Mat current_image;
    std::vector<double> result(2);
    PCLCloud current_cloud;

    while(current_image.empty() || current_cloud.empty()) {
        this->getCurrentEIHFrame(current_image, current_cloud);
    }

    // cv::imshow("Captured image", current_image);
    // cv::waitKey(0);
    // cv::destroyWindow("Captured image");

    spark_vision::tracker_3d::HalconTemplateTracker3d tracker3d;
    HalconCpp::HObject himg = Mat2HObject(current_image);

    tracker3d.GenerateTemplateFromFile(template_name);
    std::vector<double> track_result = tracker3d.TrackObject(himg);
    if(track_result[0] < 1e-5 && track_result[1] < 1e-5 && track_result[2] < 1e-5) {
        return result;
    } else {
        result[0] = track_result[1];
        result[1] = track_result[2];
        printf("The track result from track2DObject function is: \n");
        std::cout << result[0] << ", " << result[1] << std::endl;
        return result;
    }
    // TODO: draw bounding box and save resultant image to HD
}

std::vector<double> spark_vision::VisionServer::track3DObject(std::string template_name) {
    cv::Mat current_image;
    std::vector<double> result;
    PCLCloud current_cloud;
    while(current_image.empty() || current_cloud.empty()) {
        this->getCurrentEIHFrame(current_image, current_cloud);
    }
    spark_vision::tracker_3d::HalconTemplateTracker3d tracker3d;
    std::vector<double> object4DPose = tracker3d.track3dObject(current_image, current_cloud, template_name);
    std::cout << "The object pose is:" << std::endl;
    std::cout << object4DPose[0] << " " << object4DPose[1] << " " << object4DPose[2] << std::endl;
    std::string home_dir(spark_vision::getSparkDir());

    // TODO: change the load in of the hand eye calibration matrix
    Eigen::Matrix4f eye_in_hand_3D = Eigen::Matrix4f::Identity();
    std::string hand_eye_matrix_path = home_dir + "/vision/calib3d/eye_in_hand/cam_to_robot.yml";
    YAML::Node eih_node = YAML::LoadFile(hand_eye_matrix_path);
    std::cout << "Time of calibration is " << eih_node["calibration_date"] << std::endl;
    std::vector<double> calib_values = eih_node["calibration_matrix"].as<std::vector<double>>();
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        eye_in_hand_3D(i, j) = static_cast<float>(calib_values[i*4 + j]);
      }
    }
    std::cout << hand_eye_matrix_path << std::endl;
    std::cout << "The camera to robot matrix is : \n" << eye_in_hand_3D << std::endl;
    
    //call the robot service to get the current robot pose, quaternion angle pose is returned
    double cur_robot_pos[6] = {0}; // TODO: This variable seems unnecessary
    std::vector<double> current_joints, current_pos;
    this->getAuboMsg(current_joints, current_pos);
    if (current_pos.size() != 0 && current_pos.size() == 7) {
        printf("The current position values are: \n");
        for (int i = 0; i < current_pos.size(); i++) {
            std::cout << current_pos[i] << ", ";
        }
        std::cout << std::endl;
    } else {
        LOG_WARNING("There is no data in the joint vector.");
        // FIXME: return a value when the function has a error is unsafe
        return result;
    }
    cur_robot_pos[0] = current_pos[0];
    cur_robot_pos[1] = current_pos[1];
    cur_robot_pos[2] = current_pos[2];
    Eigen::Quaternionf pose;
    pose.w() = current_pos[3];
    pose.x() = current_pos[4];
    pose.y() = current_pos[5];
    pose.z() = current_pos[6];
    pose.normalize();
    Eigen::Vector3f euler_angles = spark_math::quaternionToEuler(pose);
    cur_robot_pos[3] = euler_angles[0] * 180 / M_PI;
    cur_robot_pos[4] = euler_angles[1] * 180 / M_PI;
    cur_robot_pos[5] = euler_angles[2] * 180 / M_PI;
    std::cout << euler_angles[0] * 180 / M_PI << std::endl;
    std::cout << euler_angles[1] * 180 / M_PI << std::endl;
    std::cout << euler_angles[2] * 180 / M_PI << std::endl;
    
    // calculate the current robot pose in isometry
    std::vector<double> current_rpose{
      cur_robot_pos[0], 
      cur_robot_pos[1], 
      cur_robot_pos[2], 
      cur_robot_pos[3], 
      cur_robot_pos[4], 
      cur_robot_pos[5]};
    Eigen::Isometry3f curr_robot_pose = Eigen::Isometry3f::Identity();
    curr_robot_pose.pretranslate(
      Eigen::Vector3f(
        cur_robot_pos[0], 
        cur_robot_pos[1], 
        cur_robot_pos[2]));
    curr_robot_pose.rotate(
      spark_math::Euler_to_Quaternion(
        Eigen::Vector3f(
          spark_math::deg2rad(cur_robot_pos[3]), 
          spark_math::deg2rad(cur_robot_pos[4]), 
          spark_math::deg2rad(cur_robot_pos[5]))));
    
    // calculate the flange to tool transformation
    std::string flange_to_tool_path = home_dir + "/robot/gripper/pcba2021/tcp.yml";
    YAML::Node f2t_reader = YAML::LoadFile(flange_to_tool_path);
    Eigen::Vector3f f2t_pos(
      f2t_reader["x"].as<double>(),
      f2t_reader["y"].as<double>(),
      f2t_reader["z"].as<double>()
    );
    Eigen::Vector3f f2t_rot(
      spark_math::deg2rad(f2t_reader["rx"].as<double>()),
      spark_math::deg2rad(f2t_reader["ry"].as<double>()),
      spark_math::deg2rad(f2t_reader["rz"].as<double>())
    );
    Eigen::Isometry3f flange_tool = Eigen::Isometry3f::Identity();
    flange_tool.pretranslate(f2t_pos);
    flange_tool.rotate(
      spark_math::Euler_to_Quaternion(
        Eigen::Vector3f(f2t_rot)));

    std::cout << "flange to tool matrix is:\n" << flange_tool.matrix() << std::endl;
    
    // calculate the current tool pose
    Eigen::Isometry3f curr_tool_pose = Eigen::Isometry3f::Identity();
    curr_tool_pose =  curr_robot_pose * flange_tool;
    std::cout << "curr_tool_pose is:\n" << curr_tool_pose.matrix() << std::endl;
    std::cout << "object4DPose[2]: " << object4DPose[2] << std::endl;
    std::cout << "object4DPose[3]: " << object4DPose[3] << std::endl;
    std::cout << "object4DPose[4]: " << object4DPose[4] << std::endl;

    Eigen::Vector4f obj_Trans = curr_tool_pose.matrix() * eye_in_hand_3D.inverse() * Eigen::Vector4f(object4DPose[2], object4DPose[3], object4DPose[4], 1);
    std::cout << "obj_Trans: " << obj_Trans << std::endl;
    if (std::abs(object4DPose[4]) > 0.001){ // checks if track is successful
        //move the robot to the pose position

        result.push_back(obj_Trans[0]);
        result.push_back(obj_Trans[1]);
        result.push_back(-0.05);
        result.push_back(current_rpose[3]);
        result.push_back(current_rpose[4]);
        result.push_back(
          spark_math::rad2deg(object4DPose[0]) + 
          spark_math::rad2deg(object4DPose[1]));

    } else {
        std::cout << "could not find tracked template!" << std::endl;
    }
    return result;
}

void spark_vision::VisionServer::auboMsgSubscriber() {
    ros::NodeHandle nh;
    ros::Subscriber aubo_msg_subscriber = nh.subscribe<spark_backend::AuboInfo>("aubo_wp_info", 100, &spark_vision::VisionServer::auboMsgCallback, this); // there must be "this" at the end of the subscriber call back
    ros::Rate loop_rate(30);
    while(ros::ok() && this->m_nh.ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
};

void spark_vision::VisionServer::auboMsgCallback(const spark_backend::AuboInfo::ConstPtr &msg) {
    // lock the resource first
    std::lock_guard<std::mutex> lock(this->m_aubo_msg.mtx);
    // clear the old values
    this->m_aubo_msg.way_points.clear();
    this->m_aubo_msg.current_joints.clear();
    // insert with the new values
    this->m_aubo_msg.way_points.insert(
        this->m_aubo_msg.way_points.end(),
        msg->waypoint_values.begin(),
        msg->waypoint_values.end()
    );
    this->m_aubo_msg.current_joints.insert(
        this->m_aubo_msg.current_joints.end(),
        msg->joint_values.begin(),
        msg->joint_values.end()
    );
};

// -------------------------------for PCBA end by tuomasi-------------------------------
