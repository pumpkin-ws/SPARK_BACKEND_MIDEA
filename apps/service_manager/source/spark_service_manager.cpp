#include "spark_service_manager.hpp"

using manager = spark_backend::ServiceManager;

/**
 * @brief 
 * 
 */
void manager::preemptCB() {
    ROS_INFO("The goal is preempted by the user!");
    return;
}
/**
 * @brief The goal call back will receive new goal
 * and process them accordingly
 * 
 */
void manager::goalCB() {
    if(this->m_as.isNewGoalAvailable()) {
        ROS_INFO("New service manager goal is available!");
        auto new_goal = m_as.acceptNewGoal();
        this->m_manager_tasks = static_cast<ManagerTasks>(new_goal->manager_task_type);
        switch (this->m_manager_tasks) {            
            /**
             * @brief This is bad practice, do not hard code work flow, should be 
             * designed by the user, maybe this is alright for now
             * 
             */
            case ManagerTasks::MIDEA_PCBA : {
                ROS_INFO("Executing Midea PCBA job");
                this->workflowMideaPCBA2021(new_goal->PCB_template_name, new_goal->speed_ratio);
                this->m_as.setSucceeded(this->m_result);
                break;                
            }
            case ManagerTasks::TEST_FRONTEND: {
                ROS_INFO("called test_frontend to check tracking result service!");
                std::vector<double> home {0.379398, 0.040442, 0.630250, spark_math::deg2rad(-91.0714), spark_math::deg2rad(-1.864), spark_math::deg2rad(-91.218)};
                std::vector<double> midway {0.3107, 0.04044, 0.6302, spark_math::deg2rad(-91.0714), spark_math::deg2rad(-1.864), spark_math::deg2rad(-91.218)};
                double SPEED_FRACTION = 0.05;
                if(this->cartesianMoveJointServerCall(midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                };
                if(this->cartesianMoveJointServerCall(home, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                };
                this->m_as.setSucceeded(this->m_result); 
                break;
            }
            case ManagerTasks::GET_AUBO_INFO : {
                ROS_INFO("Testing the robot subscriber..");
                std::vector<double> current_joints, current_pos;
                this->getAuboMsg(current_joints, current_pos, m_robot_ac_ptr);
                if (current_joints.size() != 0 && current_joints.size() == 6) {
                    printf("The current joint values are:\n");
                    for (int i = 0; i < current_joints.size(); i++) {
                        std::cout << current_joints[i] << ", ";
                        this->m_result.current_joints[i] = current_joints[i];
                    }
                    std::cout << std::endl;
                } else {
                    LOG_WARNING("There is no data in the joint vector.");
                }
                if (current_pos.size() != 0 && current_pos.size() == 7) {
                    printf("The current position values are: \n");
                    for (int i = 0; i < current_pos.size(); i++) { // storing result into the final 
                        // std::cout << current_pos[i] << ", ";
                        this->m_result.current_pos[i] = current_pos[i];
                    }
                    // convert the current position to euler angle representation
                    Eigen::Quaternionf angle_pose;
                    angle_pose.w() = current_pos[3];
                    angle_pose.x() = current_pos[4];
                    angle_pose.y() = current_pos[5];
                    angle_pose.z() = current_pos[6];
                    Eigen::Vector3f rotation_angle = spark_math::quaternionToEuler(angle_pose);
                    printf("The robot pose is : [%f,%f,%f,%f,%f,%f]\n", 
                            current_pos[0], 
                            current_pos[1], 
                            current_pos[2], 
                            spark_math::rad2deg(rotation_angle[0]),
                            spark_math::rad2deg(rotation_angle[1]),
                            spark_math::rad2deg(rotation_angle[2]));
                    std::cout << std::endl;
                } else {
                    LOG_WARNING("There is no data in the position vector.");
                }
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::GENERATE_MIDEA_PCB_2021_TEMPLATE: {
                // move to start position
                std::string template_name = new_goal->PCB_template_name;
                std::string pos_dir = getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos.yml";
                std::string template_dir = getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos_" + template_name + ".yml";
                // copy everything from the pos_dir and create a new file
                if(spark_filesystem::copyAndReplaceFile(pos_dir, template_dir) == false) {
                    ROS_ERROR("Unable to copy file");
                    this->m_as.setAborted();
                    break;
                };
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };
                YAML::Node temp_pos_reader = YAML::LoadFile(template_dir);
                double SPEED_FRACTION = 0.1;
                std::vector<double> image_home = yaml_pos_value_grabber(temp_pos_reader, "common", "photo_home");
                if(this->cartesianMoveJointServerCall(image_home, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                spark_backend::VisionServiceGoal goal;
                goal.PCB_template_name = new_goal->PCB_template_name;
                goal.vision_task_type = static_cast<int>(VisionTasks::GENERATE_3D_TEMPLATE);
                this->m_vision_ac_ptr->sendGoal(goal);
                this->m_vision_ac_ptr->waitForResult();
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::EIH_CALIBRATION: {
                // read in the robot poses for calibration, move the robot and record pictures
                std::string home = getSparkDir();
                std::string eih_calibration_yml = home + "/robot/waypoints/pcba2021/eih_calibration_flange.yml";
                YAML::Node calib_wp_reader = YAML::LoadFile(eih_calibration_yml);
                // A lambda expression to extract the waypoints from the yaml file
                auto waypoint_extractor = [](const YAML::Node& reader)->std::vector<std::vector<double>> {
                    std::vector<std::vector<double>> waypoints;
                    for(auto it = reader.begin(); it != reader.end(); it++) {
                        YAML::Node unparser = YAML::Load(it->second.as<std::string>());
                        std::cout << it->first.as<std::string>() << std::endl;
                        std::vector<double> curr_wp;
                        for(int i = 0; i < unparser.size(); i++) {
                            curr_wp.push_back(unparser[i].as<double>());
                        }
                        std::cout << std::endl;
                        waypoints.push_back(curr_wp);
                    }
                    return waypoints;
                };
                std::vector<std::vector<double>> waypoints = waypoint_extractor(calib_wp_reader);
                for(int i = 0; i < waypoints.size(); i++) {
                    std::cout << "pos " << i << ": ";
                    // convert the angles from degrees to radians so robot can move with
                    waypoints[i][3] = spark_math::deg2rad(waypoints[i][3]);
                    waypoints[i][4] = spark_math::deg2rad(waypoints[i][4]);
                    waypoints[i][5] = spark_math::deg2rad(waypoints[i][5]);
                }
                // The way points can be the same, but the tcp locations are probably different.
                // move to each way point and request capture and save image
                // Read in the flange to tool transformation
                std::string tcp_yaml_path = home + "/robot/gripper/pcba2021/tcp.yml";
                std::map<std::string, double> tcp;
                YAML::Node tcp_yml = YAML::LoadFile(tcp_yaml_path);
                std::cout << tcp_yml.size() << std::endl;
                printf("Extracting elements from tcp yaml..\n");
                for(auto it = tcp_yml.begin(); it != tcp_yml.end(); it++) {
                    std::cout << it->first.as<std::string>() << " : " << it->second.as<double>() << std::endl; 
                }
                tcp["x"] = tcp_yml["x"].as<double>();
                tcp["y"] = tcp_yml["y"].as<double>();
                tcp["z"] = tcp_yml["z"].as<double>();
                tcp["rx"] = tcp_yml["rx"].as<double>();
                tcp["ry"] = tcp_yml["ry"].as<double>();
                tcp["rz"] = tcp_yml["rz"].as<double>();
                Eigen::Isometry3f tcp_matrix = Eigen::Isometry3f::Identity();
                tcp_matrix.pretranslate(
                    Eigen::Vector3f(
                        tcp.find("x")->second, 
                        tcp.find("y")->second,
                        tcp.find("z")->second));
                tcp_matrix.rotate(
                    spark_math::Euler_to_Quaternion(
                        Eigen::Vector3f(
                            spark_math::deg2rad(tcp.find("rx")->second),
                            spark_math::deg2rad(tcp.find("ry")->second),
                            spark_math::deg2rad(tcp.find("rz")->second)
                        )
                    )
                );

                std::cout << "The tcp matrix is: " << std::endl;
                std::cout << tcp_matrix.matrix() << std::endl;
                std::cout << "-------------------------------\n";
                
                std::string tool_pos_yml = home + "/robot/waypoints/pcba2021/eih_calibration_tool.yml"; 
                std::ofstream tcp_pose(tool_pos_yml);
                YAML::Node tool_yml;
                for(int i = 0; i < waypoints.size(); i++) {
                    Eigen::Isometry3f flange_pose = Eigen::Isometry3f::Identity();
                    flange_pose.pretranslate(
                        Eigen::Vector3f(
                            waypoints[i][0],
                            waypoints[i][1],
                            waypoints[i][2]
                        )
                    );
                    flange_pose.rotate(
                        spark_math::Euler_to_Quaternion(
                            Eigen::Vector3f(
                                waypoints[i][3],
                                waypoints[i][4],
                                waypoints[i][5]
                            )
                        )
                    );
                    Eigen::Isometry3f tool_pose = flange_pose * tcp_matrix;
                    std::vector<double> tool_pose_euler(6);
                    tool_pose_euler[0] = tool_pose.translation()[0];
                    tool_pose_euler[1] = tool_pose.translation()[1];
                    tool_pose_euler[2] = tool_pose.translation()[2];
                    Eigen::Vector3f tool_pos_euler_rot = spark_math::quaternionToEuler(Eigen::Quaternionf(tool_pose.rotation()));
                    tool_pose_euler[3] = spark_math::rad2deg(tool_pos_euler_rot[0]);
                    tool_pose_euler[4] = spark_math::rad2deg(tool_pos_euler_rot[1]);
                    tool_pose_euler[5] = spark_math::rad2deg(tool_pos_euler_rot[2]);
                    std::string pos = "pos" + std::to_string(i);
                    tool_yml[pos] = tool_pose_euler;
                }
                tcp_pose << tool_yml;
                tcp_pose.close();
                // now we should instruct the robot to move to the proper location
                double SPEED_FRACTION = 0.05;
                spark_backend::VisionServiceGoal vis_goal;
                std::string image_path = home + "/vision/calib3d/eye_in_hand/image/";
                
                for(int i = 0; i < waypoints.size(); i++) {
                    for (auto elem : waypoints[i]) {
                        std::cout << elem << ", ";
                    }
                    std::cout << std::endl;
                    if(this->cartesianMoveJointServerCall(waypoints[i], SPEED_FRACTION, this->m_robot_ac_ptr) == false){
                        this->m_as.setAborted();
                    }
                    printf("Executing motion sequence: %d.\n", i);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // wait a second for the camera and video stream to stabilize
                    // instruct the vision service to acquire image
                    // Pass in the file save directory
                    // image save name

                    vis_goal.image_path = image_path;
                    vis_goal.image_name = std::to_string(i) + "_Color.png";
                    vis_goal.vision_task_type = static_cast<int>(VisionTasks::SAVE_CURRENT_FRAME);
                    this->m_vision_ac_ptr->sendGoal(vis_goal);
                    this->m_vision_ac_ptr->waitForResult();
                }
                // call the the calibration service
                // the directory information needed includes:
                // 1 - yaml filename of camera parameters
                // 2 - folder containing the calibration images
                // 3 - the robot pose yaml positions
                // 4 - the directory to store the calibration results
                vis_goal.vision_task_type = static_cast<int>(VisionTasks::CALIB_3D_EYE_IN_HAND);
                vis_goal.image_path = image_path;
                vis_goal.eih_calib_tool_pose_path = tool_pos_yml;
                std::string cam_param_path = home + "/vision/calib3d/eye_in_hand/cam_parameters.yaml";
                vis_goal.cam_parameter_path = cam_param_path;
                std::string calibration_result_path = home + "/vision/calib3d/eye_in_hand/";
                vis_goal.eih_calib_result_path = calibration_result_path;
                this->m_vision_ac_ptr->sendGoal(vis_goal);
                ROS_INFO("Sending eye in hand calibration service request to vision server.\n");
                auto start = std::chrono::steady_clock::now();
                this->m_vision_ac_ptr->waitForResult();
                auto end = std::chrono::steady_clock::now();
                std::chrono::duration<double, std::milli> time_elapsed = end - start;
                printf("The halcon calibration algorithm took %f ms", time_elapsed.count());
                // FIXME: is the rotoation pose from robot to camera or the other way around?
                auto calib_result = this->m_vision_ac_ptr->getResult();
                Eigen::Isometry3f eye_in_hand_calib = Eigen::Isometry3f::Identity();
                eye_in_hand_calib.pretranslate(
                    Eigen::Vector3f(
                        calib_result->eih_calib_result.position.x,
                        calib_result->eih_calib_result.position.y,
                        calib_result->eih_calib_result.position.z
                    )
                );
                Eigen::Quaternionf eih_rot = Eigen::Quaternionf::Identity();
                eih_rot.w() = calib_result->eih_calib_result.orientation.w;
                eih_rot.x() = calib_result->eih_calib_result.orientation.x;
                eih_rot.y() = calib_result->eih_calib_result.orientation.y;
                eih_rot.z() = calib_result->eih_calib_result.orientation.z;
                eye_in_hand_calib.rotate(eih_rot);
                // write the rotation matrix to a file
                std::string eih_matrix_file = home + "/vision/calib3d/eye_in_hand/cam_to_robot.yml";
                YAML::Node result_writer = YAML::LoadFile(eih_matrix_file);
                std::ofstream fwriter;
                fwriter.open(eih_matrix_file);
                std::time_t now = std::time(0);
                std::tm *ltm = localtime(&now);
                std::string time_info = std::to_string(1900 + ltm->tm_year) + "年" + 
                                        std::to_string(1 + ltm->tm_mon) + "月" + 
                                        std::to_string(ltm->tm_mday) + "日" + 
                                        std::to_string(ltm->tm_hour) + "时" + 
                                        std::to_string(ltm->tm_min) + "分" + 
                                        std::to_string(ltm->tm_sec) + "秒";
                result_writer["calibration_date"] = time_info;
                Eigen::Matrix4f eih_matrix = eye_in_hand_calib.matrix();
                std::vector<double> eih_matrix_vec;
                for(int i = 0; i < 4; i++) {
                    for(int j = 0; j < 4; j++) {
                        eih_matrix_vec.push_back(eih_matrix(i, j)); 
                    }
                }
                result_writer["calibration_matrix"] = eih_matrix_vec;
                fwriter << result_writer;
                fwriter.close();
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::MOVE_HOME: {
                //instruct the robot to move to the home photo position
                std::string wp_file = this->getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos.yml";
                YAML::Node wp_node = YAML::LoadFile(wp_file);
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };    
                std::vector<double> home_pos = yaml_pos_value_grabber(wp_node, "common","photo_home");            
                double SPEED_FRACTION = 0.05;
                if(this->cartesianMoveJointServerCall(home_pos, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                };
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::PICK_UP_BOARD : {
                //instruct the robot to move to the home photo position
                std::string template_name = new_goal->PCB_template_name;
                std::string wp_file = this->getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos_" + template_name + ".yml";
                YAML::Node wp_node = YAML::LoadFile(wp_file);
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };    
                std::vector<double> home_pos = yaml_pos_value_grabber(wp_node, "common","photo_home");         
                std::vector<double> on_belt_pos {
                    yaml_pos_value_grabber(wp_node, "common", "belt_height")
                };
                std::vector<double> robot_pick_up {
                    yaml_pos_value_grabber(wp_node, "common", "robot_pickup")
                };
                const double BELTLINE_HEIGHT = on_belt_pos[2];
                const double BELTLINE_HEIGHT_OFFSET = BELTLINE_HEIGHT + 0.1;

                double SPEED_FRACTION = 0.1;
                if(this->cartesianMoveJointServerCall(home_pos, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                };


                // perform recognition and perform grasping
                std::string tcp_yaml_path = this->getSparkDir() + "/robot/gripper/pcba2021/tcp_exec.yml";
                std::string gripper_yaml_path = getSparkDir() + "/robot/gripper/pcba2021/" + template_name + ".yml"; 
                if (!boost::filesystem::exists(gripper_yaml_path)) {
                    ROS_INFO("Template does not exist");
                    this->m_as.setAborted();
                }
                auto tcp_reader = YAML::LoadFile(tcp_yaml_path);
                auto gripper_reader = YAML::LoadFile(gripper_yaml_path);
                std::map<std::string, double> tcp, gripper_offset;
                tcp["x"] = tcp_reader["x"].as<double>();
                tcp["y"] = tcp_reader["y"].as<double>();
                tcp["z"] = tcp_reader["z"].as<double>();
                tcp["rx"] = tcp_reader["rx"].as<double>();
                tcp["ry"] = tcp_reader["ry"].as<double>();
                tcp["rz"] = tcp_reader["rz"].as<double>();
                gripper_offset["offsetx"] = gripper_reader["offsetx"].as<double>();
                gripper_offset["offsety"] = gripper_reader["offsety"].as<double>();
                gripper_offset["offsetz"] = gripper_reader["offsetz"].as<double>();
                spark_backend::VisionServiceGoal vision_goal;
                vision_goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_3D_OBJECT);
                vision_goal.PCB_template_name = template_name;
                this->m_vision_ac_ptr->sendGoal(vision_goal);
                this->m_vision_ac_ptr->waitForResult();
                auto result = this->m_vision_ac_ptr->getResult();
                while(result->number_of_tracked_object == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    vision_goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_3D_OBJECT);
                    vision_goal.PCB_template_name = template_name;
                    this->m_vision_ac_ptr->sendGoal(vision_goal);
                    this->m_vision_ac_ptr->waitForResult();
                    result = this->m_vision_ac_ptr->getResult();  
                    std::cout << "Number of tracked objects: " << result->number_of_tracked_object << std::endl;                  
                }
                Eigen::Isometry3f obj_pose = Eigen::Isometry3f::Identity();

                double rz = 0;
                int track_attempt_count{1};
                double prev_x = result->PCB_center[0].x;
                double prev_y = result->PCB_center[0].y;
                double prev_rot = result->PCB_rotation[0].z;
                double current_x{0}, current_y{0}, current_rot{0};
                bool PCB_stationary = false;
                unsigned MAXIMUM_TRACK_ATTEMPT = 50;
                while((result->number_of_tracked_object == 0) || (PCB_stationary == false)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    // try to track with new images
                    this->m_vision_ac_ptr->sendGoal(vision_goal);
                    this->m_vision_ac_ptr->waitForResult();
                    track_attempt_count++;
                    // TODO: signal warning if the track attempt is over some maximum attempts
                    if (track_attempt_count > MAXIMUM_TRACK_ATTEMPT) {
                        LOG_ERROR("Unable to track stable object after 50 attempts.");
                        return;
                    };
                    result = this->m_vision_ac_ptr->getResult(); //program will be blocked here waiting for result
                    while (result->number_of_tracked_object == 0) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        track_attempt_count++;
                        this->m_vision_ac_ptr->sendGoal(vision_goal);
                        this->m_vision_ac_ptr->waitForResult();
                        result = this->m_vision_ac_ptr->getResult();
                    }
                    current_x = result->PCB_center[0].x;
                    current_y = result->PCB_center[0].y;
                    current_rot = result->PCB_rotation[0].z;
                    double abs_dis_diff = std::sqrt(
                        std::pow(current_x - prev_x, 2.0) + std::pow(current_y - prev_y, 2.0)
                    );
                    double abs_rot_diff = std::fabs(current_rot - prev_rot);
                    prev_x = current_x;
                    prev_y = current_y;
                    prev_rot = current_rot;
                    if((abs_dis_diff < 0.002) && ((abs_rot_diff < 2) || (abs_rot_diff > 358))) PCB_stationary = true;
                }

                rz = result->PCB_rotation[0].z - 90.0;

                std::vector<double> joint_values, current_pos;
                this->getAuboMsg(joint_values, current_pos, this->m_robot_ac_ptr);
                
                Eigen::Isometry3f current_flange_pos = Eigen::Isometry3f::Identity();
                current_flange_pos.pretranslate(Eigen::Vector3f(current_pos[0], current_pos[1], current_pos[2]));
                current_flange_pos.rotate(Eigen::Quaternionf(current_pos[3], current_pos[4], current_pos[5], current_pos[6]));
                
                Eigen::Isometry3f flange_to_tool = Eigen::Isometry3f::Identity();
                flange_to_tool.pretranslate(Eigen::Vector3f(tcp["x"], tcp["y"], tcp["z"]));
                flange_to_tool.rotate(spark_math::eulerToQuaternion(Eigen::Vector3f(tcp["rx"], tcp["ry"], tcp["rz"])));

                Eigen::Isometry3f current_tool_pos = Eigen::Isometry3f::Identity();
                current_tool_pos = current_flange_pos * flange_to_tool;

                printf("The current tool pose is:\n");
                std::cout << current_tool_pos.matrix() << std::endl;

                Eigen::Vector3f dest_tool_rotation;
                dest_tool_rotation[0] = on_belt_pos[3];
                dest_tool_rotation[1] = on_belt_pos[4];
                dest_tool_rotation[2] = current_tool_pos.rotation().eulerAngles(2, 1, 0)[0] + spark_math::deg2rad(rz);
                Eigen::Isometry3f destination_tool_pose = Eigen::Isometry3f::Identity();
                Eigen::Vector3f tool_translation (result->PCB_center[0].x, result->PCB_center[0].y, result->PCB_center[0].z); 
                destination_tool_pose.pretranslate(tool_translation);  // TODO: set to the PCB board pose
                destination_tool_pose.rotate(spark_math::eulerToQuaternion(dest_tool_rotation));

                std::cout << "The destination tool pose is " << std::endl;
                std::cout << destination_tool_pose.matrix() << std::endl;

                destination_tool_pose.translate(Eigen::Vector3f(gripper_offset["offsetx"], gripper_offset["offsety"], gripper_offset["offsetz"]));
                
                Eigen::Isometry3f destination_flange_pose = (destination_tool_pose) * flange_to_tool.inverse() ;
                std::cout << "The destination flange pose is " << std::endl;
                std::cout << destination_flange_pose.matrix() << std::endl;

                std::vector<double> dest_flange_pose_rpy {
                    destination_flange_pose.translation().x(),
                    destination_flange_pose.translation().y(),
                    BELTLINE_HEIGHT_OFFSET, // TODO: change this to table height
                    on_belt_pos[3],
                    on_belt_pos[4],
                    destination_flange_pose.rotation().eulerAngles(2,1,0)[0]
                };

                std::cout << "The destination flange pose is " << std::endl;
                for (auto elem : dest_flange_pose_rpy) {
                    std::cout << elem << std::endl;
                }
                pthread_mutex_init(&m_mutex_plc, NULL);
            // OPEN GRIPPER
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 0);
                pthread_mutex_unlock(&m_mutex_plc);

            // MOVE_TO_ABOVE_PICKUP:
                if(this->cartesianMoveJointServerCall(dest_flange_pose_rpy, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                };

                std::vector<double> dest_flange_pose_rpy_down{
                    destination_flange_pose.translation().x(),
                    destination_flange_pose.translation().y(),
                    BELTLINE_HEIGHT, // get down to the belt
                    on_belt_pos[3],
                    on_belt_pos[4],
                    destination_flange_pose.rotation().eulerAngles(2,1,0)[0]
                };
            // MOVE_TO_PICKUP:
                if(this->cartesianMoveLineServerCall(dest_flange_pose_rpy_down, SPEED_FRACTION * 0.5, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    std::terminate();
                };
            // CLOSE GRIPPER
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 1);
                pthread_mutex_unlock(&m_mutex_plc);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
            // MOVE_UP_FROM_BELTLINE:
                if(this->cartesianMoveJointServerCall(robot_pick_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                }
                if(this->cartesianMoveJointServerCall(home_pos, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                }
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::MOVE_TO_RIG1 : {
                //instruct the robot to move to rig1 put down position
                std::string template_name = new_goal->PCB_template_name;
                std::string wp_file = this->getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos_" + template_name + ".yml";
                YAML::Node wp_node = YAML::LoadFile(wp_file);
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };    
                auto yaml_joint_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> joint_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            // the xyz positions
                            joint_vals[i] = unparser[i].as<double>();
                        }
                        return joint_vals;
                    };  
                std::vector<double> home_pos = yaml_pos_value_grabber(wp_node, "common", "photo_home");            
                std::vector<double> robot_out = yaml_pos_value_grabber(wp_node, "common", "robot_out");
                std::vector<double> midway = yaml_joint_value_grabber(wp_node, "rig1", "midway");
                std::vector<double> front_of_rig = yaml_pos_value_grabber(wp_node, "rig1", "front_of_rig");
                std::vector<double> in_rig_up = yaml_pos_value_grabber(wp_node, "rig1", "in_rig_up");
                
                const double SPEED_FRACTION = 0.1;

                if (this->cartesianMoveJointServerCall(home_pos, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if (this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if (this->moveJointAngleServerCall(midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if (this->cartesianMoveJointServerCall(front_of_rig, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if (this->cartesianMoveLineServerCall(in_rig_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::MOVE_TO_RIG2 : {
                //instruct the robot to move to the home photo position
                std::string template_name = new_goal->PCB_template_name;
                std::string wp_file = this->getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos_" + template_name + ".yml";
                YAML::Node wp_node = YAML::LoadFile(wp_file);
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };    
                auto yaml_joint_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> joint_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                                // the xyz positions
                                joint_vals[i] = unparser[i].as<double>();
                        }
                        return joint_vals;
                    };   
                std::vector<double> home_pos = yaml_pos_value_grabber(wp_node, "common","photo_home");   
                std::vector<double> robot_out = yaml_pos_value_grabber(wp_node, "common", "robot_out");
                std::vector<double> midway = yaml_joint_value_grabber(wp_node, "rig2", "midway");
                std::vector<double> front_of_rig = yaml_pos_value_grabber(wp_node, "rig2", "front_of_rig");
                std::vector<double> in_rig_up = yaml_pos_value_grabber(wp_node, "rig2", "in_rig_up");         
                double SPEED_FRACTION = 0.1;
                if(this->cartesianMoveJointServerCall(home_pos, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if(this->moveJointAngleServerCall(midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if(this->cartesianMoveJointServerCall(front_of_rig, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if(this->cartesianMoveLineServerCall(in_rig_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::MOVE_TO_SUCCESS : {
                //instruct the robot to move to the home photo position
                std::string template_name = new_goal->PCB_template_name;
                std::string wp_file = this->getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos_" + template_name + ".yml";
                YAML::Node wp_node = YAML::LoadFile(wp_file);
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };    
                auto yaml_joint_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> joint_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                                // the xyz positions
                                joint_vals[i] = unparser[i].as<double>();
                        }
                        return joint_vals;
                    };    
                std::vector<double> home = yaml_pos_value_grabber(wp_node, "common", "photo_home");
                std::vector<double> robot_out = yaml_pos_value_grabber(wp_node, "common", "robot_out");
                std::vector<double> pass_position_up = yaml_pos_value_grabber(wp_node, "common", "pass_position_up"); 
                double SPEED_FRACTION = 0.1;
                std::cout << "Moving to home" << std::endl;
                if (this->cartesianMoveJointServerCall(home, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    m_as.setAborted();
                    break;
                }                
                std::cout << "Moving to robot out" << std::endl;
                if (this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    m_as.setAborted();
                    break;
                }
                std::cout << "Moving to pass" << std::endl;
                if (this->cartesianMoveJointServerCall(pass_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    m_as.setAborted();
                    break;
                }
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::MOVE_TO_FAILURE: {
                //instruct the robot to move to the home photo position
                std::string template_name = new_goal->PCB_template_name;
                std::cout << "Name of the PCB is " << template_name << std::endl;
                std::string wp_file = this->getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos_" + template_name + ".yml";
                YAML::Node wp_node = YAML::LoadFile(wp_file);
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };    
                auto yaml_joint_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double> {
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> joint_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                                // the xyz positions
                                joint_vals[i] = unparser[i].as<double>();
                        }
                        return joint_vals;
                    };    
                std::vector<double> home = yaml_pos_value_grabber(wp_node, "common", "photo_home");
                std::vector<double> robot_out = yaml_pos_value_grabber(wp_node, "common", "robot_out");
                std::vector<double> fail_position_up = yaml_pos_value_grabber(wp_node, "common", "fail_position_up"); 

                double SPEED_FRACTION = 0.1;
                if (this->cartesianMoveJointServerCall(home, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    m_as.setAborted();
                    break;
                }                
                if (this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    m_as.setAborted();
                    break;
                }
                if (this->cartesianMoveJointServerCall(fail_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    m_as.setAborted();
                    break;
                }
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::MOVE_FROM_RIG1_TO_HOME: {
                std::string template_name = new_goal->PCB_template_name;
                std::string wp_file = this->getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos_" + template_name + ".yml";
                YAML::Node wp_node = YAML::LoadFile(wp_file);
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };    
                auto yaml_joint_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> joint_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            // the xyz positions
                            joint_vals[i] = unparser[i].as<double>();
                        }
                        return joint_vals;
                    };  
                std::vector<double> home_pos = yaml_pos_value_grabber(wp_node, "common", "photo_home");            
                std::vector<double> robot_out = yaml_pos_value_grabber(wp_node, "common", "robot_out");
                std::vector<double> midway = yaml_joint_value_grabber(wp_node, "rig1", "midway");
                std::vector<double> front_of_rig = yaml_pos_value_grabber(wp_node, "rig1", "front_of_rig");
                std::vector<double> in_rig_up = yaml_pos_value_grabber(wp_node, "rig1", "in_rig_up");
                
                const double SPEED_FRACTION = 0.1;

                if (this->cartesianMoveLineServerCall(in_rig_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if (this->cartesianMoveLineServerCall(front_of_rig, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };     
                if (this->moveJointAngleServerCall(midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if (this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if (this->cartesianMoveJointServerCall(home_pos, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };

                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::MOVE_FROM_RIG2_TO_HOME: {
                std::string template_name = new_goal->PCB_template_name;
                std::string wp_file = this->getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos_" + template_name + ".yml";
                YAML::Node wp_node = YAML::LoadFile(wp_file);
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };    
                auto yaml_joint_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> joint_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                                // the xyz positions
                                joint_vals[i] = unparser[i].as<double>();
                        }
                        return joint_vals;
                    };   
                std::vector<double> home_pos = yaml_pos_value_grabber(wp_node, "common","photo_home");   
                std::vector<double> robot_out = yaml_pos_value_grabber(wp_node, "common", "robot_out");
                std::vector<double> midway = yaml_joint_value_grabber(wp_node, "rig2", "midway");
                std::vector<double> front_of_rig = yaml_pos_value_grabber(wp_node, "rig2", "front_of_rig");
                std::vector<double> in_rig_up = yaml_pos_value_grabber(wp_node, "rig2", "in_rig_up");         
                double SPEED_FRACTION = 0.1;
                if(this->cartesianMoveLineServerCall(in_rig_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if(this->cartesianMoveLineServerCall(front_of_rig, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };
                if(this->moveJointAngleServerCall(midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };    
                if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };  
                if(this->cartesianMoveJointServerCall(home_pos, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    break;
                };                                      
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::URGENT_TEST: {
                this->urgentTest(new_goal->PCB_template_name);
                this->m_as.setSucceeded(this->m_result);
                break;
            }         
            case ManagerTasks::GENERATE_CLOSEUP_TEMPALTE: {
                //instruct the robot to move to the home photo position
                std::string wp_file = this->getSparkDir() + "/robot/waypoints/pcba2021/pcb_pos.yml";
                YAML::Node wp_node = YAML::LoadFile(wp_file);
                auto yaml_pos_value_grabber = 
                    [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
                        YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                        std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
                        for(int i = 0; i < unparser.size(); i++) {
                            if(i < 3) {
                                // the xyz positions
                                pos_vals[i] = unparser[i].as<double>();
                            } else {
                                // convert the degree angle to radian
                                pos_vals[i] = spark_math::deg2rad(
                                    unparser[i].as<double>()
                                );
                            }
                        }
                        return pos_vals;
                    };    
                std::vector<double> home_pos = yaml_pos_value_grabber(wp_node, "common","photo_home");         
                std::vector<double> on_belt_pos {
                    yaml_pos_value_grabber(wp_node, "common", "belt_height")
                };
                std::vector<double> robot_pick_up {
                    yaml_pos_value_grabber(wp_node, "common", "robot_pickup")
                };
                const double BELTLINE_HEIGHT = on_belt_pos[2];
                const double BELTLINE_HEIGHT_OFFSET = BELTLINE_HEIGHT + 0.03;

                double SPEED_FRACTION = 0.1;
                if(this->cartesianMoveJointServerCall(home_pos, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                };

                std::string template_name = new_goal->PCB_template_name;

                // perform recognition and perform grasping
                std::string tcp_yaml_path = this->getSparkDir() + "/robot/gripper/pcba2021/tcp.yml";
                std::string gripper_yaml_path = getSparkDir() + "/robot/gripper/pcba2021/" + template_name + ".yml"; 
                if (!boost::filesystem::exists(gripper_yaml_path)) {
                    ROS_INFO("Template does not exist");
                    this->m_as.setAborted();
                }
                auto tcp_reader = YAML::LoadFile(tcp_yaml_path);
                auto gripper_reader = YAML::LoadFile(gripper_yaml_path);
                std::map<std::string, double> tcp, gripper_offset;
                tcp["x"] = tcp_reader["x"].as<double>();
                tcp["y"] = tcp_reader["y"].as<double>();
                tcp["z"] = tcp_reader["z"].as<double>();
                tcp["rx"] = tcp_reader["rx"].as<double>();
                tcp["ry"] = tcp_reader["ry"].as<double>();
                tcp["rz"] = tcp_reader["rz"].as<double>();
                gripper_offset["offsetx"] = gripper_reader["offsetx"].as<double>();
                gripper_offset["offsety"] = gripper_reader["offsety"].as<double>();
                gripper_offset["offsetz"] = gripper_reader["offsetz"].as<double>();
                spark_backend::VisionServiceGoal vision_goal;
                vision_goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_3D_OBJECT);
                vision_goal.PCB_template_name = template_name;
                this->m_vision_ac_ptr->sendGoal(vision_goal);
                this->m_vision_ac_ptr->waitForResult();
                auto result = this->m_vision_ac_ptr->getResult();
                int track_attempt_count{1};
                unsigned MAXIMUM_TRACK_ATTEMPT = 50;
                while(result->number_of_tracked_object == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    vision_goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_3D_OBJECT);
                    vision_goal.PCB_template_name = template_name;
                    this->m_vision_ac_ptr->sendGoal(vision_goal);
                    this->m_vision_ac_ptr->waitForResult();
                    result = this->m_vision_ac_ptr->getResult();  
                    track_attempt_count++;
                    if (track_attempt_count > MAXIMUM_TRACK_ATTEMPT) {
                        LOG_ERROR("Unable to track stable object after 50 attempts.");
                        return;
                    }
                    std::cout << "Number of tracked objects: " << result->number_of_tracked_object << std::endl;                  
                }
                Eigen::Isometry3f obj_pose = Eigen::Isometry3f::Identity();

                double rz = 0;
                double prev_x = result->PCB_center[0].x;
                double prev_y = result->PCB_center[0].y;
                double prev_rot = result->PCB_rotation[0].z;
                double current_x{0}, current_y{0}, current_rot{0};
                bool PCB_stationary = false;
                
                while((result->number_of_tracked_object == 0) || (PCB_stationary == false)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    // try to track with new images
                    this->m_vision_ac_ptr->sendGoal(vision_goal);
                    this->m_vision_ac_ptr->waitForResult();
                    track_attempt_count++;
                    // TODO: signal warning if the track attempt is over some maximum attempts
                    if (track_attempt_count > MAXIMUM_TRACK_ATTEMPT) {
                        LOG_ERROR("Unable to track stable object after 50 attempts.");
                        return;
                    };
                    result = this->m_vision_ac_ptr->getResult(); //program will be blocked here waiting for result
                    while (result->number_of_tracked_object == 0) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        track_attempt_count++;
                        this->m_vision_ac_ptr->sendGoal(vision_goal);
                        this->m_vision_ac_ptr->waitForResult();
                        result = this->m_vision_ac_ptr->getResult();
                    }
                    current_x = result->PCB_center[0].x;
                    current_y = result->PCB_center[0].y;
                    current_rot = result->PCB_rotation[0].z;
                    double abs_dis_diff = std::sqrt(
                        std::pow(current_x - prev_x, 2.0) + std::pow(current_y - prev_y, 2.0)
                    );
                    double abs_rot_diff = std::fabs(current_rot - prev_rot);
                    prev_x = current_x;
                    prev_y = current_y;
                    prev_rot = current_rot;
                    if((abs_dis_diff < 0.002) && ((abs_rot_diff < 2) || (abs_rot_diff > 358))) PCB_stationary = true;
                }

                rz = result->PCB_rotation[0].z - 90.0;

                std::vector<double> joint_values, current_pos;
                this->getAuboMsg(joint_values, current_pos, this->m_robot_ac_ptr);
                
                Eigen::Isometry3f current_flange_pos = Eigen::Isometry3f::Identity();
                current_flange_pos.pretranslate(Eigen::Vector3f(current_pos[0], current_pos[1], current_pos[2]));
                current_flange_pos.rotate(Eigen::Quaternionf(current_pos[3], current_pos[4], current_pos[5], current_pos[6]));
                
                Eigen::Isometry3f flange_to_tool = Eigen::Isometry3f::Identity();
                flange_to_tool.pretranslate(Eigen::Vector3f(tcp["x"], tcp["y"], tcp["z"]));
                flange_to_tool.rotate(spark_math::eulerToQuaternion(Eigen::Vector3f(tcp["rx"], tcp["ry"], tcp["rz"])));

                Eigen::Isometry3f current_tool_pos = Eigen::Isometry3f::Identity();
                current_tool_pos = current_flange_pos * flange_to_tool;

                printf("The current tool pose is:\n");
                std::cout << current_tool_pos.matrix() << std::endl;

                Eigen::Vector3f dest_tool_rotation;
                dest_tool_rotation[0] = on_belt_pos[3];
                dest_tool_rotation[1] = on_belt_pos[4];
                dest_tool_rotation[2] = current_tool_pos.rotation().eulerAngles(2, 1, 0)[0] + spark_math::deg2rad(rz);
                Eigen::Isometry3f destination_tool_pose = Eigen::Isometry3f::Identity();
                Eigen::Vector3f tool_translation (result->PCB_center[0].x, result->PCB_center[0].y, result->PCB_center[0].z); 
                destination_tool_pose.pretranslate(tool_translation);  // TODO: set to the PCB board pose
                destination_tool_pose.rotate(spark_math::eulerToQuaternion(dest_tool_rotation));

                std::cout << "The destination tool pose is " << std::endl;
                std::cout << destination_tool_pose.matrix() << std::endl;

                destination_tool_pose.translate(Eigen::Vector3f(gripper_offset["offsetx"], gripper_offset["offsety"], gripper_offset["offsetz"]));
                
                Eigen::Isometry3f destination_flange_pose = (destination_tool_pose)* flange_to_tool.inverse() ;
                std::cout << "The destination flange pose is " << std::endl;
                std::cout << destination_flange_pose.matrix() << std::endl;

            
                std::vector<double> dest_flange_pose_rpy {
                    destination_flange_pose.translation().x(),
                    destination_flange_pose.translation().y(),
                    BELTLINE_HEIGHT_OFFSET, // TODO: change this to table height
                    on_belt_pos[3],
                    on_belt_pos[4],
                    destination_flange_pose.rotation().eulerAngles(2,1,0)[0],
                };

                std::cout << "The destination flange pose is " << std::endl;
                for (auto elem : dest_flange_pose_rpy) {
                    std::cout << elem << std::endl;
                }
            // MOVE_TO_ABOVE_PICKUP:
                if(this->cartesianMoveJointServerCall(dest_flange_pose_rpy, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                };

                std::vector<double> dest_flange_pose_rpy_down{
                    destination_flange_pose.translation().x(),
                    destination_flange_pose.translation().y(),
                    BELTLINE_HEIGHT, // get down to the belt
                    on_belt_pos[3],
                    on_belt_pos[4],
                    destination_flange_pose.rotation().eulerAngles(2,1,0)[0],
                };
            // MOVE_TO_PICKUP:
                if(this->cartesianMoveLineServerCall(dest_flange_pose_rpy_down, SPEED_FRACTION * 0.5, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    std::terminate();
                };
            // CLOSE_GRIPPER:
                while(this->getIO(IOType::DO, GRIPPER_DO, m_robot_ac_ptr) == 0) {
                    this->setIO(GRIPPER_DO, true, m_robot_ac_ptr);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            // MOVE_UP_FROM_BELTLINE:
                if(this->cartesianMoveJointServerCall(robot_pick_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }
                if(this->cartesianMoveJointServerCall(home_pos, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                spark_backend::VisionServiceGoal goal;
                goal.PCB_template_name = new_goal->PCB_template_name + "_closeup";
                goal.vision_task_type = static_cast<int>(VisionTasks::GENERATE_3D_TEMPLATE);
                this->m_vision_ac_ptr->sendGoal(goal);
                this->m_vision_ac_ptr->waitForResult();
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::RESET_DO : {
                std::cout << "Setting gripper DO to false" << std::endl;
                while(this->getIO(IOType::DO, GRIPPER_DO, m_robot_ac_ptr) == 1) {
                    this->setIO(GRIPPER_DO, false, m_robot_ac_ptr);
                }
                std::cout << "Setting rig1 move up DO to false" << std::endl;
                while(this->getIO(IOType::DO, RIG1_UP_DO, m_robot_ac_ptr) == 1) {
                    this->setIO(RIG1_UP_DO, false, m_robot_ac_ptr);
                }
                std::cout << "Setting rig1 move down DO to false" << std::endl;
                while(this->getIO(IOType::DO, RIG1_DOWN_DO, m_robot_ac_ptr) == 1) {
                    this->setIO(RIG1_DOWN_DO, false, m_robot_ac_ptr);
                }
                std::cout << "Setting rig2 move up DO to false" << std::endl;
                while(this->getIO(IOType::DO, RIG2_UP_DO, m_robot_ac_ptr) == 1) {
                    this->setIO(RIG2_UP_DO, false, m_robot_ac_ptr);
                }
                std::cout << "Setting rig1 move down DO to false" << std::endl;
                while(this->getIO(IOType::DO, RIG2_DOWN_DO, m_robot_ac_ptr) == 1) {
                    this->setIO(RIG2_DOWN_DO, false, m_robot_ac_ptr);
                }
                break;
            }
            case ManagerTasks::TOGGLE_GRIPPER :{
                pthread_mutex_init(&m_mutex_plc, NULL);
                if (m_omron_plc_ptr->getIO(102, 6) == 0) {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(102, 6, 1);
                    pthread_mutex_unlock(&m_mutex_plc);
                } else {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(102, 6, 0);
                    pthread_mutex_unlock(&m_mutex_plc);
                }
                break;
            }
            case ManagerTasks::RIG1_UP_TOGGLE: {
                pthread_mutex_init(&m_mutex_plc, NULL);
                if (m_omron_plc_ptr->getIO(101, 0) == 0) {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(101, 0, 1);
                    pthread_mutex_unlock(&m_mutex_plc);
                } else {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(101, 0, 0);
                    pthread_mutex_unlock(&m_mutex_plc);
                }
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case ManagerTasks::RIG2_UP_TOGGLE: {
                pthread_mutex_init(&m_mutex_plc, NULL);
                if (m_omron_plc_ptr->getIO(101, 3) == 0) {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(101, 3, 1);
                    pthread_mutex_unlock(&m_mutex_plc);
                } else {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(101, 3, 0);
                    pthread_mutex_unlock(&m_mutex_plc);
                }
                this->m_as.setSucceeded(this->m_result);
                break;                
            }
            case ManagerTasks::RIG1_DOWN_TOGGLE: {
                pthread_mutex_init(&m_mutex_plc, NULL);
                if (m_omron_plc_ptr->getIO(101, 2) == 0) {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(101, 2, 1);
                    pthread_mutex_unlock(&m_mutex_plc);
                } else {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(101, 2, 0);
                    pthread_mutex_unlock(&m_mutex_plc);
                }
                this->m_as.setSucceeded(this->m_result);
                break;
            }    
            case ManagerTasks::RIG2_DOWN_TOGGLE: {
                pthread_mutex_init(&m_mutex_plc, NULL);
                if (m_omron_plc_ptr->getIO(101, 4) == 0) {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(101, 4, 1);
                    pthread_mutex_unlock(&m_mutex_plc);
                } else {
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDO(101, 4, 0);
                    pthread_mutex_unlock(&m_mutex_plc);
                }
                this->m_as.setSucceeded(this->m_result);
                break;
            }            
            case ManagerTasks::DEFAULT : {
                std::cout << "The task type number is : " << new_goal->manager_task_type << std::endl;
                ROS_INFO("Not sure how the program gets here, check the task type of sent in goal.");
                break;
            }

        }
    }
    return;
}

void manager::workflowMideaPCBA2021(std::string template_name, double speed_ratio) {
    // read in the YAML file containing the speed ratio information
    std::string home = getSparkDir();
    // FIXME: These boolean values should be 
    this->rig1_empty = true;
    this->rig2_empty = true;
    this->rig1_in_queue_for_pickup = false;
    this->rig1_in_queue_for_work = false;
    this->rig2_in_queue_for_pickup = false;
    this->rig2_in_queue_for_work = false;

    using namespace spark_math;
    /**
     * @brief clear all works in queues before starting the job, 
     * in case of memory problems
     * 
     */
    for (int i = 0; i < work_queue.work_queue.size(); i++) {
        work_queue.work_queue.pop();
    }

    /**
     * @brief Load in the 2d parameters
     * 
     */
    printf("Load in 2d parameter.\n");

    std::string template_dir = this->getSparkDir() + "/vision/track_template/" + template_name + "_closeup/";
    std::string template_parameters_path = template_dir + "/track_param.yml"; 

    YAML::Node template_reader = YAML::LoadFile(template_parameters_path);
    std::vector<double> closeup_center(2);
    closeup_center[0] = template_reader["x"].as<double>();
    closeup_center[1] = template_reader["y"].as<double>();
    double closeup_phi = template_reader["phi"].as<double>();
    double closeup_length1 = template_reader["l1"].as<double>();
    double closeup_length2 = template_reader["l2"].as<double>();

    const double ALLOWABLE_CLOSEUP_OFFSET{100}; // the close up uv offset over which error will be thrown 

    auto trackCloseup = [this, template_name](std::promise<std::vector<double>>& result_flag) {
        spark_backend::VisionServiceGoal goal;
        goal.PCB_template_name = template_name + "_closeup";
        goal.vision_task_type = (int)VisionTasks::TRACK_2D_OBJECT;
        m_vision_ac_ptr->sendGoal(goal);
        m_vision_ac_ptr->waitForResult();
        auto result = m_vision_ac_ptr->getResult();
        std::cout << "The track center is (" << std::setprecision(5) << result->uv_center.x << ", " << result->uv_center.y << ")" << std::endl;
        std::vector<double> uv_result{result->uv_center.x, result->uv_center.y};
        result_flag.set_value(uv_result);
    };

    spark_backend::VisionServiceGoal vision_goal;
    //load in tcp and sucker_angle
    std::map<std::string, double> tcp;
    std::map<std::string, double> gripper_offset;
    YAML::Node tcp_reader;
    YAML::Node gripper_reader;

    //create the gripper based on the template name
    std::string tcp_yaml_path = home + "/robot/gripper/pcba2021/tcp_exec.yml";
    std::string gripper_yaml_path = getSparkDir() + "/robot/gripper/pcba2021/" + template_name + ".yml"; 
    tcp_reader = YAML::LoadFile(tcp_yaml_path);
    gripper_reader = YAML::LoadFile(gripper_yaml_path);
    tcp["x"] = tcp_reader["x"].as<double>();
    tcp["y"] = tcp_reader["y"].as<double>();
    tcp["z"] = tcp_reader["z"].as<double>();
    tcp["rx"] = tcp_reader["rx"].as<double>();
    tcp["ry"] = tcp_reader["ry"].as<double>();
    tcp["rz"] = tcp_reader["rz"].as<double>();
    gripper_offset["offsetx"] = gripper_reader["offsetx"].as<double>();
    gripper_offset["offsety"] = gripper_reader["offsety"].as<double>();
    gripper_offset["offsetz"] = gripper_reader["offsetz"].as<double>();

    YAML::Node pos_reader;
    std::string pos_yaml_path = home + "/robot/waypoints/pcba2021/pcb_pos_" + template_name + ".yml";
    pos_reader = YAML::LoadFile(pos_yaml_path);

    auto yaml_pos_value_grabber = 
        [](YAML::Node& node, std::string group, std::string position)->std::vector<double>{
            YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
            std::vector<double> pos_vals(6); // 6 - x, y, z, rx, ry, rz
            for(int i = 0; i < unparser.size(); i++) {
                if (i < 3) {
                    // the xyz positions
                    pos_vals[i] = unparser[i].as<double>();
                } else {
                    // convert the degree angle to radian
                    pos_vals[i] = spark_math::deg2rad(
                        unparser[i].as<double>()
                    );
                }
            }
            return pos_vals;
        };
    
    auto yaml_joint_value_grabber = 
            [](YAML::Node& node, std::string group, std::string position)->std::vector<double> {
                YAML::Node unparser = YAML::Load(node[group][position]["values"].as<std::string>());
                std::vector<double> joint_vals(6); // 6 - x, y, z, rx, ry, rz
                for(int i = 0; i < unparser.size(); i++) {
                    joint_vals[i] = unparser[i].as<double>();
                }
                return joint_vals;
            };

    // position to acquire image
    std::vector<double> photo_home {
        yaml_pos_value_grabber(pos_reader, "common", "photo_home")
    };
    // the robot moves out of the beltline
    std::vector<double> robot_out {
        yaml_pos_value_grabber(pos_reader, "common", "robot_out")
    };
    // position to lift up board from belt
    std::vector<double> robot_pick_up {
        yaml_pos_value_grabber(pos_reader, "common", "robot_pickup")
    };
    std::vector<double> on_belt_pos {
        yaml_pos_value_grabber(pos_reader, "common", "belt_height")
    };
    // positions to put pass/fail boards
    std::vector<double> pass_position_up {
        yaml_pos_value_grabber(pos_reader, "common", "pass_position_up")
    };
    std::vector<double> pass_position {
        yaml_pos_value_grabber(pos_reader, "common", "pass_position")
    };
    std::vector<double> fail_position_up {
        yaml_pos_value_grabber(pos_reader, "common", "fail_position_up")
    };
    std::vector<double> fail_position {
        yaml_pos_value_grabber(pos_reader, "common", "fail_position")
    };

    // waypoints in rig 1
    std::vector<double> RIG1_midway{
        yaml_joint_value_grabber(pos_reader, "rig1", "midway")
    };
    std::vector<double> front_of_RIG1{
        yaml_pos_value_grabber(pos_reader, "rig1", "front_of_rig")
    };
    std::vector<double> in_RIG1_up{
        yaml_pos_value_grabber(pos_reader, "rig1", "in_rig_up")
    };
    std::vector<double> in_RIG1_down{
        yaml_pos_value_grabber(pos_reader, "rig1", "in_rig_down")
    };

    // waypoints in rig 2
    std::vector<double> RIG2_midway{
        yaml_joint_value_grabber(pos_reader, "rig2", "midway")  
    };
    std::vector<double> front_of_RIG2{
        yaml_pos_value_grabber(pos_reader, "rig2", "front_of_rig")
    };
    std::vector<double> in_RIG2_up{
        yaml_pos_value_grabber(pos_reader, "rig2", "in_rig_up")
    };
    std::vector<double> in_RIG2_down{
        yaml_pos_value_grabber(pos_reader, "rig2", "in_rig_down")
    };
    const double BELTLINE_HEIGHT = on_belt_pos[2];
    const double BELTLINE_HEIGHT_OFFSET = BELTLINE_HEIGHT + 0.03;

    // set the speed to a low value when initializing to home position
    double SPEED_FRACTION = speed_ratio;


    if(this->cartesianMoveJointServerCall(photo_home, SPEED_FRACTION, m_robot_ac_ptr) == false) {
        this->m_as.setAborted();
        return;
    } 

    std::thread workflow_monitor_thread(&manager::auboMonitorCallback, this);
    pthread_mutex_init(&m_mutex_plc, NULL);
    u_int MAXIMUM_TRACK_ATTEMPT = 200;
    
    //open gripper
    pthread_mutex_lock(&m_mutex_plc);
    m_omron_plc_ptr->setDO(102, 6, 0);
    pthread_mutex_unlock(&m_mutex_plc);

    while(ros::ok()) { // this is a termination point, if this node is to be killed, then node will terminate
        // The YAML file needs to be reloaded in case some parameters changed
        pos_reader = YAML::LoadFile(pos_yaml_path);
        GET_NEW_TASK:
        switch (this->getTopQueueWork()) {
            case WorkType::FILL_RIG_ONE : {
                ROS_INFO("Filling rig one");
            // MOVE_TO_HOME:
                if(this->cartesianMoveJointServerCall(photo_home, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); //wiat for image to stabilize
            // PERFORM_PCB_TRACKING:
                printf("Sending signal to track 3d object on %s.\n", template_name.c_str());
                vision_goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_3D_OBJECT);
                vision_goal.PCB_template_name = template_name;
                this->m_vision_ac_ptr->sendGoal(vision_goal);
                this->m_vision_ac_ptr->waitForResult();
                auto result = this->m_vision_ac_ptr->getResult();
                int track_attempt_count{1};

                while(result->number_of_tracked_object == 0) { // with the above code, this could be a do-while loop to save some code
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    vision_goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_3D_OBJECT);
                    vision_goal.PCB_template_name = template_name;
                    this->m_vision_ac_ptr->sendGoal(vision_goal);
                    this->m_vision_ac_ptr->waitForResult();
                    result = this->m_vision_ac_ptr->getResult();  
                    std::cout << "Number of tracked objects: " << result->number_of_tracked_object << std::endl;     
                    track_attempt_count++;
                    if (track_attempt_count > MAXIMUM_TRACK_ATTEMPT) {
                        LOG_ERROR("Unable to find stable track after 100 attempts");
                        m_nh.setParam(BACKEND_WARNING, "无法追踪，退出程序");
                        this->deleteTopQueueWork();
                        while (this->board_arrive_on_beltline.load(std::memory_order_seq_cst) == true) {
                            this->board_arrive_on_beltline.store(false);
                        }
                        while(this->rig1_empty.load(std::memory_order_seq_cst) == false) {
                            this->rig1_empty.store(true);
                        }
                        while(this->rig1_in_queue_for_work.load(std::memory_order_seq_cst) == true) {
                            this->rig1_in_queue_for_work.store(false);
                        }
                        while(this->rig1_in_queue_for_pickup.load(std::memory_order_seq_cst) == true) {
                            this->rig1_in_queue_for_pickup.store(false);
                        }
                        goto GET_NEW_TASK;
                    }    
                }
                Eigen::Isometry3f obj_pose = Eigen::Isometry3f::Identity();

                double rz = 0;
                
                double prev_x = result->PCB_center[0].x;
                double prev_y = result->PCB_center[0].y;
                double prev_rot = result->PCB_rotation[0].z;
                double current_x{0}, current_y{0}, current_rot{0};
                bool PCB_stationary = false;

                while ((result->number_of_tracked_object == 0) || (PCB_stationary == false)) { // continue tracking if board is not stationary
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    // try to track with new images
                    this->m_vision_ac_ptr->sendGoal(vision_goal);
                    this->m_vision_ac_ptr->waitForResult();
                    track_attempt_count++;
                    if (track_attempt_count > MAXIMUM_TRACK_ATTEMPT) {
                        // TODO: 
                        LOG_ERROR("Unable to track stable object after 100 attempts.");
                        m_nh.setParam(BACKEND_WARNING, "无法追踪，退出程序");
                        return;
                    }; // 50 is the number of attempts
                    result = this->m_vision_ac_ptr->getResult();
                    while (result->number_of_tracked_object == 0) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        track_attempt_count++;
                        this->m_vision_ac_ptr->sendGoal(vision_goal);
                        this->m_vision_ac_ptr->waitForResult();
                        result = this->m_vision_ac_ptr->getResult();
                        if (track_attempt_count > MAXIMUM_TRACK_ATTEMPT) {
                            LOG_ERROR("Unable to track stable object after %d attempts.", MAXIMUM_TRACK_ATTEMPT);
                            m_nh.setParam(BACKEND_WARNING, "无法追踪，退出程序");
                            this->deleteTopQueueWork();
                            while (this->board_arrive_on_beltline.load(std::memory_order_seq_cst) == true) {
                                this->board_arrive_on_beltline.store(false);
                            }
                            while(this->rig1_empty.load(std::memory_order_seq_cst) == false) {
                                this->rig1_empty.store(true);
                            }
                            while(this->rig1_in_queue_for_work.load(std::memory_order_seq_cst) == true) {
                                this->rig1_in_queue_for_work.store(false);
                            }
                            while(this->rig1_in_queue_for_pickup.load(std::memory_order_seq_cst) == true) {
                                this->rig1_in_queue_for_pickup.store(false);
                            }
                            goto GET_NEW_TASK;
                        }
                    }
                    current_x = result->PCB_center[0].x;
                    current_y = result->PCB_center[0].y;
                    current_rot = result->PCB_rotation[0].z;
                    double abs_dis_diff = std::sqrt(
                        std::pow(current_x - prev_x, 2.0) + std::pow(current_y - prev_y, 2.0)
                    );
                    double abs_rot_diff = std::fabs(current_rot - prev_rot);
                    prev_x = current_x;
                    prev_y = current_y;
                    prev_rot = current_rot;
                    if((abs_dis_diff < 0.002) && ((abs_rot_diff < 2) || (abs_rot_diff > 358))) PCB_stationary = true;
                }

                // TODO: check rotation within limit
                rz = result->PCB_rotation[0].z - 90.0; // stream orientation is perpendicular to track orientation

                std::vector<double> joint_values, current_pos;
                this->getAuboMsg(joint_values, current_pos, this->m_robot_ac_ptr);
                
                Eigen::Isometry3f current_flange_pos = Eigen::Isometry3f::Identity();
                current_flange_pos.pretranslate(Eigen::Vector3f(current_pos[0], current_pos[1], current_pos[2]));
                current_flange_pos.rotate(Eigen::Quaternionf(current_pos[3], current_pos[4], current_pos[5], current_pos[6]));
                
                Eigen::Isometry3f flange_to_tool = Eigen::Isometry3f::Identity();
                flange_to_tool.pretranslate(Eigen::Vector3f(tcp["x"], tcp["y"], tcp["z"]));
                flange_to_tool.rotate(spark_math::eulerToQuaternion(Eigen::Vector3f(tcp["rx"], tcp["ry"], tcp["rz"])));

                Eigen::Isometry3f current_tool_pos = Eigen::Isometry3f::Identity();
                current_tool_pos = current_flange_pos * flange_to_tool;

                printf("The current tool pose is:\n");
                std::cout << current_tool_pos.matrix() << std::endl;

                Eigen::Vector3f dest_tool_rotation;
                dest_tool_rotation[0] = on_belt_pos[3];
                dest_tool_rotation[1] = on_belt_pos[4];
                dest_tool_rotation[2] = current_tool_pos.rotation().eulerAngles(2, 1, 0)[0] + spark_math::deg2rad(rz);
                Eigen::Isometry3f destination_tool_pose = Eigen::Isometry3f::Identity();
                Eigen::Vector3f tool_translation (result->PCB_center[0].x, result->PCB_center[0].y, result->PCB_center[0].z); 
                destination_tool_pose.pretranslate(tool_translation);  // TODO: set to the PCB board pose
                destination_tool_pose.rotate(spark_math::eulerToQuaternion(dest_tool_rotation));

                std::cout << "The destination tool pose is " << std::endl;
                std::cout << destination_tool_pose.matrix() << std::endl;

                destination_tool_pose.translate(Eigen::Vector3f(gripper_offset["offsetx"], gripper_offset["offsety"], gripper_offset["offsetz"]));
                
                Eigen::Isometry3f destination_flange_pose = (destination_tool_pose)* flange_to_tool.inverse() ;
                std::cout << "The destination flange pose is " << std::endl;
                std::cout << destination_flange_pose.matrix() << std::endl;

                std::vector<double> dest_flange_pose_rpy {
                    destination_flange_pose.translation().x(),
                    destination_flange_pose.translation().y(),
                    BELTLINE_HEIGHT_OFFSET, // TODO: change this to table height
                    on_belt_pos[3],
                    on_belt_pos[4],
                    destination_flange_pose.rotation().eulerAngles(2,1,0)[0]
                };
                std::cout << "The destination flange pose is " << std::endl;
                for (auto elem : dest_flange_pose_rpy) {
                    std::cout << elem << std::endl;
                }
                std::vector<double> dest_flange_pose_rpy_down{
                    destination_flange_pose.translation().x(),
                    destination_flange_pose.translation().y(),
                    BELTLINE_HEIGHT, // get down to the belt
                    on_belt_pos[3], 
                    on_belt_pos[4], 
                    destination_flange_pose.rotation().eulerAngles(2,1,0)[0]
                };
            // MOVE_TO_PICKUP:
                if(this->cartesianMoveLineServerCall(dest_flange_pose_rpy_down, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
            // CLOSE_GRIPPER:
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 1);
                pthread_mutex_unlock(&m_mutex_plc);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // MOVE_UP_FROM_BELTLINE:
                if(this->cartesianMoveJointServerCall(robot_pick_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
            // PERFORM CLOSE UP CHECK HERE
                std::promise<std::vector<double>> track_promise;
                auto track_future = track_promise.get_future();
                std::thread performCloseupTrack(trackCloseup, std::ref(track_promise)); // start the tread and spin off

                while(this->board_picked.load(std::memory_order_seq_cst) == false) {
                    this->board_picked.store(true);
                }
                
                while(true){
                    pthread_mutex_lock(&m_mutex_plc);
                    if (m_omron_plc_ptr->getDM(3000) == 3000) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                };
                while (this->board_picked.load() == false) {
                    this->board_picked.store(true);
                }
                // BEGIN: MAKE SURE RIG AT UPPER LIMIT
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 0, 1);
                pthread_mutex_unlock(&m_mutex_plc);
                
                while (true) {
                    pthread_mutex_lock(&m_mutex_plc);
                    if (m_omron_plc_ptr->getIO(1, 0) == 1) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                };

                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 0, 0);
                pthread_mutex_unlock(&m_mutex_plc);
                // END: MAKE SURE RIG AT UPPER LIMIT
                
                // MOVE_INTO_RIG:
                if(this->moveJointAngleServerCall(RIG1_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
                if(this->cartesianMoveJointServerCall(front_of_RIG1, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
                if(this->cartesianMoveLineServerCall(in_RIG1_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
            // CHECK THE RETURNED TRACK RESULT
                std::vector<double> track_result = track_future.get();
                if (performCloseupTrack.joinable()) {
                    performCloseupTrack.join();
                }
                // if (fabs(track_result[0] - closeup_center[1]) > ALLOWABLE_CLOSEUP_OFFSET || fabs(track_result[1] - closeup_center[0]) > ALLOWABLE_CLOSEUP_OFFSET) {
                //     LOG_ERROR("The board is off-centered on gripper. Will try capturing closeup match for another 10 times.");

                //     // TODO： try tracking for another 10 times
                //     for(int i = 0; i < 10; i++) {
                //         std::promise<std::vector<double>> track_promise;
                //         auto track_future = track_promise.get_future();
                //         std::thread performCloseupTrack(trackCloseup, std::ref(track_promise));
                //         std::vector<double> track_result = track_future.get();
                //         if (performCloseupTrack.joinable()) {
                //             performCloseupTrack.join();
                //         }
                //         if (fabs(track_result[0] - closeup_center[1]) > ALLOWABLE_CLOSEUP_OFFSET || fabs(track_result[1] - closeup_center[0]) > ALLOWABLE_CLOSEUP_OFFSET) {
                //             LOG_ERROR("The board is still off-centered on gripper on retry run #%d", i);
                //         }
                //     }
                //     LOG_ERROR("The board is off-centered. Will abort program now for safety.");
                //     std::abort();
                // }
                LOG_WARNING("The track result is u: %.4f & v: %.4f", track_result[0], track_result[1]); 
            // MOVE DOWN TO THE RIG IF RESULT IS OK
                std::vector<double> in_RIG1_down_alittle_up = in_RIG1_down;
                in_RIG1_down_alittle_up[2] = in_RIG1_down_alittle_up[2] + 0.008;
                if(this->cartesianMoveLineServerCall(in_RIG1_down_alittle_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
            // OPEN_GRIPPER:
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 0);
                pthread_mutex_unlock(&m_mutex_plc);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // MOVE_OUT:
                if(this->cartesianMoveLineServerCall(in_RIG1_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
                if(this->cartesianMoveLineServerCall(front_of_RIG1, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
            // PRESS_DOWN_RIG1, JUST MOVE AWAY AFTER PRESS DOWN THE RIG:
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 2, 1);
                pthread_mutex_unlock(&m_mutex_plc);
            // CLOSE_GRIPPER:
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 1);
                pthread_mutex_unlock(&m_mutex_plc);
            // MOVE_TO_MIDWAY_AND_CLEANUP
                if(this->moveJointAngleServerCall(RIG1_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
                if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }
            // CHECK IF RIG HAS PRESSED DOWN, IF SO, RELEASE PRESS DOWN SIGNAL
                while(true) {
                    pthread_mutex_lock(&m_mutex_plc);
                    if (m_omron_plc_ptr->getIO(1, 1) == 1) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                }
                
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 2, 0);
                pthread_mutex_unlock(&m_mutex_plc);

            // OPEN_GRIPPER:
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 0);
                pthread_mutex_unlock(&m_mutex_plc);
                
            // RESET_FILL_RIG1_MSG:
                this->deleteTopQueueWork();
                while (this->rig1_empty.load(std::memory_order_seq_cst) == true) {
                    this->rig1_empty.store(false);
                }
                while (this->rig1_in_queue_for_work.load(std::memory_order_seq_cst) == true) {
                    this->rig1_in_queue_for_work.store(false);
                }
                while (this->board_arrive_on_beltline.load(std::memory_order_seq_cst) == true) {
                    this->board_arrive_on_beltline.store(false);
                }
                break;
            }
            case WorkType::FILL_RIG_TWO : {
                ROS_INFO("Filling rig two");
            // PHOTO HOME:
                if(this->cartesianMoveJointServerCall(photo_home, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
            // PERFORM PCB TRACKING:
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                vision_goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_3D_OBJECT);
                vision_goal.PCB_template_name = template_name;
                this->m_vision_ac_ptr->sendGoal(vision_goal);
                this->m_vision_ac_ptr->waitForResult();
                auto result = this->m_vision_ac_ptr->getResult();
                int track_attempt_count{1};
                while(result->number_of_tracked_object == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    vision_goal.vision_task_type = static_cast<int>(VisionTasks::TRACK_3D_OBJECT);
                    vision_goal.PCB_template_name = template_name;
                    this->m_vision_ac_ptr->sendGoal(vision_goal);
                    this->m_vision_ac_ptr->waitForResult();
                    result = this->m_vision_ac_ptr->getResult();  
                    std::cout << "Number of tracked objects: " << result->number_of_tracked_object << std::endl;
                    track_attempt_count++;                  
                    if (track_attempt_count > MAXIMUM_TRACK_ATTEMPT) {
                        LOG_ERROR("Unable to track stable object after %d attempts.", MAXIMUM_TRACK_ATTEMPT);
                        m_nh.setParam(BACKEND_WARNING, "无法追踪，退出程序");
                        // return;
                        this->deleteTopQueueWork();
                            while (this->board_arrive_on_beltline.load(std::memory_order_seq_cst) == true) {
                                this->board_arrive_on_beltline.store(false);
                            }
                            while(this->rig2_empty.load(std::memory_order_seq_cst) == false) {
                                this->rig2_empty.store(true);
                            }
                            while(this->rig2_in_queue_for_work.load(std::memory_order_seq_cst) == true) {
                                this->rig2_in_queue_for_work.store(false);
                            }
                            while(this->rig2_in_queue_for_pickup.load(std::memory_order_seq_cst) == true) {
                                this->rig2_in_queue_for_pickup.store(false);
                            }
                        goto GET_NEW_TASK;
                    }
                }

                double rz = 0;
                
                double prev_x = result->PCB_center[0].x;
                double prev_y = result->PCB_center[0].y;
                double prev_rot = result->PCB_rotation[0].z;
                double current_x{0}, current_y{0}, current_rot{0};
                bool PCB_stationary = false;

                while((result->number_of_tracked_object == 0) || (PCB_stationary == false)) {
                    // try to track with new images
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    this->m_vision_ac_ptr->sendGoal(vision_goal);
                    this->m_vision_ac_ptr->waitForResult();
                    track_attempt_count++;
                    // TODO: signal warning if the track attempt is over some maximum attempts
                    if (track_attempt_count > MAXIMUM_TRACK_ATTEMPT) {
                        LOG_ERROR("Unable to track stable object after 1000 attempts.");
                        m_nh.setParam(BACKEND_WARNING, "无法追踪，退出程序");
                        return;
                    }; // 50 is the number of attempts
                    result = this->m_vision_ac_ptr->getResult();
                    while (result->number_of_tracked_object == 0) { 
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        track_attempt_count++;
                        this->m_vision_ac_ptr->sendGoal(vision_goal);
                        this->m_vision_ac_ptr->waitForResult();
                        result = this->m_vision_ac_ptr->getResult();
                        if(track_attempt_count > MAXIMUM_TRACK_ATTEMPT) {
                            LOG_ERROR("Unable to track stable object after 1000 attempts.");
                            m_nh.setParam(BACKEND_WARNING, "无法追踪，退出程序");
                            this->deleteTopQueueWork();
                            while (this->board_arrive_on_beltline.load(std::memory_order_seq_cst) == true) {
                                this->board_arrive_on_beltline.store(false);
                            }
                            while(this->rig2_empty.load(std::memory_order_seq_cst) == false) {
                                this->rig2_empty.store(true);
                            }
                            while(this->rig2_in_queue_for_work.load(std::memory_order_seq_cst) == true) {
                                this->rig2_in_queue_for_work.store(false);
                            }
                            while(this->rig2_in_queue_for_pickup.load(std::memory_order_seq_cst) == true) {
                                this->rig2_in_queue_for_pickup.store(false);
                            } 
                            goto GET_NEW_TASK;
                        }
                    }
                    current_x = result->PCB_center[0].x;
                    current_y = result->PCB_center[0].y;
                    current_rot = result->PCB_rotation[0].z;
                    double abs_dis_diff = std::sqrt(
                        std::pow(current_x - prev_x, 2.0) + std::pow(current_y - prev_y, 2.0)
                    );
                    double abs_rot_diff = std::fabs(current_rot - prev_rot);
                    prev_x = current_x;
                    prev_y = current_y;
                    prev_rot = current_rot;
                    if((abs_dis_diff < 0.002) && ((abs_rot_diff < 2) || (abs_rot_diff > 358))) {
                        PCB_stationary = true;
                    } else {
                        std::cout << "The absolute distance difference is " << abs_dis_diff << std::endl;
                        std::cout << "The absolute rotation difference is " << abs_rot_diff << std::endl;                        
                    }
                }

                if (result->number_of_tracked_object == 1) {
                    rz = result->PCB_rotation[0].z - 90;
                } else {
                    LOG_ERROR("Track failed");
                    return;
                }

                std::cout << "target x is : " << result->PCB_center[0].x << std::endl;
                std::cout << "target y is : " << result->PCB_center[0].y << std::endl;
                std::cout << "target rz is : " << rz << std::endl;

                std::vector<double> joint_values, current_pos;
                this->getAuboMsg(joint_values, current_pos, this->m_robot_ac_ptr);
                
                Eigen::Isometry3f current_flange_pos = Eigen::Isometry3f::Identity();
                current_flange_pos.pretranslate(Eigen::Vector3f(current_pos[0], current_pos[1], current_pos[2]));
                current_flange_pos.rotate(Eigen::Quaternionf(current_pos[3], current_pos[4], current_pos[5], current_pos[6]));
                
                Eigen::Isometry3f flange_to_tool = Eigen::Isometry3f::Identity();
                flange_to_tool.pretranslate(Eigen::Vector3f(tcp["x"], tcp["y"], tcp["z"]));
                flange_to_tool.rotate(spark_math::eulerToQuaternion(Eigen::Vector3f(tcp["rx"], tcp["ry"], tcp["rz"])));

                Eigen::Isometry3f current_tool_pos = Eigen::Isometry3f::Identity();
                current_tool_pos = current_flange_pos * flange_to_tool;

                printf("The current tool pose is:\n");
                std::cout << current_tool_pos.matrix() << std::endl;

                Eigen::Vector3f dest_tool_rotation;
                dest_tool_rotation[0] = on_belt_pos[3];
                dest_tool_rotation[1] = on_belt_pos[4];
                dest_tool_rotation[2] = current_tool_pos.rotation().eulerAngles(2, 1, 0)[0] + deg2rad(rz);
                Eigen::Isometry3f destination_tool_pose = Eigen::Isometry3f::Identity();
                Eigen::Vector3f tool_translation (result->PCB_center[0].x, result->PCB_center[0].y, result->PCB_center[0].z);
                destination_tool_pose.pretranslate(tool_translation);  // TODO: set to the PCB board pose
                destination_tool_pose.rotate(eulerToQuaternion(dest_tool_rotation));

                destination_tool_pose.translate(Eigen::Vector3f(gripper_offset["offsetx"], gripper_offset["offsety"],  gripper_offset["offsetz"]));
                
                Eigen::Isometry3f destination_flange_pose = (destination_tool_pose)* flange_to_tool.inverse() ;
                std::cout << "The destination flange pose is " << std::endl;
                std::cout << destination_flange_pose.matrix() << std::endl;

                std::vector<double> dest_flange_pose_rpy {
                    destination_flange_pose.translation().x(),
                    destination_flange_pose.translation().y(),
                    BELTLINE_HEIGHT_OFFSET,
                    on_belt_pos[3],
                    on_belt_pos[4],
                    destination_flange_pose.rotation().eulerAngles(2,1,0)[0]
                };

                std::cout << "The destination flange pose is " << std::endl;
                for (auto elem : dest_flange_pose_rpy) {
                    std::cout << elem << std::endl;
                }

                std::vector<double> dest_flange_pose_rpy_down {
                    destination_flange_pose.translation().x(),
                    destination_flange_pose.translation().y(),
                    BELTLINE_HEIGHT,
                    on_belt_pos[3],
                    on_belt_pos[4],
                    destination_flange_pose.rotation().eulerAngles(2,1,0)[0]
                };
            // MOVE DOWN TO PICKUP ON CONVEYER BELT
                if(this->cartesianMoveLineServerCall(dest_flange_pose_rpy_down, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
            // CLOSE GRIPPER
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 1);
                pthread_mutex_unlock(&m_mutex_plc);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // PICK UP THE BOARD
                if(this->cartesianMoveJointServerCall(robot_pick_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
                
                while(true) {
                    pthread_mutex_lock(&m_mutex_plc);
                    if (m_omron_plc_ptr->getDM(3000) == 3000) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                };
                
                while (this->board_picked.load() == false) {
                    this->board_picked.store(true);
                }

            // START THREAD TO CHECK PCB CLOSEUP ALIGNMENT
                std::promise<std::vector<double>> track_promise;
                auto track_future = track_promise.get_future();
                std::thread performCloseupTrack(trackCloseup, std::ref(track_promise));

            
            // MAKE SURE RIG2 IS LIFTED
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 3, 1);
                pthread_mutex_unlock(&m_mutex_plc);

                while(true) {
                    pthread_mutex_lock(&m_mutex_plc);
                    if (m_omron_plc_ptr->getIO(1, 6) == 1) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                }

                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 3, 0);
                pthread_mutex_unlock(&m_mutex_plc);
            // MOVE INTO RIG2
                if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };

                if(this->moveJointAngleServerCall(RIG2_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };

                if(this->cartesianMoveJointServerCall(front_of_RIG2, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };

                if(this->cartesianMoveLineServerCall(in_RIG2_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };

            // CHECK RESULT TO SEE IF BOARD ALIGNMENT IS OFF
                std::vector<double> track_result = track_future.get();
                if (performCloseupTrack.joinable()) {
                    performCloseupTrack.join();
                }
                // if (fabs(track_result[0] - closeup_center[1]) > ALLOWABLE_CLOSEUP_OFFSET || fabs(track_result[1] - closeup_center[0]) > ALLOWABLE_CLOSEUP_OFFSET) {
                //     LOG_ERROR("The board is off-centered on gripper. Will try capturing closeup match for another 10 times.");

                //     // TODO： try tracking for another 10 times
                //     for(int i = 0; i < 10; i++) {
                //         std::promise<std::vector<double>> track_promise;
                //         auto track_future = track_promise.get_future();
                //         std::thread performCloseupTrack(trackCloseup, std::ref(track_promise));
                //         std::vector<double> track_result = track_future.get();
                //         if (performCloseupTrack.joinable()) {
                //             performCloseupTrack.join();
                //         }
                //         if (fabs(track_result[0] - closeup_center[1]) > ALLOWABLE_CLOSEUP_OFFSET || fabs(track_result[1] - closeup_center[0]) > ALLOWABLE_CLOSEUP_OFFSET) {
                //             LOG_ERROR("The board is still off-centered on gripper on retry run #%d", i);
                //         }
                //     }
                //     LOG_ERROR("The board is off-centered. Will abort program now for safety.");
                //     std::abort();
                // }
                LOG_DEBUG("The track result is u: %.4f & v: %.4f", track_result[0], track_result[1]); 

                std::vector<double> in_RIG2_down_alittle_up = in_RIG2_down;
                in_RIG2_down_alittle_up[2] = in_RIG2_down_alittle_up[2] + 0.008; // 0.008 is the emperical distance to drop the board 

                if(this->cartesianMoveLineServerCall(in_RIG2_down_alittle_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };

            // OPEN GRIPPER
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 0);
                pthread_mutex_unlock(&m_mutex_plc);
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // sleep to make sure the gripper has opened properly
            // MOVE OUT
                if(this->cartesianMoveLineServerCall(in_RIG2_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
                // TODO: need to carefully consider the termination strategy here before pressing down the rig
                if(this->cartesianMoveLineServerCall(front_of_RIG2, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
            // PRESS DOWN RIG
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 4, 1);
                pthread_mutex_unlock(&m_mutex_plc);
            // CLOSE_GRIPPER:
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 1);
                pthread_mutex_unlock(&m_mutex_plc);
            // MOVE OUT TO WAITING POSITION
                if(this->moveJointAngleServerCall(RIG2_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
                if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                };
                while(true) {
                    pthread_mutex_lock(&m_mutex_plc);
                    if (m_omron_plc_ptr->getIO(1, 7) == 1) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                }

                while(true) {
                    pthread_mutex_lock(&m_mutex_plc);
                    if (m_omron_plc_ptr->getIO(1, 7) == 1) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                }

                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 4, 0);
                pthread_mutex_unlock(&m_mutex_plc);

            // OPEN_GRIPPER:
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 0);
                pthread_mutex_unlock(&m_mutex_plc);

            // DELETE WORK QUEUE AND RESET WORK
                this->deleteTopQueueWork();
                while (this->board_arrive_on_beltline.load(std::memory_order_seq_cst) == true) {
                    this->board_arrive_on_beltline.store(false);
                }
                while (this->rig2_in_queue_for_work.load(std::memory_order_seq_cst) == true) {
                    this->rig2_in_queue_for_work.store(false);
                }
                while (this->rig2_empty.load(std::memory_order_seq_cst) == true) {
                    this->rig2_empty.store(false);
                }
                break;
            }
            case WorkType::EMPTY_RIG_ONE : {
                ROS_INFO("Emptying rig one");
                
                while(true) {
                    pthread_mutex_lock(&m_mutex_plc);
                    if (m_omron_plc_ptr->getDM(3000) == 3000) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                };

            // CLOSE_GRIPPER:
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 1);
                pthread_mutex_unlock(&m_mutex_plc);
                
            // ENSURE RIG1 IS LIFTED  
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 2, 0);
                pthread_mutex_unlock(&m_mutex_plc); 
                
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 0, 1);
                pthread_mutex_unlock(&m_mutex_plc);
                
                while (true) {
                    pthread_mutex_lock(&m_mutex_plc);
                    if (m_omron_plc_ptr->getIO(1, 0) == 1) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                }
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 0, 0);
                pthread_mutex_unlock(&m_mutex_plc);

            // MOVE INTO RIG 1
                if(this->moveJointAngleServerCall(RIG1_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                } 
                if(this->cartesianMoveJointServerCall(front_of_RIG1, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }                
            // OPEN GRIPPER
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 0);
                pthread_mutex_unlock(&m_mutex_plc); 

                if(this->cartesianMoveLineServerCall(in_RIG1_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                } 
                if(this->cartesianMoveLineServerCall(in_RIG1_down, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                } 
            // CLOSE GRIPPER
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 1);
                pthread_mutex_unlock(&m_mutex_plc);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // MOVE OUT OF RIG 1
                if(this->cartesianMoveLineServerCall(in_RIG1_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                } 
                if(this->cartesianMoveLineServerCall(front_of_RIG1, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }                 
                if(this->moveJointAngleServerCall(RIG1_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }  

                this->deleteTopQueueWork();
                while(this->rig1_empty.load(std::memory_order_seq_cst) == false) {
                    this->rig1_empty.store(true);
                }
                while(this->rig1_in_queue_for_work.load(std::memory_order_seq_cst) == true) {
                    this->rig1_in_queue_for_work.store(false);
                }
                while(this->rig1_in_queue_for_pickup.load(std::memory_order_seq_cst) == true) {
                    this->rig1_in_queue_for_pickup.store(false);
                }

                /**
                 * @brief MOTIONS FOR PLACING PASS AND FAIL
                 * 
                 */
                if (this->rig1_result == PCBTestResult::PASS) {
                // CHECK IF THE BOARD FLIPPER IS RESET TO START POSITION
                // WAIT UNTIL THE BOARD IS RESET BEFORE GETTING TO THE PASS POSITION
                    while(true) {
                        pthread_mutex_lock(&m_mutex_plc);
                        if (m_omron_plc_ptr->getDM(3000) == 3000) {
                            pthread_mutex_unlock(&m_mutex_plc);
                            break;
                        }
                        pthread_mutex_unlock(&m_mutex_plc);
                    };
                // MOVE TO PASS POSITION
                    if(this->cartesianMoveJointServerCall(pass_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }  
                    if(this->cartesianMoveLineServerCall(pass_position, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }                
                // OPEN GRIPPER      
                    pthread_mutex_lock(&m_mutex_plc);
                    m_omron_plc_ptr->setDO(102, 6, 0);
                    pthread_mutex_unlock(&m_mutex_plc);   
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // MOVE AWAY
                    if(this->cartesianMoveJointServerCall(pass_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }  
                    if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }

                // SEND SIGNAL TO TRIGGER BOARD FLIPPER
                pthread_mutex_lock(&m_mutex_plc);
                this->m_omron_plc_ptr->setDM(3000, 0);
                pthread_mutex_unlock(&m_mutex_plc);

                } else if (this->rig1_result == PCBTestResult::FAIL) {
                // MOVE TO FAIL POSITION
                    if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }                      
                    if(this->cartesianMoveLineServerCall(fail_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }  
                    if(this->cartesianMoveLineServerCall(fail_position, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }  
                // OPEN GRIPPER
                    pthread_mutex_lock(&m_mutex_plc);
                    m_omron_plc_ptr->setDO(102, 6, 0);
                    pthread_mutex_unlock(&m_mutex_plc);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // MOVE AWAY
                    if(this->cartesianMoveLineServerCall(fail_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }  
                    if(this->moveJointAngleServerCall(RIG2_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }                      

                    if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }        
                }
                break;
            }
            case WorkType::EMPTY_RIG_TWO : {
                ROS_INFO("Emptying rig two");
            // MAKE SURE RIG2 IS LIFTED
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 4, 0);
                pthread_mutex_unlock(&m_mutex_plc);

                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 3, 1);
                pthread_mutex_unlock(&m_mutex_plc);

                while (true) {
                    pthread_mutex_lock(&m_mutex_plc);
                    if(m_omron_plc_ptr->getIO(1, 6) == 1) {
                        pthread_mutex_unlock(&m_mutex_plc);
                        break;
                    }
                    pthread_mutex_unlock(&m_mutex_plc);
                }

                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(101, 3, 0);
                pthread_mutex_unlock(&m_mutex_plc);

            // CLOSE GRIPPER
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 1);
                pthread_mutex_unlock(&m_mutex_plc);

            // MOVE INTO RIG
                if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }  
                if(this->moveJointAngleServerCall(RIG2_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }       
                if(this->cartesianMoveJointServerCall(front_of_RIG2, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }
                //OPEN GRIPPER
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 0);
                pthread_mutex_unlock(&m_mutex_plc);
                if(this->cartesianMoveLineServerCall(in_RIG2_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }   
                if(this->cartesianMoveLineServerCall(in_RIG2_down, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }   
            // CLOSE GRIPPER
                pthread_mutex_lock(&m_mutex_plc);
                m_omron_plc_ptr->setDO(102, 6, 1);
                pthread_mutex_unlock(&m_mutex_plc);                
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // a short delay to ensure that the gripper has closed properly
            // MOVE OUT OF RIG
                if(this->cartesianMoveLineServerCall(in_RIG2_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }                   
                if(this->cartesianMoveLineServerCall(front_of_RIG2, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }   
                // FIXME: Delete top work queue here is fine
                this->deleteTopQueueWork();
                while(this->rig2_empty.load(std::memory_order_seq_cst) == false) {
                    this->rig2_empty.store(true);
                }
                while(this->rig2_in_queue_for_work.load(std::memory_order_seq_cst) == true) {
                    this->rig2_in_queue_for_work.store(false);
                }
                while(this->rig2_in_queue_for_pickup.load(std::memory_order_seq_cst) == true) {
                    this->rig2_in_queue_for_pickup.store(false);
                }
                if(this->moveJointAngleServerCall(RIG2_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                    this->m_as.setAborted();
                    return;
                }  
                /**
                 * @brief MOTIONS FOR PLACING PASS AND FAIL
                 * 
                 */
                if (this->rig2_result == PCBTestResult::PASS) {
                // BLOCK WAIT UNTIL THE FLIPPER SIGNAL IS SET TO 3000
                    while(true) {
                        pthread_mutex_lock(&m_mutex_plc);
                        if (this->m_omron_plc_ptr->getDM(3000) == 3000) {
                            pthread_mutex_unlock(&m_mutex_plc);
                            break;
                        }
                        pthread_mutex_unlock(&m_mutex_plc);
                    };
                    
                // MOVE TO PASS POSITION
                    if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }  
                    if(this->cartesianMoveJointServerCall(pass_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }  
                    if(this->cartesianMoveJointServerCall(pass_position, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }  
                // OPEN GRIPPER 
                    pthread_mutex_lock(&m_mutex_plc);
                    m_omron_plc_ptr->setDO(102, 6, 0);
                    pthread_mutex_unlock(&m_mutex_plc);  
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // MOVE TO MIDDLE POSITION
                    if(this->cartesianMoveLineServerCall(pass_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }
                    if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }
                // TRIGGER THE BOARD FLIPPER
                    pthread_mutex_lock(&m_mutex_plc);
                    this->m_omron_plc_ptr->setDM(3000, 0);
                    pthread_mutex_unlock(&m_mutex_plc);
                } else if (this->rig2_result == PCBTestResult::FAIL) {
                // MOVE TO FAIL POSITION
                    if(this->cartesianMoveJointServerCall(fail_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }
                    if(this->cartesianMoveJointServerCall(fail_position, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }
                    pthread_mutex_lock(&m_mutex_plc);
                    m_omron_plc_ptr->setDO(102, 6, 0);
                    pthread_mutex_unlock(&m_mutex_plc);  
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // MOVE TO MIDDLE POSITION
                    if(this->cartesianMoveLineServerCall(fail_position_up, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }
                    if(this->moveJointAngleServerCall(RIG2_midway, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }
                    if(this->cartesianMoveJointServerCall(robot_out, SPEED_FRACTION, m_robot_ac_ptr) == false) {
                        this->m_as.setAborted();
                        return;
                    }  
                }
                break;
            }
            case WorkType::NO_WORK : {
                // prevent too rapid scanning
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                break;
            }
            default: {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                break;
            }   
        }        
        bool terminate;
        m_nh.getParam(TERMINATE_WORKFLOW, terminate);
        if(terminate == true) {
            break;
        }
    
    };
    // terminate the thread at the exit of the PCB workflow
    // pthread_cancel(workflow_monitor_thread.native_handle()); - this will terminate the program with an exception
    if(workflow_monitor_thread.joinable()) {
        workflow_monitor_thread.join();
    }
}

void manager::fillWorkQueue(WorkType work_type) {
    // Monitors the pins here and fill the work queue
    std::lock_guard<std::mutex> lk (this->work_queue.mtx);
    this->work_queue.work_queue.emplace(work_type);
}

void manager::monitorWorkStatus() {
    // ros::Subscriber sub = this->m_nh.subscribe<spark_backend::AuboInfo>("aubo_io_info", 10, &manager::auboMonitorCallback, this);
    // ros::Rate loop_rate(200);
    // while(ros::ok()) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    //     bool terminate;
    //     m_nh.getParam(TERMINATE_WORKFLOW, terminate);
    //     if(terminate == true) {
    //         break;
    //     }
    // };
}

void manager::auboMonitorCallback() {
    // The updating of the work_queue data structure can happen here
    // check roscore for use rig1 and use rig2
    bool use_rig1{false}, use_rig2{false};
    m_nh.getParam(USE_RIG1, use_rig1);
    m_nh.getParam(USE_RIG2, use_rig2);

    while (true) {
        pthread_mutex_lock(&m_mutex_plc);
    // the logic of pin decision occurs here
        if((m_omron_plc_ptr->getIO(1, 0) == 1) &&
            rig1_empty.load(std::memory_order_seq_cst) && 
            !rig1_in_queue_for_work.load(std::memory_order_seq_cst) &&
            board_arrive_on_beltline.load(std::memory_order_seq_cst) &&
            use_rig1) {
                std::cout << "Filling work queue with FILL_RIG_ONE job" << std::endl;
                this->fillWorkQueue(manager::WorkType::FILL_RIG_ONE);
                std::cout << "The work queue now has " << this->work_queue.work_queue.size() << std::endl;
                while(rig1_in_queue_for_work.load(std::memory_order_seq_cst) == false) {
                    rig1_in_queue_for_work.store(true);
                }
                while(rig1_empty.load(std::memory_order_seq_cst) == true) {
                    rig1_empty.store(false);
                }
                while(board_arrive_on_beltline.load(std::memory_order_seq_cst) == true) {
                    board_arrive_on_beltline.store(false);
                }
        } 
        if((m_omron_plc_ptr->getIO(1, 6) == 1) &&
            rig2_empty.load(std::memory_order_seq_cst) && 
            !rig2_in_queue_for_work.load(std::memory_order_seq_cst) &&
            board_arrive_on_beltline.load(std::memory_order_seq_cst) &&
            use_rig2) {
                std::cout << "Filling work queue with FILL_RIG_TWO job" << std::endl;
                this->fillWorkQueue(manager::WorkType::FILL_RIG_TWO);
                std::cout << "The work queue now has " << this->work_queue.work_queue.size() << std::endl;
                while(rig2_in_queue_for_work.load(std::memory_order_seq_cst) == false) {
                    rig2_in_queue_for_work.store(true);
                }
                while(rig2_empty.load(std::memory_order_seq_cst) == true) {
                    rig2_empty.store(false);
                }
                while(board_arrive_on_beltline.load(std::memory_order_seq_cst) == true) {
                    board_arrive_on_beltline.store(false);
                }       
        }
        if(((m_omron_plc_ptr->getIO(1, 2) == 1) || (m_omron_plc_ptr->getIO(1, 3) == 1))&&
            !rig1_in_queue_for_pickup.load(std::memory_order_seq_cst) && 
            use_rig1) {
                std::cout << "Filling work queue with EMPTY_RIG_ONE job" << std::endl;
                this->fillWorkQueue(manager::WorkType::EMPTY_RIG_ONE);
                std::cout << "The work queue now has " << this->work_queue.work_queue.size() << std::endl;
                while(this->rig1_in_queue_for_pickup.load(std::memory_order_seq_cst) == false) {
                    rig1_in_queue_for_pickup.store(true);
                }
                std::cout << "rig1_in_queue_for_pickup: " << std::boolalpha << rig1_in_queue_for_pickup.load() << std::endl;
                if (m_omron_plc_ptr->getIO(1, 2) == 1) {
                this->rig1_result = PCBTestResult::PASS;
                } else if(m_omron_plc_ptr->getIO(1, 3) == 1) {
                    this->rig1_result = PCBTestResult::FAIL;
                } else {
                    this->rig1_result = PCBTestResult::FAIL;
                }        
        } 
        if(((m_omron_plc_ptr->getIO(1, 8) == 1) || (m_omron_plc_ptr->getIO(1, 9) == 1)) &&
            !rig2_in_queue_for_pickup.load(std::memory_order_seq_cst) && use_rig2) {
                std::cout << "Filling work queue with EMPTY_RIG_TWO job" << std::endl;
                this->fillWorkQueue(manager::WorkType::EMPTY_RIG_TWO);
                std::cout << "The work queue now has " << this->work_queue.work_queue.size() << std::endl;
                while(this->rig2_in_queue_for_pickup.load(std::memory_order_seq_cst) == false) {
                    this->rig2_in_queue_for_pickup.store(true);
                }
                // board_arrive_on_beltline = false;
                if (m_omron_plc_ptr->getIO(1, 8) == 1) {
                    this->rig2_result = PCBTestResult::PASS;
                } else if(m_omron_plc_ptr->getIO(1, 9) == 1) {
                    this->rig2_result = PCBTestResult::FAIL;
                } else {
                    this->rig2_result = PCBTestResult::FAIL;
                }
        } 

        if((m_omron_plc_ptr->getIO(0, 4) == 1) &&
            (rig2_empty.load(std::memory_order_seq_cst) || rig1_empty.load(std::memory_order_seq_cst))) {
            ROS_INFO("Detected board arrival");
            while(board_arrive_on_beltline.load(std::memory_order_seq_cst) == false) {
                board_arrive_on_beltline.store(true);
            }
            while(board_picked.load(std::memory_order_seq_cst) == true) {
                board_picked.store(false);
            }
        }
        // printf("------------------------variable status are------------------------\n");
        // std::cout << "rig1_empty: " << std::boolalpha << rig1_empty.load(std::memory_order_seq_cst) << std::endl;
        // std::cout << "rig1_in_queue_for_work: " << std::boolalpha << rig1_in_queue_for_work.load(std::memory_order_seq_cst) << std::endl;
        // std::cout << "rig1_in_queue_for_pickup: " << std::boolalpha << rig1_in_queue_for_pickup.load(std::memory_order_seq_cst) << std::endl;
        // std::cout << "rig2_empty: " << std::boolalpha << rig2_empty.load(std::memory_order_seq_cst) << std::endl;
        // std::cout << "rig2_in_queue_for_work: " << std::boolalpha << rig2_in_queue_for_work.load(std::memory_order_seq_cst) << std::endl;
        // std::cout << "rig2_in_queue_for_pickup: " << std::boolalpha << rig2_in_queue_for_pickup.load(std::memory_order_seq_cst) << std::endl;
        // std::cout << "board_arrive_on_beltline: " << std::boolalpha << board_arrive_on_beltline.load(std::memory_order_seq_cst) << std::endl;
        // std::cout << "board_picked: " << std::boolalpha << board_picked.load(std::memory_order_seq_cst) << std::endl;
        // std::cout << std::endl;
        pthread_mutex_unlock(&m_mutex_plc);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    //TODO: the wait to pick up pcb should not wait until either of the rigs is available
    // if((fabs(DI_status.find(BELTLINE_TRIGGER_DI)->second - 1.0) < 0.01) &&
    //     !board_arrive_on_beltline &&
    //     (PCB_beltline_counter < 2) &&
    //     board_picked
    //     ) {
    //     ROS_INFO("Updating PCB beltline counter.");
    //     std::cout << "Pin 0 is " << DI_status.find(BELTLINE_TRIGGER_DI)->second << std::endl;

    //     PCB_beltline_counter++;
    //     std::cout << "The current counter is " << PCB_beltline_counter << std::endl;
        
    // }
}

 manager::WorkType manager::getTopQueueWork() {
    std::lock_guard<std::mutex> lk(this->work_queue.mtx);
    if (this->work_queue.work_queue.size() != 0) {
        return this->work_queue.work_queue.front();
    } else {
        return WorkType::NO_WORK;
    }
}

void manager::deleteTopQueueWork() {
    std::lock_guard<std::mutex> lk(this->work_queue.mtx);
    std::cout << "Before pop, there are " << work_queue.work_queue.size() << " tasks" << std::endl;
    this->work_queue.work_queue.pop();
    std::cout << "After popping, there are " << work_queue.work_queue.size() << " tasks" << std::endl;
    return;
}

// TODO: check if the end point difference is within the allowable difference
bool manager::progressBlocker(actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr) {
    RobotState rs = RobotState::RobotRunning;
    spark_backend::RobotServiceGoal goal;
    std::string user_robot_status;
    while(true) {
        this->m_nh.getParam(ROS_USER_ROBOT_USAGE_STATUS, user_robot_status);
        if (ac_ptr->getState() != actionlib::SimpleClientGoalState::ACTIVE) {
            goal.move_type = static_cast<int>(MoveType::GET_ROBOT_STATE);
            ac_ptr->sendGoal(goal);
            ac_ptr->waitForResult();
            auto result_robot = ac_ptr->getResult();
            rs = static_cast<RobotState>(result_robot->robot_state);
            if (rs != RobotState::RobotRunning) {
                if (user_robot_status == ROS_PARAM_USER_STOPPED) {
                    return false;
                    break; // type a break here just to ensure that return will exit the loop
                } else if (user_robot_status == ROS_PARAM_USER_RUN) {
                    // return true to move on to the next movement statement in line 
                    // when the robot has stopped running and the user has not stopped
                    // the program, then continue with the rest of work flow 

                    // need to check the robot state again to ensure the last motion has finished execution
                    // FIXME: simply sleeping  may not be a good way of ensuring the goal is resumed
                    // FIXME: The question is, how long does it take for the robot to go from pause to resume??
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    ac_ptr->sendGoal(goal);
                    ac_ptr->waitForResult();
                    auto result_robot = ac_ptr->getResult();
                    rs = static_cast<RobotState>(result_robot->robot_state);
                    if (rs == RobotState::RobotRunning) {
                        continue;
                    } else {
                        return true;
                        break;
                    }
                } else if (user_robot_status == ROS_PARAM_USER_PAUSED) {
                    // if the robot is not running and the signal from user is paused, the waiting loop will continue
                    continue;
                } else if (user_robot_status == ROS_PARAM_USER_RESUME) {
                    continue;
                }
            };
        }
    }
    // when the 
};

bool manager::pcba2021StateInspection() {
    // makes sure the safety states are met before executng the pcba workflow
    while(this->getIO(IOType::DI, RIG1_UP_DI, m_robot_ac_ptr) == 0) {
        this->setIO(RIG1_UP_DO, true, m_robot_ac_ptr);
    }
    while(this->getIO(IOType::DO, RIG1_UP_DO, m_robot_ac_ptr) == 1) {
        this->setIO(RIG1_UP_DO, false, m_robot_ac_ptr);
    }
    while(this->getIO(IOType::DI, RIG2_UP_DI, m_robot_ac_ptr) == 0) {
        this->setIO(RIG2_UP_DO, true, m_robot_ac_ptr);
    }
    while(this->getIO(IOType::DO, RIG2_UP_DO, m_robot_ac_ptr) == 1) {
        this->setIO(RIG2_UP_DO, false, m_robot_ac_ptr);
    }
    return true;      
};

void manager::urgentTest(std::string template_name) {
    ROS_INFO("Urgent test to check 2D tracker");
    {
        std::promise<std::vector<double>> result_promise;
        auto future = result_promise.get_future();

        auto trackCloseup = [this, template_name](std::promise<std::vector<double>>& result_flag) {
            spark_backend::VisionServiceGoal goal;
            goal.PCB_template_name = template_name + "_closeup";
            goal.vision_task_type = int(VisionTasks::TRACK_2D_OBJECT);
            m_vision_ac_ptr->sendGoal(goal);
            m_vision_ac_ptr->waitForResult();
            auto result = m_vision_ac_ptr->getResult();
            std::cout << "The track center is (" << std::setprecision(5) << result->uv_center.x << ", " << result->uv_center.y << ")" << std::endl;
            std::vector<double> uv_result{result->uv_center.x, result->uv_center.y};
            result_flag.set_value(uv_result);
        };

        std::thread trackThread(trackCloseup, std::ref(result_promise));

        // auto trackObject = [this](std::promise<std::vector<double>>& result_promise, std::string template_name){
        //     spark_backend::VisionServiceGoal goal;
        //     goal.PCB_template_name = template_name + "_closeup";
        //     goal.vision_task_type = int(VisionTasks::TRACK_2D_OBJECT);
        //     m_vision_ac_ptr->sendGoal(goal);
        //     m_vision_ac_ptr->waitForResult();
        //     auto result = m_vision_ac_ptr->getResult();
        //     std::cout << "The track center is (" << std::setprecision(5) << result->uv_center.x << ", " << result->uv_center.y << std::endl;

        //     // load in the tracker parameters 
        //     std::string home_dir = this->getSparkDir();
        //     std::string template_dir = home_dir + "/vision/track_template/" + template_name + "_closeup/";

        //     std::string template_parameters_path = template_dir + "/track_param.yml"; 

        //     YAML::Node template_reader = YAML::LoadFile(template_parameters_path);
        //     std::vector<double> center(2);
        //     center[0] = template_reader["x"].as<double>();
        //     center[1] = template_reader["y"].as<double>();
        //     double phi = template_reader["phi"].as<double>();
        //     double length1 = template_reader["l1"].as<double>();
        //     double length2 = template_reader["l2"].as<double>();

        //     double u = result->uv_center.x;
        //     double v = result->uv_center.y;
        //     LOG_DEBUG("The track position is:");
        //     std::cout << "u: " << u << " v: " << v << std::endl;
        //     result_promise.set_value(std::vector<double>{u, v});
        // };
        // std::thread track2d(trackObject, std::ref(result_promise), template_name);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::vector<double> result = future.get();
        std::cout << "Finished tracking" << std::endl;
        printf("The track result is u=%.4f, v=%.4f.\n", result[0], result[1]);
        if (trackThread.joinable()) {
            trackThread.join();
        }
    }
    
    // track object with template name

}

bool manager::cartesianMoveJointServerCall(const std::vector<double> position, const double& speed_ratio,
                                           actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr) {
    spark_backend::RobotServiceGoal goal;
    goal.use_joint = false;
    goal.set_fraction = true;
    goal.move_rate = speed_ratio;
    goal.move_type = int(MoveType::MOVE_J);

    const double PDISTANCE_TOL {0.001};

    std::vector<double> joint_values, current_pos;
    this->getAuboMsg(joint_values, current_pos, ac_ptr);

    goal.current_joints[0] = joint_values[0];
    goal.current_joints[1] = joint_values[1];
    goal.current_joints[2] = joint_values[2];
    goal.current_joints[3] = joint_values[3];
    goal.current_joints[4] = joint_values[4];
    goal.current_joints[5] = joint_values[5];

    goal.move_joint_pos[0] = position[0];
    goal.move_joint_pos[1] = position[1];
    goal.move_joint_pos[2] = position[2];
    goal.move_joint_pos[3] = position[3];
    goal.move_joint_pos[4] = position[4];
    goal.move_joint_pos[5] = position[5];

    ac_ptr->sendGoal(goal);

    if(ac_ptr->waitForResult() == true) {
        ROS_INFO("Cartesian joint move sent successful.");
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool progress_status = this->progressBlocker(ac_ptr);
        // if false is returned by the progress blocker; it means that the user has stopped the robot, return false immediately
        if (progress_status == false) {
            return progress_status;
        }
        // get the current robot arm position and check for position tolerance
        goal.move_type = int(MoveType::GET_AUBO_MSG);
        this->m_robot_ac_ptr->sendGoal(goal);
        this->m_robot_ac_ptr->waitForResult();
        auto result = m_robot_ac_ptr->getResult();
        double position_tolerance = 
            std::sqrt(
                std::pow((position[0] - result->waypoint_values[0]), 2.0) + 
                std::pow((position[1] - result->waypoint_values[1]), 2.0) + 
                std::pow((position[2] - result->waypoint_values[2]), 2.0)
            );
        auto start_wait = std::chrono::steady_clock::now(); // the time to start waiting if position tolerance is greater than a certain value
        if (position_tolerance < PDISTANCE_TOL) {
            return progress_status;
        }

        while (position_tolerance > PDISTANCE_TOL) {
            this->m_robot_ac_ptr->sendGoal(goal);
            this->m_robot_ac_ptr->waitForResult();
            auto result = m_robot_ac_ptr->getResult();
            position_tolerance = 
                std::sqrt(
                    std::pow((position[0] - result->waypoint_values[0]), 2.0) + 
                    std::pow((position[1] - result->waypoint_values[1]), 2.0) + 
                    std::pow((position[2] - result->waypoint_values[2]), 2.0)
                );
            if (position_tolerance < PDISTANCE_TOL) {
                return progress_status;
            }
            auto cur_time = std::chrono::steady_clock::now(); 
            std::chrono::duration<double, std::milli> wait_time = cur_time - start_wait;
            if (wait_time.count() > 2000) { // if wait time is greater than 2000 ms before the end point converges
                ROS_ERROR("Robot end point motion does not converge after 2s wait.");
                std::terminate();
            }
        }
    } else {
        ROS_INFO("Cartesian joint move sent failed, not sure what happened.");
        return false;
    }
    return false;
    
};

bool manager::cartesianMoveLineServerCall(const std::vector<double> position, const double& speed_ratio,
                                          actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr) {
    spark_backend::RobotServiceGoal goal;
    goal.use_joint = false;
    goal.set_fraction = true;
    goal.move_rate = speed_ratio;
    goal.move_type = int(MoveType::MOVE_L);

    const double DISTANCE_TOL {0.001};

    std::vector<double> joint_values, current_pos;
    this->getAuboMsg(joint_values, current_pos, ac_ptr);

    goal.current_joints[0] = joint_values[0];
    goal.current_joints[1] = joint_values[1];
    goal.current_joints[2] = joint_values[2];
    goal.current_joints[3] = joint_values[3];
    goal.current_joints[4] = joint_values[4];
    goal.current_joints[5] = joint_values[5];

    goal.move_line_pos[0] = position[0];
    goal.move_line_pos[1] = position[1];
    goal.move_line_pos[2] = position[2];
    goal.move_line_pos[3] = position[3];
    goal.move_line_pos[4] = position[4];
    goal.move_line_pos[5] = position[5];

    ac_ptr->sendGoal(goal);

    if(ac_ptr->waitForResult() == true) {
        ROS_INFO("Cartesian joint move sent successfully, the robot should be executing the move.");
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool progress_status = this->progressBlocker(ac_ptr);
        if (progress_status == false) {
            return progress_status;
        }
        // get the current robot arm position and check for position tolerance
        goal.move_type = int(MoveType::GET_AUBO_MSG);
        this->m_robot_ac_ptr->sendGoal(goal);
        this->m_robot_ac_ptr->waitForResult();
        auto result = m_robot_ac_ptr->getResult();
        double position_tolerance = 
            std::sqrt(
                std::pow((position[0] - result->waypoint_values[0]), 2.0) + 
                std::pow((position[1] - result->waypoint_values[1]), 2.0) + 
                std::pow((position[2] - result->waypoint_values[2]), 2.0)
            );
        auto start_wait = std::chrono::steady_clock::now(); // the time to start waiting if position tolerance is greater than a certain value
        if (position_tolerance < DISTANCE_TOL) {
            std::cout << "position tolerance less than 0.01m in cartesian move line" << std::endl;
            return progress_status;
        }

        while (position_tolerance > DISTANCE_TOL) {
            this->m_robot_ac_ptr->sendGoal(goal);
            this->m_robot_ac_ptr->waitForResult();
            auto result = m_robot_ac_ptr->getResult();
            position_tolerance = 
                std::sqrt(
                    std::pow((position[0] - result->waypoint_values[0]), 2.0) + 
                    std::pow((position[1] - result->waypoint_values[1]), 2.0) + 
                    std::pow((position[2] - result->waypoint_values[2]), 2.0)
                );
            if (position_tolerance < DISTANCE_TOL) {
                return progress_status;
            }
            auto cur_time = std::chrono::steady_clock::now(); 
            std::chrono::duration<double, std::milli> wait_time = cur_time - start_wait;
            if (wait_time.count() > 2000) { // if wait time is greater than 2000 ms before the end point converges
                ROS_ERROR("Robot end point motion does not converge after 2s wait.");
                std::terminate();
            }
        }
    } else {
        ROS_INFO("Cartesian joint move snet failed, not sure what happened.");
        return false;
    }
    return false;
};

bool manager::moveJointAngleServerCall(const std::vector<double> joints, const double& speed_ratio, 
                                       actionlib::SimpleActionClient<spark_backend::RobotServiceAction>* ac_ptr) {
    spark_backend::RobotServiceGoal goal;
    goal.use_joint = true;
    goal.set_fraction = true;
    goal.move_rate = speed_ratio;
    goal.move_type = int(MoveType::MOVE_JOINT_ANGLE);


    goal.move_joint_pos[0] = joints[0];
    goal.move_joint_pos[1] = joints[1];
    goal.move_joint_pos[2] = joints[2];
    goal.move_joint_pos[3] = joints[3];
    goal.move_joint_pos[4] = joints[4];
    goal.move_joint_pos[5] = joints[5];

    ac_ptr->sendGoal(goal);

    if(ac_ptr->waitForResult() == true) {
        ROS_INFO("Joint angle move sent successfully.");
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool progress_status = this->progressBlocker(ac_ptr);
        if (progress_status == false) {
            return progress_status;
        }
        
        // get the aubo info and check for angle tolerance
        goal.move_type = int(MoveType::GET_AUBO_MSG);
        this->m_robot_ac_ptr->sendGoal(goal);
        this->m_robot_ac_ptr->waitForResult();
        auto result = m_robot_ac_ptr->getResult();
        double angle_tol = spark_math::deg2rad(6); // the angle tolerance for each joint, if not reached an error will be reported
        // check if all joints are within angle tolerances
        bool joint_consistency = fabs(result->joint_vals[0] - joints[0]) < angle_tol &&
                                 fabs(result->joint_vals[1] - joints[1]) < angle_tol &&
                                 fabs(result->joint_vals[2] - joints[2]) < angle_tol &&
                                 fabs(result->joint_vals[3] - joints[3]) < angle_tol &&
                                 fabs(result->joint_vals[4] - joints[4]) < angle_tol &&
                                 fabs(result->joint_vals[5] - joints[5]) < angle_tol;
        auto start = std::chrono::steady_clock::now();
        while(joint_consistency == false) {
            goal.move_type = int(MoveType::GET_AUBO_MSG);
            this->m_robot_ac_ptr->sendGoal(goal);
            this->m_robot_ac_ptr->waitForResult();
            auto result = m_robot_ac_ptr->getResult();
            joint_consistency = fabs(result->joint_vals[0] - joints[0]) < angle_tol &&
                                            fabs(result->joint_vals[1] - joints[1]) < angle_tol &&
                                            fabs(result->joint_vals[2] - joints[2]) < angle_tol &&
                                            fabs(result->joint_vals[3] - joints[3]) < angle_tol &&
                                            fabs(result->joint_vals[4] - joints[4]) < angle_tol &&
                                            fabs(result->joint_vals[5] - joints[5]) < angle_tol;
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> time_elps = end - start;
            if (time_elps.count() > 2000) {
                ROS_ERROR("Exceeds maximum joint angle converge wait time");
                exit(-1);
            }
        }
        if (joint_consistency) {
            return progress_status;
        } else {
            ROS_ERROR("Robot joints does not converge");
            exit(-1);
        }
    } else {
        ROS_ERROR("Cartesian joint move sent failed, not sure what happened.");
        exit(-1);
        return false;
    }
    return false;
};

int manager::getIO(const IOType io_type, const int pin_num,
                   actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr) {
    spark_backend::RobotServiceGoal goal;
    if (io_type == IOType::DI) {
        goal.move_type = static_cast<int>(MoveType::GET_DI);
        goal.io_type = static_cast<int>(IOType::DI);
        goal.pin_num = pin_num;
        ac_ptr->sendGoal(goal);
        ac_ptr->waitForResult();
        auto result = ac_ptr->getResult();
        return result->io_state;
    } else if (io_type == IOType::DO) {
        goal.move_type = static_cast<int>(MoveType::GET_DO);
        goal.io_type = static_cast<int>(IOType::DO);
        goal.pin_num = pin_num;
        ac_ptr->sendGoal(goal);
        ac_ptr->waitForResult();
        auto result = ac_ptr->getResult();
        return result->io_state;
    } else {
        LOG_ERROR("Invalid IO type specified.");
        return -1;
    }
};

bool manager::setIO(const int pin_num, const bool state,
                    actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr) {
    spark_backend::RobotServiceGoal goal;
    goal.move_type = static_cast<int>(MoveType::SET_DO);
    goal.pin_num = pin_num;
    goal.io_type = static_cast<int>(IOType::DO);
    goal.DO_value = (int)state;
    ac_ptr->sendGoal(goal);
    ac_ptr->waitForResult();
    return true; // TODO: the return value should base on something better than true
};


void manager::getAuboMsg(std::vector<double> &joint_vals, std::vector<double> &current_pos,
                         actionlib::SimpleActionClient<spark_backend::RobotServiceAction> * ac_ptr) {
    spark_backend::RobotServiceGoal goal;
    goal.move_type = static_cast<int>(MoveType::GET_AUBO_MSG);
    ac_ptr->sendGoal(goal);
    ac_ptr->waitForResult();
    auto result = ac_ptr->getResult();
    joint_vals.insert(
        joint_vals.end(),
        result->joint_vals.begin(),
        result->joint_vals.end()
    );
    current_pos.insert(
        current_pos.end(),
        result->waypoint_values.begin(),
        result->waypoint_values.end()
    );
}
