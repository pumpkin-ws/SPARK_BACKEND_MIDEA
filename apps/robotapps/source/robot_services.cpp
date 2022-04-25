#include "robot_services.hpp"

using rbs = spark_robot::AuboServer;

void rbs::goalCB() {
    if(this->m_as.isNewGoalAvailable()) {
        // ROS_INFO("New robot service goal is available!");
        auto new_goal = m_as.acceptNewGoal();
        this->m_move_type = static_cast<MoveType>(new_goal->move_type);
        std::vector<double> pos;
        std::vector<double> current_joints;
        std::vector<std::vector<double>> track_joint;
        std::vector<double> joint_values, current_pos;

        // TODO: This may not be a good way to check the validity of a enumerator
        if(this->m_move_type == MoveType::DEFAULT) {
            ROS_INFO("Invalid goal received!");
            m_as.setAborted();
        }
        // create a thread here to perform the actions
        switch(this->m_move_type) {
            case MoveType::PAUSE : {
                this->pause();
                break;
            }
            case MoveType::STOP : {
                this->stop();
                break;
            }
            case MoveType::RESUME : {
                this->resume();
                break;
            }
            case MoveType::MOVE_L : {
                // TODO: first check if the joint value is used, get the position values
                pos.clear();
                for (int i = 0; i < 6; i++) {
                    pos.push_back(new_goal->move_line_pos[i]);
                }       
                current_joints.clear();
                for (int i = 0; i < 6; i++) {
                    current_joints.push_back(new_goal->current_joints[i]);
                }             
                this->moveLine(pos, current_joints, new_goal->use_joint, new_goal->move_rate, new_goal->set_fraction);
                break;
            }
            // added by WS on July 2, 2021
            case MoveType::MOVE_JOINT_ANGLE : {
                std::vector<double> joints;
                for (int i = 0; i < 6; i++) {
                    joints.push_back(new_goal->move_joint_pos[i]);
                }
                this->moveJointAngle(joints, new_goal->move_rate, new_goal->set_fraction);

                break;
            }
            case MoveType::MOVE_J : {
                pos.clear();
                for (int i = 0; i < 6; i++) {
                    pos.push_back(new_goal->move_joint_pos[i]);
                }
                current_joints.clear();
                for (int i = 0; i < 6; i++) {
                    current_joints.push_back(new_goal->current_joints[i]);
                }
                this->moveJoint(pos, current_joints, new_goal->use_joint, new_goal->move_rate, new_goal->set_fraction);
                break;
            }
            case MoveType::TRACK_P : {
                std::vector<std::vector<double>> track_cartesian;
                for(int i = 0; i < new_goal->num_of_track_points; i++) {
                    std::vector<double> point;
                    point.clear();
                    for(int j = 0; j < 6; j++) {
                        // TODO: how to properly check for out of range indexing
                        point.push_back(new_goal->move_track_pos[i*6 + j]);
                    }
                    track_cartesian.push_back(point);
                }
                current_joints.clear();
                for (int i = 0; i < 6; i++) {
                    current_joints.push_back(new_goal->current_joints[i]);
                }
                this->moveTrackP(track_cartesian, current_joints, new_goal->use_joint);
                break;
            }
            case MoveType::TRACK_JOINT : {
               std::vector<std::vector<double>> track_cartesian;
                for(int i = 0; i < new_goal->num_of_track_points; i++) {
                    std::vector<double> point;
                    point.clear();
                    for(int j = 0; j < 6; j++) {
                        // TODO: how to properly check for out of range indexing
                        point.push_back(new_goal->move_track_pos[i*6 + j]);
                    }
                    track_cartesian.push_back(point);
                }
                current_joints.clear();
                for (int i = 0; i < 6; i++) {
                    current_joints.push_back(new_goal->current_joints[i]);
                }                
                this->moveTrackJoint(track_cartesian, current_joints, new_goal->use_joint);
                break;
            }
            case MoveType::GET_DI : {
                //get the DI number
                printf("Here are the DI values!\n");
                spark_robot::AuboMover::IOType io_type = static_cast<spark_robot::AuboMover::IOType>(new_goal->io_type);
                this->m_result.io_state = this->m_aubo_utils.getSingleIOStatus(io_type, new_goal->pin_num);
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case MoveType::GET_DO : {
                spark_robot::AuboMover::IOType io_type = static_cast<spark_robot::AuboMover::IOType>(new_goal->io_type);
                this->m_result.io_state = this->m_aubo_utils.getSingleIOStatus(io_type, new_goal->pin_num);
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case MoveType::SET_DO : {
                spark_robot:AuboMover::IOType io_type = static_cast<spark_robot::AuboMover::IOType>(new_goal->io_type);
                int do_val = new_goal->DO_value;
                this->m_result.DO_set_status = this->m_aubo_utils.setSingleIOStatus(io_type, new_goal->pin_num, do_val);
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            case MoveType::GET_ROBOT_STATE : {
                RobotState rs;
                this->m_aubo_utils.getRobotStatus(rs);
                // std::cout << "The current robot status is : " << std::endl;
                // switch(rs) {
                //     case RobotState::RobotPaused : {
                //         printf("Robot is paused.\n");
                //         break;
                //     }
                //     case RobotState::RobotResumed : {
                //         printf("Robot is resumed.\n");
                //         break;
                //     }
                //     case RobotState::RobotRunning : {
                //         printf("Robot is running.\n");
                //         break;
                //     }
                //     case RobotState::RobotStopped : {
                //         printf("Robot is stopped.\n");
                //         break;
                //     }
                // }
                this->m_result.robot_state = static_cast<int>(rs);
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            // Add by Sheng Wei 2021.05.21
            case MoveType::GET_AUBO_MSG : {
                joint_values.clear();
                current_pos.clear();
                this->getAuboMsg(joint_values, current_pos);

                this->m_result.joint_vals.clear();
                this->m_result.joint_vals.insert(
                    this->m_result.joint_vals.end(), 
                    joint_values.begin(), 
                    joint_values.end());

                this->m_result.waypoint_values.clear();
                this->m_result.waypoint_values.insert(
                    this->m_result.waypoint_values.end(),
                    current_pos.begin(),
                    current_pos.end()
                );
                this->m_as.setSucceeded(this->m_result);
                break;
            }
            //
            case MoveType::DEFAULT : {
                ROS_INFO("Not sure how the program gets here! Did you miss a break somewhere?");
                break;
            }
        } 
    } else {
        ROS_INFO("The goal callback is triggered but no new goal is available! Something unexpected!!");
    }
    return;
}

void rbs::pause() {
    ROS_INFO("CALLING PAUSE");
    this->m_aubo_utils.pause();
    this->m_result.has_arrived = false;
    this->m_as.setSucceeded(this->m_result);    
}

void rbs::resume() {
    ROS_INFO("CALLING RESUME");
    this->m_aubo_utils.resume();
    // FIXME: This is not the proper way to return result, but the service must be set succeded for a new service to be accepted
    this->m_result.has_arrived = true;
    this->m_as.setSucceeded(this->m_result);
}

void rbs::stop() {
    ROS_INFO("CALLING STOP");
    this->m_aubo_utils.stop();
    this->m_result.has_arrived = false;
    this->m_as.setSucceeded(this->m_result);
}

void rbs::moveLine(
    std::vector<double> position, 
    std::vector<double> current_joints, 
    bool use_joint_coord, 
    double rate_fraction,
    bool set_fraction
    ){
    ROS_INFO("CALLING MOVE LINE");
    this->m_aubo_utils.setMoveType(this->m_move_type);
    this->m_aubo_utils.setTargetLinePos(position, current_joints, use_joint_coord);

    if (set_fraction == true) {
        // profile the time spent on setting the rate fraction
        auto start = std::chrono::steady_clock::now();
        this->m_aubo_utils.set_fraction(rate_fraction);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> time_setting_rate_fraction = end - start;
        std::cout << "Time spent on setting rate fraction is " << time_setting_rate_fraction.count() << " ms." << std::endl;
    }
    this->m_aubo_utils.start();
    this->m_result.has_arrived = true;
    this->m_as.setSucceeded(this->m_result);
}

void rbs::moveJointAngle(std::vector<double> target_joints, double rate_fraction, bool set_fraction) {
    ROS_INFO("Calling move joint angle");
    this->m_aubo_utils.setMoveType(this->m_move_type);
    this->m_aubo_utils.setTargetJointPos(target_joints, std::vector<double>(), true);
    if (set_fraction == true) {
        this->m_aubo_utils.set_fraction(rate_fraction);
    }
    this->m_aubo_utils.start();
    this->m_result.has_arrived = true;
    this->m_as.setSucceeded(this->m_result);
};


void rbs::moveJoint(
    std::vector<double> target_pos, 
    std::vector<double> current_pos, 
    bool use_joint_coord,
    double rate_fraction,
    bool set_fraction) {
    ROS_INFO("CALLING MOVE JOINT");
    this->m_aubo_utils.setMoveType(this->m_move_type);
    this->m_aubo_utils.setTargetJointPos(target_pos, current_pos, use_joint_coord);
    if (set_fraction == true) {
        this->m_aubo_utils.set_fraction(rate_fraction);
    }
    this->m_aubo_utils.start();
    this->m_result.has_arrived = true;
    this->m_as.setSucceeded(this->m_result);
}

void rbs::moveTrackP(std::vector<std::vector<double>> positions, std::vector<double> current_joints, bool use_joint_coord) {
    ROS_INFO("CALLING MOVE TRACK P");
    this->m_aubo_utils.setMoveType(this->m_move_type);
    this->m_aubo_utils.setTrack(positions, current_joints, use_joint_coord);
    this->m_aubo_utils.start();
    this->m_result.has_arrived = true;
    this->m_as.setSucceeded(this->m_result);
}

void rbs::moveTrackJoint(std::vector<std::vector<double>> positions, std::vector<double> current_joints, bool use_joint_coord) {
    ROS_INFO("CALLING MOVE JOINT");
    this->m_aubo_utils.setMoveType(this->m_move_type);
    this->m_aubo_utils.setTrack(positions, current_joints, use_joint_coord);
    this->m_aubo_utils.start();
    this->m_result.has_arrived = true;
    this->m_as.setSucceeded(this->m_result);
}

void rbs::moveTrackArc(std::vector<std::vector<double>> positions, bool use_joint_coord) {

}

void rbs::preemptCB() {
    ROS_INFO("Client has stopped service.");
    // this->m_aubo_utils.stop();
    this->m_as.setPreempted();
}

void rbs::auboMsgCallback(const spark_backend::AuboInfo::ConstPtr &msg) {
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

void rbs::auboMsgSubscriber() {
    ros::NodeHandle nh;
    ros::Subscriber aubo_msg_subscriber = nh.subscribe<spark_backend::AuboInfo>("aubo_wp_info", 10, &rbs::auboMsgCallback, this); // there must be "this" at the end of the subscriber call back
    ros::Rate loop_rate(30);
    while(ros::ok() && this->m_nh.ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
};

void rbs::getAuboMsg(std::vector<double> &joint_vals, std::vector<double> &current_pos) {
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

