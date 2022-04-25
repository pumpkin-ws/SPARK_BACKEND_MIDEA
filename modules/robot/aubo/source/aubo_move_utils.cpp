#include "aubo_move_utils.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

using util = spark_robot::AuboUtils;

void util::moveConfig(double fraction) {
    // set max joint velocity and acceleration to the fraction as specified by the user
    aubo_robot_namespace::JointVelcAccParam set_joint_vel;
    set_joint_vel.jointPara[0] = fraction * MAX_JOINT_VEL;
    set_joint_vel.jointPara[1] = fraction * MAX_JOINT_VEL;
    set_joint_vel.jointPara[2] = fraction * MAX_JOINT_VEL;
    set_joint_vel.jointPara[3] = fraction * MAX_JOINT_VEL;
    set_joint_vel.jointPara[4] = fraction * MAX_JOINT_VEL;
    set_joint_vel.jointPara[5] = fraction * MAX_JOINT_VEL;
    this->m_robot_service.robotServiceSetGlobalMoveJointMaxVelc(set_joint_vel);

    aubo_robot_namespace::JointVelcAccParam set_joint_acc;
    set_joint_acc.jointPara[0] = fraction * MAX_JOINT_ACC;
    set_joint_acc.jointPara[1] = fraction * MAX_JOINT_ACC;
    set_joint_acc.jointPara[2] = fraction * MAX_JOINT_ACC;
    set_joint_acc.jointPara[3] = fraction * MAX_JOINT_ACC;
    set_joint_acc.jointPara[4] = fraction * MAX_JOINT_ACC;
    set_joint_acc.jointPara[5] = fraction * MAX_JOINT_ACC;

    this->m_robot_service.robotServiceSetGlobalMoveJointMaxAcc(set_joint_acc);

    // set max linear velocity and maximum linear acceleration
    double max_linear_vel = fraction * 2.0;
    double max_linear_acc = fraction * 2.0;
    this->m_robot_service.robotServiceSetGlobalMoveEndMaxLineVelc(max_linear_vel);
    this->m_robot_service.robotServiceSetGlobalMoveEndMaxLineAcc(max_linear_acc);
    
};


void util::setTargetJointPos(std::vector<double> target_pos, std::vector<double> current_joints, bool use_joint_coord) {
    // all movements defaulted to use the joint coordinate
    if(use_joint_coord == true) {
        /**
         * @brief If the joint coordinate is used, the values are passed on to m_target_joint_pos directly
         * 
         */
        // if joint coordinate is used, the values 
        this->m_target_joint_pos.jointpos[0] = target_pos[0];
        this->m_target_joint_pos.jointpos[1] = target_pos[1];
        this->m_target_joint_pos.jointpos[2] = target_pos[2];
        this->m_target_joint_pos.jointpos[3] = target_pos[3];
        this->m_target_joint_pos.jointpos[4] = target_pos[4];
        this->m_target_joint_pos.jointpos[5] = target_pos[5];
        this->m_joint_pos_set = true;
    } else {
        // cartesian coordinate is used
        aubo_robot_namespace::Rpy rpy_tmp;
        rpy_tmp.rx = target_pos[3];
        rpy_tmp.ry = target_pos[4];
        rpy_tmp.rz = target_pos[5];
        /**
         * @brief calculate the inverse kinematics and 
         * 
         */
        double start_point_joint_angle[6];
        start_point_joint_angle[0] = current_joints[0];
        start_point_joint_angle[1] = current_joints[1];
        start_point_joint_angle[2] = current_joints[2];
        start_point_joint_angle[3] = current_joints[3];
        start_point_joint_angle[4] = current_joints[4];
        start_point_joint_angle[5] = current_joints[5];

        aubo_robot_namespace::Pos pos;
        pos.x = target_pos[0];
        pos.y = target_pos[1];
        pos.z = target_pos[2];

        aubo_robot_namespace::Ori ori;
        this->m_robot_service.RPYToQuaternion(rpy_tmp, ori);
        this->m_robot_service.robotServiceRobotIk(start_point_joint_angle, pos, ori, this->m_target_joint_pos);
        std::cout << "The calculated joint angles are : " << std::endl;
        std::cout << this->m_target_joint_pos.jointpos[0] << std::endl;
        std::cout << this->m_target_joint_pos.jointpos[1] << std::endl;
        std::cout << this->m_target_joint_pos.jointpos[2] << std::endl;
        std::cout << this->m_target_joint_pos.jointpos[3] << std::endl;
        std::cout << this->m_target_joint_pos.jointpos[4] << std::endl;
        std::cout << this->m_target_joint_pos.jointpos[5] << std::endl;
        
        this->m_joint_pos_set = true;
    }
}; 

void util::setTargetLinePos(std::vector<double> target_pos, std::vector<double> current_joints, bool use_joint_coord) {
    if(use_joint_coord == true) {
        /**
         * @brief in this case, needs to use the forward kinematics to solve for the 
         * line position of the target. Maybe it is unnecessary to calculate the forward kinematics,
         * given the joint values, the controller programs will calculate the inverse kinematics. 
         * 
         */
        this->m_target_line_pos.jointpos[0] = target_pos[0];
        this->m_target_line_pos.jointpos[1] = target_pos[1];
        this->m_target_line_pos.jointpos[2] = target_pos[2];
        this->m_target_line_pos.jointpos[3] = target_pos[3];
        this->m_target_line_pos.jointpos[4] = target_pos[4];
        this->m_target_line_pos.jointpos[5] = target_pos[5];

        double joint_angles[6];
        joint_angles[0] = target_pos[0];
        joint_angles[1] = target_pos[1];
        joint_angles[2] = target_pos[2];
        joint_angles[3] = target_pos[3];
        joint_angles[4] = target_pos[4];
        joint_angles[5] = target_pos[5];

        this->m_robot_service.robotServiceRobotFk(joint_angles, aubo_robot_namespace::ARM_DOF, this->m_target_line_pos);
        std::cout << "The calculated forward kinematic EE position is: \n";
        std::cout << "x: " << this->m_target_line_pos.cartPos.position.x << std::endl;
        std::cout << "y: " << this->m_target_line_pos.cartPos.position.y << std::endl;
        std::cout << "z: " << this->m_target_line_pos.cartPos.position.z << std::endl;

        
        this->m_line_pos_set = true;
    } else {
        // cartesian coordinate is used
        this->m_target_line_pos.cartPos.position.x = target_pos[0];
        this->m_target_line_pos.cartPos.position.y = target_pos[1];
        this->m_target_line_pos.cartPos.position.z = target_pos[2];
        aubo_robot_namespace::Rpy rpy_tmp;
        rpy_tmp.rx = target_pos[3];
        rpy_tmp.ry = target_pos[4];
        rpy_tmp.rz = target_pos[5];
        this->m_robot_service.RPYToQuaternion(rpy_tmp, this->m_target_line_pos.orientation);
        
        // calculate the inverse kinematic and set the joint value here
        aubo_robot_namespace::Pos pos;
        pos.x = target_pos[0];
        pos.y = target_pos[1];
        pos.z = target_pos[2];
        aubo_robot_namespace::Ori ori;
        this->m_robot_service.RPYToQuaternion(rpy_tmp, ori);

        double start_point_joint_angle[6];
        start_point_joint_angle[0] = current_joints[0];
        start_point_joint_angle[1] = current_joints[1];
        start_point_joint_angle[2] = current_joints[2];
        start_point_joint_angle[3] = current_joints[3];
        start_point_joint_angle[4] = current_joints[4];
        start_point_joint_angle[5] = current_joints[5];

        // TODO: estimate the time spent on calculating the inverse kinematics
        auto start = std::chrono::steady_clock::now();
        this->m_robot_service.robotServiceRobotIk(start_point_joint_angle, pos, ori, this->m_target_line_pos);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> time_elapsed = end - start;
        std::cout << "Time spent on estimating move L kinematics is " << time_elapsed.count() << std::endl;

        this->m_line_pos_set = true;
    }
};

void util::setTrack(std::vector<std::vector<double>> way_points, std::vector<double> current_joints, bool use_joint_coord) {
    this->m_target_track.clear();
    aubo_robot_namespace::wayPoint_S wp;
    if(use_joint_coord == true) {
        // the joint coordinate is used
        for(int i = 0; i < way_points.size(); i++) {
            wp.jointpos[0] = way_points[i][0];
            wp.jointpos[1] = way_points[i][1];
            wp.jointpos[2] = way_points[i][2];
            wp.jointpos[3] = way_points[i][3];
            wp.jointpos[4] = way_points[i][4];
            wp.jointpos[5] = way_points[i][5];
            this->m_target_track.push_back(wp);
        }
        this->m_track_set = true;
    } else {
        // the cartesian coordinate is used
        // TODO: the inverse kinematic of each joint position needs to be calculated
        // set the start joint angle to the current joint angle
        double start_point_joint_angle[6];
        start_point_joint_angle[0] = current_joints[0];
        start_point_joint_angle[1] = current_joints[1];
        start_point_joint_angle[2] = current_joints[2];
        start_point_joint_angle[3] = current_joints[3];
        start_point_joint_angle[4] = current_joints[4];
        start_point_joint_angle[5] = current_joints[5];

        for(int i = 0; i < way_points.size(); i++) {
            wp.cartPos.position.x = way_points[i][0];
            wp.cartPos.position.y = way_points[i][1];
            wp.cartPos.position.y = way_points[i][2];
            aubo_robot_namespace::Rpy rpy_tmp;
            rpy_tmp.rx = way_points[i][3];
            rpy_tmp.ry = way_points[i][4];
            rpy_tmp.rz = way_points[i][5];
            this->m_robot_service.RPYToQuaternion(rpy_tmp, wp.orientation);
            // this->m_target_track.push_back(wp);
            this->m_robot_service.robotServiceRobotIk(start_point_joint_angle, wp.cartPos.position, wp.orientation, wp);
            //update the start joint angle
            start_point_joint_angle[0] = wp.jointpos[0];
            start_point_joint_angle[1] = wp.jointpos[1];
            start_point_joint_angle[2] = wp.jointpos[2];
            start_point_joint_angle[3] = wp.jointpos[3];
            start_point_joint_angle[4] = wp.jointpos[4];
            start_point_joint_angle[5] = wp.jointpos[5];
            printf("The current joint angles are: \n");
            std::cout << start_point_joint_angle[0] << std::endl;
            std::cout << start_point_joint_angle[1] << std::endl;
            std::cout << start_point_joint_angle[2] << std::endl;
            std::cout << start_point_joint_angle[3] << std::endl;
            std::cout << start_point_joint_angle[4] << std::endl;
            std::cout << start_point_joint_angle[5] << std::endl;
            this->m_target_track.push_back(wp);
        }

        this->m_track_set = true;
    }
}; 

void util::moveJoint() {
    // check if the joint position has been set yet
    if(this->m_joint_pos_set == false) {
        LOG_ERROR("Set the joint position before performing joint move!!");
    }
    // initialize the motion properties
    this->m_robot_service.robotServiceInitGlobalMoveProfile();
    this->moveConfig(this->m_rate_fraction);
    // perform the joint movement, the robot service joint move will only 
    // this call will get the joint components of the wayPoint_S. 
    this->m_robot_service.robotServiceSetNoArrivalAhead();
    this->m_robot_service.robotServiceJointMove(this->m_target_joint_pos, false);
    this->m_joint_pos_set = false;
}

// added by Sheng on July 2, 2021
void util::moveJointAngle() {
    if(this->m_joint_pos_set == false) {
        LOG_ERROR("Set the joint position before performing joint move");
    }
    // initialize the motion properties
    this->m_robot_service.robotServiceInitGlobalMoveProfile();
    this->moveConfig(this->m_rate_fraction);
    this->m_robot_service.robotServiceSetNoArrivalAhead();
    this->m_robot_service.robotServiceJointMove(this->m_target_joint_pos, false);
    this->m_joint_pos_set = false;
}

void util::moveLine() {
    // check if the line position has been set yet
    if(this->m_line_pos_set == false) {
        LOG_ERROR("Set the line position before performing line move");
    }
    // initialize the motion properties
    auto start = std::chrono::steady_clock::now();
    this->m_robot_service.robotServiceInitGlobalMoveProfile();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> time_elapsed = end - start;
    std::cout << time_elapsed.count() << std::endl;
    this->moveConfig(this->m_rate_fraction);
    // perform the line movement
    // arrival ahead 
    // this->m_robot_service.robotServiceSetArrivalAheadDistanceMode(0.003);
    this->m_robot_service.robotServiceSetNoArrivalAhead();
    this->m_robot_service.robotServiceLineMove(this->m_target_line_pos, true);
    this->m_line_pos_set = false;
}

void util::moveTrackCartesianP() {
    // check if the track position set has been set yet
    if(this->m_track_set == false) {
        LOG_ERROR("Set the track point set before performing track move");
    }
    // initialize the motion properties
    this->m_robot_service.robotServiceInitGlobalMoveProfile();
    // clear the global way point container
    this->m_robot_service.robotServiceClearGlobalWayPointVector();
    for(int i = 0; i < this->m_target_track.size(); i++) {
        this->m_robot_service.robotServiceAddGlobalWayPoint(m_target_track[i]);
    }
    // this->m_robot_service.robotServiceSetArrivalAheadDistanceMode(0.003);
    // this->m_robot_service.robotServiceSetArrivalAheadTimeMode(0.01);
    this->m_robot_service.robotServiceSetGlobalBlendRadius(this->m_blend_radius);
    this->m_robot_service.robotServiceTrackMove(aubo_robot_namespace::CARTESIAN_MOVEP, false);
    this->m_track_set = false;
}

void util::moveTrackJoint() {
    if(this->m_track_set == false) {
        LOG_ERROR("Set the track point set before performing track move");
    }
    // initialize the motion properties
    this->m_robot_service.robotServiceInitGlobalMoveProfile();
    // clear the global way point container
    this->m_robot_service.robotServiceClearGlobalWayPointVector();
    for(int i = 0; i < this->m_target_track.size(); i++) {
        this->m_robot_service.robotServiceAddGlobalWayPoint(m_target_track[i]);
    }
    this->m_robot_service.robotServiceSetGlobalBlendRadius(this->m_blend_radius);
    this->m_robot_service.robotServiceSetArrivalAheadDistanceMode(0.003);
    this->m_robot_service.robotServiceSetArrivalAheadTimeMode(0.01);
    this->m_robot_service.robotServiceTrackMove(aubo_robot_namespace::JIONT_CUBICSPLINE, false);
    this->m_track_set = false;
}

void util::run() {
    if(this->m_move_type_set == false) {
        LOG_ERROR("Set the move type before performing move service");
        return;
    } 
    std::cout << static_cast<int>(this->m_move_type) << std::endl;
    switch(this->m_move_type) {
        case MoveType::MOVE_J : {
            this->moveJoint();
            break;
        }
        case MoveType::MOVE_JOINT_ANGLE : {
            this->moveJointAngle();
            break;
        }
        case MoveType::MOVE_L : {
            std::cout << "Executing move line command!" << std::endl;
            this->moveLine();
            break;
        }
        case MoveType::TRACK_P : {
            this->moveTrackCartesianP();
            break;
        }
        case MoveType::TRACK_JOINT : {
            this->moveTrackJoint();
            // this->stop();
            break;
        }
        default: {
            printf("No valid case selected!\n");
            break;
        }
    }
    return;
}

/**
 * @brief TODO: Define this function later
 * 
 */
void util::moveTrackArcCir() {

}

