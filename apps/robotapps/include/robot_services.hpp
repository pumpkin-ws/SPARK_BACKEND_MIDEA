#ifndef ROBOT_SERVICES_HPP_
#define ROBOT_SERVICES_HPP_

#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"
#include "spark_backend/RobotServiceAction.h"
#include "robot/aubo/include/aubo_move_utils.hpp"
#include "common.hpp"
#include "spark_backend/AuboInfo.h"
#include "std_msgs/String.h"

namespace spark_robot {
    class AuboServer {
    private:
        // TODO: Delete the copy constructors here
        ros::NodeHandle m_nh; // declare the node handle before declaring the action server
        actionlib::SimpleActionServer<spark_backend::RobotServiceAction> m_as;
        spark_backend::RobotServiceFeedback m_feedback;
        spark_backend::RobotServiceResult m_result;
        std::string m_action_name;
        AuboUtils m_aubo_utils;
        MoveType m_move_type;

        std::string m_sub_name;  // Add by Hudi 2021.05.21

        void goalCB();
        void preemptCB();
        void moveLine(std::vector<double> position, std::vector<double> current_joints, bool use_joint_coord, double rate_fraction, bool set_fraction);
        void moveJoint(std::vector<double> target_pos, std::vector<double> current_joints, bool use_joint_coord, double rate_fraction, bool set_fraction);
        void moveJointAngle(std::vector<double> target_joints, double rate_fraction, bool set_fraction);
        void moveTrackP(std::vector<std::vector<double>> positions, std::vector<double> current_joints, bool use_joint_coord);
        void moveTrackArc(std::vector<std::vector<double>> positions, bool use_joint_coord);
        void moveTrackJoint(std::vector<std::vector<double>> positions, std::vector<double> current_joints, bool use_joint_coord);
        void pause();
        void stop();
        void resume();

        void auboMsgCallback(const spark_backend::AuboInfo::ConstPtr &msg);
        void auboMsgSubscriber();


        void getAuboMsg(std::vector<double> &joint_vals, std::vector<double> &current_pos);  // Add by Hudi 2021.05.21
        // void subCallback(const spark_backend::AuboInfo::ConstPtr &msg);  // Add by Hudi 2021.05.21
        void startAuboMsg();
        struct AuboMessage {
            std::vector<float> way_points;
            std::vector<float> current_joints;
            std::mutex mtx;
        };
        AuboMessage m_aubo_msg;

        ros::Subscriber aubo_msg_subscriber;
                   
        
    public:
        // Initialize the robot in the AuboServer Constructor
        // Changed by Hudi 2021.05.21
        AuboServer(std::string ac_name, std::string sub_name, std::string robot_host = "192.168.1.36", int robot_port = 8899) :
            m_as(m_nh, ac_name, false),
            m_aubo_utils(robot_host, robot_port),
            m_action_name(ac_name),
            m_sub_name(sub_name) {
                m_as.registerGoalCallback(boost::bind(&AuboServer::goalCB, this));
                m_as.registerPreemptCallback(boost::bind(&AuboServer::preemptCB, this));
                m_as.start();
        std::thread t_aubo_subscriber(&AuboServer::auboMsgSubscriber, this); // need "this" at the end of starting a thread in a member function
        t_aubo_subscriber.detach();
        }

        // The destructor
        ~AuboServer(void) {
        }

    };
}

#endif