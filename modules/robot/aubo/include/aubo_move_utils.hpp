#ifndef AUBO_MOVE_UTILS_HPP_
#define AUBO_MOVE_UTILAS_HPP_

#include "aubo_move.hpp"
#include <cmath>
#include "common.hpp"

namespace spark_robot {
    class AuboUtils :  public AuboMover{
    private:
        // TODO: delete the copy assignment and constructor here
        double m_rate_fraction{0.01};
        aubo_robot_namespace::wayPoint_S m_target_joint_pos; // the target position for moving with moveJ
        aubo_robot_namespace::wayPoint_S m_target_line_pos; // the target position for moving with moveL
        std::vector<aubo_robot_namespace::wayPoint_S> m_target_track; // the set of way points for moveTrack

        bool m_joint_pos_set{false};
        bool m_line_pos_set{false};
        bool m_track_set{false};
        bool m_move_type_set{false};

        float m_blend_radius{0.03};
        MoveType m_move_type = MoveType::MOVE_J;

    public:
        AuboUtils(){};  // Add by Hudi 2021.05.21
        AuboUtils(std::string, int){};  // Add by Hudi 2021.05.21

        void run() override;
        void moveConfig(double fraction) override;
        void moveLine() override;
        void moveJoint() override;
        void moveJointAngle();
        void moveTrackCartesianP() override;
        void moveTrackArcCir() override;
        void moveTrackJoint() override;

        void startLine();
        void startJoint();
        void startTrackP();

        void set_fraction(double frac) {
            if ((frac > 0) && (frac <= 1)) {
                this->m_rate_fraction = frac;
            }
            else {
                LOG_ERROR("Invalid rate fraction, fraction needs to be between 0 and 1.");
            }
        }
        double get_fraction() const {
            return m_rate_fraction;
        }
        /**
         * @brief Set the Target Joint Pos object
         * 
         * @param pos the target position to move the robot to, the pos can either be in cartesian coordinates or in joint coordinates
         * @param current_joints the current joint position, used as a reference for calculating inverse kinematic
         * @param use_joint_coord whether joint coordinate for the target pos is used, if yes, then update joint directly, if no, calculate the inverse 
         * kinematics first and then pass on the joints
         */
        void setTargetJointPos(std::vector<double> target_pos, std::vector<double> current_joints, bool use_joint_coord = true); 

        /**
         * @brief Set the Target Line Pos 
         * 
         * @param taget_pos the target position to move to, the target position can be in either cartesian coordinate or in joint coordinate
         * if the target position is in joint coordinate, then the position needs to be transformed into cartesian coordinate by calculating the 
         * forward kinematics
         * @param current_joints the current joint position used as a reference for calculating the inverse kinematic
         * @param use_joint_coord whether joint coordinate for the target position is used, if yes, then calculate the forward kinematcis first then pass
         * on the cartesian coordinate, if no, then pass on the cartesian directly. The user needs to decide and tell the function which coordinate
         * is being used!
         */
        void setTargetLinePos(std::vector<double> taget_pos, std::vector<double> current_joints, bool use_joint_coord = true);
        void setTrack(std::vector<std::vector<double>> way_points, std::vector<double> current_joints, bool use_joint_coord = true); 
        void setMoveType(MoveType mt) {
            this->m_move_type = mt;
            this->m_move_type_set = true;
        }        
        void setJointPosState(bool status) {
            this->m_joint_pos_set = status;
        }; 
        void setLinePosState(bool status) {
            this->m_line_pos_set = status;
        };
        void setTrackPosState(bool status) {
            this->m_track_set = status;
        };
        void setMoveTypeState(bool status) {
            this->m_move_type_set = status;
        }
    };
}

#endif