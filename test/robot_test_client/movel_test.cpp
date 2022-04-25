#include "robot/aubo/include/aubo_move_utils.hpp"
#include "common.hpp"
#include <unistd.h>

int main(int argc, char** argv) {
    spark_robot::AuboUtils au;
    au.set_fraction(0.01);
    au.setMoveType(MoveType::MOVE_L);
    std::vector<double> joints(6);
    joints[0] = 24.45 * M_PI / 180.0;
    joints[1] = -21.9 * M_PI / 180.0;
    joints[2] = 58.04 * M_PI / 180.0;
    joints[3] = -10.61 * M_PI / 180.0;
    joints[4] = 90.94915 * M_PI / 180.0;
    joints[5] = 174 * M_PI / 180.0;
    // au.setTargetJointPos(joints);
    au.start();
    sleep(2);
    au.pause();
    sleep(2);
    au.resume();
    return EXIT_SUCCESS;
}