#include "halcon_eye_in_hand_3d.hpp"
#include <chrono>

int main() {
    spark_vision::CalibrateEyeInHand3D_Halcon calib_eih;

    auto start = std::chrono::steady_clock::now();
    calib_eih.readCalibParametersFromYAML("./parameters.yaml");
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> time_passed = end - start;
    std::cout << "Time spent on reading yaml is " << time_passed.count() << " ms" << std::endl;

    start = std::chrono::steady_clock::now();
    calib_eih.calculateCalibrationPlatePoses("./pic/");
    end = std::chrono::steady_clock::now();
    time_passed = end - start;
    std::cout << "Time spent on calculating each board pose is " << time_passed.count() << " ms" << std::endl;

    start = std::chrono::steady_clock::now();
    calib_eih.readRobotPoseFromYAML("./RobotPose.yaml");
    end = std::chrono::steady_clock::now();
    time_passed = end - start;
    std::cout << "Time spent on reading robot pose is " << time_passed.count() << " ms" << std::endl;

    start = std::chrono::steady_clock::now();
    calib_eih.calibrateAndSaveResult("./result/");
    end = std::chrono::steady_clock::now();
    time_passed = end - start;
    std::cout << "Time spent on calibrating hand-eye transformation is " << time_passed.count() << " ms" << std::endl;

    Eigen::Matrix4d result = calib_eih.readCalibrateResult("./result/");
    std::cout << result << std::endl;

    return EXIT_SUCCESS;
}