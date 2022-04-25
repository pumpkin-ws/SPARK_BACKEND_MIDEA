#include "chessboard_intrinsics.hpp"

using calib2d = spark_vision::calibration2d::CalibrateIntrinsicsCB;

void calib2d::help() {
    std::printf("\n---------------------------------help message------------------------------------\n");
    std::printf("This class provides utilities to calibrate the camera intrinsics with a chessboard.\n");
    std::printf("To get image intrinsic results, do the following:\n");
    std::printf("1 - load image data with load_data(std::string file_dir, std::string filename_appendix)\n");
    std::printf("2 - set the cv::Size board size and the double square width. \n");
    std::printf("3 - set the window size for the corner refinement. \n");
    std::printf("4 - run calibration and the results will be saved in a yaml file.\n");
    std::printf("5 - get the camera matrix and the distortion coefficients.\n");
    std::printf("-----------------------------------help message-----------------------------------\n\n");
}

bool calib2d::loadData(std::string file_dir, std::string appendix) {
    if(this->m_has_data == false) {
        std::vector<std::string> image_filenames = spark_filesystem::getAllFileName(file_dir, appendix);
        for(auto filename : image_filenames) {
            this->m_image_data.push_back(cv::imread(file_dir + filename, cv::IMREAD_COLOR));
        }
        // check if the first image is empty or if the number of images in the folder is less than 2
        if(this->m_image_data[0].empty()) {
            std::printf("Unable to load images, will not set the m_has_data flag to true!\n");
            std::printf("Try with another valid directory!");
            this->help();
            return false;
        } else if(this->m_image_data.size() < 2) {
            std::printf("Need at least 2 images to get intrinsic calibration. Check folder again!\n");
            this->help();
            this->m_has_data = false;
            return false;
        } else {
            std::printf("Has loaded a total of %d images.\n", (int)m_image_data.size());
            this->m_has_data = true;
            return true;
        }
    } else if(this->m_image_data.size() == 0) {
        std::printf("No data has been loaded, try to load data again!");
        this->m_has_data = false;
        return false;
    } else {
        std::printf("This object is already loaded with data. Create a new object for another calibration.\n");
        return true;
    }
}

bool calib2d::runCalibration(std::string save_dir, bool display_track_result) {
    /**
     * @brief before calibration, check the following items
     * 1 - whether images are loaded
     * 2 - whether the corner dimensions has been set
     * 3 - whether the square width has been set
     * 4 - whether the window size for corner refinement has been set
     * 5 - whether calibration has already been performed
     */
    if(this->m_has_data == false) {
        std::printf("Images has not beed loaded yet, use load_data to load image!\n");
        this->help();
        return false;
    }
    if(this->m_set_board_size == false) {
        std::printf("The board size has not been set, use set_board_size to set the board size!\n");
        this->help();
        return false;
    }
    if(this->m_set_square_width == false) {
        std::printf("The square size has not been set, use set_square_size to set the square size!\n");
        this->help();
        return false;
    }
    if(this->m_set_subpixel_window == false) {
        std::printf("The subpixel window size has not been set, use set_sub_pixel_width to set the subpixel width!\n");
        this->help();
        return false;
    }
    if(this->m_calibrated == true) {
        std::printf("Calibration performed. To perform another calibration, create a new object");
        return false;
    }
    /**
     * @brief perform calibration after passing the above checks
     * 
     */
    std::vector<std::vector<cv::Point2f>> corner_points_vec;
    for(int i{0}; i < m_image_data.size(); i++) {
        std::vector<cv::Point2f> corner_points;
        bool find_pattern = cv::findChessboardCorners(
                                this->m_image_data[i], 
                                this->m_board_size, 
                                corner_points, 
                                this->m_find_corner_flags);
        corner_points_vec.push_back(corner_points);
        if(find_pattern == false) {
            std::printf("Patterns cannot be found for some images, check the images to see if some are incomplete.\n");
            return find_pattern;
        }
    }
    // refine the corner points
    for(int i{0}; i < corner_points_vec.size(); i++) {
        cv::Mat gray = this->m_image_data[i].clone();
        cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(
            gray, 
            corner_points_vec[i], 
            cv::Size(this->m_subpixel_window, this->m_subpixel_window), 
            cv::Size(-1, -1), 
            // TODO: not sure if I should make the term criteria a parameter modifiable?
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30.0, 0.0001));
    }
    // depending on whether the user wants to display the image, default is false, so not displaying
    if(display_track_result) {
        for(int i{0}; i < this->m_image_data.size(); i++) {
            cv::Mat img = m_image_data[i].clone();
            cv::drawChessboardCorners(img, this->m_board_size, cv::Mat(corner_points_vec[i]), true);
            std::string window_name = {"corner detected for image " + std::to_string(i+1)};
            cv::imshow(window_name, img);
            cv::waitKey(0);
            cv::destroyWindow(window_name);
        }
    }
    std::vector<cv::Point3f> object_points;
    this->calcBoardCorners(this->m_board_size, this->m_square_width, object_points);
    std::vector<std::vector<cv::Point3f>> object_point_vecs;
    object_point_vecs.resize(corner_points_vec.size(), object_points);
    this->m_calibration_rms = cv::calibrateCameraRO(
                                object_point_vecs,
                                corner_points_vec,
                                m_image_data[0].size(),
                                -1,
                                this->m_camera_matrix,
                                this->m_distortion_coeffs,
                                this->m_rotation_vecs,
                                this->m_translation_vecs,
                                object_point_vecs,
                                m_calibration_flags);
    if(this->m_calibration_rms > 0) {
        std::printf("Calibrated successfully!\n");
        this->m_calibrated = true;
        /**
         * @brief write the result of tracking to a yaml file in the specified directory
         * Only write if the directory is valid
         */
        if(spark_filesystem::checkDirectoryExists(save_dir)) {
            cv::FileStorage fs((save_dir + "cam_intrinsics_result.yaml"), cv::FileStorage::WRITE);
            std::time_t rawtime; // record the time of calibration
            std::time(&rawtime);
            fs << "CalibrationDate" << std::asctime(std::localtime(&rawtime));
            fs << "CameraMatrix" << this->m_camera_matrix << "DistortionCoeffs" << this->m_distortion_coeffs;
            fs.release();
        }

        return true;
    } else {
        return false;
    }
}

void calib2d::calcBoardCorners(cv::Size board_size, float square_size, std::vector<cv::Point3f>& object_points) {
    object_points.clear();
    for(int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            object_points.push_back(cv::Point3f(j*square_size, i*square_size, 0));
        }
    }    
};