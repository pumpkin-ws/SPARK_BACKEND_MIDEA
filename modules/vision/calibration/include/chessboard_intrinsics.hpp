#ifndef CHESSBOARD_INTRINSICS_HPP_
#define CHESSBOARD_INTRINSICS_HPP_

#include <opencv2/opencv.hpp>
#include <filesystem/include/file_system.hpp>
#include <time.h>


namespace spark_vision {
    /**
     * @brief namespace calibration 2D will contain the 
     * needed functionality for performing 2D calibration
     * 
     */
    namespace calibration2d{
        /**
         * @brief CalibrateIntrinsicsCB will contain the necessary utilities
         * for calibrating the camera with check boards
         * Note: this object cannot be used to calibrate fisheye cameras. 
         * 
         */
        class CalibrateIntrinsicsCB{
        public:
            explicit CalibrateIntrinsicsCB() : m_has_data(false){
                printf("A camera intrinsics calibration object is initialized. No image data has been loaded.\n");
            }; 

            /**
             * @brief print out the help message for using this class
             * 
             */
            void help();
            
            /**
             * @brief load the calibration images into an image vector
             * 
             * @param file_dir - the file directory from which the calibration images are stored
             * @param appendix - the image appendix, i.e., .jpg or .png
             * 
             * @return bool indicating whether the image data has been loaded successfully
             */
            bool loadData(std::string file_dir, std::string appendix);
            
            /**
             * @brief Run calibration based on the prestored images
             * 
             * @param save_dir : The directory where the result yaml file will be saved
             * @param display_track_result : whether to display the tracked corner points
             * @return true calibration is successful
             * @return false 
             */
            bool runCalibration(std::string save_dir, bool display_track_result = false);

            /**
             * @brief Set the board size object
             * 
             * @param board_size : cv::Size containing the number of corner rows and cols - the board pattern
             */
            void set_board_size(cv::Size board_size) {
                this->m_board_size = board_size;
                if(this->m_board_size.empty()) {
                    printf("Passed in empty board size.\n");
                    this->m_set_board_size = false;
                } else {
                    this->m_set_board_size = true;
                }
            }
            /**
             * @brief Set the square width object
             * 
             * @param square_width : size of a single chessboard square, the unit is chosen by user, 
             *                       the intrinsic calibration result will be in the same unit
             */
            void set_square_width(double square_width) {
                this->m_square_width = square_width;
                if(this->m_square_width <= 0) {
                    std::printf("Invalid square width!\n");
                    this->m_set_square_width = false; 
                } else {
                    this->m_set_square_width = true;
                }
            }
            /**
             * @brief Set the sub pixel window object
             * 
             * @param window_size : a double representing the window used in finding refined corners
             */
            void set_sub_pixel_window(double window_size) {
                this->m_subpixel_window = window_size;
                if(this->m_subpixel_window <= 0) {
                    std::printf("Invalid subpixel window size!\n");
                    this->m_set_subpixel_window = false;
                } else {
                    this->m_set_subpixel_window = true;
                }
            }
            /**
             * @brief Get the board size object
             * 
             * @return cv::Size : dimension of the corner points
             */
            cv::Size get_board_size() const {
                return this->m_board_size;
            }
            /**
             * @brief Get the square width object
             * 
             * @return double : the square width of the chessboard squares
             */
            double get_square_width() const {
                return this->m_square_width;
            }
            /**
             * @brief Get the camera matrix object
             * 
             * @return cv::Mat a 3x3 camera intrinsics matrix
             * fx 0  cx
             * 0  fy cy
             * 0  0  1
             */
            cv::Mat get_camera_matrix() const {
                if(m_calibrated == false) {
                    std::printf("The camera has not been calibrated, check for empty Mat before use.\n");
                }
                return m_camera_matrix;
            }
            /**
             * @brief Get the distortion coeffs object
             * 
             * @return cv::Mat returns a 5x1 matrix containing the distortion coefficients
             */
            cv::Mat get_distortion_coeffs() const {
                if(m_calibrated == false) {
                    std::printf("The camera has not been calibrated, check for empty Mat before use.\n");
                }
                return m_distortion_coeffs;
            }

            void set_find_corner_flags(int flag) {
                this->m_find_corner_flags = flag;
            }
            /**
             * @brief undistort the image based on the camera matrix and calculated 
             * 
             * @param inupt 
             * @param output 
             */
            void undistortImage(cv::Mat input, cv::Mat output) {
                if(m_calibrated == false) {
                    std::printf("Perform calibration first!!");
                    this->help();
                    return;
                } else {
                    cv::undistort(input, output, this->m_camera_matrix, this->m_distortion_coeffs);
                    return;
                }
                
            };
            
        private:
            bool m_has_data = false;
            bool m_set_board_size = false;
            bool m_set_square_width = false;
            bool m_set_subpixel_window = false;
            bool m_calibrated = false;
            std::vector<cv::Mat> m_image_data;
            cv::Size m_board_size;
            double m_square_width{0}; // the length of the square be in the user desired unit
            double m_subpixel_window{11}; // size of the subpixel window
            double m_calibration_rms{0};
            cv::Mat m_camera_matrix;
            cv::Mat m_distortion_coeffs;
            cv::Mat m_rotation_vecs;
            cv::Mat m_translation_vecs;
            int m_find_corner_flags{cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE};
            int m_calibration_flags{0 | cv::CALIB_USE_LU};
            /**
             * @brief calculate the ideal object points given the actual width of a chessboard cell
             * 
             * @param board_size : the pattern size of the board, this is stored as a private member
             * @param square_size : the actual length of a square cell, this is stored as a private member
             * @param object_points : passed in as a reference, storing the object points 
             */
            void calcBoardCorners(cv::Size board_size, float square_size, std::vector<cv::Point3f>& object_points);
        };
    }
    
}


#endif