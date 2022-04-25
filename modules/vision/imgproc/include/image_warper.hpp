#ifndef IMAGE_WARPER_HPP_
#define IMAGE_WARPER_HPP_

#include <opencv2/opencv.hpp>

namespace spark_vision {
    /**
     * @brief This class will find the perspective transformation based on the mapping relations on the image
     * , and then perform transformations on the new images
     * 
     * This function will be matching 4 pixel points on the original image to 4 new points on the transformed pixel
     * The user can either directly call the calcMatrixFromPoints to get the perspective transform matrix, or the 
     * user can select points on an image and pass in the points to transform to.
     * 
     */
    class ImageWarper {
    public:
        explicit ImageWarper(){};
        void help();
        /**
         * @brief calculate the homography matrix from matching original points to the objective points
         * 
         * @param original_points : the original points needed to be transformed
         * @param objective_points : the destination points to be transformed to
         * @param homo_mat : the returned homography matrix
         */
        void calcMatrixFromPoints(
            const std::vector<cv::Point2f>& original_points, 
            const std::vector<cv::Point>& objective_points, 
            cv::Mat& homo_mat
        );
        /**
         * @brief warp image if the homography matrix is already calculated
         * 
         * @param input_image : the original image
         * @param output_image : the transformed image
         */
        void warpImage(const cv::Mat& input_image, cv::Mat& output_image);
        /**
         * @brief Set the image dims object, unwarping should only be performed on image of a certain size.
         * 
         * @param dims : the desired image dimensions
         */
        void set_image_dims(cv::Size dims) {
            if((dims.width < 0) || (dims.height) < 0) {
                std::printf("Invalid values for the image dimensions!\n");
                return;
            } else {
                this->m_image_dims == dims;
            }
        };
        
    private:

        bool m_has_matrix; // check whether the perspective matrix is available
        cv::Mat m_calibration_image; // the image from which the transformed points are selected 
        cv::Mat m_homo_mat;
        std::vector<cv::Point2f> original_points;
        std::vector<cv::Point2f> object_points;

        // TODO: the dimensions of the image needs to be fixed
        cv::Size m_image_dims;
    };
}


#endif