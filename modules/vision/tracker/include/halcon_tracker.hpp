/**
 * @file vision_tracker.hpp
 * @author Sheng Wei (pumpkin_wang@foxmail.com)
 * @brief contains code used for 2D tracking, this version is still mainly dependent on Halcon,
 * eventually will need to get rid of halcon
 * @version 0.1
 * @date 2021-02-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef HALCON_TRACKER_HPP_
#define HALCON_TRACKER_HPP_
#include "spark_vision.hpp"

#include <map>
#include <opencv2/opencv.hpp>
#include "common.hpp"

/**
 * @brief The name space for vision related utilities
 * 
 */
namespace spark_vision {
    /**
     * @brief The name space for 2d trackers, GET RID OF HALCON, USE OPENCV!!
     * 
     */
    namespace tracker_2d {
        /**
         * @brief Different tracking algorithms using halcon, shape based,
         * light tracking, or others
         * 
         */
        class HalconTemplateTracker_v0 {
        public:
            /**
             * @brief Construct a new HalconTemplateTracker_v0 object
             * 
             * 
             * @param TemplateImage 
             */
            explicit HalconTemplateTracker_v0(HalconCpp::HObject TemplateImage) : m_ho_model_image(TemplateImage) {

            };

            explicit HalconTemplateTracker_v0() {

            };

            ~HalconTemplateTracker_v0(){
                
            }

            /**
             * @brief Select image template by subtracting background from foreground
             * BgSub stands for Background Subtraction, this is not a pun! hehe
             * 
             * @param foreground_image : image of the foreground, with object to track
             * @param background_image : image of the background, without object to track
             */
            void selectTemplateViaBgSub(HalconCpp::HObject foreground_image, HalconCpp::HObject background_image);

            /**
             * @brief track object based on templates already generated
             * 
             * @param image : the image may or may not contain the 
             * @return std::map<std::string, double> 
             */
            std::map<std::string, double> trackObject(HalconCpp::HObject image);

            /**
             * @brief track multiple objects of the same object template generated previously
             * 
             * @param image the opencv image containing the objects to be tracked
             * @param number_of_object the number of objects should be searched for  
             * @return std::map<int, std::map<std::string, double>> : int is the ordering number of the result, std::map<std::string, double> is 
             * the corresponding tracked result
             */
            std::map<int, std::map<std::string, double>> trackMultipleObject(cv::Mat image, int number_of_objects);

            /**
             * @brief Set the min matching score object
             * If the min_matching_score is not within 0 and 1, then do not
             * set the the minimum matching score
             * 
             * @param d the value of the minimum matching score to be set
             */
            void set_min_matching_score(double d) {
                // if the minimum matching score is not within 0 and 1, do not set
                if((d > 0) && (d <= 1)) {
                    m_ht_min_matching_score = d;
                } else {
                    printf("The minimum matching score is not in (0,1].\n");
                    printf("Program will use default minimum matching score of 0.5.\n");
                }
            }

            /**
             * @brief Get the min matching score object
             * 
             * @return double : the minimum matching score, should be in (0,1]
             */
            double get_min_matching_score() const {
                return static_cast<double>(m_ht_min_matching_score);
            }

            /**
             * @brief Set the greediness object
             * The input greediness score should be in (0,1]
             * 
             * @param d 
             */
            void set_greediness(double d) {
                if((d > 0) && (d <= 1)) {
                    m_ht_greediness = d;
                } else {
                    printf("The minimum matching score is not in (0,1].\n");
                    printf("Program will use default greediness of 0.9.\n");
                }
            }

            /**
             * @brief Get the greediness object
             * Greediness used in template object tracking
             * @return double 
             */
            double get_greediness() const {
                return static_cast<double>(m_ht_greediness);
            }

            /**
             * @brief Set the num matches object
             * Set if the number of matches is greater than 0, do not set if the number of matches is less than 0
             * 
             * @param d 
             */
            void set_num_matches(int matches) {
                if(matches > 0) {
                    this->m_ht_num_matches = matches;
                } else {
                    printf("The number of templates is not greater than 0.\n");
                    printf("Program will use default number of matches 1.\n");
                }
            }

            /**
             * @brief Get the num matches object
             * Return the number of matches that the tracker needs to find
             * 
             * @return int 
             */
            int get_num_matches() const {
                return static_cast<int>(this->m_ht_num_matches);
            };

            /**
             * @brief Get the model image object
             * 
             * @return HalconCpp::HObject - The halcon object storing the template image
             */
            HalconCpp::HObject get_model_image() const {
                return this->m_ho_model_image;
            }

            void set_model_image(cv::Mat model_image) {
                this->m_ho_model_image = spark_vision::cvMatToHobject(model_image);
            }
            /**
             * @brief Select the image template interactively by allowing the user
             * to draw a bounding box on the image
             * 
             * @param template_image : the image from which track model is selected
             */
            void selectTemplateInteractive(std::string template_name);
            
            

            // XIAOGE LV
			/**
             * @brief  read template from file
             *
             * 
             * @param template_path : the input path should be the absolute system path
			 * @param template_name : the name of the template
             */
            void readTemplateFromFile(std::string template_path,std:: string template_name);

            // XIAOGE LV
			/**
             * @brief  save template to file
             *
             * 
             * @param path_name : the path which use to save the template
			 * @param template_name : the name of the template
             */
			void saveTemplateToFile(std::string path_name,std:: string template_name);

			// XIAOGE LV
			/**
			 * @brief Get the Handle of the Template 
			 *
			 * @return Tuple : the Handle of the Template
			 */
			HalconCpp::HTuple getTemplateHandle()const{
				return this->m_ht_modelID;
			}

        private:
            // delete the copy constructors to avoid any default copy behaviors
            HalconTemplateTracker_v0(const HalconTemplateTracker_v0&) = delete;
            HalconTemplateTracker_v0& operator=(const HalconTemplateTracker_v0&) = delete;
            
            // use ho_ as hobject, hv_ as htuple
            HalconCpp::HObject m_ho_model_image, 
                               m_ho_image_gauss,
                               m_ho_image_reduced, 
                               m_ho_model_ROI,
                               m_ho_image_emphasis,
                               m_ho_image_illuminate,
                               m_ho_image_shocked;

            /**
             * @brief the ROI rect is a rectangle containing the ROI selected either by the user or 
             * automatically from file or by background subtraction
             * 
             */
            HalconCpp::HObject m_ho_ROI_rect; 
            HalconCpp::HTuple m_ht_window_handle;
            HalconCpp::HTuple m_ht_model_phi, 
                              m_ht_model_length, 
                              m_ht_model_width,
                              m_ht_modelID, 
                              m_ht_model_rows, 
                              m_ht_model_cols,
                              m_ht_tracked_row{0},
                              m_ht_tracked_col{0},
                              m_ht_tracked_angle{0},
                              m_ht_tracked_scale_R,
                              m_ht_tracked_scale_C,
                              m_ht_tracked_score{0},
                              m_ht_tracked_model;

            HalconCpp::HTuple m_ht_min_matching_score{0.5}; // the minimum matching score for a track to be considered successful
            HalconCpp::HTuple m_ht_greediness{0.9}; // greediness used in tracking
            HalconCpp::HTuple m_ht_num_matches{1}; // the number of matches to find
            /**
             * @brief check whether a template has been generated, track can only be performed 
             * after a template has been successfully generated, otherwise abort track
             * 
             */
            bool m_has_template = false;

            std::string getSparkDir() {
                const char* home = getenv("HOME");
                std::string home_dir(home);
                home_dir += "/Documents/spark";
                return home_dir;
            };

        };
    }
}


#endif