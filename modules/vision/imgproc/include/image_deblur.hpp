#ifndef IMAGE_DEBLUR_HPP_
#define IMAGE_DEBLUR_HPP_

#include <opencv2/opencv.hpp>

namespace spark_vision {

    /**
     * @brief use a circular point spread function to estimate the resotration filter(the Weiner filter, Hw)
     * The restore the original image in frequency domain
     * S = HU + N
     * S represents the blurred image in frequency domain, U presents the original image, H represents
     * the frequency response of the point spread function (PSF), and N is the spectrum of additive noise. 
     * To resotre image, do U = Hw*S, where Hw is the Weiner filter.
     */
    class OutOfFocusDeblur{
    public:
        /**
         * @brief Construct a new Out Of Focus Deblur object
         * The explicit default constructor, delete the copy constructors
         */
        explicit OutOfFocusDeblur(){}; 
        explicit OutOfFocusDeblur(cv::Mat input) : m_input_image(input.clone()) {
            if(this->m_input_image.empty()) {
                std::printf("Invalid image path!\n");
                this->m_get_input_image = false;
                return;
            } else {
                this->m_get_input_image = true;
                return;
            }
        };
        /**
         * @brief Prints out the help function for using this class
         * 
         */
        void help();
        /**
         * @brief load image if the input image is not initialized
         * 
         * @param file_name : the file name of the input image
         */
        void loadImage(std::string file_name);
        /**
         * @brief calculate the point spread function, 
         * 
         * @param output_image : the image representing the circular point spread function
         * @param filter_size : the image size of the filter
         * @param circle_radius : the radius of the circle
         */
        void calculatePSF(cv::Mat& output_image, cv::Size filter_size, int circle_radius);
        /**
         * @brief preparing the image for fast Fourier transform, shifting the 0th quadrant and the 3rd quadrant
         * and shifting the 2nd and 4th quardrant of the input image pixel
         * 
         * @param input_image : the image to be domian-shifted
         * @param output_image : the result of the domain-shifted image
         */
        void fftShift(const cv::Mat& input_image, cv::Mat &output_image);
        /**
         * @brief calculate the Weiner filter used for filtering the image
         * 
         * @param input_h_PSF : the input circular point spread function
         * @param Hw : the output Weiner filter 
         * @param nsr : the noise to signal ratio
         */
        void calcWeinerFilter(const cv::Mat& input_h_PSF, cv::Mat& Hw, double nsr);
        /**
         * @brief perform filtering
         * 
         * @param input_image : the input blurred imag
         * @param output_image : the output sharp image
         * @param Hw : the Weiner filter
         */
        void filter2DFreq(const cv::Mat& input_image, cv::Mat& output_image, const cv::Mat& Hw);
        /**
         * @brief perform image filtering
         * 
         */
        void filterImage();

        void set_nsr(double nsr) {
            this->m_nsr = nsr;
            if(this->m_nsr <= 0) {
                std::printf("Invalid noise to signal ratio, nsr needs to be greater than 0!\n");
                this->m_get_nsr = false;
                return;
            } else {
                this->m_get_nsr = true;
                return;
            }
        };

        void set_PSF_radius(int radius) {
            this->m_circle_radius = radius;
            if((this->m_input_image.empty()) || (this->m_get_input_image == false)) {
                std::printf("Load image before setting the PSF radius, as the length of the PSF circle\n");
                std::printf("cannot be larger than the length/width of the image");
                this->m_get_circle_radius = false;
                return;
            }
            if(this->m_circle_radius <= 0) {
                std::printf("Invalid value for PSF radius, needs to be greater than 0!\n");
                this->m_get_circle_radius = false;
                return;
            }
            int max_len = (this->m_input_image.cols >= this->m_input_image.rows) ? this->m_input_image.rows : this->m_input_image.cols;
            if(radius > max_len) {
                this->m_circle_radius = (max_len & -2) / 2;
                this->m_get_circle_radius = true;
                return;
            } else {
                this->m_circle_radius = (radius & -2) / 2;
                this->m_get_circle_radius = true;
                return;
            }
        }

        cv::Mat get_result_image() const {
            return this->m_output_image.clone();
        }

        cv::Mat get_input_image() const {
            return this->m_input_image.clone();
        }
        
        
    private:
        OutOfFocusDeblur(const OutOfFocusDeblur&) = delete; // delete the () copy constructor
        OutOfFocusDeblur operator=(const OutOfFocusDeblur&) = delete; // delete the = copy constructor

        bool m_get_circle_radius{false};
        bool m_get_input_image{false};
        bool m_get_nsr{false};
        cv::Mat m_input_image;
        cv::Mat m_output_image;
        cv::Mat m_processed_image;
        int m_circle_radius;
        double m_nsr;
    };

    /**
     * @brief The motion deblur class will use a linear PSF function to deblur the image
     * 
     * 
     */
    class MotionDeblur{
    public:
        explicit MotionDeblur(){};
        explicit MotionDeblur(cv::Mat input) : m_input_image(input.clone()) {
            if(m_input_image.empty()) {
                std::printf("Unable to load image!\n");
                this->m_get_image = false;
                return;
            } else {
                std::printf("Mat is not empty. Image loaded successfully!\n");
                this->m_get_image = true;
                return;
            }
        }
        /**
         * @brief load image given the string name
         * 
         * @param file_name : the full directory and filename of the image!
         */
        void loadImage(std::string file_name) {
            this->m_input_image = cv::imread(file_name, cv::IMREAD_GRAYSCALE);
            if(this->m_input_image.empty()) {
                std::printf("Unable to load image!\n");
                this->m_get_image = false;
                return;
            } else {
                std::printf("Mat is not empty. Image loaded successfully!\n");
                this->m_get_image = true;
                return;
            }
        };
        /**
         * @brief how to use this class to get result
         * 
         */
        void help();

        /**
         * @brief create the line PSF function 
         * 
         * @param output_image : the PSF mask
         * @param filter_size : size of the PSF mask
         * @param len : length of the line
         * @param theta : rotation of the line
         */
        void calcPSF(cv::Mat& output_image, cv::Size filter_size, int len, double theta);

        /**
         * @brief shifting the four quadrants of the image to prepare the image for Fourier transform
         * 
         * @param input_image : the input image
         * @param output_image : the output image
         */
        void fftShift(const cv::Mat& input_image, cv::Mat& output_image);
 
        /**
         * @brief calculate the Weiner filter
         * 
         * @param H the PSF mask representing a rotated line at the center of the mask
         * @param Hw the weiner filter
         * @param nsr the noise to signal ratio
         */
        void calcWeinerFilter(const cv::Mat& H, cv::Mat& Hw, double nsr);

        /**
         * @brief tapers the image's edge to reduce ringing effect?
         * 
         * @param input_image 
         * @param output_image 
         * @param gamma 
         * @param beta 
         */
        void edgeTaper(const cv::Mat& input_image, cv::Mat& output_image, double gamma, double beta);

        /**
         * @brief Perform frequency space filtering
         * 
         * @param input_image 
         * @param output_image 
         * @param Hw 
         */
        void filter2DFreq(const cv::Mat& input_image, cv::Mat& output_image, const cv::Mat& Hw);

        /**
         * @brief perform the actual filtering here
         * TODO: write this later
         * 
         */
        void filterImage();
        
        void set_length(int length) {
            this->m_len = length;
            if(this->m_len <= 0) {
                std::printf("Invalid value for the length of PSF!\n");
                this->m_get_PSF_length = false;
                return;
            } else {
                this->m_get_PSF_length = true;
                return;
            }
        }

        void set_theta(double theta, bool in_radian = true) {
            if(in_radian == true) {
                this->m_theta = theta;
            } else {
                // convert the angle from degree to radian if in radian is not true
                this->m_theta = theta * 180.0 / M_PI;
            }
            
            if(this->m_theta <= 0) {
                std::printf("Invalid value for the rotation of the angle!\n");
                m_get_theta = false;
                return;
            } else {
                this->m_get_theta = true;
            }
        }

        void set_nsr(double nsr) {
            this->m_nsr = nsr;
            if(this->m_nsr <= 0) {
                std::printf("Invalid value for nsr!\n");
                this->m_get_nsr = false;
                return;
            } else {
                this->m_get_nsr = true;
                return;
            }
        }

        cv::Mat get_output_image() const {
            return m_output_image.clone();
        }

        cv::Mat get_input_image() const {
            return m_input_image.clone();
        }
        
    private:
        // TODO: delete the copy constructors here
        cv::Mat m_input_image;
        cv::Mat m_output_image;
        bool m_get_image;
        bool m_get_PSF_length;
        bool m_get_theta;
        bool m_get_nsr;
        // TODO: have default values for the following params
        int m_len;
        double m_theta;
        double m_nsr;
        double m_gamma{5.0}, m_beta{0.2}; // gamma and beta are used for edge taping

    };
}
#endif