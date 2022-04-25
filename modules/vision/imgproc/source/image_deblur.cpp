#include "image_deblur.hpp"


using oof_deblur = spark_vision::OutOfFocusDeblur;
using motion_deblur = spark_vision::MotionDeblur;

void oof_deblur::help() {
    std::printf("-----------------oof deblur help-----------------\n");
    std::printf("1 - load the image either with the constructor or the the loadImage() function\n");
    std::printf("2 - set the radius of the circular PSF, if the radius is lager than the image, the radius will be according to the largest width\n");
    std::printf("NOTE: out of focus deblurring is very sensitive to the size of the circular PSF");
    std::printf("3 - set the noise to signal ratio\n");
}

void oof_deblur::calculatePSF(cv::Mat& output_image, cv::Size filter_size, int circle_radius) {
    cv::Mat h(filter_size, CV_32F, cv::Scalar(0)); // the mask does not have to be rectangular, the data type is CV_32F
    cv::Point point(filter_size.width / 2.0, filter_size.height / 2.0);
    cv::circle(h, point, circle_radius, cv::Scalar(255), -1, 8);
    cv::Scalar summa = cv::sum(h);
    output_image = h / summa[0];
};

void oof_deblur::fftShift(const cv::Mat& input_image, cv::Mat& output_image) {
    output_image = input_image.clone();
    int cx = output_image.cols / 2.0;
    int cy = output_image.rows / 2.0;
    cv::Mat quarter_0(output_image, cv::Rect(0, 0, cx, cy));
    cv::Mat quarter_1(output_image, cv::Rect(cx, 0, cx, cy));
    cv::Mat quarter_2(output_image, cv::Rect(0, cy, cx, cy));
    cv::Mat quarter_3(output_image, cv::Rect(cx, cy, cx, cy));
    cv::Mat tmp;
    quarter_0.copyTo(tmp);
    quarter_3.copyTo(quarter_0);
    tmp.copyTo(quarter_3);
    quarter_1.copyTo(tmp);
    quarter_2.copyTo(quarter_1);
    tmp.copyTo(quarter_2);
};

void oof_deblur::calcWeinerFilter(const cv::Mat& input_h_PSF, cv::Mat& Hw, double nsr) {
    cv::Mat h_PSF_shifted;
    // shift the image domain to prepare for Fourier transform
    this->fftShift(input_h_PSF, h_PSF_shifted);
    // cv::Mat array of size 2, the first layer stores the image, and the second layer will be initialized to 0s
    cv::Mat planes[2] = {
        cv::Mat_<float>(h_PSF_shifted.clone()),
        cv::Mat::zeros(h_PSF_shifted.size(), CV_32F)
    };
    cv::Mat complex_image;
    cv::merge(planes, 2, complex_image);
    cv::dft(complex_image, complex_image);
    cv::split(complex_image, planes);
    cv::Mat denom;
    cv::pow(cv::abs(planes[0]), 2, denom);
    denom += nsr;
    cv::divide(planes[0], denom, Hw);
};

void oof_deblur::filter2DFreq(const cv::Mat& input_image, cv::Mat& output_image, const cv::Mat& Hw) {
    cv::Mat planes[2] = {
        cv::Mat_<float>(input_image.clone()),
        cv::Mat::zeros(input_image.size(), CV_32F)
    };
    cv::Mat complex_image;
    cv::merge(planes, 2, complex_image);
    cv::dft(complex_image, complex_image, cv::DFT_SCALE);
    // construct the Weiner filter
    cv::Mat planesH[2] = {
        cv::Mat_<float>(Hw.clone()),
        cv::Mat::zeros(Hw.size(), CV_32F)
    };
    cv::Mat complexH;
    cv::merge(planesH, 2, complexH);
    cv::Mat complexIH;
    cv::mulSpectrums(complex_image, complexH, complexIH, 0); // how to make sure the two complex arrays are of the same size?

    cv::idft(complexIH, complexIH);
    cv::split(complexIH, planes);
    output_image = planes[0];
};

void oof_deblur::loadImage(std::string file_name) {
    this->m_input_image = cv::imread(file_name, cv::IMREAD_COLOR);
    if(this->m_input_image.empty()) {
        std::printf("Failed to load image, check path validity!\n");
        this->m_get_input_image = false;
        return;
    } else {
        this->m_get_input_image = true;
        return;
    }
}

void oof_deblur::filterImage() {
    /**
     * @brief check the following requirements before performing 
     * 1 - has image been loaded
     * 2 - was PSF radius function set
     * 3 - was nsr set
     */
    if(this->m_get_input_image == false) {
        std::printf("Image has not been loaded, load image first!\n");
        this->help();
        return;
    }
    if(this->m_get_circle_radius == false) {
        std::printf("Set the radius for the PSF kernel before calibrating.\n");
        this->help();
        return;
    }
    if(this->m_get_nsr == false) {
        std::printf("Set the noise to signal ratio first!\n");
        this->help();
        return;
    }
    // make sure the image is a single channel image, if not, convert to a single channel image
    if(this->m_input_image.channels() == 3) {
        cv::cvtColor(this->m_input_image, this->m_input_image, cv::COLOR_BGR2GRAY);
    }

    /**
     * @brief start performing out of focus deblurring after the preconditions are satisfied
     * 
     */
    cv::Rect roi = cv::Rect(0, 0, this->m_input_image.cols & -2, this->m_input_image.rows & -2);
    
    // calculate the PSF given the image size, and then use the PSF to calculate the Weiner filter
    cv::Mat H, Hw; 
    this->calculatePSF(H, roi.size(), this->m_circle_radius);
    this->calcWeinerFilter(H, Hw, 1.0 / this->m_nsr);
    // actually perform the image deblurring
    this->filter2DFreq(this->m_input_image(roi), this->m_output_image, Hw);
    cv::imshow("before normalize", this->m_output_image);
    cv::waitKey(0);
    this->m_output_image.convertTo(this->m_output_image, CV_8U);
    cv::normalize(this->m_output_image, this->m_output_image, 0, 255, cv::NORM_MINMAX);
};

void motion_deblur::help() {
    std::printf("\n------------------------------------help on motion deblur------------------------------------\n");
    std::printf("To use this class to get the result of motion deblurring, do the following...\n");
    std::printf("1 - Load image with the constructor or loadImage()\n");
    std::printf("2 - Set length (represents how fast the camera or the object is moving) for the linear PSF\n");
    std::printf("3 - Set theta (the direction of rotation, representing towards which direction the object or the ) for the linear PSF.\n");
    std::printf("4 - Set nsr - signal to noise ratio.\n");
    std::printf("5 - Use filterImage() to perform deblurring.\n");
    std::printf("6 - Use get_output_image() to get the result image. \n");
    std::printf("\n------------------------------------help on motion deblur------------------------------------\n");
};

void motion_deblur::calcPSF(cv::Mat& output_image, cv::Size filter_size, int len, double theta) {
    cv::Mat h(filter_size, CV_32F, cv::Scalar(0));
    cv::Point point(filter_size.width / 2, filter_size.height / 2);
    // why use ellipse here but not use a line, cv::Size in the ellipse defines the length of the ellipse's main axes
    cv::ellipse(h, point, cv::Size(0, cvRound(float(len) / 2.0)), 90.0 - theta, 0, 360, cv::Scalar(255), cv::FILLED);
    // normalize the mat
    cv::Scalar summa = cv::sum(h);
    output_image = h / summa[0];
};

void motion_deblur::fftShift(const cv::Mat& input_image, cv::Mat& output_image) {
    output_image = input_image.clone();
    // fftShift can only perform on images with even rows and columns
    int cx = output_image.cols / 2;
    int cy = output_image.rows / 2;
    cv::Mat quarter0(output_image, cv::Rect(0,  0,  cx, cy));
    cv::Mat quarter1(output_image, cv::Rect(cx, 0,  cx, cy));
    cv::Mat quarter2(output_image, cv::Rect(0,  cy, cx, cy));
    cv::Mat quarter3(output_image, cv::Rect(cx, cy, cx, cy));
    cv::Mat tmp;
    // switch quarter 0 and quarter 3
    quarter0.copyTo(tmp);
    quarter3.copyTo(quarter0);
    tmp.copyTo(quarter3);
    // switch quarter 1 and quarter 2
    quarter2.copyTo(tmp);
    quarter1.copyTo(quarter2);
    tmp.copyTo(quarter1);
}

void motion_deblur::calcWeinerFilter(const cv::Mat& H, cv::Mat& Hw, double nsr) {
    cv::Mat H_shifted;
    this->fftShift(H, H_shifted);
    cv::Mat planes[2]{
        cv::Mat_<float>(H_shifted.clone()),
        cv::Mat::zeros(H_shifted.size(), CV_32F)
    };
    /**
     * @brief merge the two planes and perform discrete Fourier transform. 
     * 
     */
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);
    // what does the real part and the imaginary part of the discrete Fourier transformation mean?
    // how was discrete fourier transform performed on a matrix
    cv::dft(complexI, complexI);
    cv::split(complexI, planes);
    cv::Mat denom;
    cv::pow(cv::abs(planes[0]), 2, denom);
    denom += nsr;
    cv::divide(planes[0], denom, Hw);
};

void motion_deblur::edgeTaper(const cv::Mat& input_image, cv::Mat& output_image, double gamma, double beta) {
    int Nx = input_image.cols;
    int Ny = input_image.rows;
    cv::Mat w1(1, Nx, CV_32F, cv::Scalar(0));
    cv::Mat w2(Ny, 1, CV_32F, cv::Scalar(0));
    
    float* p1 = w1.ptr<float>(0);
    float* p2 = w2.ptr<float>(0);
    float dx = float(2.0 * CV_PI / Nx);
    float x = float(-CV_PI);
    for(int i = 0; i < Nx; i++) {
        p1[i] = float(0.5 * ( std::tanh((x + gamma/2) / beta) - std::tanh((x - gamma/2) / beta) ));
        x += dx;
    }
    float dy = float(2.0 * CV_PI / Ny);
    float y = float(-CV_PI);
    for(int i = 0; i < Ny; i++) {
        p2[i] = float(0.5 * (tanh((y + gamma / 2) / beta) - tanh((y - gamma / 2) / beta)));
        y += dy;
    }
    cv::Mat w = w2 * w1;
    cv::multiply(input_image, w, output_image);
}

void motion_deblur::filter2DFreq(const cv::Mat& input_image, cv::Mat& output_image, const cv::Mat& Hw) {
    cv::Mat planes[2] {
        cv::Mat_<float>(input_image.clone()), 
        cv::Mat::zeros(Hw.size(), CV_32F)
    };
    // perform discrete fourier transform on the original image
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);
    cv::dft(complexI, complexI, cv::DFT_SCALE);
    
    // compute the product of weiner filter and the complex image after fourier transformation
    cv::Mat planesH[2] {
        cv::Mat_<float>(Hw.clone()),
        cv::Mat::zeros(Hw.size(), CV_32F)
    };
    cv::Mat complexH;
    cv::merge(planesH, 2, complexH);

    // multiply complex H with the image, generate complexIH
    cv::Mat complexIH;
    cv::mulSpectrums(complexI, complexH, complexIH, 0);

    cv::idft(complexIH, complexIH);
    cv::split(complexIH, planes);
    output_image = planes[0];
};

void motion_deblur::filterImage() {
    /**
     * @brief check the following before performing image filtering
     * 
     */
    if(this->m_get_image == false) {
        std::printf("Did not get image! Use the constructor or loadImage to get image!\n");
        this->help();
        return;
    }
    if(this->m_get_PSF_length == false) {
        std::printf("Linear PSF length not set! Set PSF!\n");
        this->help();
        return;
    }
    if(this->m_get_theta == false) {
        std::printf("Linear PSF theta not set! Set theta of linear PSF.\n");
        this->help();
        return;
    }
    if(this->m_get_nsr == false) {
        std::printf("Noise to signal ratio not set yet! Set nsr!\n");
        this->help();
        return;
    }
    /**
     * @brief start performing image filtering
     * 
     */
    cv::Rect roi(0, 0, this->m_input_image.cols & -2, this->m_input_image.rows & -2);
    // calculate the Weiner filter
    cv::Mat H, Hw;
    this->calcPSF(H, roi.size(), this->m_len, this->m_theta);
    this->calcWeinerFilter(H, Hw, 1.0 / this->m_nsr);
    // FIXME: check if the image is a three channel image, if yes, then convert to a single channel grayscale
    if(this->m_input_image.channels() > 1) {
        cv::cvtColor(this->m_input_image, this->m_input_image, cv::COLOR_RGB2GRAY);
    }
    this->m_input_image.convertTo(this->m_input_image, CV_32F);
    this->edgeTaper(this->m_input_image, this->m_input_image, this->m_gamma, this->m_beta);
    std::cout << Hw.size() << std::endl;
    this->filter2DFreq(this->m_input_image(roi), this->m_output_image, Hw);
    this->m_output_image.convertTo(this->m_output_image, CV_8U);
    cv::normalize(this->m_output_image, this->m_output_image, 0, 255, cv::NORM_MINMAX);
};
