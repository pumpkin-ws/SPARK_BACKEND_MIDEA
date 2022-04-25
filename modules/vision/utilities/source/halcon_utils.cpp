#include "halcon_utils.hpp"
#include <fstream>
HalconCpp::HObject spark_vision::cvMatToHobject(cv::Mat& image) {
    using namespace HalconCpp;
    HObject HObject_image = HObject();
    int height = image.rows;
    int width = image.cols;
    
    if(image.type() == CV_8UC3) {
        // for a three channel color image, 8 bit integers
        std::vector<cv::Mat> image_channels;
        cv::split(image, image_channels);
        cv::Mat image_blue = image_channels[0];
        cv::Mat image_green = image_channels[1];
        cv::Mat image_red = image_channels[2];
        uchar* data_red = new uchar[height * width];
        uchar* data_green = new uchar[height * width];
        uchar* data_blue = new uchar[height * width];
        for(int i = 0; i < height; i++) {
            std::memcpy(data_red + width*i, image_red.data + image_red.step*i, width);
            std::memcpy(data_green + width*i, image_green.data + image_green.step*i, width);
            std::memcpy(data_blue + width*i, image_blue.data + image_blue.step*i, width);
        }
        
        try{
            GenImage3(&HObject_image, "byte", width, height, (Hlong)data_red, (Hlong)data_green, (Hlong)data_blue);
        } catch(HalconCpp::HOperatorException& except) {
            std::cout << except.ErrorMessage() << std::endl;
        }
        
        delete[] data_red;
        delete[] data_green;
        delete[] data_blue;
        data_red = NULL;
        data_green = NULL;
        data_blue = NULL;
    } else if(image.type() == CV_8UC1) {
        uchar* data = new uchar[height*width];
        for(int i = 0; i < height; i++) {
            // for a one channel gray iamge, 8 bit intergers
            uchar* data = new uchar[height * width];
            for(int i = 0; i < height; i++) {
                std::memcpy(data + width*i, image.data + image.step*i, width);
            }
            GenImage1(&HObject_image, "byte", width, height, (Hlong)data);
            delete[] data;
            data = NULL;
        }
    }
    return HObject_image;
};

cv::Mat spark_vision::hobjectToCvMat(HalconCpp::HObject H_img) {
       cv::Mat cv_img;
	HalconCpp::HTuple channels, w, h;

	HalconCpp::ConvertImageType(H_img, &H_img, "byte");
	HalconCpp::CountChannels(H_img, &channels);

	if (channels.I() == 1)
	{
		HalconCpp::HTuple pointer;
		GetImagePointer1(H_img, &pointer, nullptr, &w, &h);
		int width = w.I(), height = h.I();
		int size = width * height;
		cv_img = cv::Mat::zeros(height, width, CV_8UC1);
		memcpy(cv_img.data, (void*)(pointer.L()), size);
	}

	else if (channels.I() == 3)
	{
		HalconCpp::HTuple pointerR, pointerG, pointerB;
		HalconCpp::GetImagePointer3(H_img, &pointerR, &pointerG, &pointerB, nullptr, &w, &h);
		int width = w.I(), height = h.I();
		int size = width * height;
		cv_img = cv::Mat::zeros(height, width, CV_8UC3);
		uchar* R = (uchar*)(pointerR.L());
		uchar* G = (uchar*)(pointerG.L());
		uchar* B = (uchar*)(pointerB.L());
		for (int i = 0; i < height; ++i)
		{
			uchar *p = cv_img.ptr<uchar>(i);
			for (int j = 0; j < width; ++j)
			{
				p[3 * j] = B[i * width + j];
				p[3 * j + 1] = G[i * width + j];
				p[3 * j + 2] = R[i * width + j];
			}
		} 
	}
	return cv_img;
};

void spark_vision::displayHalconImage(HalconCpp::HObject image, HalconCpp::HTuple &h_window_handle) {
    using namespace HalconCpp;
    HTuple hobject_channels = HTuple();
    HTuple channel_type;
    CountChannels(image, &hobject_channels);
    HTuple h_width;
    HTuple h_height;
    int width, height;
    GetImageSize(image, &h_width, &h_height);
    // close all windows that are opened    
    if(HDevWindowStack::IsOpen()) {
        CloseWindow(HDevWindowStack::Pop());
    }
    // display image
    try{
        deviceOpenWindowFitImage(image, 0, 0, h_width, h_height, &h_window_handle);
    } catch(HalconCpp::HOperatorException& except) {
       std::cout << except.ErrorMessage() << std::endl;
    } 
    SetWindowParam(h_window_handle, "window_title", "Display Window");
    if(HDevWindowStack::IsOpen()){
        ClearWindow(HDevWindowStack::GetActive());
    }   
    if(HDevWindowStack::IsOpen()){
        DispObj(image, HDevWindowStack::GetActive());
    }
    if(HDevWindowStack::IsOpen()){
        SetLineWidth(HDevWindowStack::GetActive(), 2);
    }
};   

void spark_vision::deviceOpenWindowFitImage (HalconCpp::HObject ho_Image,       HalconCpp::HTuple hv_Row, 
                                             HalconCpp::HTuple hv_Column,       HalconCpp::HTuple hv_WidthLimit, 
                                             HalconCpp::HTuple hv_HeightLimit,  HalconCpp::HTuple *hv_WindowHandle)
{
  
    // Local iconic variables
    using namespace HalconCpp;
    // Local control variables
    HTuple  hv_MinWidth, hv_MaxWidth, hv_MinHeight;
    HTuple  hv_MaxHeight, hv_ResizeFactor, hv_ImageWidth, hv_ImageHeight;
    HTuple  hv_TempWidth, hv_TempHeight, hv_WindowWidth, hv_WindowHeight;

    if (0 != (HTuple(int((hv_WidthLimit.TupleLength())==0)).TupleOr(int(hv_WidthLimit<0))))
    {
        hv_MinWidth = 500;
        hv_MaxWidth = 800;
    }
    else if (0 != (int((hv_WidthLimit.TupleLength())==1)))
    {
        hv_MinWidth = 0;
        hv_MaxWidth = hv_WidthLimit;
    }
    else
    {
        hv_MinWidth = ((const HTuple&)hv_WidthLimit)[0];
        hv_MaxWidth = ((const HTuple&)hv_WidthLimit)[1];
    }
    //Parse input tuple HeightLimit
    if (0 != (HTuple(int((hv_HeightLimit.TupleLength())==0)).TupleOr(int(hv_HeightLimit<0))))
    {
        hv_MinHeight = 400;
        hv_MaxHeight = 600;
    }
    else if (0 != (int((hv_HeightLimit.TupleLength())==1)))
    {
        hv_MinHeight = 0;
        hv_MaxHeight = hv_HeightLimit;
    }
    else
    {
        hv_MinHeight = ((const HTuple&)hv_HeightLimit)[0];
        hv_MaxHeight = ((const HTuple&)hv_HeightLimit)[1];
    }
    //
    //Test, if window size has to be changed.
    hv_ResizeFactor = 1;
    GetImageSize(ho_Image, &hv_ImageWidth, &hv_ImageHeight);
    //First, expand window to the minimum extents (if necessary).
    if (0 != (HTuple(int(hv_MinWidth>hv_ImageWidth)).TupleOr(int(hv_MinHeight>hv_ImageHeight))))
    {
        hv_ResizeFactor = (((hv_MinWidth.TupleReal())/hv_ImageWidth).TupleConcat((hv_MinHeight.TupleReal())/hv_ImageHeight)).TupleMax();
    }
    hv_TempWidth = hv_ImageWidth*hv_ResizeFactor;
    hv_TempHeight = hv_ImageHeight*hv_ResizeFactor;
  //Then, shrink window to maximum extents (if necessary).
    if (0 != (HTuple(int(hv_MaxWidth<hv_TempWidth)).TupleOr(int(hv_MaxHeight<hv_TempHeight))))
    {
        hv_ResizeFactor = hv_ResizeFactor*((((hv_MaxWidth.TupleReal())/hv_TempWidth).TupleConcat((hv_MaxHeight.TupleReal())/hv_TempHeight)).TupleMin());
    }
    hv_WindowWidth = hv_ImageWidth*hv_ResizeFactor;
    hv_WindowHeight = hv_ImageHeight*hv_ResizeFactor;
    //Resize window
    SetWindowAttr("background_color","black");
    OpenWindow(hv_Row,hv_Column,hv_WindowWidth,hv_WindowHeight,0,"visible","",&(*hv_WindowHandle));
    HDevWindowStack::Push((*hv_WindowHandle));
    if (HDevWindowStack::IsOpen())
        SetPart(HDevWindowStack::GetActive(),0, 0, hv_ImageHeight-1, hv_ImageWidth-1);
    return;
}


//yujin add
void spark_vision::htupleToEigenMatrix(HalconCpp::HTuple hv_tuple, Eigen::MatrixXd &eigen_matrix) {
    int elements_num = hv_tuple.Length();
    int matrix_size=sqrt(elements_num);
    eigen_matrix.resize(matrix_size, matrix_size);
    int index=0;
    int row=0;
    int column=0;
    double matrix_value=0;
    if (elements_num == (matrix_size*matrix_size)){
        for (row = 0; row < matrix_size; ++row) {
            for (column = 0; column < matrix_size; ++column){
                if (index < elements_num) {  
                    HalconCpp::HTuple tuple_value;
                    tuple_value.Clear();
                    tuple_value = ((const HalconCpp::HTuple&)hv_tuple)[index];
                    matrix_value = tuple_value.D();
                    eigen_matrix(row,column) = matrix_value;
                }
                ++index;
            }    
        }
    }
    else {
        printf("Wrong number of elements\n");
        printf("Please check if the number of rows and columns of the input matrix are equal\n");
         
    }
 };


void spark_vision::eigenMatrixToHtuple(Eigen::MatrixXd eigen_matrix, HalconCpp::HTuple & hv_tuple) {
    int matrix_row = eigen_matrix.rows();
    int matrix_column = eigen_matrix.cols();
    hv_tuple.Clear();
    for (int row = 0; row < matrix_row; ++row) {
        for (int column = 0; column < matrix_column; ++column) {
            double matrix_value = eigen_matrix(row,column);
            hv_tuple[row * matrix_column + column] = matrix_value;
        }
    }
};

void spark_vision::htupleToVector(HalconCpp::HTuple hv_tuple, std::vector <double>& vec) {
    int elements_num = hv_tuple.Length();
    int index = 0;       
    for (int i=1;i <= elements_num;++i ) {
        if (index < elements_num) {
            HalconCpp::HTuple tuple_value;
            tuple_value.Clear();
            tuple_value = ((const HalconCpp::HTuple&)hv_tuple)[index];
            double vec_value = tuple_value.D();
            vec.push_back(vec_value);
        }
    ++index;
    };

};


void spark_vision::vectorToHtuple(std::vector <double> vec,HalconCpp::HTuple & hv_tuple) {
    int element_num=vec.size();
    for (int index=0; index < element_num; ++index){
        double vec_value = vec[index];
        hv_tuple[index] = vec_value;
    }
};

void spark_vision::htupleToCvPoint2d(HalconCpp::HTuple point_row,
                                     HalconCpp::HTuple point_column,
                                     cv::Point2d & point_coordinate){
    double row = point_row.D();
    double column = point_column.D();
    point_coordinate.x=column;
    point_coordinate.y=row;
};

void spark_vision::cvPoint2dtoHtuple(cv::Point2d point_coordinate,
                                     HalconCpp::HTuple & point_row,
                                     HalconCpp::HTuple & point_column){
    double row = point_coordinate.y;
    double column = point_coordinate.x;
    point_row=row;
    point_column=column;
};

void spark_vision::halconPickPointsForCalib(cv::Mat image,
                                            int points_num,
                                            double threshold_value,
                                            std::vector<cv::Point2d> & points_coordinate){
    HalconCpp::HObject mark_point_image = spark_vision::cvMatToHobject(image);
    HalconCpp::HTuple window_handle;
    spark_vision::displayHalconImage(mark_point_image, window_handle);
    for (int index=1; index <= points_num; ++index) {
        HalconCpp::HTuple roi_center_row;
        HalconCpp::HTuple roi_center_column;
        HalconCpp::HTuple roi_radius;
        HalconCpp::DrawCircle(window_handle, &roi_center_row, &roi_center_column, &roi_radius);
        HalconCpp::HObject roi_circle;
        HalconCpp::GenCircle(&roi_circle, roi_center_row, roi_center_column, roi_radius);
        HalconCpp::HImage roi_image;
        HalconCpp::ReduceDomain(mark_point_image, roi_circle, &roi_image);
        HalconCpp::HImage roi_gray_image;
        HalconCpp::Rgb1ToGray(roi_image, &roi_gray_image);
        HalconCpp::HRegion threshold_region;
        HalconCpp::Threshold(roi_gray_image, &threshold_region, 0, threshold_value);
        HalconCpp::HRegion fillup_region;
        HalconCpp::FillUp(threshold_region, &fillup_region);
        HalconCpp::HRegion connect_regions;
        HalconCpp::Connection(fillup_region, &connect_regions);
        HalconCpp::HRegion mark_region;
        HalconCpp::SelectShapeStd(connect_regions, &mark_region, "max_area", 70);
        HalconCpp::HTuple mark_point_area;
        HalconCpp::HTuple mark_point_row;
        HalconCpp::HTuple mark_point_column;
        HalconCpp::AreaCenter(mark_region, &mark_point_area, &mark_point_row, &mark_point_column);
        HalconCpp::HXLD target_center;
        HalconCpp::GenCrossContourXld(&target_center, mark_point_row, mark_point_column, 20, 0);
        HalconCpp::DispObj(target_center, HalconCpp::HDevWindowStack::GetActive());
        cv::Point2d mark_coordinate;
        spark_vision::htupleToCvPoint2d(mark_point_row,mark_point_column,mark_coordinate);
        points_coordinate.push_back(mark_coordinate);
    }
};

void spark_vision::txtToVector(const std::string& pathname, std::vector< std::vector< double > >& res)
{
    std::ifstream infile;
    infile.open(pathname.data());   
    assert(infile.is_open()); 
    std::vector<double> suanz;
    std::string s;  // file content
    while (std::getline(infile, s)){
        if(s.empty()){
        continue;
        }
        char *s_input = (char*)s.c_str();
        const char *split = ",";
        char *p = strtok(s_input,split);
        double d;
        while(p != NULL) {
        d = atof(p);
        suanz.push_back(d);
        p = strtok(NULL, split);
        }
        res.push_back(suanz);
        suanz.clear();
        s.clear();
    }
    infile.close();
    return;
}