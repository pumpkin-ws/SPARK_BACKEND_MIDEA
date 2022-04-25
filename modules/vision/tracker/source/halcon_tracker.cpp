#include "halcon_tracker.hpp"

void spark_vision::tracker_2d::HalconTemplateTracker_v0::selectTemplateInteractive(std::string template_name) {
    /**
     * @brief close all windows if any is open
     * 
     */
    if(HalconCpp::HDevWindowStack::IsOpen()) {
        HalconCpp::CloseWindow(HalconCpp::HDevWindowStack::Pop());
    }
    spark_vision::displayHalconImage(this->m_ho_model_image, this->m_ht_window_handle);
    HalconCpp::SetWindowParam(this->m_ht_window_handle, "window_title", "Interactive Template Selection");

    HalconCpp::DrawRectangle2(this->m_ht_window_handle, &this->m_ht_model_rows, &this->m_ht_model_cols, 
                              &this->m_ht_model_phi, &this->m_ht_model_length, &this->m_ht_model_width);

    HalconCpp::GenRectangle2(&this->m_ho_ROI_rect, this->m_ht_model_rows,
                             this->m_ht_model_cols, this->m_ht_model_phi,
                             this->m_ht_model_length, this->m_ht_model_width);
    // HalconCpp::GaussFilter(this->m_ho_model_image, &this->ho_image_gauss, 5);

    // CAUTION: it is very easy to make run time mistake with halcon's unspecific type programming, OpenCV is much more friendly to programmers
    // 1 - preprocessing of the images
    HalconCpp::HTuple em_height{7}, em_width{7}, em_factor{1.0};
    HalconCpp::Emphasize(this->m_ho_model_image, &this->m_ho_image_emphasis, em_height, em_width, em_factor);
    
    // adjust the illumination of the image to make the image shine more consistently
    HalconCpp::HTuple il_width{20}, il_height{20}, il_factor{0.55};
    HalconCpp::Illuminate(this->m_ho_image_emphasis, &this->m_ho_image_illuminate, il_width, il_height, il_factor);   

    HalconCpp::HTuple shock_theta{0.5}, shock_iterations{5}, shock_mode{"canny"} , shock_sigma{1.0};
    HalconCpp::ShockFilter(this->m_ho_image_illuminate, &this->m_ho_image_shocked, shock_theta, shock_iterations, shock_mode, shock_sigma); 
    
    HalconCpp::ReduceDomain(this->m_ho_image_shocked, this->m_ho_ROI_rect, &this->m_ho_image_reduced);
    // spark_vision::displayHalconImage(this->m_ho_image_reduced, this->m_ht_window_handle);

    /**
     * @brief define the variables here used for creating the anisoshape model, 
     * as it is very confusing to have "ghost" constants passing in as function arguments
     * The anisotropic shape creation is the key to the track model generation process
     */
    const HalconCpp::HTuple num_levels = "auto"; // the number of pyramid levels to generate 
    const HalconCpp::HTuple angle_start = HalconCpp::HTuple(0).TupleRad(); 
    const HalconCpp::HTuple angel_extent = HalconCpp::HTuple(360).TupleRad(); 
    const HalconCpp::HTuple angle_step = "auto";
    const HalconCpp::HTuple scale_rmin = 0.5;
    const HalconCpp::HTuple scale_rmax = 1.5;
    const HalconCpp::HTuple scale_rstep = "auto";
    const HalconCpp::HTuple scale_cmin = 0.5;
    const HalconCpp::HTuple scale_cmax = 1.5;
    const HalconCpp::HTuple scale_cstep = "auto";
    const HalconCpp::HTuple optimization = "auto";
    const HalconCpp::HTuple metric = "use_polarity";
    const HalconCpp::HTuple contrast = "auto"; 
    const HalconCpp::HTuple min_contrast = "auto";

    try{
        HalconCpp::CreateAnisoShapeModel(
            this->m_ho_image_reduced, 
            num_levels, 
            angle_start,
            angel_extent, 
            angle_step, 
            scale_rmin, 
            scale_rmax, 
            scale_rstep, 
            scale_cmin, 
            scale_cmax,
            scale_cstep, 
            optimization, 
            metric, 
            contrast, 
            min_contrast, 
            &this->m_ht_modelID); 
        this->m_has_template = true;
        // save the anisotropic shape mode
        // check for directory if it exists
        std::string home = getSparkDir();
        std::string template_dir = home + "/vision/track_template/2d/" + template_name + "/";
        spark_filesystem::createDirectory(template_dir);
        // save the template to the directory
        this->saveTemplateToFile(template_dir, template_name);
        // use YAML to save the other parameters
        std::string param_file_name = template_dir + template_name + ".yml";
        cv::FileStorage fs(param_file_name, cv::FileStorage::WRITE);
        fs << "rows" << this->m_ht_model_rows.D();
        fs << "cols" << this->m_ht_model_cols.D();
        fs << "phi" << this->m_ht_model_phi.D();
        fs << "length" << this->m_ht_model_length.D();
        fs << "width" << this->m_ht_model_width.D();
        fs.release();        
        cv::Mat template_image = spark_vision::hobjectToCvMat(this->m_ho_image_reduced);
        std::string reduced_image_path = template_dir + template_name + ".png";
        cv::imwrite(reduced_image_path, template_image);
    } catch (HalconCpp::HException &except) {
        std::cout << except.ErrorMessage() << std::endl;
    }
    // close all halcon windows
    if(HalconCpp::HDevWindowStack::IsOpen()) {
        HalconCpp::CloseWindow(HalconCpp::HDevWindowStack::Pop());
    }
    // set m_has_template to true if template generation is successful
};

std::map<std::string, double> spark_vision::tracker_2d::HalconTemplateTracker_v0::trackObject(HalconCpp::HObject image) {
    /**
     * @brief The msg_pair used as return messages for the track result
     * 
     */
    typedef std::pair<std::string, double> msg_pair;
    std::map<std::string, double> result;

    // check if the object has already generated 
    if(this->m_has_template == false) {
        printf("The track template has not been generated yet!\n");
        result.insert(msg_pair("track_success", 0));
        return result;
    } else {    
        // perform tracking        
        // emphasize the image width, emphasize height, factor
        HalconCpp::HTuple em_height{7}, em_width{7}, em_factor{1};
        HalconCpp::Emphasize(image, &this->m_ho_image_emphasis, em_height, em_width, em_factor);
        
        // adjust the illumination of the image to make the image shine more consistently
        HalconCpp::HTuple il_width{20}, il_height{20}, il_factor{0.55};
        HalconCpp::Illuminate(this->m_ho_image_emphasis, &this->m_ho_image_illuminate, il_width, il_height, il_factor);
        
        // start peforming track
        // first define the variables
        const HalconCpp::HTuple start_angle(HalconCpp::HTuple{0}.TupleRad());
        const HalconCpp::HTuple end_angle(HalconCpp::HTuple{360}.TupleRad());
        const HalconCpp::HTuple scale_r_min{0.5};
        const HalconCpp::HTuple scale_r_max{1.5};
        const HalconCpp::HTuple scale_c_min{0.5};
        const HalconCpp::HTuple scale_c_max{1.5}; 
        const HalconCpp::HTuple max_overlap{0.5};
        const HalconCpp::HTuple sub_pixel{"least_squares"};
        const HalconCpp::HTuple num_levels{0};
        try{

            HalconCpp::FindAnisoShapeModels(this->m_ho_image_illuminate, 
                                            this->m_ht_modelID, 
                                            start_angle, 
                                            end_angle,
                                            scale_r_min,
                                            scale_r_max,
                                            scale_c_min,
                                            scale_c_max,
                                            this->m_ht_min_matching_score,
                                            this->m_ht_num_matches,
                                            max_overlap,
                                            sub_pixel,
                                            num_levels,
                                            this->m_ht_greediness,
                                            &this->m_ht_tracked_row,
                                            &this->m_ht_tracked_col,
                                            &this->m_ht_tracked_angle,
                                            &this->m_ht_tracked_scale_R,
                                            &this->m_ht_tracked_scale_C,
                                            &this->m_ht_tracked_score,
                                            &this->m_ht_tracked_model);
        } catch(HalconCpp::HException& except) {
            std::cout << "An halcon error occured during tracking!" << std::endl;
            std::cout << except.ErrorMessage() << std::endl;
        }
    }
    std::cout << "Trying to store data if successful!" << std::endl;
    std::cout << m_ht_tracked_score.Length() << std::endl;
    if (static_cast<double>(this->m_ht_tracked_score.Length() != 0)) {
        if(static_cast<double>(this->m_ht_tracked_score.D()) > 0) {
            result.insert(msg_pair("row", static_cast<double>(this->m_ht_tracked_row.D())));
            result.insert(msg_pair("col", static_cast<double>(this->m_ht_tracked_col.D())));
            result.insert(msg_pair("angle", static_cast<double>(this->m_ht_tracked_angle.D())));
            result.insert(msg_pair("track_score", static_cast<double>(this->m_ht_tracked_score.D())));
            result.insert(msg_pair("track_success", 1));
            return result;
        } else {
            result.insert(msg_pair("track_success", 0));
            return result;
        }
    } else {
        result.insert(msg_pair("track_success", 0));
        return result;
    }

};

std::map<int, std::map<std::string, double>> spark_vision::tracker_2d::HalconTemplateTracker_v0::trackMultipleObject(cv::Mat image, int number_of_objects) {
    // 1 - convert image to hobject
    HalconCpp::HObject input_image = spark_vision::cvMatToHobject(image);
    // 2 - define message pair
    typedef std::pair<std::string, double> msg_pair;
    typedef std::pair<int, std::map<std::string, double>> result_pair;
    std::map<int, std::map<std::string, double>> final_result;
    if (this->m_has_template == false) {
        printf("The track template has not been generated yet!\n");
        std::map<std::string, double> fail_result;
        fail_result.insert(msg_pair("track_success", 0));
        final_result.insert(result_pair(-1, fail_result));
        return final_result;
    } else {
        // perform tracking
        // 1 - enhance the image: first perform image emphasizing, the image illumination
        HalconCpp::HTuple em_height{7}, em_width{7}, em_factor{1.0};
        HalconCpp::Emphasize(input_image, &this->m_ho_image_emphasis, em_height, em_width, em_factor);
        
        // adjust the illumination of the image to make the image shine more consistently
        HalconCpp::HTuple il_width{20}, il_height{20}, il_factor{0.55};
        HalconCpp::Illuminate(this->m_ho_image_emphasis, &this->m_ho_image_illuminate, il_width, il_height, il_factor);   

        HalconCpp::HTuple shock_theta{0.5}, shock_iterations{5}, shock_mode{"canny"} , shock_sigma{1.0};
        HalconCpp::ShockFilter(this->m_ho_image_illuminate, &this->m_ho_image_shocked, shock_theta, shock_iterations, shock_mode, shock_sigma); 
        
        // HalconCpp::HTuple window_handle;
        // spark_vision::displayHalconImage(this->m_ho_image_shocked, window_handle);

        // 2 - define related parameters
        const HalconCpp::HTuple start_angle(HalconCpp::HTuple{0}.TupleRad());
        const HalconCpp::HTuple end_angle(HalconCpp::HTuple{360}.TupleRad());
        const HalconCpp::HTuple scale_r_min{0.5};
        const HalconCpp::HTuple scale_r_max{1.5};
        const HalconCpp::HTuple scale_c_min{0.5};
        const HalconCpp::HTuple scale_c_max{1.5}; 
        const HalconCpp::HTuple max_overlap{0.5};
        const HalconCpp::HTuple sub_pixel{"least_squares"};
        const HalconCpp::HTuple num_levels{0};
        const HalconCpp::HTuple num_matches{0}; // set the number of matches to be 0 means finding all the available templates

        // 3 - perform tracking
        try {
            HalconCpp::FindAnisoShapeModels(this->m_ho_image_shocked, 
                                            this->m_ht_modelID, 
                                            start_angle, 
                                            end_angle,
                                            scale_r_min,
                                            scale_r_max,
                                            scale_c_min,
                                            scale_c_max,
                                            this->m_ht_min_matching_score,
                                            num_matches,
                                            max_overlap,
                                            sub_pixel,
                                            num_levels,
                                            this->m_ht_greediness,
                                            &this->m_ht_tracked_row,
                                            &this->m_ht_tracked_col,
                                            &this->m_ht_tracked_angle,
                                            &this->m_ht_tracked_scale_R,
                                            &this->m_ht_tracked_scale_C,
                                            &this->m_ht_tracked_score,
                                            &this->m_ht_tracked_model);
        // 4 - perform track result extraction
        printf("The number of items tracked is %d\n", (int)this->m_ht_tracked_score.Length()); // how did I get the number of tracked items here?
        printf("The tuple length of the above data is %d\n", (int)this->m_ht_tracked_score.TupleLength());
        int num_of_results = (int)this->m_ht_tracked_score.Length();
        if (num_of_results != 0) {
            for(int i = 0; i < num_of_results; i++) {
                std::map<std::string, double> this_result;
                this_result.insert(msg_pair("row", static_cast<double>(this->m_ht_tracked_row[i].D())));
                this_result.insert(msg_pair("col", static_cast<double>(this->m_ht_tracked_col[i].D())));
                this_result.insert(msg_pair("angle", static_cast<double>(this->m_ht_tracked_angle[i].D())));
                this_result.insert(msg_pair("width", static_cast<double>(this->m_ht_model_width.D())));
                this_result.insert(msg_pair("length", static_cast<double>(this->m_ht_model_length.D())));
                this_result.insert(msg_pair("phi", static_cast<double>(this->m_ht_model_phi.D())));
                this_result.insert(msg_pair("track_score", static_cast<double>(this->m_ht_tracked_score[i].D())));
                this_result.insert(msg_pair("track_success", 1));
                final_result.insert(result_pair(i, this_result));
            }
            return final_result;
        } else {
            std::map<std::string, double> fail_result;
            fail_result.insert(msg_pair("track_success", 0));
            final_result.insert(result_pair(-1, fail_result));
            return final_result;
        }         

        } catch (HalconCpp::HException& except) {
            LOG_ERROR("An halcon error has occurred during multi-object tracking.");
            std::cout << except.ErrorMessage() << std::endl;
            std::map<std::string, double> fail_result;
            fail_result.insert(msg_pair("track_success", 0));
            final_result.insert(result_pair(-1, fail_result));
            return final_result;
        }
    }
};
// XIAOGE LV
void spark_vision::tracker_2d::HalconTemplateTracker_v0::readTemplateFromFile(std::string template_path,std:: string template_name) {
    try{
        std::string ReadModelPathString = template_path + template_name + "/" + template_name + ".shm";
        const char* ModelPathChar = ReadModelPathString.c_str();
        std::cout << ModelPathChar << std::endl;
        const HalconCpp::HTuple RModelPath(ModelPathChar);
        HalconCpp::ReadShapeModel(RModelPath, &this->m_ht_modelID);

        // read in and set the template geometric parameters as well
        std::string template_yml = template_path + template_name + "/" + template_name + ".yml";
        std::cout << template_yml << std::endl;
        cv::FileStorage fs(template_yml, cv::FileStorage::READ);
        std::cout << "Is the read file open? : " << std::boolalpha << fs.isOpened() << std::endl;
        this->m_ht_model_rows = (double)fs["rows"];
        this->m_ht_model_cols = (double)fs["cols"];
        this->m_ht_model_phi = (double)fs["phi"];
        this->m_ht_model_width = (double)fs["width"];
        this->m_ht_model_length = (double)fs["length"];
        fs.release();  
        this->m_has_template = true;
	}
	catch(HalconCpp::HException& except)
	{
        // set m_has_template to true if template generation is successful
		std::cout << except.ErrorMessage() << std::endl;
	}
}

void spark_vision::tracker_2d::HalconTemplateTracker_v0::saveTemplateToFile(std::string path_name,std:: string template_name) {
	std::string SaveModelPathString = path_name + template_name + ".shm";
	const char* ModelPathChar = SaveModelPathString.c_str();
	const HalconCpp::HTuple SaveModelPath(ModelPathChar);
	// check if the object has already generated 
    if(this->m_has_template == false) {
        printf("The track template has not been generated yet!\n");
		// recreate the new Template
		selectTemplateInteractive(template_name);
		//Save the model
         WriteShapeModel(this->m_ht_modelID, SaveModelPath);
    } else {
		 //Save the model
         WriteShapeModel(this->m_ht_modelID, SaveModelPath);
	}    
}
