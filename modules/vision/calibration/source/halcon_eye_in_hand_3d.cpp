#include "halcon_eye_in_hand_3d.hpp"

using eih_calib = spark_vision::CalibrateEyeInHand3D_Halcon;

void eih_calib::readCalibParametersFromYAML(const std::string& calib_param_YAML) {
    YAML::Node reader;
    reader = YAML::LoadFile(calib_param_YAML);
    this->m_calibrateParameters[0] = reader["num_of_images"].as<double>();
    this->m_calibrateParameters[1] = reader["board_dim"].as<double>();
    this->m_calibrateParameters[2] = reader["f"].as<double>();
    this->m_calibrateParameters[3] = reader["cx"].as<double>();
    this->m_calibrateParameters[4] = reader["cy"].as<double>();
    this->m_calibrateParameters[5] = reader["cols"].as<double>();
    this->m_calibrateParameters[6] = reader["rows"].as<double>();
	printf("The read-in camera parameters are : \n");
	for (auto it = reader.begin(); it != reader.end(); it++) {
		std::cout << it->first.as<std::string>() << " : " << it->second.as<double>() << std::endl; 
	}
}

void eih_calib::readRobotPoseFromYAML(const std::string& robot_pose_YAML) {
    YAML::Node reader;
    reader = YAML::LoadFile(robot_pose_YAML);
    // TODO: check if the number of poses in the YAML file if yes, then continue with calibration; if no, then abort and throw error
    if(fabs(static_cast<double>(reader.size()) - m_calibrateParameters[0].D()) > 1e-3) {
        std::cerr << "The specified number of images and the robot yaml have different dimensions." << std::endl;
        return;
    } else {
        int idx = 0;
        HalconCpp::HTuple hv_I;
        HalconCpp::HTuple hv_PoseNum;
        for (YAML::iterator it = reader.begin(); it != reader.end(); it++) {
            hv_I = idx;
			try{
				HalconCpp::GetCalibData(
					m_camCalibrateID,
					"calib_obj_pose",
					HalconCpp::HTuple(0).TupleConcat(hv_I),
					"pose",
					&m_calculateObjInCamPose
				);
			} catch(HalconCpp::HException& e) {
				std::cerr << e.ErrorMessage() << std::endl;
			}

            HalconCpp::SetCalibDataObservPose(
                m_calibrateID,
                0,
                0,
                hv_I,
                m_calculateObjInCamPose
            );

            std::vector<double> current_pose = it->second.as<std::vector<double>>();
			for(int i = 0; i < current_pose.size(); i++) {
                hv_PoseNum[i] = current_pose[i];
            }
            HalconCpp::CreatePose(
                hv_PoseNum[0], 
                hv_PoseNum[1], 
                hv_PoseNum[2], 
                hv_PoseNum[3], 
                hv_PoseNum[4], 
                hv_PoseNum[5], 
                "Rp+T",
                "abg",
                "point",
                &m_toolInBasePose);
            HalconCpp::SetCalibData(
                m_calibrateID,
                "tool",
                hv_I,
                "tool_in_base_pose",
                m_toolInBasePose
            );
			idx++;
        }
    }
	printf("Robot poses successfully created.\n");

}

void eih_calib::calculateCalibrationPlatePoses(const std::string& image_file_path) {
    // Local control variables
    HalconCpp::HTuple  hv_Data, hv_NumImages;
    HalconCpp::HTuple  hv_CalTabFile, hv_f, hv_StartCamParam;
    HalconCpp::HTuple  hv_I, hv_RCoord, hv_CCoord, hv_Index;
    HalconCpp::HTuple  hv_Errors, hv_CamParam, hv_FileHandle;
    HalconCpp::HTuple  hv_PoseStr, hv_IsEOF, hv_PoseSubstrings, hv_PoseNum;
    HalconCpp::HTuple  hv_data, hv_ToolInBasePose, hv_Warnings, hv_PoseErrors;
    HalconCpp::HTuple  hv_ToolInCamPose, hv_ObjInBasePose, hv_Exception;
    HalconCpp::HTuple  hv_Message, hv_ResultErrors;
    HalconCpp::HObject ho_Image;
    //Directories with calibration images and data files
    const char *dir = image_file_path.c_str();
    hv_Data = dir;
    hv_NumImages = m_calibrateParameters[0];
    hv_CalTabFile = "caltab_"+m_calibrateParameters[1]+"mm.descr";
    hv_f = m_calibrateParameters[2]*8.3e-6;
    hv_StartCamParam[0] = hv_f;
    hv_StartCamParam[1] = 0;
    hv_StartCamParam[2] = 8.3e-6;
    hv_StartCamParam[3] = 8.3e-6;
    hv_StartCamParam[4] = m_calibrateParameters[3];
    hv_StartCamParam[5] = m_calibrateParameters[4];
    hv_StartCamParam[6] = m_calibrateParameters[5];
    hv_StartCamParam[7] = m_calibrateParameters[6];
    // int ImageNums = hv_NumImages;
    CreateCalibData("calibration_object", 1, 1, &m_camCalibrateID);
    //Set the camera type used
    SetCalibDataCamParam(m_camCalibrateID, 0, HalconCpp::HTuple(), hv_StartCamParam);
    //Set the calibration object
    SetCalibDataCalibObject(m_camCalibrateID, 0, hv_CalTabFile);

    //Extract the calibration object points
    { //why is this performed in a code block
    HalconCpp::HTuple end_val28 = hv_NumImages-1;
    HalconCpp::HTuple step_val28 = 1;
    for (hv_I=0; hv_I.Continue(end_val28, step_val28); hv_I += step_val28)
    {
        ReadImage(&ho_Image, (hv_Data + hv_I+ "_Color")+".png");
        //Image Acquisition 01: Do something
        //Search for the calibration plate
        FindCalibObject(ho_Image, m_camCalibrateID, 0, 0, hv_I, HalconCpp::HTuple(), HalconCpp::HTuple());
        GetCalibDataObservPoints(m_camCalibrateID, 0, 0, hv_I, &hv_RCoord, &hv_CCoord, 
            &hv_Index, &m_calculateObjInCamPose);
    }
    }

    CalibrateCameras(m_camCalibrateID, &hv_Errors); // Does the calibration of camera intrinsics happen here?
    GetCalibData(m_camCalibrateID, "camera", 0, "params", &hv_CamParam); // Get the calibration data
    CreateCalibData("hand_eye_moving_cam", 0, 0, &m_calibrateID);
    SetCalibData(m_calibrateID, "model", "general", "optimization_method", "nonlinear");
	printf("Done with plane object location and camera intrinsics calibration.\n");
}

void eih_calib::calibrateAndSaveResult(const std::string& result_path) {
    HalconCpp::HTuple hv_Warnings, hv_PoseErrors, hv_ObjInBasePose, hv_ToolInBasePose, hv_Exception, toolInCamPose;
    HalconCpp::HTuple calibrateResultPath;
    const char *dir = result_path.c_str();
    calibrateResultPath = dir;
    //Check the input poses for consistency
    this->checkInputposes(m_calibrateID, 0.005, 0.008, &hv_Warnings);
    if (0 != (int((hv_Warnings.TupleLength())!=0)))
    {
    // dev_inspect_ctrl(...); only in hdevelop
    // stop(...); only in hdevelop
    }
    //
    //Perform the hand eye calibration and store the results to file
    CalibrateHandEye(m_calibrateID, &hv_PoseErrors);
    //Get poses computed by the hand eye calibration
    GetCalibData(m_calibrateID, "camera", 0, "tool_in_cam_pose", &toolInCamPose);
    GetCalibData(m_calibrateID, "calib_obj", 0, "obj_in_base_pose", &hv_ObjInBasePose);
    GetCalibData(m_calibrateID, "tool", 0, "tool_in_base_pose", &hv_ToolInBasePose);
    PoseToHomMat3d(toolInCamPose, &m_toolInCamPose);
    m_toolInCamPose[12] = 0;
    m_toolInCamPose[13] = 0;
    m_toolInCamPose[14] = 0;
    m_toolInCamPose[15] = 1;
    try
    {
    //Save the hand eye calibration results to file
        WriteTuple(m_toolInCamPose, calibrateResultPath + "ToolInCamPose.dat");
    }
    // catch (Exception) 
    catch (HalconCpp::HException &HDevExpDefaultException)
    {
    HDevExpDefaultException.ToHTuple(&hv_Exception);
    //Do nothing
    }
    m_resultErrors[0] = HalconCpp::HTuple(hv_PoseErrors[0]);
    m_resultErrors[1] = HalconCpp::HTuple(hv_PoseErrors[1]);
    WriteTuple(m_resultErrors, calibrateResultPath+" ResultErrors.dat");
	printf("Done calibrating hand to eye transformation. \n Caution: This is the hand to eye calibration, use the inverse for eye to hand calibration.\n");
}

void eih_calib::checkInputposes(
    HalconCpp::HTuple hv_CalibDataID, 
    HalconCpp::HTuple hv_RotationTolerance,
	HalconCpp::HTuple hv_TranslationTolerance, 
    HalconCpp::HTuple *hv_Warnings) {
	// Local iconic variables

	// Local control variables
	HalconCpp::HTuple  hv_MinLargeRotationFraction, hv_MinLargeAnglesFraction;
	HalconCpp::HTuple  hv_StdDevFactor, hv_Type, hv_Exception, hv_IsHandEyeScara;
	HalconCpp::HTuple  hv_IsHandEyeArticulated, hv_NumCameras, hv_NumCalibObjs;
	HalconCpp::HTuple  hv_I1, hv_PosesIdx, hv_RefCalibDataID, hv_UseTemporaryCopy;
	HalconCpp::HTuple  hv_CamPoseCal, hv_SerializedItemHandle, hv_TmpCalibDataID;
	HalconCpp::HTuple  hv_Error, hv_Index, hv_CamDualQuatCal, hv_BasePoseTool;
	HalconCpp::HTuple  hv_BaseDualQuatTool, hv_NumCalibrationPoses, hv_LX2s;
	HalconCpp::HTuple  hv_LY2s, hv_LZ2s, hv_TranslationToleranceSquared;
	HalconCpp::HTuple  hv_RotationToleranceSquared, hv_Index1, hv_CamDualQuatCal1;
	HalconCpp::HTuple  hv_Cal1DualQuatCam, hv_BaseDualQuatTool1, hv_Tool1DualQuatBase;
	HalconCpp::HTuple  hv_Index2, hv_CamDualQuatCal2, hv_DualQuat1, hv_BaseDualQuatTool2;
	HalconCpp::HTuple  hv_DualQuat2, hv_LX1, hv_LY1, hv_LZ1, hv_MX1, hv_MY1;
	HalconCpp::HTuple  hv_MZ1, hv_Rot1, hv_Trans1, hv_LX2, hv_LY2, hv_LZ2;
	HalconCpp::HTuple  hv_MX2, hv_MY2, hv_MZ2, hv_Rot2, hv_Trans2, hv_MeanRot;
	HalconCpp::HTuple  hv_MeanTrans, hv_SinTheta2, hv_CosTheta2, hv_SinTheta2Squared;
	HalconCpp::HTuple  hv_CosTheta2Squared, hv_ErrorRot, hv_StdDevQ0, hv_ToleranceDualQuat0;
	HalconCpp::HTuple  hv_ErrorDualQuat0, hv_StdDevQ4, hv_ToleranceDualQuat4;
	HalconCpp::HTuple  hv_ErrorDualQuat4, hv_Message, hv_NumPairs, hv_NumPairsMax;
	HalconCpp::HTuple  hv_LargeRotationFraction, hv_NumPairPairs, hv_NumPairPairsMax;
	HalconCpp::HTuple  hv_Angles, hv_Idx, hv_LXA, hv_LYA, hv_LZA, hv_LXB;
	HalconCpp::HTuple  hv_LYB, hv_LZB, hv_ScalarProduct, hv_LargeAngles;
	HalconCpp::HTuple  hv_LargeAnglesFraction;
	HalconCpp::HTupleVector  hvec_CamDualQuatsCal(1), hvec_BaseDualQuatsTool(1);

	//This procedure checks the hand-eye calibration input poses that are stored in
	//the calibration data model CalibDataID for consistency.
	//
	//For this check, it is necessary to know the accuracy of the input poses.
	//Therefore, the RotationTolerance and TranslationTolerance must be
	//specified that approximately describe the error in the rotation and in the
	//translation part of the input poses, respectively. The rotation tolerance must
	//be passed in RotationTolerance in radians. The translation tolerance must be
	//passed in TranslationTolerance in the same unit in which the input poses were
	//given, i.e., typically in meters. Therefore, the more accurate the
	//input poses are, the lower the values for RotationTolerance and
	//TranslationTolerance should be chosen. If the accuracy of the robot's tool
	//poses is different from the accuracy of the calibration object poses, the
	//tolerance values of the poses with the lower accuracy (i.e., the higher
	//tolerance values) should be passed.

	//Typically, check_hand_eye_calibration_input_poses is called after all
	//calibration poses have been set in the calibration data model and before the
	//hand eye calibration is performed. The procedure checks all pairs of robot
	//tool poses and compares them to the corresponding pair of calibration object
	//poses. For each inconsistent pose pair, a string is returned in Warnings that
	//indicates the inconsistent pose pair. For larger values for RotationTolerance
	//or TranslationTolerance, i.e., for less accurate input poses, fewer warnings
	//will be generated because the check is more tolerant, and vice versa. The
	//procedure is also helpful if the errors that are returned by the hand-eye
	//calibration are larger than expected to identify potentially erroneous poses.
	//Note that it is not possible to check the consistency of a single pose but
	//only of pose pairs. Nevertheless, if a certain pose occurs multiple times in
	//different warning messages, it is likely that the pose is erroneous.
	//Erroneous poses that result in inconsistent pose pairs should removed
	//from the calibration data model by using remove_calib_data_observ and
	//remove_calib_data before performing the hand-eye calibration.
	//
	//check_hand_eye_calibration_input_poses also checks whether enough calibration
	//pose pairs are passed with a significant relative rotation angle, which
	//is necessary for a robust hand-eye calibration.
	//
	//check_hand_eye_calibration_input_poses also verifies that the correct
	//calibration model was chosen in create_calib_data. If a model of type
	//'hand_eye_stationary_cam' or 'hand_eye_moving_cam' was chosen, the calibration
	//of an articulated robot is assumed. For 'hand_eye_scara_stationary_cam' or
	//'hand_eye_scara_moving_cam', the calibration of a SCARA robot is assumed.
	//Therefore, if all input poses for an articulated robot are parallel or if some
	//robot poses for a SCARA robot are tilted, a corresponding message is returned
	//in Warnings. Furthermore, if the number of tilted input poses for articulated
	//robots is below a certain value, a corresponding message in Warnings indicates
	//that the accuracy of the result of the hand-eye calibration might be low.
	//
	//If no problems have been detected in the input poses, an empty tuple is
	//returned in Warnings.
	//
	//
	//Define the minimum fraction of pose pairs with a rotation angle exceeding
	//2*RotationTolerance.
	hv_MinLargeRotationFraction = 0.1;
	//Define the minimum fraction of screw axes pairs with an angle exceeding
	//2*RotationTolerance for articulated robots.
	hv_MinLargeAnglesFraction = 0.1;
	//Factor that is used to multiply the standard deviations to obtain an error
	//threshold.
	hv_StdDevFactor = 3.0;
	//
	//Check input control parameters.
	if (0 != (int((hv_CalibDataID.TupleLength()) != 1)))
	{
		throw HalconCpp::HException("Wrong number of values of control parameter: 1");
	}
	if (0 != (int((hv_RotationTolerance.TupleLength()) != 1)))
	{
		throw HalconCpp::HException("Wrong number of values of control parameter: 2");
	}
	if (0 != (int((hv_TranslationTolerance.TupleLength()) != 1)))
	{
		throw HalconCpp::HException("Wrong number of values of control parameter: 3");
	}
	try
	{
		GetCalibData(hv_CalibDataID, "model", "general", "type", &hv_Type);
	}
	// catch (Exception) 
	catch (HalconCpp::HException &HDevExpDefaultException)
	{
		HDevExpDefaultException.ToHTuple(&hv_Exception);
		throw HalconCpp::HException("Wrong value of control parameter: 1");
	}
	if (0 != (int(hv_RotationTolerance < 0)))
	{
		throw HalconCpp::HException("Wrong value of control parameter: 2");
	}
	if (0 != (int(hv_TranslationTolerance < 0)))
	{
		throw HalconCpp::HException("Wrong value of control parameter: 3");
	}
	//
	//Read out the calibration data model.
	hv_IsHandEyeScara = HalconCpp::HTuple(int(hv_Type == HalconCpp::HTuple("hand_eye_scara_stationary_cam"))).TupleOr(int(hv_Type == HalconCpp::HTuple("hand_eye_scara_moving_cam")));
	hv_IsHandEyeArticulated = HalconCpp::HTuple(int(hv_Type == HalconCpp::HTuple("hand_eye_stationary_cam"))).TupleOr(int(hv_Type == HalconCpp::HTuple("hand_eye_moving_cam")));
	//This procedure only works for hand-eye calibration applications.
	if (0 != (HalconCpp::HTuple(hv_IsHandEyeScara.TupleNot()).TupleAnd(hv_IsHandEyeArticulated.TupleNot())))
	{
		throw HalconCpp::HException("check_hand_eye_calibration_input_poses only works for hand-eye calibrations");
	}
	GetCalibData(hv_CalibDataID, "model", "general", "num_cameras", &hv_NumCameras);
	GetCalibData(hv_CalibDataID, "model", "general", "num_calib_objs", &hv_NumCalibObjs);
	//
	//Get all valid calibration pose indices.
	QueryCalibDataObservIndices(hv_CalibDataID, "camera", 0, &hv_I1, &hv_PosesIdx);
	hv_RefCalibDataID = hv_CalibDataID;
	hv_UseTemporaryCopy = 0;
	//If necessary, calibrate the interior camera parameters.
	if (0 != hv_IsHandEyeArticulated)
	{
		//For articulated (non-SCARA) robots, we have to check whether the camera
		//is already calibrated. Otherwise, the queried poses might not be very
		//accurate.
		try
		{
			GetCalibData(hv_CalibDataID, "calib_obj_pose", HalconCpp::HTuple(0).TupleConcat(HalconCpp::HTuple(hv_PosesIdx[0])),
				"pose", &hv_CamPoseCal);
		}
		// catch (Exception) 
		catch (HalconCpp::HException &HDevExpDefaultException)
		{
			HDevExpDefaultException.ToHTuple(&hv_Exception);
			if (0 != (HalconCpp::HTuple(int(hv_NumCameras != 0)).TupleAnd(int(hv_NumCalibObjs != 0))))
			{
				//If the interior camera parameters are not calibrated yet, perform
				//the camera calibration by using a temporary copy of the calibration
				//data model.
				SerializeCalibData(hv_CalibDataID, &hv_SerializedItemHandle);
				DeserializeCalibData(hv_SerializedItemHandle, &hv_TmpCalibDataID);
				ClearSerializedItem(hv_SerializedItemHandle);
				hv_RefCalibDataID = hv_TmpCalibDataID;
				hv_UseTemporaryCopy = 1;
				CalibrateCameras(hv_TmpCalibDataID, &hv_Error);
			}
		}
	}
	//Query all robot tool and calibration object poses.
	{
		HalconCpp::HTuple end_val120 = (hv_PosesIdx.TupleLength()) - 1;
		HalconCpp::HTuple step_val120 = 1;
		for (hv_Index = 0; hv_Index.Continue(end_val120, step_val120); hv_Index += step_val120)
		{
			try
			{
				//For an articulated robot with a camera and a calibration object,
				//a calibrated poses should always be available.
				GetCalibData(hv_RefCalibDataID, "calib_obj_pose", HalconCpp::HTuple(0).TupleConcat(HalconCpp::HTuple(hv_PosesIdx[hv_Index])),
					"pose", &hv_CamPoseCal);
			}
			// catch (Exception) 
			catch (HalconCpp::HException &HDevExpDefaultException)
			{
				HDevExpDefaultException.ToHTuple(&hv_Exception);
				//For a SCARA robot or for an articulated robots with a general
				//sensor and no calibration object, directly use the observed poses.
				GetCalibDataObservPose(hv_RefCalibDataID, 0, 0, HalconCpp::HTuple(hv_PosesIdx[hv_Index]),
					&hv_CamPoseCal);
			}
			//Transform the calibration object poses to dual quaternions.
			PoseToDualQuat(hv_CamPoseCal, &hv_CamDualQuatCal);
			hvec_CamDualQuatsCal[hv_Index] = HalconCpp::HTupleVector(hv_CamDualQuatCal);
			//Transform the robot tool pose to dual quaternions.
			GetCalibData(hv_RefCalibDataID, "tool", HalconCpp::HTuple(hv_PosesIdx[hv_Index]), "tool_in_base_pose",
				&hv_BasePoseTool);
			PoseToDualQuat(hv_BasePoseTool, &hv_BaseDualQuatTool);
			hvec_BaseDualQuatsTool[hv_Index] = HalconCpp::HTupleVector(hv_BaseDualQuatTool);
		}
	}
	hv_NumCalibrationPoses = hv_PosesIdx.TupleLength();
	if (0 != hv_UseTemporaryCopy)
	{
		ClearCalibData(hv_TmpCalibDataID);
	}
	//
	//In the first test, check the poses for consistency. The principle of
	//the hand-eye calibration is that the movement of the robot from time
	//i to time j is represented by the relative pose of the calibration
	//object from i to j in the camera coordinate system and also by the
	//relative pose of the robot tool from i to j in the robot base
	//coordinate system. Because both relative poses represent the same 3D
	//rigid transformation, but only seen from two different coordinate
	//systems, their screw axes differ but their screw angle and their
	//screw translation should be identical. This knowledge can be used to
	//check the consistency of the input poses. Furthermore, remember the
	//screw axes for all robot movements to later check whether the
	//correct calibration model (SCARA or articulated) was selected by the
	//user.
	(*hv_Warnings) = HalconCpp::HTuple();
	hv_LX2s = HalconCpp::HTuple();
	hv_LY2s = HalconCpp::HTuple();
	hv_LZ2s = HalconCpp::HTuple();
	hv_TranslationToleranceSquared = hv_TranslationTolerance * hv_TranslationTolerance;
	hv_RotationToleranceSquared = hv_RotationTolerance * hv_RotationTolerance;
	{
		HalconCpp::HTuple end_val162 = hv_NumCalibrationPoses - 2;
		HalconCpp::HTuple step_val162 = 1;
		for (hv_Index1 = 0; hv_Index1.Continue(end_val162, step_val162); hv_Index1 += step_val162)
		{
			hv_CamDualQuatCal1 = hvec_CamDualQuatsCal[hv_Index1].T();
			DualQuatConjugate(hv_CamDualQuatCal1, &hv_Cal1DualQuatCam);
			hv_BaseDualQuatTool1 = hvec_BaseDualQuatsTool[hv_Index1].T();
			DualQuatConjugate(hv_BaseDualQuatTool1, &hv_Tool1DualQuatBase);
			{
				HalconCpp::HTuple end_val167 = hv_NumCalibrationPoses - 1;
				HalconCpp::HTuple step_val167 = 1;
				for (hv_Index2 = hv_Index1 + 1; hv_Index2.Continue(end_val167, step_val167); hv_Index2 += step_val167)
				{
					//For two robot poses, ...
					//... compute the movement of the calibration object in the
					//camera coordinate system.
					hv_CamDualQuatCal2 = hvec_CamDualQuatsCal[hv_Index2].T();
					DualQuatCompose(hv_Cal1DualQuatCam, hv_CamDualQuatCal2, &hv_DualQuat1);
					//
					//... compute the movement of the tool in the robot base
					//coordinate system.
					hv_BaseDualQuatTool2 = hvec_BaseDualQuatsTool[hv_Index2].T();
					DualQuatCompose(hv_Tool1DualQuatBase, hv_BaseDualQuatTool2, &hv_DualQuat2);
					//
					//Check whether the two movements are consistent. If the two
					//movements are consistent, the scalar parts of the corresponding
					//dual quaternions should be equal. For the equality check, we
					//have to take the accuracy of the input poses into account, which
					//are given by RotationTolerance and TranslationTolerance.
					DualQuatToScrew(hv_DualQuat1, "moment", &hv_LX1, &hv_LY1, &hv_LZ1, &hv_MX1,
						&hv_MY1, &hv_MZ1, &hv_Rot1, &hv_Trans1);
					DualQuatToScrew(hv_DualQuat2, "moment", &hv_LX2, &hv_LY2, &hv_LZ2, &hv_MX2,
						&hv_MY2, &hv_MZ2, &hv_Rot2, &hv_Trans2);
					while (0 != (int(hv_Rot1 > (HalconCpp::HTuple(180.0).TupleRad()))))
					{
						hv_Rot1 = hv_Rot1 - (HalconCpp::HTuple(360.0).TupleRad());
					}
					while (0 != (int(hv_Rot2 > (HalconCpp::HTuple(180.0).TupleRad()))))
					{
						hv_Rot2 = hv_Rot2 - (HalconCpp::HTuple(360.0).TupleRad());
					}
					//
					hv_Rot1 = hv_Rot1.TupleFabs();
					hv_Trans1 = hv_Trans1.TupleFabs();
					hv_Rot2 = hv_Rot2.TupleFabs();
					hv_Trans2 = hv_Trans2.TupleFabs();
					hv_MeanRot = 0.5*(hv_Rot1 + hv_Rot2);
					hv_MeanTrans = 0.5*(hv_Trans1 + hv_Trans2);
					hv_SinTheta2 = (0.5*hv_MeanRot).TupleSin();
					hv_CosTheta2 = (0.5*hv_MeanRot).TupleCos();
					hv_SinTheta2Squared = hv_SinTheta2 * hv_SinTheta2;
					hv_CosTheta2Squared = hv_CosTheta2 * hv_CosTheta2;
					//
					//1. Check the scalar part of the real part of the dual quaternion,
					//which encodes the rotation component of the screw:
					//  q[0] = cos(theta/2)
					//Here, theta is the screw rotation angle.
					hv_ErrorRot = (hv_Rot1 - hv_Rot2).TupleFabs();
					while (0 != (int(hv_ErrorRot > (HalconCpp::HTuple(180.0).TupleRad()))))
					{
						hv_ErrorRot = hv_ErrorRot - (HalconCpp::HTuple(360.0).TupleRad());
					}
					hv_ErrorRot = hv_ErrorRot.TupleFabs();
					//Compute the standard deviation of the scalar part of the real part
					//by applying the law of error propagation.
					hv_StdDevQ0 = (0.5*hv_SinTheta2)*hv_RotationTolerance;
					//Multiply the standard deviation by a factor to increase the certainty.
					hv_ToleranceDualQuat0 = hv_StdDevFactor * hv_StdDevQ0;
					hv_ErrorDualQuat0 = ((HalconCpp::HTuple(hv_DualQuat2[0]).TupleFabs()) - (HalconCpp::HTuple(hv_DualQuat1[0]).TupleFabs())).TupleFabs();
					//
					//2. Check the scalar part of the dual part of the dual quaternion,
					//which encodes translation and rotation components of the screw:
					//  q[4] = -d/2*sin(theta/2)
					//Here, d is the screw translation.
					//
					//Compute the standard deviation of the scalar part of the dual part
					//by applying the law of error propagation.
					hv_StdDevQ4 = (((0.25*hv_SinTheta2Squared)*hv_TranslationToleranceSquared) + ((((0.0625*hv_MeanTrans)*hv_MeanTrans)*hv_CosTheta2Squared)*hv_RotationToleranceSquared)).TupleSqrt();
					//Multiply the standard deviation by a factor to increase the certainty.
					hv_ToleranceDualQuat4 = hv_StdDevFactor * hv_StdDevQ4;
					hv_ErrorDualQuat4 = ((HalconCpp::HTuple(hv_DualQuat2[4]).TupleFabs()) - (HalconCpp::HTuple(hv_DualQuat1[4]).TupleFabs())).TupleFabs();
					//If one of the two errors exceeds the computed thresholds, return
					//a warning for the current pose pair.
					if (0 != (HalconCpp::HTuple(int(hv_ErrorDualQuat0 > hv_ToleranceDualQuat0)).TupleOr(int(hv_ErrorDualQuat4 > hv_ToleranceDualQuat4))))
					{
						hv_Message = ((("Inconsistent pose pair (" + (HalconCpp::HTuple(hv_PosesIdx[hv_Index1]).TupleString("2d"))) + HalconCpp::HTuple(",")) + (HalconCpp::HTuple(hv_PosesIdx[hv_Index2]).TupleString("2d"))) + ")";
						(*hv_Warnings) = (*hv_Warnings).TupleConcat(hv_Message);
					}
					//
					//Remember the screw axes (of the robot tool movements) for screws
					//with a significant rotation part. For movements without rotation
					//the direction of the screw axis is determined by the translation
					//part only. Hence, the direction of the screw axis cannot be used
					//to decide whether an articulated or a SCARA robot is used.
					if (0 != (int(hv_Rot2 > (hv_StdDevFactor*hv_RotationTolerance))))
					{
						hv_LX2s = hv_LX2s.TupleConcat(hv_LX2);
						hv_LY2s = hv_LY2s.TupleConcat(hv_LY2);
						hv_LZ2s = hv_LZ2s.TupleConcat(hv_LZ2);
					}
				}
			}
		}
	}
	//
	//In the second test, we check whether enough calibration poses with a
	//significant rotation part are available for calibration.
	hv_NumPairs = hv_LX2s.TupleLength();
	hv_NumPairsMax = (hv_NumCalibrationPoses*(hv_NumCalibrationPoses - 1)) / 2;
	if (0 != (int(hv_NumPairs < 2)))
	{
		hv_Message = "There are not enough rotated calibration poses available.";
		(*hv_Warnings) = (*hv_Warnings).TupleConcat(hv_Message);
		//In this case, we can skip further test.
		return;
	}
	hv_LargeRotationFraction = (hv_NumPairs.TupleReal()) / hv_NumPairsMax;
	if (0 != (HalconCpp::HTuple(int(hv_NumPairs < 4)).TupleOr(int(hv_LargeRotationFraction < hv_MinLargeRotationFraction))))
	{
		hv_Message = HalconCpp::HTuple("Only few rotated robot poses available, which might result in a reduced accuracy of the calibration results.");
		(*hv_Warnings) = (*hv_Warnings).TupleConcat(hv_Message);
	}
	//
	//In the third test, we compute the angle between the screw axes with
	//a significant rotation part. For SCARA robots, this angle must be 0 in
	//all cases. For articulated robots, for a significant fraction of robot
	//poses, this angle should exceed a certain threshold. For this test, we
	//use the robot tool poses as they are assumed to be more accurate than the
	//calibration object poses.
	hv_NumPairPairs = (hv_NumPairs*(hv_NumPairs - 1)) / 2;
	hv_NumPairPairsMax = (hv_NumPairsMax*(hv_NumPairsMax - 1)) / 2;
	hv_Angles = HalconCpp::HTuple(hv_NumPairPairs, 0);
	hv_Idx = 0;
	{
		HalconCpp::HTuple end_val277 = hv_NumPairs - 2;
		HalconCpp::HTuple step_val277 = 1;
		for (hv_Index1 = 0; hv_Index1.Continue(end_val277, step_val277); hv_Index1 += step_val277)
		{
			hv_LXA = HalconCpp::HTuple(hv_LX2s[hv_Index1]);
			hv_LYA = HalconCpp::HTuple(hv_LY2s[hv_Index1]);
			hv_LZA = HalconCpp::HTuple(hv_LZ2s[hv_Index1]);
			{
				HalconCpp::HTuple end_val281 = hv_NumPairs - 1;
				HalconCpp::HTuple step_val281 = 1;
				for (hv_Index2 = hv_Index1 + 1; hv_Index2.Continue(end_val281, step_val281); hv_Index2 += step_val281)
				{
					hv_LXB = HalconCpp::HTuple(hv_LX2s[hv_Index2]);
					hv_LYB = HalconCpp::HTuple(hv_LY2s[hv_Index2]);
					hv_LZB = HalconCpp::HTuple(hv_LZ2s[hv_Index2]);
					//Compute the scalar product, i.e. the cosine of the screw
					//axes. To obtain valid values, crop the cosine to the
					//interval [-1,1].
					hv_ScalarProduct = ((((((hv_LXA*hv_LXB) + (hv_LYA*hv_LYB)) + (hv_LZA*hv_LZB)).TupleConcat(1)).TupleMin()).TupleConcat(-1)).TupleMax();
					//Compute the angle between the axes in the range [0,pi/2].
					hv_Angles[hv_Idx] = (hv_ScalarProduct.TupleFabs()).TupleAcos();
					hv_Idx += 1;
				}
			}
		}
	}
	//Large angles should significantly exceed the RotationTolerance.
	hv_LargeAngles = (hv_Angles.TupleGreaterElem(hv_StdDevFactor*hv_RotationTolerance)).TupleSum();
	//Calculate the fraction of pairs of movements, i.e., pairs of pose
	//pairs, that have a large angle between their corresponding screw
	//axes.
	hv_LargeAnglesFraction = (hv_LargeAngles.TupleReal()) / hv_NumPairPairsMax;
	//For SCARA robots, all screw axes should be parallel, i.e., no
	//two screw axes should have a large angle.
	if (0 != (hv_IsHandEyeScara.TupleAnd(int(hv_LargeAngles > 0))))
	{
		hv_Message = HalconCpp::HTuple("The robot poses indicate that this might be an articulated robot, although a SCARA robot was selected in the calibration data model.");
		(*hv_Warnings) = (*hv_Warnings).TupleConcat(hv_Message);
	}
	//For articulated robots, the screw axes should have a large
	//angles.
	if (0 != hv_IsHandEyeArticulated)
	{
		if (0 != (int(hv_LargeAngles == 0)))
		{
			//If there is no pair of movements with a large angle between
			//their corresponding screw axes, this might be a SCARA robot.
			hv_Message = HalconCpp::HTuple("The robot poses indicate that this might be a SCARA robot (no tilted robot poses available), although an articulated robot was selected in the calibration data model.");
			(*hv_Warnings) = (*hv_Warnings).TupleConcat(hv_Message);
		}
		else if (0 != (int(hv_LargeAngles < 3)))
		{
			//If there are at most 2 movements with a large angle between
			//their corresponding screw axes, the calibration might be
			//unstable.
			hv_Message = "Not enough tilted robot poses available for an accurate calibration of an articulated robot.";
			(*hv_Warnings) = (*hv_Warnings).TupleConcat(hv_Message);
		}
		else if (0 != (int(hv_LargeAnglesFraction < hv_MinLargeAnglesFraction)))
		{
			//If there is only a low fraction of pairs of movements with
			//a large angle between their corresponding screw axes, the
			//accuracy of the calibration might be low.
			hv_Message = HalconCpp::HTuple("Only few tilted robot poses available, which might result in a reduced accuracy of the calibration results.");
			(*hv_Warnings) = (*hv_Warnings).TupleConcat(hv_Message);
		}
	}
	return;
}

Eigen::Matrix4d eih_calib::readCalibrateResult(const std::string& result_path) {
	HalconCpp::HTuple Calibrate_ResultPath,Calibrate_Matrix;
	const char* dir = result_path.c_str();
	Calibrate_ResultPath = dir;
	HalconCpp::ReadTuple(Calibrate_ResultPath + "ToolInCamPose.dat", &Calibrate_Matrix);
	int count = 0;
	for (int i = 0;i < 4;i++)
	{
		for (int j=0;j<4;j++)
		{
			m_calibrateMatrix(i,j) = Calibrate_Matrix[count].D();
			count++;
		}
	}
	return m_calibrateMatrix;
}
