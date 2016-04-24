/*****************************************************************************/
/* filename: 	Dist_correct.cpp                                             */
/* author: 		Alexander Treib (TR)                                         */
/* description:	Distortion correction for camera                             */
/*---------------------------------------------------------------------------*/
/* revision history:                                                         */
/* date       | author | modification                                        */
/*____________|________|_____________________________________________________*/
/* 2015-11-20 | TR     | initial revision                                    */
/*****************************************************************************/

#include "Dist_correct.hpp"


//void read(const FileNode& node, Dist_correct& x, const Dist_correct& default_value = Dist_correct())
//{
//	if (node.empty())
//		x = default_value;
//	else
//		x.read(node);
//}

bool runCalibrationAndSave(Dist_correct& s, Size imageSize, Mat&  m_cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2f> > imagePoints);

Dist_correct::Dist_correct(){
	//Initialization of member variables
	if (debugging == "on")
	{
		m_show_undistorted_image = true;
		m_undistorted_output_in_loop = true;
	}
	else
	{
		m_show_undistorted_image = false;
		m_undistorted_output_in_loop = false;
	}
	m_undistorted_delay = 25;	//default value
	m_goodInput = false;
	m_inputSettingsFile = "settings_" + camera_model + ".xml";
	m_mode = m_inputType == Dist_correct::IMAGE_LIST ? CAPTURING : DETECTION;
	m_prevTimestamp = 0;
	m_blinkOutput = false;
	m_readCameraParams = 0;
	m_readin_set = 0;
}

void Dist_correct::information()
{
	cout << "Use input \"default.xml\" to recalibrate" << endl
		<< "Use input \"calibration_data_F100.xml\" for the Widecam F100" << endl;
}

void Dist_correct::write(FileStorage& fs) const                        //Write settings 
{
	fs << "{" << "BoardSize_Width" << m_boardSize.width
		<< "BoardSize_Height" << m_boardSize.height
		<< "Square_Size" << m_squareSize
		<< "Calibrate_Pattern" << m_patternToUse
		<< "Calibrate_NrOfFrameToUse" << m_nrFrames
		<< "Calibrate_FixAspectRatio" << m_aspectRatio
		<< "Calibrate_AssumeZeroTangentialDistortion" << m_calibZeroTangentDist
		<< "Calibrate_FixPrincipalPointAtTheCenter" << m_calibFixPrincipalPoint

		<< "Write_DetectedFeaturePoints" << m_bwritePoints
		<< "Write_extrinsicParameters" << m_bwriteExtrinsics
		<< "Write_outputFileName" << m_outputFileName

		<< "Show_UndistortedImage" << m_showUndistorsed

		<< "Input_FlipAroundHorizontalAxis" << m_flipVertical
		<< "Input_Delay" << m_delay
		<< "Input" << m_input
		<< "}";
}

void Dist_correct::readSettings(const FileNode& node)                          //Read settings from the given node
{
	node["BoardSize_Width"] >> m_boardSize.width;
	node["BoardSize_Height"] >> m_boardSize.height;
	node["Calibrate_Pattern"] >> m_patternToUse;
	node["Square_Size"] >> m_squareSize;
	node["Calibrate_NrOfFrameToUse"] >> m_nrFrames;
	node["Calibrate_FixAspectRatio"] >> m_aspectRatio;
	node["Write_DetectedFeaturePoints"] >> m_bwritePoints;
	node["Write_extrinsicParameters"] >> m_bwriteExtrinsics;
	node["Write_outputFileName"] >> m_outputFileName;
	node["Calibrate_AssumeZeroTangentialDistortion"] >> m_calibZeroTangentDist;
	node["Calibrate_FixPrincipalPointAtTheCenter"] >> m_calibFixPrincipalPoint;
	node["Input_FlipAroundHorizontalAxis"] >> m_flipVertical;
	node["Show_UndistortedImage"] >> m_showUndistorsed;
	node["Input"] >> m_input;
	node["Input_Delay"] >> m_delay;
	node["Framerate"] >> m_framerate;
	interpret();
}

void Dist_correct::interpret()
{
	//is just called at the readSettings() function to check whether the input is valid
	m_goodInput = true;
	if (m_boardSize.width <= 0 || m_boardSize.height <= 0)
	{
		cerr << "Invalid Board size: " << m_boardSize.width << " " << m_boardSize.height << endl;
		m_goodInput = false;
	}
	if (m_squareSize <= 10e-6)
	{
		cerr << "Invalid square size " << m_squareSize << endl;
		m_goodInput = false;
	}
	if (m_nrFrames <= 0)
	{
		cerr << "Invalid number of frames " << m_nrFrames << endl;
		m_goodInput = false;
	}

	if (m_input.empty())      // Check for valid input
		m_inputType = INVALID;
	else
	{
		if (m_input[0] >= '0' && m_input[0] <= '9')
		{
			stringstream ss(m_input);
			ss >> m_cameraID;
			m_inputType = CAMERA;
		}
		else
		{
			if (readStringList(m_input, m_imageList))
			{
				m_inputType = IMAGE_LIST;
				m_nrFrames = (m_nrFrames < (int)m_imageList.size()) ? m_nrFrames : (int)m_imageList.size();
			}
			else
				m_inputType = VIDEO_FILE;
		}
		if (m_inputType == CAMERA)
			m_inputCapture.open(m_cameraID);
		if (m_inputType == VIDEO_FILE)
			m_inputCapture.open(m_input);
		if (m_inputType != IMAGE_LIST && !m_inputCapture.isOpened())
			m_inputType = INVALID;
	}
	if (m_inputType == INVALID)
	{
		cerr << " Inexistent input: " << m_input;
		m_goodInput = false;
	}

	m_flag = 0;
	if (m_calibFixPrincipalPoint) m_flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
	if (m_calibZeroTangentDist)   m_flag |= CV_CALIB_ZERO_TANGENT_DIST;
	if (m_aspectRatio)            m_flag |= CV_CALIB_FIX_ASPECT_RATIO;


	m_calibrationPattern = NOT_EXISTING;
	if (!m_patternToUse.compare("CHESSBOARD")) m_calibrationPattern = CHESSBOARD;
	if (!m_patternToUse.compare("CIRCLES_GRID")) m_calibrationPattern = CIRCLES_GRID;
	if (!m_patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) m_calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
	if (m_calibrationPattern == NOT_EXISTING)
	{
		cerr << " Inexistent camera calibration mode: " << m_patternToUse << endl;
		m_goodInput = false;
	}
	m_atImageList = 0;

}

Mat Dist_correct::nextImage()
{
	//returns a Matrix with the actual input of the camera
	if (m_inputCapture.isOpened())
	{
		m_inputCapture >> m_temp;
	}
	else if (m_atImageList < (int)m_imageList.size())
		m_temp = imread(m_imageList[m_atImageList++], CV_LOAD_IMAGE_COLOR);

	return m_temp;
}

bool Dist_correct::readStringList(const string& filename, vector<string>& l)
{
	//Simplification for the function interpret()
	l.clear();
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}

bool Dist_correct::readin()
{
	//overall function to read the settings from the settings_'cameramodel'.xml file
	FileStorage fs(m_inputSettingsFile, FileStorage::READ); // Read the Settings of the model;

	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << m_inputSettingsFile << "\"" << endl;
		return 0;
	}

	readSettings(fs["Settings"]);					   	//accessing through operator[] with nodename "Settings" ->read 
	fs.release();                                         // close Dist file

	if (!m_goodInput)
	{
		cout << "invalid input detected. application stopping. " << endl;
		return 0;
	}

	m_undistorted_delay = (int)(1000 / m_framerate);
	m_readin_set = true;
	return 1;
}

int Dist_correct::readCameraParams()
{
	//reading camera_calibration_'cameramodel'.xml if previous camaera_calibration is existing
	if (m_readin_set == 0)
	{
		readin();
	}
	if (m_readin_set = 1)
	{
		FileStorage fs_calibrated("camera_calibration_" + camera_model + ".xml", FileStorage::READ);
		if (!fs_calibrated.isOpened())
		{
			cout << "Could not open the configuration file: \"" << m_inputSettingsFile << "\"" << endl;
			m_readCameraParams = 0;
			return 0;
		}
		fs_calibrated["image_Width"] >> m_imageSize.width;
		fs_calibrated["image_Height"] >> m_imageSize.height;
		fs_calibrated["Camera_Matrix"] >> m_cameraMatrix;
		fs_calibrated["Distortion_Coefficients"] >> m_distCoeffs;
		fs_calibrated["Extrinsic_Parameters"] >> m_extrinsic_params;

		m_readCameraParams = 1;
		return 1;
	}
	else{
		m_readCameraParams = 0;
		cout << "Error in readCameraParams" << endl;
		return 0;
	}
}

bool Dist_correct::undistortion_gray(Mat &output)
{
	//Dewarp the input image and convert it to a gray image
	if (m_readCameraParams == 0)
	{
		readCameraParams();
	}
	//no else if here! -> after successfully executing readCameraParams m_readCameraParams should be 1
	if (m_readCameraParams == 1)
	{
		do	//do at least once, if m_undistored_output_in_loop=1 -> referenz to output will be updated every m_undistorted_delay
		{
			m_view = nextImage();
			undistort(m_view, m_undistorted_image, m_cameraMatrix, m_distCoeffs);
			cvtColor(m_undistorted_image, output, COLOR_BGR2GRAY);

			//Display image in a loop (doesn't wait -> always ready to provide an image)
			if (m_show_undistorted_image == 1)
			{
				imshow("undistorted gray image", output);
				imshow("original", m_view);
				waitKey(m_undistorted_delay);
			}

		} while (m_undistorted_output_in_loop);
		return 1;
	}
	else
	{
		cout << "Error in undistortion_gray()...please check the camera_calibration_'model'.xml" << endl;
		return 0;
	}
}

bool Dist_correct::camera_calibration()
{
	//returns 1 for a calibration successfully completed and 0 for abortion during capturing

	char key = '0';
	readin();
	//no else if here! -> after successfully executing readCameraParams m_readCameraParams should be 1
	m_blinkOutput = false;
	while (!((key == 'b') && (m_mode == CALIBRATED)))
	{
		m_view = nextImage();
		//Is called at the end of CAPTURING; if enough Images are available 
		if (m_mode == CAPTURING && m_imagePoints.size() >= (unsigned)m_nrFrames)
		{
			m_blinkOutput = false;
			if (runCalibrationAndSave(*this, m_imageSize, m_cameraMatrix, m_distCoeffs, m_imagePoints))
				m_mode = CALIBRATED;
			else
				m_mode = DETECTION;
		}
		if (m_view.empty())          // If no more images then run calibration, save and stop loop.
		{
			if (m_imagePoints.size() > 0)
				runCalibrationAndSave(*this, m_imageSize, m_cameraMatrix, m_distCoeffs, m_imagePoints);
			return -1;
		}

		m_imageSize = m_view.size();  // Format input image.
		if (m_flipVertical)    
			flip(m_view, m_view, 0);

		vector<Point2f> pointBuf;

		bool found=0;
		switch (m_calibrationPattern) // Find feature points on the input format
		{
		case Dist_correct::CHESSBOARD:
			found = findChessboardCorners(m_view, m_boardSize, pointBuf,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			break;
		case Dist_correct::CIRCLES_GRID:
			found = findCirclesGrid(m_view, m_boardSize, pointBuf);
			break;
		case Dist_correct::ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid(m_view, m_boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
			break;
		default:
			found = false;
			break;
		}

		if (found)                // If done with success,
		{
			// improve the found corners' coordinate accuracy for chessboard
			if (m_calibrationPattern == Dist_correct::CHESSBOARD)
			{
				Mat viewGray;
				cvtColor(m_view, viewGray, COLOR_BGR2GRAY);
				cornerSubPix(viewGray, pointBuf, Size(11, 11),
					Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			}

			if (m_mode == CAPTURING &&  // For camera only take new samples after delay time
				(!m_inputCapture.isOpened() || clock() - m_prevTimestamp > m_delay*1e-3*CLOCKS_PER_SEC))
			{
				m_imagePoints.push_back(pointBuf);
				m_prevTimestamp = clock();
				m_blinkOutput = m_inputCapture.isOpened();
			}

			// Draw the corners.
			drawChessboardCorners(m_view, m_boardSize, Mat(pointBuf), found);
		}

		//----------------------------- Output Text ------------------------------------------------
		string msg = (m_mode == CAPTURING) ? "100/100" :
			m_mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
		int baseLine = 0;
		Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
		Point textOrigin(m_view.cols - 2 * textSize.width - 10, m_view.rows - 2 * baseLine - 10);

		if (m_mode == CAPTURING)
		{
			if (m_showUndistorsed)
				msg = format("%d/%d Undist", (int)m_imagePoints.size(), m_nrFrames);
			else
				msg = format("%d/%d", (int)m_imagePoints.size(), m_nrFrames);
		}

		putText(m_view, msg, textOrigin, 1, 1, m_mode == CALIBRATED ? GREEN : RED);

		if (m_blinkOutput)
			bitwise_not(m_view, m_view);

		//------------------------- Video capture  output  undistorted ------------------------------
		if (m_mode == CALIBRATED && m_showUndistorsed)
		{
			Mat temp = m_view.clone();
			//Transforms (distorted image, output image, m_cameraMatrix, m_distCoeffs
			undistort(temp, m_view, m_cameraMatrix, m_distCoeffs);
		}

		//------------------------------ Show image and check for input commands -------------------
		imshow("Image View", m_view);
		key = (char)waitKey(m_inputCapture.isOpened() ? 50 : m_delay);

		if (key == ESC_KEY)
		{
			if (m_mode == CALIBRATED)
				return 1;
			if ((m_mode == CAPTURING) || (m_mode == DETECTION))
				return 0;
		}

		if (key == 'u' && m_mode == CALIBRATED)
			m_showUndistorsed = !m_showUndistorsed;

		if (m_inputCapture.isOpened() && key == 'g')
		{
			m_mode = CAPTURING;
			m_imagePoints.clear();
		}


		// -----------------------Show the undistorted image for the image list ------------------------
		/*if (calib.obj_dist.m_inputType == Dist_correct::IMAGE_LIST && calib.obj_dist.m_showUndistorsed)
		{
		Mat m_view, rview, map1, map2;
		initUndistortRectifyMap(m_cameraMatrix, m_distCoeffs, Mat(),
		getOptimalNewm_cameraMatrix(m_cameraMatrix, m_distCoeffs, m_imageSize, 1, m_imageSize, 0),
		m_imageSize, CV_16SC2, map1, map2);

		for (int i = 0; i < (int)calib.obj_dist.m_imageList.size(); i++)
		{
		m_view = imread(calib.obj_dist.m_imageList[i], 1);
		if (m_view.empty())
		continue;
		remap(m_view, rview, map1, map2, INTER_LINEAR);
		imshow("Image View", rview);
		char c = (char)waitKey();
		if (c == ESC_KEY || c == 'q' || c == 'Q')
		break;
		}
		}*/
	}
	return 0;
}

bool Dist_correct::go(Mat &output)
{
	//overall function that is called by external classes
	if (calibration == "off")
	{
		undistortion_gray(output);
	}
	if (calibration == "on")
	{
		camera_calibration();
	}
	return 0;
}

//int main(int argc, char* argv[])
//{
//	return 0;
//}

//Compute the mean of the Error
static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
	Dist_correct::Pattern patternType /*= Dist_correct::CHESSBOARD*/)
{
	corners.clear();

	switch (patternType)
	{
	case Dist_correct::CHESSBOARD:
	case Dist_correct::CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; ++i)
			for (int j = 0; j < boardSize.width; ++j)
				corners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
		break;

	case Dist_correct::ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
		break;
	default:
		break;
	}
}

static bool runCalibration(Dist_correct& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs, double& totalAvgErr)
{
	//Initialization of cameraMatrix and distCoeffs
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (s.m_flag & CV_CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = 1.0;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcBoardCornerPositions(s.m_boardSize, s.m_squareSize, objectPoints[0], s.m_calibrationPattern);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	/*Find intrinsic and extrinsic camera parameters*/
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, s.m_flag | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}

// Print camera parameters to the output file
static void saveCameraParams(Dist_correct& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
	double totalAvgErr)
{
	FileStorage fs(s.m_outputFileName, FileStorage::WRITE);

	time_t tm;
	time(&tm);
	struct tm *t2 = localtime(&tm);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_Time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_Width" << imageSize.width;
	fs << "image_Height" << imageSize.height;
	fs << "board_Width" << s.m_boardSize.width;
	fs << "board_Height" << s.m_boardSize.height;
	fs << "square_Size" << s.m_squareSize;

	if (s.m_flag & CV_CALIB_FIX_ASPECT_RATIO)
		fs << "FixAspectRatio" << s.m_aspectRatio;

	if (s.m_flag)
	{
		sprintf(buf, "flags: %s%s%s%s",
			s.m_flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
			s.m_flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
			s.m_flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
			s.m_flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
		cvWriteComment(*fs, buf, 0);

	}

	fs << "flagValue" << s.m_flag;

	fs << "Camera_Matrix" << cameraMatrix;
	fs << "Distortion_Coefficients" << distCoeffs;

	fs << "Avg_Reprojection_Error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
		fs << "Extrinsic_Parameters" << bigmat;
	}

	if (!imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "Image_points" << imagePtMat;
	}
}

bool runCalibrationAndSave(Dist_correct& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs, vector<vector<Point2f> > imagePoints)
{
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
		reprojErrs, totalAvgErr);
	cout << (ok ? "Calibration succeeded" : "Calibration failed")
		<< ". avg re projection error = " << totalAvgErr;

	if (ok)
		saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs,
		imagePoints, totalAvgErr);
	return ok;
}