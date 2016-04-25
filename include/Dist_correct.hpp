/*****************************************************************************/
/* filename: 	Dist_correct.hpp                                             */
/* author: 		Alexander Treib (TR)                                         */
/* description:	Distortion correction for camera                             */
/*---------------------------------------------------------------------------*/
/* revision history:                                                         */
/* date       | author | modification                                        */
/*____________|________|_____________________________________________________*/
/* 2015-11-20 | TR     | initial revision                                    */
/*****************************************************************************/


#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <vector>
//OpenCV Header
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

//modification parameters
const string camera_model = "F100";
const string calibration = "off";
const string debugging = "off";

const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
const char ESC_KEY = 27;
enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };


class Dist_correct
{
public:
	enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
	
	enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };
	Dist_correct();

	//functions
	void information();
	void write(FileStorage& fs) const;
	void readSettings(const FileNode& node);
	void interpret();
	Mat nextImage();
	bool readStringList(const string& filename, vector<string>& l);
	bool readin();
	int readCameraParams();
	bool undistortion_gray(Mat &output);
	bool camera_calibration();
	bool go(Mat &input_output);
	

	//members
public:
	Size m_boardSize;            // The size of the board -> Number of items by width and height
	Pattern m_calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
	float m_squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
	int m_nrFrames;              // The number of frames to use from the input for calibration
	float m_aspectRatio;         // The aspect ratio
	int m_delay;                 // In case of a video input
	bool m_bwritePoints;         //  Write detected feature points
	bool m_bwriteExtrinsics;     // Write extrinsic parameters
	bool m_calibZeroTangentDist; // Assume zero tangential distortion
	bool m_calibFixPrincipalPoint;// Fix the principal point at the center
	bool m_flipVertical;          // Flip the captured images around the horizontal axis
	string m_outputFileName;      // The name of the file where to write
	bool m_showUndistorsed;       // Show undistorted images after calibration
	string m_input;               // The input ->


	
	string m_inputSettingsFile;
	int m_cameraID;
	int m_framerate;
	vector<string> m_imageList;
	int m_atImageList;
	VideoCapture m_inputCapture;
	InputType m_inputType;
	bool m_goodInput;
	int m_flag;
	int m_undistorted_delay;
	

	vector<vector<Point2f> > m_imagePoints;
	Mat m_cameraMatrix, m_distCoeffs, m_extrinsic_params;
	Size m_imageSize;
	int m_mode;
	clock_t m_prevTimestamp;


	Mat m_view;
	Mat m_temp;
	Mat  m_undistorted_image;
	bool m_blinkOutput;
	bool m_readin_set;

//modifications members
	bool m_undistorted_output_in_loop;
	bool m_show_undistorted_image;
	bool m_calibration;


private:
	string m_patternToUse;
	bool m_readCameraParams;

};

