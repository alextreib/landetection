/*****************************************************************************/
/* filename: 	Bird_eye.hpp                                                 */
/* author: 		Alexander Treib (TR)                                         */
/* description:	Implementation of the Bird's-eye view algorithm              */
/*---------------------------------------------------------------------------*/
/* revision history:                                                         */
/* date       | author | modification                                        */
/*____________|________|_____________________________________________________*/
/* 2015-11-20 | TR     | initial revision                                    */
/*****************************************************************************/


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>

#include <math.h>
#include <vector>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

#define PI 3.14
const int cameraID = 0;
const string H_filename = "H_calibration.xml";

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

class Bird_eye
{
public:
	Bird_eye();
	bool read_H_file();
	bool write_H_file(Mat &H);
	Mat nextImage();
	bool chessboard_H_calibration();
	bool manual_H_calibration();
	void Transformation(Mat &input, Mat &output);
	void go(Mat &input_output);
public:
	Mat m_H;
	Mat m_input, m_output;
	bool m_read_H_file;
	VideoCapture m_inputCapture;
	Mat m_temp;
};

