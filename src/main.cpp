/*
 * main.cpp
 *
 *  Created on: 18.11.2015
 *      Author: Robert
 */

#include "../include/imageProcessing.h"

imageProcessing::module myImgProc;

int angle = 40;

void trackbar(int, void*) {
	myImgProc.deltaTheta() = angle * (CV_PI / 180);

	myImgProc.Tloop();

	myImgProc.drawLinesInMat(myImgProc.houghLines(), myImgProc.imgMat(), 0, 255, 0, 1);
	myImgProc.drawLinesInMat(myImgProc.meanLines(), myImgProc.imgMat(), 0, 0, 255, 1);

	imshow ("Bild Canny", myImgProc.cannyMat());
	myImgProc.drawLinesInMat(myImgProc.houghLines(), myImgProc.cannyMat(), 255, 255, 255, 1);
	imshow ("Bild Canny Lines", myImgProc.cannyMat());

	imshow ("Bild", myImgProc.imgMat());
}


void main( int argc, char** argv){
	myImgProc.Bild(argv); 

	namedWindow("Parameters", WINDOW_AUTOSIZE );
	createTrackbar( "Blur  ", "Parameters", &myImgProc.blurThreshold(), 100, trackbar );
	createTrackbar( "Canny ", "Parameters", &myImgProc.cannyThreshold(), 100, trackbar );
	createTrackbar( "Hough ", "Parameters", &myImgProc.houghThreshold(), 200, trackbar );
	createTrackbar( "Rho   ", "Parameters", &myImgProc.deltaRho(), 500, trackbar );
	createTrackbar( "Theta ", "Parameters", &angle, 360, trackbar );
	trackbar(0, 0);
	waitKey(50);
}
