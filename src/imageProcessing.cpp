#include "../include/imageProcessing.h"


imageProcessing::module::module(void) :m_lineContainer(vector< vector<Vec2f> >(100)), m_meanLines(vector<Vec3f>(100)) {

	/*****************************************************************************/
	/* Initializing variables with parameters:                                   */
	/*		m_imgPath = Path of Original Image                                   */
	/*		m_blurSizeX = Blur Matrix Size in X                                  */
	/* 		m_blurSizeY = Blur Matrix Size in Y                                  */
	/*		m_lowThreshhold = Canny lower Threshold                              */
	/*		m_highThreshhold = Canny high Threshold                              */
	/*****************************************************************************/

	/* Path of the Image */
	//m_imgPath = "F:\\Programme\\Eclipse\\Workspace\\Bildverarbeitung_v4.2\\Debug\\IMG.jpg";
	//m_imgPath = "..\\Debug\\IMG.jpg";

	/* Size of the Kernel to Blur*/
	m_blurSizeX = 3;
	m_blurSizeY = 3;

	/* Variables for Canny Operation */
	m_cannyLowThreshold = 100;
	m_cannyHighThreshold = 300;
	m_cannyApertureSize = 3;

	/* Variables for Hough Operation */
	m_houghRho = 1;
	m_houghTheta = CV_PI / 180;
	m_houghThreshold = 40;	// wichtig
	m_houghSrn = 0;
	m_houghStn = 0;

	/* Variables for lineFittting */
	m_deltaRho = 40;		//wichtig
	m_deltaTheta = 40 * (CV_PI / 180);
	m_foundLine = false;

	m_Bild = 0;

}

imageProcessing::module::~module(void) {
}


void imageProcessing::module::Tloop(void) {
	double t = (double)getTickCount();

	resetVectors();
	getImg();
	loadImg();
	imgEditing();
	streetFinding();

	t = ((double)getTickCount() - t) / getTickFrequency() * 1000;
	cout << "\n\nTime:\t\t\t" << t << " ms" << endl;
}

void imageProcessing::module::getImg(void)
{
	Mat m_readinImg;
	m_dist_correct_obj.go(m_readinImg);
	m_bird_eye_obj.go(m_readinImg);

	m_readinImg.copyTo(m_matImg);
}

void imageProcessing::module::loadImg(void) {
	//	if (!m_imgPath.empty()) {
	//		m_matImg = imread(m_imgPath.c_str());								//load Picture in a Matrix
	//	}
	//	m_matImg = imread( m_Bild[1] );

	float l_gesamt;
	float v_fahrzeug;
	float t_delay;
	float l_0;
	float Neigung_Kamera;
	float Hoehe_Kamera;
	float l_Bild;

	v_fahrzeug = 1;			//	in m/s
	t_delay = 0.1;			//	in s
	Neigung_Kamera = 45;	//	in ° ?????
	Hoehe_Kamera = 0.015;	//	in m


	l_gesamt = v_fahrzeug * t_delay;
	l_0 = tan(Neigung_Kamera) * Hoehe_Kamera;
	l_Bild = l_gesamt - l_0;
	//Hier muss noch die Umrechnung von Pixel in reale Entfernung rei um die errechnete Distanz auch auf das Bild zu übertragen

	Rect croppedRect;
	croppedRect.x = 1;
	croppedRect.y = m_matImg.rows / 2 - 50;
	croppedRect.width = m_matImg.cols - 1;
	croppedRect.height = 100;

	m_matImg = m_matImg(croppedRect).clone();
}

void imageProcessing::module::imgEditing(void) {
	if (!m_matImg.empty()) {
		//cvtColor(m_matImg, m_matGray, CV_BGR2GRAY);
		m_matGray = m_matImg;
		blur(m_matGray, m_matBlur, Size(m_blurSizeX, m_blurSizeY));
		Canny(m_matBlur, m_matCanny, m_cannyLowThreshold, m_cannyHighThreshold, m_cannyApertureSize);
	}
}

void imageProcessing::module::streetFinding(void) {
	HoughLines(m_matCanny, m_houghLines, m_houghRho, m_houghTheta, m_houghThreshold, m_houghSrn, m_houghStn);
	sortLines();
	buildMeanLines();
}


void imageProcessing::module::sortLines(void) {

	m_lineContainer.push_back(vector<Vec2f>());

	unsigned int i;
	for (i = 0; i < m_houghLines.size(); i++)
	{
		m_containerIterator = m_lineContainer.begin();
		m_foundLine = false;

		Vec2f tmpLine = m_houghLines[i];
		convertLine(tmpLine);

		while ((m_containerIterator != m_lineContainer.end()) && (m_foundLine == false))
		{
			if (m_containerIterator->empty())
			{
				m_containerIterator->push_back(tmpLine);
				m_foundLine = true;
			}
			else
			{
				Vec2f tmpRef = m_containerIterator->at(0);

				if (lineInRange(tmpRef, tmpLine))
				{
					m_containerIterator->push_back(tmpLine);
					m_foundLine = true;
				}
			}
			m_containerIterator++;
		}
		if (m_foundLine == false)
		{
			m_lineContainer.push_back(vector<Vec2f>());
			(m_containerIterator++)->push_back(tmpLine);
		}
	}
}


void imageProcessing::module::convertLine(Vec2f &line) {
	if (line.val[RHO] < 0)
	{
		line.val[RHO] = abs(line.val[RHO]);
		line.val[THETA] = line.val[THETA] + CV_PI;
	}
}


bool imageProcessing::module::lineInRange(Vec2f &lineReference, Vec2f &lineCompare) {
	float rhoLowerLimit = lineReference.val[RHO] - m_deltaRho;
	float rhoUpperLimit = lineReference.val[RHO] + m_deltaRho;
	float thetaLowerLimit = lineReference.val[THETA] - m_deltaTheta;
	float thetaUpperLimit = lineReference.val[THETA] + m_deltaTheta;

	if ((lineCompare.val[RHO] <= rhoUpperLimit) && (lineCompare.val[RHO] >= rhoLowerLimit))
	{
		if (thetaLowerLimit < 0)
		{
			if (lineCompare.val[THETA] >= (2 * CV_PI) + thetaLowerLimit)
			{
				lineCompare.val[THETA] = lineCompare.val[THETA] - (2 * CV_PI);
			}
		}
		else if (thetaUpperLimit >(2 * CV_PI))
		{
			if (lineCompare.val[THETA] <= thetaUpperLimit - (2 * CV_PI))
			{
				lineCompare.val[THETA] = lineCompare.val[THETA] + (2 * CV_PI);
			}
		}

		if ((lineCompare.val[THETA] <= thetaUpperLimit) && (lineCompare.val[THETA] >= thetaLowerLimit))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}


void imageProcessing::module::buildMeanLines(void) {
	m_meanLines.resize(m_lineContainer.size());
	//cout << "Start\n\n" << endl;
	unsigned int k = 0;
	for (m_containerIterator = m_lineContainer.begin(); m_containerIterator != m_lineContainer.end(); m_containerIterator++)
	{
		m_meanLines[k][RHO] = 0;
		m_meanLines[k][THETA] = 0;
		m_meanLines[k][SIZE] = 0;
		//cout << "\n\nnewVector:" << endl;
		for (m_vectorIterator = m_containerIterator->begin(); m_vectorIterator != m_containerIterator->end(); m_vectorIterator++)
		{
			m_meanLines[k][RHO] += m_vectorIterator->val[RHO];
			m_meanLines[k][THETA] += m_vectorIterator->val[THETA];
			m_meanLines[k][SIZE] = m_meanLines[k][SIZE] + 1;
			//cout << m_vectorIterator->val[RHO] << "\t\t" << (m_vectorIterator->val[THETA]) * (180/CV_PI) << endl;
		}

		m_meanLines[k][RHO] = m_meanLines[k][RHO] / m_meanLines[k][SIZE];
		m_meanLines[k][THETA] = m_meanLines[k][THETA] / m_meanLines[k][SIZE];
		k++;
	}
}

void imageProcessing::module::drawLinesInMat(vector<Vec3f> linesToDraw, Mat matToDraw, int color_blue, int color_green, int color_red, int lineWidth) {
	unsigned int i;
	for (i = 0; i < linesToDraw.size(); i++)
	{
		float rho = linesToDraw[i][0], theta = linesToDraw[i][1];
		Point pt1, pt2;
		double a, b;

		a = cos(theta);
		b = sin(theta);

		double x0 = a*rho, y0 = b*rho;

		pt1.x = cvRound(x0 + 10000 * (-b));
		pt1.y = cvRound(y0 + 10000 * (a));
		pt2.x = cvRound(x0 - 10000 * (-b));
		pt2.y = cvRound(y0 - 10000 * (a));
		line(matToDraw, pt1, pt2, Scalar(color_blue, color_green, color_red), lineWidth, CV_AA);
	}
}


void imageProcessing::module::drawLinesInMat(vector<Vec2f> linesToDraw, Mat matToDraw, int color_blue, int color_green, int color_red, int lineWidth) {
	unsigned int i;

	for (i = 0; i < linesToDraw.size(); i++)
	{
		float rho = linesToDraw[i][0], theta = linesToDraw[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 10000 * (-b));
		pt1.y = cvRound(y0 + 10000 * (a));
		pt2.x = cvRound(x0 - 10000 * (-b));
		pt2.y = cvRound(y0 - 10000 * (a));
		line(matToDraw, pt1, pt2, Scalar(color_blue, color_green, color_red), lineWidth, CV_AA);
	}
}


void imageProcessing::module::resetVectors() {
	m_lineContainer.clear();
	m_meanLines.clear();
}

