/*****************************************************************************/
/* filename: 	Bird_eye.cpp                                                 */
/* author: 		Alexander Treib (TR)                                         */
/* description:	Implementation of the Bird's-eye view algorithm              */
/*---------------------------------------------------------------------------*/
/* revision history:                                                         */
/* date       | author | modification                                        */
/*____________|________|_____________________________________________________*/
/* 2015-11-20 | TR     | initial revision                                    */
/*****************************************************************************/
#include "Bird_eye.hpp"


Bird_eye::Bird_eye()
{
	//Initialization
	m_read_H_file = 0;
}


bool Bird_eye::read_H_file()
{
	//In case a previous H calibration file exists, the function copy the Matrix into the member variable
	FileStorage fs(H_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << H_filename << "\"" << endl;
		m_read_H_file = 0;
		return 0;
	}
	fs["H_matrix"] >> m_H;
	fs.release();
	m_read_H_file = 1;
	return 1;
}

bool Bird_eye::write_H_file(Mat &H)
{
	//Write the calculated H Matrix into a file
	FileStorage fs(H_filename, FileStorage::WRITE);
	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << H_filename << "\"" << endl;
		return 0;
	}
	fs << "H_matrix" << H;
	fs.release();
	return 1;
}

Mat Bird_eye::nextImage()
{
	//returns the current image from the camera of the type Mat
	if (m_inputCapture.isOpened())
	{
		m_inputCapture >> m_temp;
	}

	return m_temp;
}

bool Bird_eye::chessboard_H_calibration()
{
	//calibration of the H Matrix with a given chessboard
	int board_w = 9;
	int board_h = 6;

	Size board_sz(board_w, board_h);

	Mat gray_image, tmp, birds_image;
	Point2f objPts[4], imgPts[4];
	vector<Point2f> corners;
	float Z; //have experimented from values as low as .1 and as high as 100

	int alpha_ = 1;

	int H_set = 0;
	char key = 0;
	int found_chess = 0;
	m_inputCapture.open(cameraID);
	Mat image;

	//Until the Key c for completed is pressed
	while (key != 'b')
	{
		//get image
		image = nextImage();
		
		//Display rawimage
		if (!H_set)
		{
			namedWindow("RawImage");
			imshow("RawImage", image);
		}

		//Search Chessboard (CPU intense)
		found_chess = findChessboardCorners(image, board_sz, corners);

		if (found_chess) {
			destroyWindow("RawImage");
			drawChessboardCorners(image, board_sz, corners, 1);
			cvtColor(image, gray_image, CV_RGB2GRAY);
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			resize(image, tmp, Size(), .5, .5);
			namedWindow("Chessboard");
			imshow("Chessboard", tmp);

			objPts[0].x = 0;
			objPts[0].y = 0;
			objPts[1].x = board_w - 1;
			objPts[1].y = 0;
			objPts[2].x = 0;
			objPts[2].y = board_h - 1;
			objPts[3].x = board_w - 1;
			objPts[3].y = board_h - 1;

			imgPts[0] = corners.at(0);
			imgPts[1] = corners.at(board_w - 1);
			imgPts[2] = corners.at((board_h - 1) * board_w);
			imgPts[3] = corners.at((board_h - 1) * board_w + board_w - 1);

			m_H = cv::getPerspectiveTransform(objPts, imgPts);
			cout << "Calculated H Matrix: " << m_H << endl;
			birds_image = image;
			H_set = 1;
		}
		if ((H_set == 1) || found_chess)
		{
			//Output text
			string msg = "Press 'b' when completed";
			int baseLine = 0;
			Size textSize = getTextSize(msg, 1, 1, 3, &baseLine);
			Point textOrigin(image.cols - 2 * textSize.width - 10, image.rows - 2 * baseLine - 10);
			putText(image, msg, textOrigin, 1, 1, (0, 0, 255));

			namedWindow("Result", 1);
			createTrackbar("Alpha", "Result", &alpha_, 40);
			double alpha = (double)(alpha_ - 20);	//range: -20 to 20
			Z = alpha;

			m_H.at<double>(2, 2) = Z;

			warpPerspective(image, birds_image, m_H, Size(3 * image.cols, 2 * tmp.rows),
				CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);

			imshow("Result", birds_image);
		}
		key = (char)waitKey(20);
	}
	destroyAllWindows();
	if (!found_chess)
	{
		cout << "No chessboard found -> No H_calibration.xml" << endl;
		return 0;
	}
	//Save the configuration
	if (H_set)
	{
		write_H_file(m_H);
		return 1;
	}
	else
	{
		cout << "No H Matrix to write" << endl;
		return 0;
	}

	//something not ok
	cout << "Unknown error in fkt. chess_H_calibration" << endl;
	return 0;
}

bool Bird_eye::manual_H_calibration()
{
	//the function calculates the H matrix through a manual configuration tool
	// read the camera input
	VideoCapture cap(cameraID);
	if (!cap.isOpened())
		return 0;

	// default values of the homography parameters
	int alpha_ = 90., beta_ = 90., gamma_ = 90.;
	int f_ = 500, dist_ = 200;
	char key = 0;

	// images
	Mat destination;
	Mat source;

	/// Create Window
	namedWindow("Result", 1);

	/// Add trackbars for different aspects of the 
	createTrackbar("Alpha", "Result", &alpha_, 180);
	//createTrackbar("Beta", "Result", &beta_, 180);
	//createTrackbar("Gamma", "Result", &gamma_, 180);
	//createTrackbar("f", "Result", &f_, 2000);
	//createTrackbar("Distance", "Result", &dist_, 2000);

	while (key != 'b') {

		//grab and retrieve each frames of the video sequentially 
		cap >> source;

		double f, dist;
		double alpha, beta, gamma;
		alpha = ((double)alpha_ - 90.)*PI / 180;
		beta = ((double)beta_ - 90.)*PI / 180;
		gamma = ((double)gamma_ - 90.)*PI / 180;
		f = (double)f_;
		dist = (double)dist_;

		Size taille = source.size();
		double w = (double)taille.width, h = (double)taille.height;

		/*****Calculation of the H_Matrix***/
		// Projection 2D -> 3D matrix
		Mat A1 = (Mat_<float>(4, 3) <<
			1, 0, -w / 2,
			0, 1, -h / 2,
			0, 0, 0,
			0, 0, 1);

		// Rotation matrices around the X,Y,Z axis
		Mat RX = (Mat_<float>(4, 4) <<
			1, 0, 0, 0,
			0, cos(alpha), -sin(alpha), 0,
			0, sin(alpha), cos(alpha), 0,
			0, 0, 0, 1);

		Mat RY = (Mat_<float>(4, 4) <<
			cos(beta), 0, -sin(beta), 0,
			0, 1, 0, 0,
			sin(beta), 0, cos(beta), 0,
			0, 0, 0, 1);

		Mat RZ = (Mat_<float>(4, 4) <<
			cos(gamma), -sin(gamma), 0, 0,
			sin(gamma), cos(gamma), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1);

		// Composed rotation matrix with (RX,RY,RZ)
		Mat R = RX * RY * RZ;

		// Translation matrix on the Z axis change dist will change the height
		Mat T = (Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, dist, 0, 0, 0, 1);       // Camera Intrisecs matrix 3D -> 2D
		// Camera Intrisecs matrix 3D -> 2D
		Mat A2 = (Mat_<float>(3, 4) <<
			f, 0, w / 2, 0,
			0, f, h / 2, 0,
			0, 0, 1, 0);

		// Final and overall transformation matrix
		// Basis matrix is multiplied with A1 for the projection in homogenous coordinates
		// Basis matrix is multiplied with Rotationsmatrix (R * A1) -> rotation around the calculated rotation matrix
		// Result is multiplied with T for the Translation matrix -> translation (just for changing the distance)
		// Result is multiplied
		m_H = A2 * (T * (R * A1));

		// Apply matrix transformation
		warpPerspective(source, destination, m_H, taille, WARP_INVERSE_MAP);

		//Output text
		string msg = "Press 'b' when completed";
		int baseLine = 0;
		Size textSize = getTextSize(msg, 1, 1, 3, &baseLine);
		Point textOrigin(destination.cols - 2 * textSize.width - 10, destination.rows - 2 * baseLine - 10);
		putText(destination, msg, textOrigin, 1, 1, (0, 0, 255));

		imshow("Original", source);
		imshow("Result", destination);

		key = (char)waitKey(30);
	}
	destroyAllWindows();

	if (write_H_file(m_H))
	{
		return 1;
	}
	else
	{
		cout << "Could not write H file" << endl;
		return 0;
	}
	//everyhting not ok
	return 0;
}

void Bird_eye::Transformation(Mat &input, Mat &output)
{
	//Transformation of the input image through the H Matrix
	cv::warpPerspective(input, output, m_H, input.size(),
		CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
}

void Bird_eye::go(Mat &input_output)
{
	//H Matrix exists already?
	while (!m_read_H_file)
	{
		if (read_H_file())
		{
			cout << "Reading H_file" << endl;
		}
		else
		{
			cout << H_filename << " doesn't exist" << endl;
			cout << "->Calibration is needed." << endl;
			cout << "-Press 'c' for chessboard calbiration" << endl;
			cout << "-Press 'm' for manual calibration" << endl;
			char calib_select;
			cin >> calib_select;
			if (calib_select == 'm')
			{
				manual_H_calibration();
			}
			else if (calib_select == 'c')
			{
				chessboard_H_calibration();
			}
			else
			{
				cout << "Unknown key...new try!" << endl;
			}
		}
	}

	//Transformation
	m_input = input_output;
	Transformation(m_input, m_output);
	input_output = m_output;
}



//int main() {
//	Bird_eye bird;
//	Mat input, output;
//
//	while (1)
//	{
//		VideoCapture cap(1);
//		cap >> input;
//
//		bird.go(input, output);
//		imshow("Test", output);
//		waitKey(10);
//
//	}
//}