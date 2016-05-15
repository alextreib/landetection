/*****************************************************************************/
/* Dateiname: imageProcessing.h                                              */
/* Autor(en): Robert Ostertag (RO)                                           */
/* Beschreibung: Klassen zur Bildbearbeitung                                 */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/* Revisionshistorie:                                                        */
/*                                                                           */
/* Datum      | Name | Änderung                                              */
/*____________|______|_______________________________________________________*/
/* 2015-11-02 | RO   | Erstellung und Übernahme des alten Standes            */
/* 2015-05-15 | AT   | Adding functionality for calibration and bird_eye     */
/*            |      |                                                       */
/*****************************************************************************/

#ifndef IMAGEPROCESSING_H_
#define IMAGEPROCESSING_H_

#include "../include/definitions.h"
#include "../include/Bird_eye.hpp"
#include "../include/Dist_correct.hpp"

namespace imageProcessing{

	class module
	{
	private:
		/*****************************************************************************/
		/* variable name: 	m_membervariable                                         */
		/* description: 	stuff...                                                 */
		/*****************************************************************************/
		//char * m_imgPath;					//Pointer on Path of the Img
		string m_imgPath;

		
		Mat m_matImg;						//Matrix with the original Img
		Mat m_matGray;						//Matrix with the Img in Gray
		Mat m_matBlur;						//Matrix with Img blurred


		int m_blurSizeX, m_blurSizeY;		//Blur intensity

		Mat m_matCanny;						//Matrix with the detected Edges
		int m_cannyLowThreshold, m_cannyHighThreshold;//Thresholds for Hysteresis procedure
		int m_cannyApertureSize;					//Size of Sobel Operator

		vector<Vec2f> m_houghLines;
		double m_houghRho;
		double m_houghTheta;
		int m_houghThreshold;
		double m_houghSrn;
		double m_houghStn;
		vector< vector<Vec2f> > m_lineContainer;
		vector< vector<Vec2f> >::iterator m_containerIterator;
		vector<Vec2f>::iterator m_vectorIterator;
		vector<Vec3f> m_meanLines;
		int m_deltaRho;
		float m_deltaTheta;
		bool m_foundLine;
		char** m_Bild;
		enum{ RHO = 0, THETA = 1, SIZE = 2 };

		Dist_correct m_dist_correct_obj;
		Bird_eye m_bird_eye_obj;


	protected:
	public:
		/*****************************************************************************/
		/* function name: 	module                                                   */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	default constructor                                      */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | RO     | initial revision                                    */
		/*****************************************************************************/
		module(void);


		/*****************************************************************************/
		/* function name: 	~module                                                  */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	destructor                                               */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		~module(void);


		/*****************************************************************************/
		/* function name:	loop                                                     */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	load Img into Matrix                                     */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		void Tloop(void);


		/*****************************************************************************/
		/* function name:	getImg                                                   */
		/* author: 			Alexander Treib (TR)                                     */
		/* description: 	get a calibrated, undistorted and bird_eye image         */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-05-15 | AT     | initial revision                                    */
		/*****************************************************************************/
		void getImg(void);


		/*****************************************************************************/
		/* function name:	loadImg                                                  */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	load Img into Matrix                                     */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		void loadImg(void);


		/*****************************************************************************/
		/* function name: 	imgEditing                                               */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	Image Editing                                            */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		void imgEditing(void);


		/*****************************************************************************/
		/* function name: 	streetFinding                                            */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	Searching for 3 lines                                    */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		void streetFinding(void);


		/*****************************************************************************/
		/* function name: 	lineFitting                                              */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	Sort found similar lines in vectoes                      */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		void sortLines(void);


		/*****************************************************************************/
		/* function name: 	buildMeanLines                                           */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	Build one mean line for each vector of lines in container*/
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		void buildMeanLines(void);


		/*****************************************************************************/
		/* function name: 	drawLinesInMat                                           */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	Draw the Lines in the Matrix                             */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		void drawLinesInMat(vector<Vec3f> linesToDraw, Mat matToDraw, int color_blue, int color_green, int color_red, int lineWidth);


		/*****************************************************************************/
		/* function name: 	drawLinesInMat                                           */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	Draw the Lines in the Matrix                             */
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		void drawLinesInMat(vector<Vec2f> linesToDraw, Mat matToDraw, int color_blue, int color_green, int color_red, int lineWidth);


		/*****************************************************************************/
		/* function name: 	ThetaInRange                                             */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	Build one mean line for each vector of lines in container*/
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		bool lineInRange(Vec2f &lineReference, Vec2f &lineCompare);


		/*****************************************************************************/
		/* function name: 	ThetaInRange                                             */
		/* author: 			Robert Ostertag (RO)                                     */
		/* description: 	Build one mean line for each vector of lines in container*/
		/*---------------------------------------------------------------------------*/
		/* date       | author | modification                                        */
		/*____________|________|_____________________________________________________*/
		/* 2015-11-02 | BM     | initial revision                                    */
		/*****************************************************************************/
		void convertLine(Vec2f &line);



		const Mat& cannyMat(void) const { return m_matCanny; }

		const Mat& imgMat(void) const { return m_matImg; }

		const vector<Vec3f>& meanLines(void) const { return m_meanLines; }

		const vector<Vec2f>& houghLines(void) const { return m_houghLines; }


		int& blurThreshold(void) { m_blurSizeY = m_blurSizeX; return m_blurSizeX; }

		int& cannyThreshold(void) { m_cannyHighThreshold = 3 * m_cannyLowThreshold; return m_cannyLowThreshold; }

		int& houghThreshold(void) { return m_houghThreshold; }

		int& deltaRho(void) { return m_deltaRho; }

		float& deltaTheta(void) { return m_deltaTheta; }


		char** Bild(char** blabla) { m_Bild = blabla; return 0; }


		void resetVectors(void);

	};

}
#endif /* IMAGEPROCESSING_H_ */


