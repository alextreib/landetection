#include "Bildverarbeitung_sim.hpp"


Bildverarbeitung_sim::Bildverarbeitung_sim()
{
}


Bildverarbeitung_sim::~Bildverarbeitung_sim()
{
}

int main()
{
	Bildverarbeitung_sim sim;
	Mat mat;
	float summe = 0;
	float mittelwert;
	float i=0;

	while (1)
	{
		i++;
		//double t = (double)getTickCount();
		sim.dist_correct.go(mat);	//60ms

		//imshow("Undistorted", mat);

		sim.bird_eye.go(mat);	//110ms
		//Robertfkt
		imshow("Bird_eye", mat);
		/*t = ((double)getTickCount() - t) / getTickFrequency();
		cout << "Times passed in ms: " << t * 1000 << endl;
		summe = summe + t;
		i++;
		mittelwert = summe / i;
		cout << "mittel: " << mittelwert << endl;*/
		stringstream k;
		float j = i;
		k << j;

		//imwrite("testbild_manualcalib_gerade_rechts1_" + k.str() + ".png", mat);
		waitKey(50);
	}
	return 0;
}