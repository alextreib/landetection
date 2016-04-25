#include "Bird_eye.hpp"
#include "Dist_correct.hpp"

class Bildverarbeitung_sim
{
public:
	Bildverarbeitung_sim();
	~Bildverarbeitung_sim();
public:
	Bird_eye bird_eye;
	Dist_correct dist_correct;
};

