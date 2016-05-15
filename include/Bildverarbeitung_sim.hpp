/*****************************************************************************/
/* filename: 	Bird_eye.cpp                                                 */
/* author: 		Alexander Treib (TR)                                         */
/* description:	Simulation of the superior class                             */
/*---------------------------------------------------------------------------*/
/* revision history:                                                         */
/* date       | author | modification                                        */
/*____________|________|_____________________________________________________*/
/* 2015-11-20 | TR     | initial revision                                    */
/*****************************************************************************/

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

