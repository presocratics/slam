#ifndef  slamsensor_INC
#define  slamsensor_INC
#include <cv.h>
#include "ourerr.hpp"
#include "featureIO.h"

#define MAXLINE 1024            /*  */

struct slamprojection {
    slamprojection(){};
    slamprojection( const cv::Point3d& s, int i ) : source(s), id(i) {};
    cv::Point3d source;
    int id;
}; 

/*
 * =====================================================================================
 *        Class:  SlamSensor
 *  Description:  Retrieves input from image processing.
 * =====================================================================================
 */
class SlamSensor : public FeatureIO
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        SlamSensor ( const char *fn, bool ih) : FeatureIO( fn, ih) { }

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        int get_projections();

        /* ====================  PUB MEMBER VARIABLES      ======================================= */
	cv::Vec3d X,V;
	std::vector<slamprojection> slammatches;
}; /* -----  end of class SlamSensor  ----- */

#endif   /* ----- #ifndef slamsensor_INC  ----- */

