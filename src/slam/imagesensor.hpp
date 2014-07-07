#ifndef  imagesensor_INC
#define  imagesensor_INC
#include <cv.h>
#include "ourerr.hpp"
#include "featureIO.h"

#define MAXLINE 1024            /*  */


/*
 * =====================================================================================
 *        Class:  ImageSensor
 *  Description:  Retrieves input from image processing.
 * =====================================================================================
 */
class ImageSensor : public FeatureIO
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        ImageSensor ()
        {

        }                             /* constructor */
        ImageSensor ( int nm, const char *fn, bool ih) : FeatureIO( nm, fn, ih) 
        {

        }

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        void get_projections();

}; /* -----  end of class ImageSensor  ----- */

#endif   /* ----- #ifndef imagesensor_INC  ----- */

