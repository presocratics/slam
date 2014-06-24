#ifndef  imagesensor_INC
#define  imagesensor_INC
#include <cv.h>
#include "ourerr.hpp"

#define MAXLINE 1024            /*  */

/*
 * Normalized pixel coordinates of the a point feature in the camera *
 * coordinate frame.
 */
struct projection {
    cv::Point2d source, reflection;
    unsigned int id;
};				/* ----------  end of struct match  ---------- */
typedef struct projection Projection;

/*
 * =====================================================================================
 *        Class:  ImageSensor
 *  Description:  Retrieves input from image processing.
 * =====================================================================================
 */
class ImageSensor
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        ImageSensor (){
            matches.resize(5);
        }                             /* constructor */
        ImageSensor ( const char *fn, bool ih) {
            matches.resize(5);
            set_file( fn, ih );
        }/* constructor */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        void set_file( const char *fn, bool ih ) {
            isHex = ih;
            fp=open_source(fn); 
        }
        void update();
        void get_projections();

        /* ====================  OPERATORS     ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        std::vector<projection> matches;
    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */
        FILE* open_source ( const char *fn );
        void get_val ( FILE* fp, const char *str, const char *fmt, ... );

        /* ====================  DATA MEMBERS  ======================================= */
        FILE *fp;
        bool isHex;

}; /* -----  end of class ImageSensor  ----- */

#endif   /* ----- #ifndef imagesensor_INC  ----- */
