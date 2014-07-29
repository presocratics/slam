/*
 * =====================================================================================
 *
 *       Filename:  featureIO.h
 *
 *    Description:  base class for sensors
 *
 *        Version:  1.0
 *        Created:  06/27/2014 11:50:31 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Hong-Bin Yoon (), yoon48@illinois.edu
 *   Organization:  
 *
 * =====================================================================================
 */ 

  #ifndef FEATUREIO_H
#define  FEATUREIO_H
#include <cv.h>
#include "ourerr.hpp"

#define MAXLINE 1024            /*  */

/*
 * Normalized pixel coordinates of the a point feature in the camera *
 * coordinate frame.
 */
struct projection {
    projection(){};
    projection( const cv::Point2d& s, const cv::Point2d& r, int i ) : source(s), reflection(r), id(i) {};
    cv::Point2d source, reflection;
    int id;
};				/* ----------  end of struct match  ---------- */
typedef struct projection Projection;

/*
 * =====================================================================================
 *        Class: featureIO 
 *        Description: 
 * =====================================================================================
 */
class FeatureIO
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        FeatureIO ( const char *fn, bool ih) : isHex(ih) {
            set_file( fn );
        }/* constructor */
        ~FeatureIO() { matches.clear(); };

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        void set_file( const char *fn ) {
            fp=open_source(fn); 
        }
        void update();
        virtual void get_projections();

        /* ====================  OPERATORS     ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        std::vector<projection> matches;
    protected:

        /* ====================  METHODS       ======================================= */
        FILE* open_source ( const char *fn );
        int get_val ( FILE* fp, const char *str, const char *fmt, ... );

        /* ====================  DATA MEMBERS  ======================================= */
        FILE *fp;
        bool isHex;

}; /* -----  end of class ImageSensor  ----- */

#endif   /* ----- #ifndef imagesensor_INC  ----- */

