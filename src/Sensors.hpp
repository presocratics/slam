#ifndef  sensors_INC
#define  Sensors_INC
#define MAXLINE 1024            /*  */
#include <cv.h>
#include "Quaternion.hpp"
/*
 * =====================================================================================
 *        Class:  Sensors
 *  Description:  Store sensor data.
 * =====================================================================================
 */
class Sensors
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        Sensors () /* constructor */
        {
            index=0;
        }

        /* ====================  ACCESSORS     ======================================= */
        double get_altitude();

        double get_dt();

        cv::Vec3d get_acceleration ( );  
        cv::Vec3d get_angular_velocity ( int timestep ) { 
            return Mat2Vec3d( angular_velocityHist, timestep ); 
        }
        cv::Vec4d get_quaternion ( int timestep ) { 
            return Mat2Vec4d( quaternionHist, timestep ); 
        }


        /* ====================  MUTATORS      ======================================= */
        void update();
        void set_index( int i ) { index=i; }
        void set_acceleration( const char *fn ) { 
            acceleration_fp=open_source(fn); 
        }

        void set_altitude ( const char *fn ) {
            altitude_fp = open_source(fn);
        }		

        void set_quaternion( cv::Mat& src ) { quaternionHist=src; }

        void set_dt ( const char *fn ) {
            dt_fp = open_source(fn);
        }		

        void set_angular_velocity( cv::Mat& src ) { angular_velocityHist=src; }

        /* ====================  OPERATORS     ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        double altitude;
        cv::Vec3d acceleration, angular_velocity;
        Quaternion quaternion;
        double dt;

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */
        cv::Vec3d Mat2Vec3d( cv::Mat src, int timestep );
        cv::Vec4d Mat2Vec4d( cv::Mat src, int timestep );
        FILE* open_source ( const char *fn );
        void get_val ( FILE* fp, const char *str, const char *fmt, ... );

        /* ====================  DATA MEMBERS  ======================================= */
        int index;

        /* Sources */
        FILE *altitude_fp, *dt_fp, *acceleration_fp;
        cv::Mat quaternionHist, angular_velocityHist;

}; /* -----  end of class Sensors  ----- */

#endif   /* ----- #ifndef Sensors_INC  ----- */
