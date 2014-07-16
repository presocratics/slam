#ifndef  Sensors_INC
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
        Sensors (){;} /* constructor */

        /* ====================  ACCESSORS     ======================================= */
        double get_altitude();

        double get_dt();

        cv::Vec3d get_acceleration ( );  
        cv::Vec3d get_angular_velocity ( );
        cv::Vec4d get_quaternion ( );


        /* ====================  MUTATORS      ======================================= */
        void update();
        void set_acceleration( const char *fn, bool isHex ) { 
            acceleration_fn = fn;
            accelerationIsHex = isHex;
        }

        void set_altitude ( const char *fn, bool isHex ) {
            altitude_fn = fn;
            altitudeIsHex = isHex;
        }		

        void set_quaternion( const char *fn, bool isHex ) { 
            quaternion_fn = fn;
            quaternionIsHex = isHex;
        }

        void set_dt ( const char *fn, bool isHex ) {
            dtIsHex = isHex;
            dt_fp = open_source(fn);
        }		

        void set_angular_velocity( const char *fn, bool isHex ) { 
            angular_velocity_fn = fn;
            angular_velocityIsHex = isHex;
        }

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
        bool accelerationIsHex, altitudeIsHex, angular_velocityIsHex, dtIsHex,
             quaternionIsHex;

        std::string altitude_fn, acceleration_fn,
            quaternion_fn, angular_velocity_fn;
        FILE *dt_fp;
        /* Sources */

}; /* -----  end of class Sensors  ----- */

#endif   /* ----- #ifndef Sensors_INC  ----- */
