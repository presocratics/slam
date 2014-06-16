#ifndef  sensors_INC
#define  Sensors_INC

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
            i=0;
        }

        /* ====================  ACCESSORS     ======================================= */
        double get_altitude(int i) { return altitudeHist.at<double>(0,i); }
        cv::Vec3d get_acceleration(int i);
        cv::Vec3d get_angular_velocity(int i);
        cv::Vec4d get_quaternion(int i);
        double get_dt( int i );

        /* ====================  MUTATORS      ======================================= */
        void update();
        void set_index( int index ) { i=index; }
        void set_acceleration( cv::Mat& src ) { accelerationHist=src; }
        void set_altitude( cv::Mat& src ) { altitudeHist=src; }
        void set_quaternion( cv::Mat& src ) { quaternionHist=src; }
        void set_dt( cv::Mat& src ) { dtHist=src; }
        void set_angular_velocity( cv::Mat& src ) { angular_velocityHist=src; }

        /* ====================  OPERATORS     ======================================= */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        int i;
        double altitude;
        cv::Vec3d acceleration, angular_velocity;
        Quaternion quaternion;
        double dt;

        /* Sources */
        cv::Mat accelerationHist, altitudeHist, quaternionHist, dtHist,
            angular_velocityHist;

}; /* -----  end of class Sensors  ----- */

#endif   /* ----- #ifndef Sensors_INC  ----- */
