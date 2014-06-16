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
            index=0;
        }

        /* ====================  ACCESSORS     ======================================= */
        double get_altitude(int timestep) { 
            return altitudeHist.at<double>(0,timestep); 
        }

        double get_dt(int timestep) { 
            return dtHist.at<double>(0,timestep); 
        }

        cv::Vec3d get_acceleration ( int timestep ) { 
            return Mat2Vec3d( accelerationHist, timestep ); 
        }
        cv::Vec3d get_angular_velocity ( int timestep ) { 
            return Mat2Vec3d( angular_velocityHist, timestep ); 
        }
        cv::Vec4d get_quaternion ( int timestep ) { 
            return Mat2Vec4d( quaternionHist, timestep ); 
        }


        /* ====================  MUTATORS      ======================================= */
        void update();
        void set_index( int i ) { index=i; }
        void set_acceleration( cv::Mat& src ) { accelerationHist=src; }
        void set_altitude( cv::Mat& src ) { altitudeHist=src; }
        void set_quaternion( cv::Mat& src ) { quaternionHist=src; }
        void set_dt( cv::Mat& src ) { dtHist=src; }
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

        /* ====================  DATA MEMBERS  ======================================= */
        int index;

        /* Sources */
        cv::Mat accelerationHist, altitudeHist, quaternionHist, dtHist,
            angular_velocityHist;

}; /* -----  end of class Sensors  ----- */

#endif   /* ----- #ifndef Sensors_INC  ----- */
