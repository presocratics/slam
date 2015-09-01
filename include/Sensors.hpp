#ifndef  Sensors_INC
#define  Sensors_INC
#define MAXTYPE 32            /*  */
#define MAXVALS 256            /*  */
#include <cv.h>
#include "Sensor.hpp"
#include "Quaternion.hpp"
#define EPS 0.005 // Maximum delta to consider part of the same timestep.
#define UPDATE_ACC 1
#define UPDATE_ANG 1<<1
#define UPDATE_QUAT 1<<2
#define UPDATE_IMG 1<<3
#define UPDATE_ALT 1<<4
#define UPDATE_INIT 1<<5
#define UPDATE_POS 1<<6            /*  */
#define UPDATE_VEL 1<<7
#define UPDATE_VELB 1<<8


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
        Sensors() {time=-1;};

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        int update();
        double get_time() { return time;} ;

        /* ====================  OPERATORS     ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        Sensor<Quaternion> quat;
        Sensor<cv::Vec3d> acc;
        Sensor<cv::Vec3d> ang;
        Sensor<double> alt;
        Sensor<int> img;
        Sensor<cv::Vec3d> init;
        Sensor<cv::Vec3d> vel,velb;
        Sensor<cv::Vec3d> pos;

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        double t; /* Start time timestep */
        double time; /* Timestamp of current sensor */
        char type[MAXTYPE];
        char vals[MAXVALS];
}; /* -----  end of class Sensors  ----- */

#endif   /* ----- #ifndef Sensors_INC  ----- */
