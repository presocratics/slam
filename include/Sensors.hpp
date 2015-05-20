#ifndef  Sensors_INC
#define  Sensors_INC
#define MAXTYPE 32            /*  */
#define MAXVALS 256            /*  */
#include <cv.h>
#include "Sensor.hpp"
#include "Quaternion.hpp"
#define EPS 0.005 // Maximum delta to consider part of the same timestep.
#define UPDATE_ACC 0x1
#define UPDATE_ANG 0x2
#define UPDATE_QUAT 0x4
#define UPDATE_IMG 0x8
#define UPDATE_ALT 0x10
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
        Sensors() {time=-1;}

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        int update();

        /* ====================  OPERATORS     ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        Sensor<Quaternion> quat;
        Sensor<cv::Vec3d> acc;
        Sensor<cv::Vec3d> ang;
        Sensor<double> alt;
        Sensor<int> img;

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
