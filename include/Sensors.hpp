#ifndef  Sensors_INC
#define  Sensors_INC
#define MAXTYPE 32            /*  */
#define MAXVALS 256            /*  */
#include <cv.h>
#include "Sensor.hpp"
#include "Quaternion.hpp"
#define EPS 0.01 // Maximum delta to consider part of the same timestep.
#define UPDATE_ACC 1
#define UPDATE_ANG 1<<1
#define UPDATE_QUAT 1<<2
#define UPDATE_IMG 1<<3
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
