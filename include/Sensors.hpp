#ifndef  Sensors_INC
#define  Sensors_INC
#define MAXTYPE 32            /*  */
#define MAXVALS 256            /*  */
#include <cv.h>
#include "Sensor.hpp"
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
        Sensors() {time=-1;}

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        void update();

        /* ====================  OPERATORS     ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        Sensor<Quaternion> quat;
        Sensor<cv::Vec3d> acc;
        Sensor<cv::Vec3d> ang;
        Sensor<cv::Vec3d> img;

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
