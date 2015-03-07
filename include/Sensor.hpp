/*
 * =====================================================================================
 *
 *       Filename:  Sensor.hpp
 *
 *    Description:  A sensor template.
 *
 *        Version:  1.0
 *        Created:  03/06/2015 03:38:50 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (MHM), miller7@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */

#ifndef  Sensor_INC
#define  Sensor_INC

/*
 * =====================================================================================
 *        Class:  Sensor
 *  Description:  Implements a sensor.
 * =====================================================================================
 */
template < class T >
class Sensor
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        Sensor () {timestamp=0;}                             /* constructor */

        /* ====================  ACCESSORS     ======================================= */
        double get_timestamp ( ) const { return timestamp; }
        double get_dt ( ) const { return dt; }
        T get_value() const { return value; } 

        /* ====================  MUTATORS      ======================================= */
        void set_value( double ts, T v )
        {
            dt=(timestamp==0) ? 0 : ts-timestamp;
            timestamp=ts;
            value=v;
        }

        /* ====================  OPERATORS     ======================================= */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        T value;
        double timestamp;
        double dt;

}; /* ----------  end of template class Sensor  ---------- */

#endif   /* ----- #ifndef Sensor_INC  ----- */
