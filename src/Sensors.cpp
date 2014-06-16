#include "Sensors.hpp"

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Sensors
 *      Method:  Sensors :: update
 * Description:  Get current sensor data.
 *--------------------------------------------------------------------------------------
 */
    void
Sensors::update ( )
{
    altitude = get_altitude( i );
    acceleration = get_acceleration( i );
    quaternion = get_quaternion( i );
    //dt = get_dt( i );
    //angular_velocity = get_angular_velocity( i );
    ++i;
    return ;
}		/* -----  end of method Sensors::update  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Sensors
 *      Method:  Sensors :: get_acceleration
 * Description:  Returns acceleration at timestep i.
 *--------------------------------------------------------------------------------------
 */
    cv::Vec3d
Sensors::get_acceleration ( int i )
{
    cv::Vec3d a;
    for( int j=0; j<3; ++j )
    {
        a[j] = accelerationHist.at<double>(j, i);
    }
    return a;
}		/* -----  end of method Sensors::get_acceleration  ----- */

    cv::Vec4d
Sensors::get_quaternion ( int i )
{
    cv::Vec4d q;
    for( int j=0; j<4; ++j )
    {
        q[j] = quaternionHist.at<double>(j, i);
    }
    return q ;
}		/* -----  end of method Sensors::get_quaternion  ----- */

