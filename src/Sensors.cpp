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
    altitude = get_altitude( index );
    acceleration = get_acceleration( index );
    quaternion = get_quaternion( index );
    dt = get_dt( index );
    angular_velocity = get_angular_velocity( index );
    ++index;
    return ;
}		/* -----  end of method Sensors::update  ----- */

    cv::Vec3d
Sensors::Mat2Vec3d ( cv::Mat src, int timestep )
{
    cv::Vec3d r;
    for( int i=0; i<3; ++i )
    {
        r[i] = src.at<double>(i, timestep);
    }
    return r;
}		/* -----  end of method Sensors::Mat2Vec  ----- */

    cv::Vec4d
Sensors::Mat2Vec4d ( cv::Mat src, int timestep )
{
    cv::Vec4d r;
    for( int i=0; i<4; ++i )
    {
        r[i] = src.at<double>(i, timestep);
    }
    return r;
}		/* -----  end of method Sensors::Mat2Vec  ----- */
