#include "Sensors.hpp"
#include "ourerr.hpp"
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
    altitude = get_altitude( );
    acceleration = get_acceleration( index );
    quaternion = get_quaternion( index );
    dt = get_dt( );
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


    FILE*
Sensors::open_source ( const char *fn )
{
    FILE *fp;
    char line[MAXLINE];
    if( (fp=fopen(fn,"r"))==NULL )
        err_sys("fopen altitude");
    //TODO: Bogus hack to skip first 1700
    for( int i=0; i<1699; ++i )
        fgets( line, MAXLINE, fp );
    return fp ;
}		/* -----  end of method Sensors::open_source  ----- */


    void
Sensors::get_val ( FILE* fp, const char *str, const char *fmt, ... )
{
    char *line = new char[MAXLINE];
    va_list ap;
    va_start(ap,fmt);

    if( (fgets(line,MAXLINE,fp ))==NULL )
        err_sys("fgets %s", str);
    vsscanf( line, fmt, ap );
    va_end(ap);

    return ;
}		/* -----  end of method Sensors::get_val  ----- */



    double
Sensors::get_altitude( )
{ 
    double alt;
    get_val( altitude_fp, "alt", "%lf", &alt );
    return alt;
}		/* -----  end of method Sensors::get_altitude  ----- */

    double
Sensors::get_dt ( )
{
    double d;
    get_val( dt_fp, "dt", "%lf", &d );
    return d;
}		/* -----  end of method Sensors::get_dt  ----- */

