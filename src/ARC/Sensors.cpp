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
    acceleration = get_acceleration( );
    quaternion = get_quaternion( );
    dt = get_dt( );
    angular_velocity = get_angular_velocity(  );
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
    if( (fp=fopen(fn,"r"))==NULL )
        err_sys("fopen %s", fn);
    return fp ;
}		/* -----  end of method Sensors::open_source  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Sensors
 *      Method:  Sensors :: get_val
 * Description:  Returns value from next line of file pointer in specified
 * format. Functions that call this can take advantage of the fact the the type of
 * the vars in the va_list is unknown to get_val. This allows hex to be read
 * into a double e.g.
 *
 * Suppose the string stored in line is the hex value of some double:
 *
 * double val;
 * get_val(fp,"foo","%lx",&val);
 *
 * Note that this would not work if vsscanf were used in the calling function in
 * this way.
 *
 *--------------------------------------------------------------------------------------
 */
    void
Sensors::get_val ( FILE* fp, const char *str, const char *fmt, ... )
{
    char line[MAXLINE];
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
    FILE *fp;
    double alt;
    char str[4];

    if( altitudeIsRealTime )
    {
        fp = open_source( altitude_fn.c_str() );
    }
    else
    {
        fp = altitude_fp;
    }
    strcpy( str, (altitudeIsHex) ? "%lx" : "%lf" );
    get_val( fp, "alt", str, &alt );
    if( altitudeIsRealTime ) fclose(fp);
    return alt;
}		/* -----  end of method Sensors::get_altitude  ----- */

    double
Sensors::get_dt ( )
{
    FILE *fp;
    double d;
    char str[4];

    if( dtIsRealTime )
    {
        fp = open_source(dt_fn.c_str());
    }
    else
    {
        fp = dt_fp;
    }
    strcpy( str, (dtIsHex) ? "%lx" : "%lf" );
    get_val( fp, "dt", str, &d );
    if( dtIsRealTime ) fclose(fp);
    return d;
}		/* -----  end of method Sensors::get_dt  ----- */


    cv::Vec3d
Sensors::get_acceleration ( )
{
    FILE *fp;
    cv::Vec3d acc;
    char str[12];

    if( accelerationIsRealTime )
    {
        fp = open_source( acceleration_fn.c_str() );
    }
    else
    {
        fp = acceleration_fp;
    }
    strcpy( str, (accelerationIsHex) ? "%lx,%lx,%lx" : "%lf,%lf,%lf" );
    get_val( fp, "acc", str, &acc[0], &acc[1], &acc[2] );
    if( accelerationIsRealTime ) fclose(fp);
    return acc;
}		/* -----  end of method Sensors::get_acceleration  ----- */

    cv::Vec3d
Sensors::get_angular_velocity ( )
{
    FILE *fp;
    cv::Vec3d w;
    char str[12];

    if( angular_velocityIsRealTime )
    {
        fp = open_source(angular_velocity_fn.c_str());
    }
    else
    {
        fp = angular_velocity_fp;
    }
    strcpy( str, (angular_velocityIsHex) ? "%lx,%lx,%lx" : "%lf,%lf,%lf" );
    get_val( fp, "w", str, &w[0], &w[1], &w[2] );
    if( angular_velocityIsRealTime ) fclose(fp);
    return w;
}		/* -----  end of method Sensors::get_angular_velocity  ----- */

    cv::Vec4d
Sensors::get_quaternion ( )
{
    FILE *fp;
    cv::Vec4d q;
    char str[16];

    if( quaternionIsRealTime )
    {
        fp = open_source( quaternion_fn.c_str() );
    }
    else
    {
        fp = quaternion_fp;
    }
    strcpy( str, (quaternionIsHex) ? "%lx,%lx,%lx,%lx" : "%lf,%lf,%lf,%lf" );
    get_val( fp, "quat", str, &q[0], &q[1], &q[2], &q[3] );
    if( quaternionIsRealTime ) fclose(fp);
    return q;
}		/* -----  end of method Sensors::get_acceleration  ----- */

