/*
 * =====================================================================================
 *
 *       Filename:  imagesensor.cpp
 *
 *    Description:  ImageSensor class implementation. Retrieves latest body
 *    frame data from a file and stores matched features in a vector.
 *
 *        Version:  1.0
 *        Created:  06/24/2014 11:39:30 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (MHM), miller7@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */
#include "imagesensor.hpp"

    void
ImageSensor::update ( )
{
    get_projections();
    return ;
}		/* -----  end of method imageSensor::update  ----- */


    FILE *
ImageSensor::open_source ( const char *fn )
{
    FILE *fp;
    char line[MAXLINE];
    if( (fp=fopen(fn,"r"))==NULL )
        err_sys("fopen body");
    return fp ;
}		/* -----  end of method imageSensor::open_source  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ImageSensor
 *      Method:  ImageSensor :: get_projections
 * Description:  Writes latest data to vector of matches.
 *--------------------------------------------------------------------------------------
 */
    void
ImageSensor::get_projections ( )
{
    char str[4];
    strcpy( str, (isHex) ? "%lx,%lx,%lx,%lx" : "%lf,%lf,%lf,%lf" );
    std::vector<projection>::iterator it=matches.begin();
    for( ; it!=matches.end(); ++it )
    {
        get_val( fp, "image", str, &it->source.x, &it->source.y,
                &it->reflection.x, &it->reflection.y );
    }
    return ;
}		/* -----  end of method imageSensor::get_projections  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ImageSensor
 *      Method:  ImageSensor :: get_val
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
ImageSensor::get_val ( FILE* fp, const char *str, const char *fmt, ... )
{
    char *line = new char[MAXLINE];
    va_list ap;
    va_start(ap,fmt);

    if( (fgets(line,MAXLINE,fp ))==NULL )
        err_sys("fgets %s", str);
    vsscanf( line, fmt, ap );
    va_end(ap);

    return ;
}		/* -----  end of method ImageSensor::get_val  ----- */
