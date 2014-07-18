/*
 * =====================================================================================
 *
 *       Filename:  featureIO.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/27/2014 12:09:22 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Hong-Bin Yoon (), yoon48@illinois.edu
 *   Organization:  
 *
 * =====================================================================================
 */


#include "featureIO.h"

    void
FeatureIO::update ( )
{
    get_projections();
    return ;
}		/* -----  end of method imageSensor::update  ----- */


    FILE *
FeatureIO::open_source ( const char *fn )
{
    FILE *fp;
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
FeatureIO::get_projections ( )
{
    char str[20];
    strcpy( str, (isHex) ? "%d,%lx,%lx,%lx" : "%d,%lf,%lf,%lf" );
    std::vector<projection>::iterator it=matches.begin();
    for( ; it!=matches.end(); ++it )
    {
        get_val( fp, "image", str, &it->id,&it->source.x, &it->source.y,
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
    int
FeatureIO::get_val ( FILE* fp, const char *str, const char *fmt, ... )
{
    int rv;
    char line[MAXLINE];
    va_list ap;
    va_start(ap,fmt);

    if( (fgets(line,MAXLINE,fp ))==NULL )
        err_sys("fgets %s", str);
    rv=vsscanf( line, fmt, ap );
    va_end(ap);

    return rv ;
}		/* -----  end of method ImageSensor::get_val  ----- */
