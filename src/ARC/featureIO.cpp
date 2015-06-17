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

#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include "featureIO.h"

    int
FeatureIO::update ( )
{
    int rv;
    rv=get_projections();
    return rv;
}        /* -----  end of method imageSensor::update  ----- */


//    FILE *
//FeatureIO::open_source ( const char *fn )
//{
//    char cmd[1024];
//    sprintf(cmd, "tail -f -n+0 %s", fn);
//
//    FILE *fp = popen( cmd, "r");
//    int fd = fileno(fp);
//    int flags;
//    flags = fcntl(fd, F_GETFL, 0);
//    flags |= O_NONBLOCK;
//    fcntl(fd, F_SETFL, flags);
//
//    return fp ;
//}        /* -----  end of method imageSensor::open_source  ----- */

    FILE *
FeatureIO::open_source ( const char *fn )
{
    FILE *fp;
    if( (fp=fopen(fn,"r"))==NULL )
        err_sys("fopen body");
    return fp ;
}        /* -----  end of method imageSensor::open_source  ----- */
/*
 *--------------------------------------------------------------------------------------
 *       Class:  ImageSensor
 *      Method:  ImageSensor :: get_projections
 * Description:  Writes latest data to vector of matches.
 *--------------------------------------------------------------------------------------
 */
   int
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

    return 1;
}        /* -----  end of method imageSensor::get_projections  ----- */

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
 * Returns -1 on newline (end of input set).
 * Returns -2 on EOF.
 *
 *--------------------------------------------------------------------------------------
 */
    int
FeatureIO::get_val ( FILE* fp, const char *str, const char *fmt, ... )
{
    char* line = new char[MAXLINE];
    va_list ap;
    va_start(ap,fmt);

    char *r;
    if ((r=fgets(line,1024,fp))==NULL) {
        if (feof(fp)) {
            return -2;
        } else {
            err_msg("fgets");
        }
    } else {
        int rv=vsscanf( line, fmt, ap );
        va_end(ap);
        free(line);
        line = NULL;
        return rv;
    }
    std::cout << "-(!) get val error" << std::endl;
    return 0;

}        /* -----  end of method ImageSensor::get_val  ----- */
