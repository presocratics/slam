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
    char str[20];
    strcpy( str, (isHex) ? "%d,%lx,%lx,%lx,%lx" : "%d,%lf,%lf,%lf,%lf" );
    std::vector<projection>::iterator it=matches.begin();
    for( ; it!=matches.end(); ++it )
    {
        get_val( fp, "image", str, &it->id,&it->source.x, &it->source.y,
                &it->reflection.x, &it->reflection.y );
    }
    return ;
}		/* -----  end of method imageSensor::get_projections  ----- */


