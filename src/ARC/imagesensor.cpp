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
 * Description:  Writes latest data to vector of matches. Returns -1 when eof
 * and -2 when fifo would block.
 *--------------------------------------------------------------------------------------
 */
    int
ImageSensor::get_projections ( )
{
    std::cout << "IMGSENSE gp" << std::endl;
    int rv;
    matches.clear();
    char str[20];
    strcpy( str, (isHex) ? "%d,%lx,%lx,%lx,%lx" : "%d,%lf,%lf,%lf,%lf" );
    while(1)
    {
        projection pj;
        pj.reflection=NONREF; // Initialize to NONREF
        rv = get_val( fp, "image", str, &pj.id,&pj.source.x, &pj.source.y, 
                &pj.reflection.x, &pj.reflection.y );
        if( rv<0 )
            return rv;
        matches.push_back(pj);
    }

    return rv ;

}		/* -----  end of method imageSensor::get_projections  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ImageSensor
 *      Method:  ImageSensor :: getNumRef
 * Description:  Returns the number of reflected and nonreflected features.
 *--------------------------------------------------------------------------------------
 */
    void
ImageSensor::getNumFeatures ( int *refl, int *nonrefl ) const
{
    *refl=0;
    *nonrefl=0;
    for( cMatchIter mi=matches.begin(); mi!=matches.end(); ++mi )
    {
        ( mi->isRef() ) ? ++(*refl) : ++(*nonrefl);
    }
}		/* -----  end of method ImageSensor::getNumRef  ----- */

