/*
 * =====================================================================================
 *
 *       Filename:  slamsensor.cpp
 *
 *    Description:  SlamSensor class implementation. Handles Slam Input and Output.
 *
 *        Version:  1.0
 *        Created:  07/30/2014 11:39:30 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Hong-Bin Yoon
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */
#include "slamsensor.hpp"


typedef struct slamprojection Slamprojection;

/*
 *--------------------------------------------------------------------------------------
 *       Class:  SlamSensor
 *      Method:  SlamSensor :: get_projections
 * Description:  Writes latest data to vector of matches.
 *--------------------------------------------------------------------------------------
 */
    void
SlamSensor::get_projections ( )
{
    slammatches.clear();
    slamprojection pj;

    /* get X and V */
    char str[32];
    strcpy( str, (isHex) ? "%lx,%lx,%lx,%lx,%lx,%lx" : "%lf,%lf,%lf,%lf,%lf,%lf" );
    get_val(fp, "XV", str, &X[0], &X[1], &X[2], &V[0], &V[1], &V[2]  );

    /* get Feature Points */
    strcpy( str, (isHex) ? "%d,%lx,%lx,%lx" : "%d,%lf,%lf,%lf" );
    while( get_val( fp, "image", str, &pj.id,&pj.source.x, &pj.source.y, 
                &pj.source.z )!=-1 )
    {
        slammatches.push_back(pj);
    }
    return ;
}		/* -----  end of method SlamSensor::get_projections  ----- */


