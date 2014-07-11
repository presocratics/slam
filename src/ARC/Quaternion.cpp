#include "Quaternion.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Quaternion
 *      Method:  Quaternion :: euler
 * Description:  Returns Euler angles of Quaternion.
 *--------------------------------------------------------------------------------------
 */
    cv::Vec3d
Quaternion::euler ( )
{
    cv::Vec3d r;
    r[0] = atan2( 2*coord[1]*coord[2]+2*coord[3]*coord[0],pow(coord[2],2) 
            - pow(coord[1],2)-pow(coord[0],2)+ pow(coord[3],2) );
    r[1] = -1*asin( 2*coord[0]*coord[2]-2*coord[3]*coord[1] );
    r[2] = atan2( 2*coord[0]*coord[1]+2*coord[3]*coord[2],pow(coord[0],2) 
            + pow(coord[3],2)-pow(coord[2],2)-pow(coord[1],2) );

    return r ;
}		/* -----  end of method Quaternion::euler  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Quaternion
 *      Method:  Quaternion :: rotation
 * Description:  Returns rotation matrix from quaternion.
 *--------------------------------------------------------------------------------------
 */
    cv::Matx33d
Quaternion::rotation ( )
{
    cv::Matx33d m;
	// -(!) Different From Wikipedia ... ??
	m(0, 0) = pow(coord[3], 2) + pow(coord[0], 2) - pow(coord[1], 2) - pow(coord[2], 2);
	m(1, 0) = 2 * (coord[0]*coord[1] + coord[3]*coord[2]);
	m(2, 0) = 2 * (coord[0]*coord[2] - coord[3]*coord[1]);
	m(0, 1) = 2 * (coord[0]*coord[1] - coord[3]*coord[2]);
	m(1, 1) = pow(coord[3], 2) - pow(coord[0], 2) + pow(coord[1], 2) - pow(coord[2], 2);
	m(2, 1) = 2 * (coord[3]*coord[0] + coord[1]*coord[2]);
	m(0, 2) = 2 * (coord[3]*coord[1] + coord[0]*coord[2]);
	m(1, 2) = 2 * (coord[1]*coord[2] - coord[3]*coord[0]);
	m(2, 2) = pow(coord[3], 2) - pow(coord[0], 2) - pow(coord[1], 2) + pow(coord[2], 2);
    return m ;
}		/* -----  end of method Quaternion::rotation  ----- */

