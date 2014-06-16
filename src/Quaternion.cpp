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
void quaternion2Euler(Mat src, Mat& dst)
    cv::Vec3d r;
    r[0] = atan2( 2*q[1]*q[2]+2*q[3]*q[0],pow(q[2],2) 
            - pow(q[1],2)-pow(q[0],2)+ pow(q[3],2) );
    r[1] = -1*asin( 2*q[0]*q[2]-2*q[3]*q[1] );
    r[2] = atan2( 2*q[0]*q[1]+2*q[3]*q[2],pow(q[0],2) 
            + pow(q[3],2)-pow(q[2],2)-pow(q[1],2) );

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
	m(0, 0) = pow(q[3], 2) + pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2);
	m(1, 0) = 2 * (q[0]*q[1] + q[3]*q[2]);
	m(2, 0) = 2 * (q[0]*q[2] - q[3]*q[1]);
	m(0, 1) = 2 * (q[0]*q[1] - q[3]*q[2]);
	m(1, 1) = pow(q[3], 2) - pow(q[0], 2) + pow(q[1], 2) - pow(q[2], 2);
	m(2, 1) = 2 * (q[3]*q[0] + q[1]*q[2]);
	m(0, 2) = 2 * (q[3]*q[1] + q[0]*q[2]);
	m(1, 2) = 2 * (q[1]*q[2] - q[3]*q[0]);
	m(2, 2) = pow(q[3], 2) - pow(q[0], 2) - pow(q[1], 2) + pow(q[2], 2);
    return ;
}		/* -----  end of method Quaternion::rotation  ----- */

