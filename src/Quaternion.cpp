#include "Quaternion.hpp"

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

