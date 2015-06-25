/*
 * =====================================================================================
 *
 *       Filename:  arc.cpp
 *
 *    Description:  General function needed for arc work.
 *
 *        Version:  1.0
 *        Created:  06/22/2015 03:08:02 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (MHM), miller7@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */
#include "arc.hpp"

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  blockAssign
 *  Description:  Writes a submatrix to dst at the starting point tl
 * =====================================================================================
 */
    void
blockAssign ( cv::Mat dst, cv::Mat block, cv::Point tl )
{
    cv::Rect roi;
    cv::Mat sub_dst;

    roi = cv::Rect(tl, block.size());
    sub_dst = dst(roi);
    block.copyTo(sub_dst);
    return;
}        /* -----  end of function blockAssign  ----- */


