/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  SLAM algorithm for reflection code.
 *          Input should be files/FIFO in the following format:
 *          SENSORS:
 *                  altitude: float
 *              acceleration: float,float,float
 *                        dt: float
 *                quaternion: float,float,float,float
 *          angular velocity: float,float,float
 *
 *          IMAGE:
 *          ID1,X1,Y1,X2,Y2
 *          ...
 *          IDN,X1,Y1,X2,Y2
 *          \n (end of timestep denoted by newline)
 *
 *          Output to STDOUT:
 *          X,V
 *          ID1,X,Y,Z
 *          ...
 *          IDN,X,Y,Z
 *          \n (end of timestep denoted by newline)
 *
 *        Version:  1.0
 *        Created:  07/07/2014 05:45:59 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (MHM), miller7@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */

#include "main.h"
#include <iostream>
#include <fstream>

//#include "ourerr.hpp"
#define PINIT 1e-4            /*  */
#define THRESH 2e-10           /*  */
#define MAXLINE 1024
using std::cout; 
using std::cerr; 
using std::endl;

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  help
 *  Description:  
 * =====================================================================================
 */
    void
help ( const char *str)
{
    fprintf(stderr, "Usage: %s [measurements] < sensors\n", str);
    return;
}		/* -----  end of function help  ----- */

int main( int argc, char **argv )
{
    /* Handle input */
    if (argc>1)
    {
        if (!strcmp(argv[1],"-h" ))
        {
            help(argv[0]);
            exit(EXIT_FAILURE);
        }
        else
        {
            //TODO: get meas filename
        }
    }

    /* Intialize */
    const double Q0=28;
    const double R0=25;
    States mu, mu_prev;
    Sensors sense;

    mu.X[2] = -1; /* Constant altitude for now */

    cv::Mat P=cv::Mat::eye(9,9,CV_64F);
    blockAssign(P, PINIT*cv::Mat::eye(3,3,CV_64F), cv::Point(0,0));
    blockAssign(P, PINIT*cv::Mat::eye(3,3,CV_64F), cv::Point(6,6));

    mu.setb(cv::Vec3d(0,0,0));

    const double scaleW=1;
    const double scaleH=1;
    const int width=640;
    const int height=480;
    cv::Mat rtplot=cv::Mat::zeros(width, height, CV_8UC3);


    /* Enter main loop */
    int u;
    while ((u=sense.update())!=-1)
    {
        double dt;
        cv::Vec3d old_pos;
        cv::Mat G, Q, R, K, H, F;
        States f, kmh;

        if ((u & UPDATE_ACC)>0) dt=sense.acc.get_dt();
        else if ((u & UPDATE_ANG)>0) dt=sense.ang.get_dt();
        else if ((u & UPDATE_QUAT)>0) dt=sense.quat.get_dt();
        else
        {
            fprintf(stderr, "No update, so what are we doing here?\n");
            exit(EXIT_FAILURE);
        }
        old_pos=mu.X;
        f=mu.dynamics(sense);
    }
    return 0;
}

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

