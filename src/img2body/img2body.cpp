/*
 * =====================================================================================
 *
 *       Filename:  img2body.cpp
 *
 *    Description:  takes in img frame coords and outputs body frame coords
 *
 *        Version:  1.0
 *        Created:  06/24/2014 03:38:49 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Hong-Bin Yoon (), yoon48@illinois.edu
 *   Organization:  
 *
 * =====================================================================================
 */
#include    <math.h>
#include    <iostream>
#include    <stdlib.h>
#include    <cstdlib>
#include    <cstdio>
#include    "Quaternion.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#define MAXLINE 512 

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description: img2body IO format
 *               ref, ID, x1, y1, x2, y2
 *               
 *               ref = 0 for relection
 *               ref = 1 for good features
 *               ID = signature value assigned from vision algorithm
 *               x1 = real object coord
 *               x2 = reflection coord IF it exists. Otherwise set to 0.
 *
 * =====================================================================================
 */
    int
main ( int argc, char *argv[] )
{
    char* line;
    ssize_t rs;

    float u0 = 314.27445;
    float v0 = 219.26341;
    float fu = 772.09201;
    float fv = 768.33574;

    Quaternion qbc = Quaternion(cv::Vec4d(0.50798 , 0.50067, 0.49636, -0.49489) );
    cv::Matx33d Rb2c = qbc.rotation();
    cv::Matx33d Rc2b = Rb2c.t();

    line	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

        while( fgets( line, MAXLINE, stdin )!=NULL )
    {
    int ref, ID;
    float xp, yp, xpp, ypp;

        if( line[0] == '\n' )
        {
            printf("%s",line);

            continue;
        }
        sscanf( line, "%d, %d,%f,%f,%f,%f", &ref, &ID, &xp, &yp, &xpp, &ypp );

        /* calculate body frame coordinates */
        float p1 = (xp-u0)/fu;
        float p2 = (yp-v0)/fv;
        float Zs = 1/sqrt(pow(p1,2)+pow(p2,2)+1);
        float Xs = p1*Zs;
        float Ys = p2*Zs;
        cv::Matx31d Yic = cv::Matx31d( Xs, Ys, Zs);
        cv::Mat Yib = (cv::Mat)Rc2b * (cv::Mat)Yic;

        float p11 = (xpp-u0)/fu;
        float p21 = (ypp-v0)/fv;
        float Zs1 = 1/sqrt(pow(p11,2)+pow(p21,2)+1);
        float Xs1 = p11*Zs1;
        float Ys1 = p21*Zs1;
        cv::Matx31d Yic1 = cv::Matx31d( Xs1, Ys1, Zs1);
        cv::Mat Yib1 = (cv::Mat)Rc2b * (cv::Mat)Yic1;
        
        printf("%d, %d,%f,%f,%f,%f,%f\n",ref, ID,Yib.at<double>(1,0)/ Yib.at<double>(0,0), Yib.at<double>(2,0)/ Yib.at<double>(0,0) ,Yib1.at<double>(1,0)/ Yib1.at<double>(0,0), Yib1.at<double>(2,0)/ Yib1.at<double>(0,0) );
        fflush(stdout);

    }

    free (line);
    line	= NULL;
	return 0;

    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
