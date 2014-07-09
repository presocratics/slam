/*
 * =====================================================================================
 *
 *       Filename:  img2body.cpp
 *
 *    Description:  Reads in img frame coords from STDIN and writes body frame
 *    coords to STDOUT.
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
    if( argc>1 )
    {
        printf("Usage: %s\n\
Reads image frame coordinates from STDIN in format:\n\
    id,x1,y1,x2,y2\n", argv[0] );
        exit(EXIT_FAILURE);
    }
    char* line;
    ssize_t rs;

    const double u0 = 314.27445;
    const double v0 = 219.26341;
    const double fu = 772.09201;
    const double fv = 768.33574;

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
        int ID;
        double xp, yp, xpp, ypp;
        double p1, p2,Zs, Xs, Ys;
        cv::Matx31d Yic, Yic1;
        cv::Mat Yib, Yib1;

        if( line[0] == '\n' )
        {
            printf("%s",line);
            continue;
        }
        sscanf( line, "%d,%lf,%lf,%lf,%lf", &ID, &xp, &yp, &xpp, &ypp );

        /* calculate body frame coordinates */
        p1 = (xp-u0)/fu;
        p2 = (yp-v0)/fv;
        Zs = 1/sqrt(pow(p1,2)+pow(p2,2)+1);
        Xs = p1*Zs;
        Ys = p2*Zs;
        Yic = cv::Matx31d( Xs, Ys, Zs);
        Yib = (cv::Mat)Rc2b * (cv::Mat)Yic;

        double p11 = (xpp-u0)/fu;
        double p21 = (ypp-v0)/fv;
        double Zs1 = 1/sqrt(pow(p11,2)+pow(p21,2)+1);
        double Xs1 = p11*Zs1;
        double Ys1 = p21*Zs1;
        Yic1 = cv::Matx31d( Xs1, Ys1, Zs1);
        Yib1 = (cv::Mat)Rc2b * (cv::Mat)Yic1;
        
        printf("%d,%.17lf,%.17lf,%.17lf,%.17lf\n",ID,Yib.at<double>(1,0)/ Yib.at<double>(0,0), Yib.at<double>(2,0)/ Yib.at<double>(0,0) ,Yib1.at<double>(1,0)/ Yib1.at<double>(0,0), Yib1.at<double>(2,0)/ Yib1.at<double>(0,0) );
        fflush(stdout);

    }

    free (line);
    line	= NULL;
	return 0;

    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
