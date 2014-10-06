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

// Camera calibration parameters
const cv::Point2d center( 314.27445, 219.26341 ); // Center pixel?
const cv::Point2d focal(  772.09201, 768.33574 ); // focal length
// Camera orientation w.r.t. IMU
Quaternion qbc(cv::Vec4d(0.50798 , 0.50067, 0.49636, -0.49489) );
cv::Matx33d Rb2c = qbc.rotation();
cv::Matx33d Rc2b = Rb2c.t();

void image2body ( const cv::Point2d source, cv::Point2d& y );
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  image2body
 *  Description:  
 * =====================================================================================
 */
    void
image2body ( const cv::Point2d source, cv::Point2d& y )
{
    cv::Point2d p;
    cv::Point3d S;
    cv::Matx31d Yic;
    cv::Matx31d Yib;

    // Center coordinates, adjust for focal length, and normalize z-coordinate
    p=source-center;
    p.x/=focal.x;
    p.y/=focal.y;
    S = cv::Point3d( p.x, p.y, 1 );
    S *= 1/sqrt(p.x*p.x+p.y*p.y+1);

    Yic = cv::Matx31d( S.x, S.y, S.z);
    // Transform to body
    Yib = Rc2b*Yic;
    y.x=Yib(1)/Yib(0);
    y.y=Yib(2)/Yib(0);
    return;
}		/* -----  end of function image2body  ----- */

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

    // TODO: move these to a config file.
    // Camera calibration parameters

    line	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

    while( fgets( line, MAXLINE, stdin )!=NULL )
    {
        int ID;
        cv::Point2d source, reflection;
        cv::Point2d sbody, rbody;
        if( line[0] == '\n' )
        {
            printf("%s",line);
            continue;
        }
        reflection = cv::Point2d(0,0);
        sscanf( line, "%d,%lf,%lf,%lf,%lf", &ID, &source.x, &source.y, &reflection.x, &reflection.y );

        /* calculate body frame coordinates */
        image2body( source, sbody );
        image2body( reflection, rbody );
        
        if(reflection == cv::Point2d(0,0))
            printf("%d,%.17lf,%.17lf\n",ID,sbody.x, sbody.y);
        else
            printf("%d,%.17lf,%.17lf,%.17lf,%.17lf\n",ID,sbody.x, sbody.y ,rbody.x, rbody.y);
    }

    free (line);
    line	= NULL;
	return 0;

    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
