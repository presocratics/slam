/*
 * =====================================================================================
 *
 *       Filename:  rotate.c
 *
 *    Description:  Rotate coordinates by euler angles. Use radians.
 *
 *        Version:  1.0
 *        Created:  06/04/2014 11:23:20 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (), witsquash@lavabit.com
 *   Organization:  
 *
 * =====================================================================================
 */


#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <Quaternion.hpp>
#include <cv.h>
#include "ourerr.h"
#define USE_MATH_DEFINES
#define MAXLINE 8192            /*  */
#define EULER_ARG "-e"            /*  */
#define EULER 0            /*  */
#define QUATERNION 1            /*  */
#define QUATERNION_ARG "-q"            /*  */
#define E2QPATH "/home/marty/ARC/slam/bin/euler2qbw"            /* TODO: Fix this garbage. */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  help
 *  Description:  
 * =====================================================================================
 */
    void
help ( )
{
    printf("OPTIONS\n");
    printf("\t%-s\t\t%s\n", QUATERNION_ARG, "Provide 4 quaternion parameters.");
    printf("\t%-s\t\t%s\n", EULER_ARG, "Provide 3 Euler parameters.");
    printf("SYMBOLS\n");
    printf("\t%-s\t\t%s\n", "p", "Use pi for parameter.");
    printf("\t%-s\t\t%s\n", "h", "Use pi/2 for parameter.");
    printf("\t%-s\t\t%s\n", "f", "Use pi/4 for parameter.");
    return ;
}		/* -----  end of function help  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  
 * =====================================================================================
 */
    int
main ( int argc, char *argv[] )
{
    int mode;
    int nparams;
    double params[4];
    Quaternion *quat;
    if( argc==1 )
    {
        printf("Usage: %s [options] [parameters]\n", argv[0]);
        help();
        exit(EXIT_FAILURE);
    }
    if( !strcmp(argv[1], QUATERNION_ARG) )
        mode=QUATERNION;
    else if( !strcmp(argv[1], EULER_ARG) )
        mode=EULER;
    else
    {
        printf("-q or -e required\n");
        printf("Usage: %s [options] [parameters]\n", argv[0]);
        help();
        exit(EXIT_FAILURE);
    }
    if( mode==EULER )
    {
        nparams=3;
        if( argc!=5 )
        {
            printf("Euler requires exactly 3 parameters.\n");
            exit(EXIT_FAILURE);
        }
    }
    else if( mode==QUATERNION )
    {
        nparams=4;
        if( argc!=6 )
        {
            printf("Quaternion requires exactly 4 parameters.\n");
            exit(EXIT_FAILURE);
        }
    }

    int i;
    for( i=0; i<nparams; ++i )
    {
        switch ( argv[i+2][0] ) {
            case 'p':
                params[i]=M_PI;
                break;

            case 'h':	
                params[i]=M_PI_2;
                break;

            case 'f':	
                params[i]=M_PI_4;
                break;

            default:	
                sscanf(argv[i+2], "%lf", params+i);
                break;
        }				/* -----  end switch  ----- */
    }
    if( mode==EULER )
    {
        char *line;
        FILE *fp;
        char cmd[MAXLINE];
        double q[4];

        
        line	= new char[MAXLINE];
        sprintf(cmd, "echo %f,%f,%f|%s", params[0], params[1], params[2], E2QPATH );
        fp = popen(cmd, "r");
        fgets(line, MAXLINE, fp);
        pclose(fp);
        sscanf( line, "%lf,%lf,%lf,%lf", q, q+1, q+2, q+3 );


        delete line;
        quat= new Quaternion(q[0], q[1], q[2], q[3]);
    }
    else
    {
        quat = new Quaternion(params[0], params[1], params[2], params[3]);
    }
    const cv::Matx33d rot = quat->rotation();
    char *line;
    line = new char[MAXLINE];
    while( fgets( line, MAXLINE, stdin )!=NULL )
    {
        cv::Vec3d in, out;
        sscanf( line, "%lf,%lf,%lf", &in[0], &in[1], &in[2] );
        out = rot*in;
        printf("%f,%f,%f\n", out[0], out[1], out[2] );
    }
    delete line;
    delete quat;

    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
