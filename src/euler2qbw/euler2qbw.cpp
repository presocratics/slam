/*
 * =====================================================================================
 *
 *       Filename:  euler2qbw.cpp
 *
 *    Description:  Reads euler angles frome STDIN and writes corresponding
 *    quaternion to STDOUT.
 *    Input expected as a triple x,y,z
 *
 *        Version:  1.0
 *        Created:  06/10/2014 03:28:04 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (MHM), miller7@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */

#include <cmath>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#define MAXLINE 1024            /*  */

void euler2quaternion ( double x, double y, double z, double *q );
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  euler2quaternion
 *  Description:  Converts euler angles provided by x,y,z to quaternion.
 * =====================================================================================
 */
    void
euler2quaternion ( double x, double y, double z, double *q )
{
    q[0] = sin(x/2)*cos(y/2)*cos(z/2)-cos(x/2)*sin(y/2)*sin(z/2);
    q[1] = cos(x/2)*sin(y/2)*cos(z/2)+sin(x/2)*cos(y/2)*sin(z/2);
    q[2] = cos(x/2)*cos(y/2)*sin(z/2)-sin(x/2)*sin(y/2)*cos(z/2);
    q[3] = cos(x/2)*cos(y/2)*cos(z/2)+sin(x/2)*sin(y/2)*sin(z/2);

    return;
}		/* -----  end of function euler2quaternion  ----- */

int main( int argc, char **argv )
{
    char *line;
    double *quaternion;

    if( argc!=1 )
    {
        printf("Usage: %s\n\
Reads Euler angles as triple x,y,z from STDIN.\n", argv[0] );
        exit(EXIT_FAILURE);
    }
    line	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    
    quaternion	= (double *)calloc ( (size_t)(4), sizeof(double) );
    if ( quaternion==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }



    while( fgets( line, MAXLINE, stdin )!=NULL )
    {
        double x, y, z;
        sscanf( line, "%lf,%lf,%lf", &x, &y, &z );
        euler2quaternion(x,y,z, quaternion);
        printf("%.8e,%.8e,%.8e,%.8e\n", quaternion[0], quaternion[1], quaternion[2], quaternion[3] );
        fflush(stdout);
    }
    free (line);
    line	= NULL;
    free (quaternion);
    quaternion	= NULL;
	return 0;
}

