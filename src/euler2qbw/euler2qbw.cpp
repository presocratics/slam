/*
 * =====================================================================================
 *
 *       Filename:  euler2qbw.cpp
 *
 *    Description:  Reads euler angles frome STDIN and writes corresponding
 *    quaternion to STDOUT.
 *    TODO: Determine type of input. How are the angles defined?
 *    
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

#include <cstring>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#define MAXLINE 1024            /*  */

void euler2quaternion ( double phi, double theta, double psi, double *q );
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  euler2quaternion
 *  Description:  Converts euler angles provided by x,y,z to quaternion.
 * =====================================================================================
 */
    void
euler2quaternion ( double phi, double theta, double psi, double *q )
{
    q[0] = sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2);
    q[1] = cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2);
    q[2] = cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2);
    q[3] = cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2);

    return;
}		/* -----  end of function euler2quaternion  ----- */

int main( int argc, char **argv )
{
    char *line;
    double *quaternion;
    int degrees;

    if (argc>1 && !strcmp(argv[1],"-h")) {
        printf("Usage: %s [-d]\n\
Reads Euler angles as triple x,y,z from STDIN in radians unless -d used.\n", argv[0] );
        exit(EXIT_FAILURE);
    }
    degrees=0;
    if (argc>1 && (!strcmp(argv[1], "-d"))) degrees=1;
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
        double phi, theta, psi;
        sscanf( line, "%lf,%lf,%lf", &phi, &theta, &psi );
        if (degrees) {
            phi*=M_PI/180;
            theta*=M_PI/180;
            psi*=M_PI/180;
        }
        euler2quaternion( phi, theta, psi, quaternion );
        printf("%.15lf,%.15lf,%.15lf,%.15lf\n", quaternion[0], quaternion[1], quaternion[2], quaternion[3] );
    }
    free (line);
    line	= NULL;
    free (quaternion);
    quaternion	= NULL;
	return 0;
}

