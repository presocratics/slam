/*
 * =====================================================================================
 *
 *       Filename:  kalman.cpp
 *
 *    Description:  Applies low pass filter to STDIN. Writes to STDOUT.
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


#include <iostream>
#include <cstdio>
#include <cstdlib>
#define MAXLINE 32            /*  */
int main( int argc, char **argv )
{
    char *line;
    ssize_t rs;
    double x, y, z;
    double xp, yp, zp;
    double alpha;

    if( argc!=2 )
    {
        printf("Usage: %s alpha\n", argv[0] );
        exit(EXIT_FAILURE);
    }
    sscanf( argv[1], "%lf", &alpha );

    xp=yp=zp=0;
    
    line	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

    while( fgets( line, MAXLINE, stdin )!=NULL )
    {
        xp=x;
        yp=y;
        zp=z;
        sscanf( line, "%lf,%lf,%lf", &x, &y, &z );
        printf("%lf,%lf,%lf\n", alpha*x+(1-alpha)*xp, alpha*y+(1-alpha)*yp, alpha*z+(1-alpha)*zp);
        fflush(stdout);
    }

    free (line);
    line	= NULL;
	return 0;
}

