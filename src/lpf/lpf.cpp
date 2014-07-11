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
#define MAXLINE 1024            /*  */
#define FILTER_ORDER 4            /*  */

double filter ( double *x, const double *coeffs, int index, int order );

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  filter
 *  Description:  Implements a low pass filter.
 *  x: inputs
 *  coeffs: filter coeffs
 *  index: index of first value
 *  order: number of elements in x and coeffs
 * =====================================================================================
 */
    double
filter ( double *x, const double *coeffs, int index, int order )
{
    double y;
    int i;
    for( i=0, y=0; i<order; ++i,++index )
    {
        y+=x[i]*coeffs[index%order];
    }
    return y;
}		/* -----  end of function filter  ----- */

int main( int argc, char **argv )
{
    char *line;
    // coeff for n-0 at top, n-N at end.
    const double coeffs[FILTER_ORDER]={
        0.50,
        0.25,
        0.125,
        0.125,
    };
    double *x,*y,*z;
    
    // Callocs init to zeros.
    x	= (double *) calloc ( (size_t)(FILTER_ORDER), sizeof(double) );
    if ( x==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    y	= (double *) calloc ( (size_t)(FILTER_ORDER), sizeof(double) );
    if ( y==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    z	= (double *) calloc ( (size_t)(FILTER_ORDER), sizeof(double) );
    if ( z==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    double out[3];

    if( argc!=1 )
    {
        printf("Usage: %s <input\n", argv[0] );
        printf("Applies LPF on an input triple. Filter coeffs are compiled in.\n");
        exit(EXIT_FAILURE);
    }

    line	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

    int i;
    for( i=0; fgets( line, MAXLINE, stdin )!=NULL; ++i )
    {
        sscanf( line, "%lf,%lf,%lf", x+i%FILTER_ORDER, y+i%FILTER_ORDER, z+i%FILTER_ORDER );
        out[0] = filter(x,coeffs, i, FILTER_ORDER);
        out[1] = filter(y,coeffs, i, FILTER_ORDER);
        out[2] = filter(z,coeffs, i, FILTER_ORDER);
        printf("%lf,%lf,%lf\n", out[0], out[1], out[2] );
    }

    free (line);
    line	= NULL;
    free (x);
    x	= NULL;
    free (y);
    y	= NULL;
    free (z);
    z	= NULL;
	return 0;
}

