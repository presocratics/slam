/*
 * =====================================================================================
 *
 *       Filename:  kalman.cpp
 *
 *    Description:  Applies FIR filter to STDIN using provided coefficients. Writes to STDOUT.
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
#include <ourerr.hpp>
#define MAXLINE 1024            /*  */

double filter ( double *x, const double *coeffs, int index, int order );

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  filter
 *  Description:  Implements a FIR filter.
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
    for( i=0, y=0; i<order; ++i )
    {
        y+=x[(order+index-i)%order]*coeffs[i];
    }
    return y;
}		/* -----  end of function filter  ----- */

int main( int argc, char **argv )
{
    if( argc<2 )
    {
        printf("Usage: %s coeffs < data\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    FILE *coeff_fd;
    int order;
    double *coeffs;
    char *line;
    double *x,*y,*z;

    line	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

    if( (coeff_fd=fopen( argv[1], "r" ))==NULL )
        err_sys("fopen: coeffs");
    for( order=0; fgets( line, MAXLINE, coeff_fd ); ++order ) ; // Get number of coefficients by counting lines
    
    coeffs	= (double *) calloc ( (size_t)(order), sizeof(double) );
    if ( coeffs==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    fclose(coeff_fd);

    if( (coeff_fd=fopen( argv[1], "r" ))==NULL )
        err_sys("fopen: coeffs");
    for( int i=0; i<order; ++i ) // Load coefficients
    {
        fscanf( coeff_fd, "%lf", coeffs+i );
    }
    fclose(coeff_fd);
    
    // Assume callocs init to 0s
    x	= (double *) calloc ( (size_t)(order), sizeof(double) );
    if ( x==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    y	= (double *) calloc ( (size_t)(order), sizeof(double) );
    if ( y==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    z	= (double *) calloc ( (size_t)(order), sizeof(double) );
    if ( z==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

    int i;
    for( i=0; fgets( line, MAXLINE, stdin )!=NULL; ++i )
    {
        double out[3];
        sscanf( line, "%lf,%lf,%lf", x+(i%order), y+(i%order), z+(i%order) );
        out[0] = filter(x,coeffs, i%order, order);
        out[1] = filter(y,coeffs, i%order, order);
        out[2] = filter(z,coeffs, i%order, order);
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
    free (coeffs);
    coeffs	= NULL;
	return 0;
}

