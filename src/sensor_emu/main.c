/*
 * =====================================================================================
 *
 *       Filename:  imu.c
 *
 *    Description:  Simulate the IMU and return gyro values. Uses nanosleep() to
 *    supply values at the same frequency as recorded. Gyro values are printed
 *    to stdout. Can also be run on acc.
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


#include	<stdlib.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include "ourerr.h"
#define MAXLINE 1024            /*  */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  
 * =====================================================================================
 */
    int
main ( int argc, char *argv[] )
{
    FILE *fp;
    char *line;
    int del;
    struct timespec req, rem;
    long cur, next;
    
    line	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    
    int sz = MAXLINE-1;
    //cur.t=9;

    if( argc!=2 )
    {
        printf("Usage: %s gyrodata\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    if( (fp=fopen(argv[1], "r"))==NULL )
        err_sys("fopen");
    if( (fgets( line, sz, fp ))==NULL ) // Get the header
        err_sys("fgets");
    if( (fgets( line, sz, fp ))==NULL ) // Get first line
        err_sys("fgets");
    sscanf( line, "%d", &next );
    while( fgets( line, sz, fp )!=NULL )
    {
        cur = next;
        printf("%s", line );
        sscanf( line, "%d", &next );
        del = next-cur;
        req.tv_sec=0;
        req.tv_nsec=(int)1e3*del; /* convert microseconds to nanoseconds */
        if( nanosleep(&req,&rem)==-1 )
            err_sys("nanosleep");
        fflush(stdout);
    }
    fclose(fp);
    free (line);
    line	= NULL;

    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
