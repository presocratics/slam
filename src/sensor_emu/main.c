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
    int usedel=0;
    
    line	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    
    int sz = MAXLINE-1;

    if( argc<2 )
    {
        printf("Usage: %s data [-d]\n", argv[0]);
        printf("OPTIONS:\n");
        printf("-d\tinputs are deltas, not absolute timestamps\n");
        exit(EXIT_FAILURE);
    }
    else if( argc>2 )
    {
        usedel=1;
    }
    if( (fp=fopen(argv[1], "r"))==NULL )
        err_sys("fopen");
    if( (fgets( line, sz, fp ))==NULL ) // Get first line
        err_sys("fgets");
    sscanf( line, "%d", &next );
    printf( "%s", line );
    while( fgets( line, sz, fp )!=NULL )
    {
        cur = next;
        sscanf( line, "%d", &next );
        del = (usedel) ? next : next-cur;
        req.tv_sec=del/1e6;
        req.tv_nsec=(int)1e3*(del%(int)1e6); /* convert microseconds to nanoseconds */
        if( nanosleep(&req,&rem)==-1 )
            err_sys("nanosleep");
        printf( "%s", line );
        fflush(stdout);
    }
    fclose(fp);
    free (line);
    line	= NULL;

    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
