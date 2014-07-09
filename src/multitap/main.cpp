/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  Reads from stdin. Outputs to files that can be opened for
 *    writing, i.e. normal files or FIFOs that are already open for reading.
 *
 *        Version:  1.0
 *        Created:  06/09/2014 10:52:51 AM
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
//#include <sys/time.h>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>
//#include <sys/stat.h>
#include <fcntl.h>
#include <cerrno>
#include "ourerr.hpp"

#define BUFSIZE 8192           /*  */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  
 * =====================================================================================
 */
    int
main ( int argc, char *argv[] )
{
    char buf[BUFSIZE];
    int *fd;
    ssize_t rs;
    if( argc<2 )
    {
        printf( "Usage: %s file1 ... fileN\n", argv[0] );
        exit(EXIT_FAILURE);
    }
    
    fd	= (int *) calloc ( (size_t)(argc-1), sizeof(int) );
    if ( fd==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }
    // Read from stdin
    while( (rs=read( 0, buf, BUFSIZE))!=0 )
    {
        for(int i=0; i<(argc-1); ++i )
        {
            if( (fd[i]=open(argv[i+1], O_WRONLY | O_CREAT | O_NONBLOCK | O_APPEND, 0644))==-1 && (errno!=ENXIO) )
                err_sys("Cannot open %s", argv[i+1]);
            write(fd[i], buf, rs);
            close(fd[i]);
        }
    }

    free (fd);
    fd	= NULL;
    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
