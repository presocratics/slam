/*
 * =====================================================================================
 *
 *       Filename:  stamp.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/14/2014 03:31:58 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Hong-Bin Yoon (HBY), yoon48@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */


#include <iostream>
#include <stdlib.h>
#include <cstdlib>
#include <cstdio>
#include <string.h>
#define MAXLINE 512


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  trim
 *  Description:  removes trailing newline char 
 * =====================================================================================
 */
    void
trim ( char* line )
{
    int new_line = strlen(line) -1;
    if (line[new_line] == '\n')
        line[new_line] = '\0';
    return;
}		/* -----  end of function trim  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  
 * =====================================================================================
 */
    int
main ( int argc, char *argv[] )
{
    char* line;
    line	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

    char* timestamp;
    timestamp	= (char *) calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

    if(argc < 3)
    {
        printf("Usage: %s timestamp features_w/o_ID \n", argv[0]);
        exit(EXIT_FAILURE);
    }

    FILE* fp_t = fopen(argv[1] , "r"); 
    FILE* fp_f = fopen(argv[2] , "r");  

    while(fgets(timestamp,MAXLINE,fp_t )!=NULL)
    {
        trim(timestamp);
        while(fgets(line,MAXLINE,fp_f )!=NULL)
        {
            if(line[0] =='\n')
            {
                printf("%s",line);
                break;
            }
            else
            {
                printf("%s,%s",timestamp,line);
            }
        }
    }

    free (line);
    line	= NULL;
    free (timestamp);
    timestamp	= NULL;
    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */

