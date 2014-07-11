/*
 * =====================================================================================
 *
 *       Filename:  ourerr.c
 *
 *    Description:  Error handling code from Stevens Advanced Programming in the
 *    UNIX Environment. Appendix B page 682
 *
 *        Version:  1.0
 *        Created:  06/02/2014 10:50:05 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (), miller7@illinois.edu
 *   Organization:  
 *
 * =====================================================================================
 */

#include <errno.h>
#include <stdarg.h>
#include "ourhdr.hpp"
#include "ourerr.hpp"
static void err_doit(int, const char *, va_list);
char *pname = NULL;


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  err_ret
 *  Description:  Nonfatal error related to a system call. Print a message and
 *  return.
 * =====================================================================================
 */
    void
err_ret ( const char *fmt, ... )
{
    va_list ap;
    va_start(ap,fmt);
    err_doit(1, fmt, ap);
    va_end(ap);
    return;
}		/* -----  end of function err_ret  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  err_sys
 *  Description:  Fatal error related to a system call. Print a message and
 *  terminate.
 * =====================================================================================
 */
    void
err_sys ( const char *fmt, ... )
{
    va_list ap;

    va_start(ap,fmt);
    err_doit(1, fmt, ap);
    va_end(ap);
    exit(EXIT_FAILURE);
}		/* -----  end of function err_sys  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  err_dump
 *  Description:  Fatal error related to a system call. Print a message, dump
 *  core, and terminate.
 * =====================================================================================
 */
    void
err_dump ( const char *fmt, ... )
{
    va_list ap;

    va_start(ap,fmt);
    err_doit(1, fmt, ap);
    va_end(ap);
    abort();
    exit(EXIT_FAILURE);
    return;
}		/* -----  end of function err_dump  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  err_msg
 *  Description:  Nonfatal error related to a system call. Print a message and
 *  return.
 * =====================================================================================
 */
    void
err_msg ( const char *fmt, ... )
{
    va_list ap;

    va_start(ap,fmt);
    err_doit(0, fmt, ap);
    va_end(ap);

    return;
}		/* -----  end of function err_msg  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  err_quit
 *  Description:  
 * =====================================================================================
 */
    void
err_quit ( const char *fmt, ... )
{
    va_list ap;

    va_start(ap,fmt);
    err_doit(0, fmt, ap);
    va_end(ap);

    exit(EXIT_FAILURE);
}		/* -----  end of function err_quit  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  err_doit
 *  Description:  Print a message and return to caller. Caller specifies
 *  "errnoflag"
 * =====================================================================================
 */
    static void
err_doit ( int errnoflag, const char *fmt, va_list ap )
{
    int errno_save;
    char buf[MAXLINE];
    
    errno_save = errno;
    vsprintf( buf, fmt, ap );
    if( errnoflag )
        sprintf( buf+strlen(buf), ": %s", strerror(errno_save));
    strcat(buf, "\n");
    fflush(stdout);
    fputs(buf, stderr);
    fflush(NULL);

    return;
}		/* -----  end of function err_doit  ----- */
