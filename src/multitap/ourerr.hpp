#ifndef  ourerr_INC
#define  ourerr_INC
void err_ret ( const char *fmt, ... );
void err_sys ( const char *fmt, ... );
void err_dump ( const char *fmt, ... );
void err_msg ( const char *fmt, ... );
void err_quit ( const char *fmt, ... );
#endif   /* ----- #ifndef ourerr_INC  ----- */
