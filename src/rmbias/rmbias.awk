#!/usr/bin/awk -f
BEGIN { 
    if( ARGC<4 )
    {
        printf("Usage: %s v1 v2 v3 infile\n", ARGV[0]);
        exit;
    }
    FS=",";
    OFS=",";
    v1=ARGV[1];
    v2=ARGV[2];
    v3=ARGV[3];
    ARGV[1]=ARGV[4];
    ARGC=2;
}
{ print $1-v1, $2-v2, $3-v3 }
