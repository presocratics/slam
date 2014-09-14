#!/usr/bin/awk -f
# Martin Miller
# mean.awk
# Calculates mean of values on the interval [t0,t1]
BEGIN { 
    if( ARGC<4 )
    {
        printf("Usage: %s t0 t1 infile\n", ARGV[0]);
        exit;
    }
    FS=",";
    OFS=",";
    t0=ARGV[1];
    t1=ARGV[2];
    ARGV[1]=ARGV[3];
    ARGC=2;
    mean=0;
    i=1;
}
NR>=t0 && NR<=t1 { 
        mean=((i-1)*mean+$1)/i;
    ++i;
}
END {
    print mean
}
