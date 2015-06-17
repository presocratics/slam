#!/usr/bin/awk -f
# wrap.awk
# Created: 6/15/2015
# Martin Miller
# Removes discontinuities due to angle changing between 0<->360
# Input: <degrees>
BEGIN {
    prev=$1;
    maxdel=270;
    n=0;
}
{
    diff=$1-prev;
    if (diff>maxdel && diff>0) {
        n-=1;
    } else if (-diff>maxdel && diff<0) {
        n+=1;
    }
    printf("%0.9f\n", $1+360*n);
    prev=$1;
}
