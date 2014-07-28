#!/usr/bin/awk -f
BEGIN { 
    dt=0;
    FS=",";
    OFS=",";
}
{
    dt=$1-old;
    old=$1;
    print dt,dt/1e6;
}

