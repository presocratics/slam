#!/usr/bin/env sh
# makemap.sh
# Martin Miller
# Created: 2015/07/30
# Runs SLAM code and outputs to KML
numwaypoints=200
tmpfile=$(date +%s%N)
slam $1 $2 $3 $4|awk -F, 'BEGIN {printf("No,Time,UTM-Zone,UTM-Ch,UTM-East,UTM-North\n")} \
    {printf("%d,%s,16,T,%s,%s\n",NR,strftime("%T",$1),$3,$2)}' > $tmpfile
gpsbabel -iunicsv,grid=utm -f $tmpfile -x position,distance=6m,time=10 -x transform,rte=wpt \
-x simplify,count=$numwaypoints  -okml -F $5
rm $tmpfile
