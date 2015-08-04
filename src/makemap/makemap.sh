#!/usr/bin/env sh
# makemap.sh
# Martin Miller
# Created: 2015/07/30
# Runs SLAM code and outputs to KML
numwaypoints=200
tmpfile=$(date +%s%N)
slam $1 $2 $3 $4>$tmpfile

grep "^[0-9]*\.[0-9]*," $tmpfile | \
    awk -F, 'BEGIN {printf("No,Time,UTM-Zone,UTM-Ch,UTM-East,UTM-North\n")} \
    {printf("%d,%s,16,T,%s,%s\n",NR,strftime("%T",$1),$3,$2)}' > ${tmpfile}2
gpsbabel -iunicsv,grid=utm -f ${tmpfile}2 -x position,distance=6m,time=10 -x transform,trk=wpt,del \
 -x track,pack,sdistance=0.1k -okml,points=0 -F ${tmpfile}3

grep "^[^.]*," $tmpfile |sort -t, -n -k1,1 -s  -r|sed 's/,/        /'|tac| \
    uniq -w 9 |tr -s ' ' ','| \
    awk -F, 'BEGIN {printf("No,UTM-Zone,UTM-Ch,UTM-East,UTM-North\n")} \
    {printf("%s,16,T,%s,%s\n",$1,$3,$2)}'>${tmpfile}4
gpsbabel -iunicsv,grid=utm -f ${tmpfile}4 -okml -F ${tmpfile}5

gpsbabel -ikml -f${tmpfile}3 -ikml -f${tmpfile}5 -okml,points=0 -F $5

rm ${tmpfile}{,2,3,4,5}
