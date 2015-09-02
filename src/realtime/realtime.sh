#!/usr/bin/env bash
# realtime.sh
# Martin Miller
# Created: 2015/08/28
# Runs SLAM from raw inputs
# realtime <gps input file> <img input dir> <pics file> <Q0> <R0> <P0>
export sen=$1
export imgdir=$2
export pics=$3
find . -type p | xargs rm
mkfifo acc.ff ang.ff att.ff quat.ff img.ff

grep CORRIMU $sen | \
    cut -d'*' -f1 | \
   awk -F, '{printf("%s,ACC,%0.9f,%0.9f,%0.9f\n",$4,200*$9,200*$8,-200*$10)}' > acc.ff &

grep CORRIMU $sen | \
    awk -F, '{printf("%s,ANG,%0.9f,%0.9f,%0.9f\n",$4,200*$6,200*$5,-200*$7)}' > ang.ff &

#paste -d,\
#    <(grep INSPVA $sen|cut -d, -f4) \
#    <(paste -d, \
#        <(grep INSPVA $sen | \
#            cut -d, -f11,12) \
#        <(grep INSPVA $sen | \
#            cud -d, -f13|wrap) | \
#        euler2qbw -d) | \
#        sed 's/,/,QUAT,/' > quat.ff &
sort -t, -n -m  \
    <( grep "CORRIMU" $sen | \
        awk -F, \
        '{printf("%s,ANG,%0.9f,%0.9f,%0.9f\n",$4,200*$6,200*$5,-200*$7)}') \
    <(paste -d, \
        <( grep "INSPVA" $sen | \
            cut -d, -f4) \
        <(paste -d, \
            <( grep INSPVA $sen | \
                cut -d, -f11,12) \
            <( grep INSPVA $sen | \
                cut -d, -f13 | \
                wrap) | \
            euler2qbw -d) | \
            sed 's/,/,QUAT,/') | \
    /home/marty/ARC/slam/src/quatKF/quatKF.py | \
    sed 's/,/,QUAT,/' > quat.ff &

#TODO: Incorporate this
rectify <(grep MARK2 $sen |cut -d, -f7|rmglitch|tail -n \
 +2|cut -d, -f1) <(grep "^1" $pics |cut -d, \
 -f2|proccam|cut -d' ' -f1) | sed 's/$/,IMG/' > img.ff &
#grep MARK2 $sen | \
#    cut -d, -f7 | \
#    rmglitch | \
#   cut -d, -f1 | \
#    sed 's/$/,IMG/' > img.ff &

sort -t, -n -m ang.ff acc.ff quat.ff img.ff | \
    slam <(find $2 -name "*-0.png"|sort| \
        ~/ARC/trackingPoints/bin/trackingPoints -f -m 12 100 | \
        sed -n '/--/,+20 p'|sed 's/--.*//'|sed -n '2,$ p'|img2body) 1e-4 1e-3 1e-5

