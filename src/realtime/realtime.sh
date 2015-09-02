#!/usr/bin/env bash
# realtime.sh
# Martin Miller
# Created: 2015/08/28
# Runs SLAM from raw inputs
# realtime <gps input file> <img input dir> <first line number> <Q0> <R0> <P0>
export sen=$1
export imgdir=$2
export line=$3
find . -type p | xargs rm
mkfifo acc.ff ang.ff att.ff quat.ff img.ff

tail -n+$line $sen | \
    grep CORRIMU | \
    cut -d'*' -f1 | \
   awk -F, '{printf("%s,ACC,%0.9f,%0.9f,%0.9f\n",$4,200*$9,200*$8,-200*$10)}' > acc.ff &

tail -n+$line $sen | \
    grep CORRIMU | \
    awk -F, '{printf("%s,ANG,%0.9f,%0.9f,%0.9f\n",$4,200*$6,200*$5,-200*$7)}' > ang.ff &

#paste -d, <(grep INSPVA $sen|cut -d, -f4) \
#    <(grep
sort -t, -n -m  \
    <( tail -n+$line $sen | \
        grep CORRIMU | \
        awk -F, \
        '{printf("%s,ANG,%0.9f,%0.9f,%0.9f\n",$4,200*$6,200*$5,-200*$7)}') \
    <(paste -d, \
        <(tail -n+$line $sen | \
            grep INSPVA | \
            cut -d, -f4) \
        <(paste -d, \
            <(tail -n+$line $sen | \
                grep INSPVA | \
                cut -d, -f11,12) \
            <(tail -n+$line $sen | \
                grep INSPVA | \
                cut -d, -f13 | \
                wrap) | \
            euler2qbw -d) | \
            sed 's/,/,QUAT,/') | \
    /home/marty/ARC/slam/src/quatKF/quatKF.py | \
    sed 's/,/,QUAT,/' > quat.ff &

#TODO: Incorporate this
rectify <(grep MARK2 2015-05-06T17\:28-0500.gps |cut -d, -f7|rmglitch|tail -n \
 +2|cut -d, -f1) <(grep "^1" 2015-05-06T17\:28-0500.pics |cut -d, \
 -f2|proccam|cut -d' ' -f1) | sed 's/$/,IMG/' > img.ff &
#tail -n+$line $sen | \
#    grep MARK2 | \
#    cut -d, -f7 | \
#    rmglitch | \
#   cut -d, -f1 | \
#    sed 's/$/,IMG/' > img.ff &

sort -t, -n -m ang.ff acc.ff quat.ff img.ff | \
    slam <(find $2 -name "*-0.png"|sort| \
        ~/ARC/trackingPoints/bin/trackingPoints -f -m 12 100 | \
        sed -n '/--/,+10 p'|sed 's/--.*//'|sed -n '2,$ p'|img2body) 1e-4 1e-3 1e-5






#sort -t, -n -m \
#    <(tail -n+$3 $1| \
#        grep CORRIMU |\
#        cut -d'*' -f1|\
#        awk -F, '{printf("%s,ACC,%0.9f,%0.9f,%0.9f\n",$4,200*$9,200*$8,-200*$10)}') \
#    <(tail -n +$3 $1|\
#        grep CORRIMU |\
#        awk -F, '{printf("%s,ANG,%0.9f,%0.9f,%0.9f\n",$4,200*$6,200*$5,-200*$7)}') \
#    <(paste -d, \
#        <(tail -n+$line $sen| \
#            grep INSPVA| \
#            cut -d, -f4) \
#        <(paste -d, \
#            <(tail -n+$line $sen| \
#            grep INSPVA| \
#            cut -d, -f11,12) \
#            <(tail -n+$line $sen|\
#            grep INSPVA|\
#            cut -d, -f13|\
#            wrap) | \
#            euler2qbw -d)|  \
#        sed 's/,/,QUAT,/') \
#    <(tail -n+$3 $1|\
#        grep MARK2| \
#        cut -d, -f7| \
#        rmglitch| \
#        cut -d, -f1| \
#        sed 's/$/,IMG/') |
#        slam <(find $2 -name "*-0.png"|sort| \
#            ~/ARC/trackingPoints/bin/trackingPoints -f -m 12 100 | \
#            sed -n '/--/,+25 p'|sed 's/--.*//'|sed -n '2,$ p'|img2body) 1e-2 1e-3 1e-6
#
#
#
